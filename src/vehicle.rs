use crate::ardupilot::ArduPilotDomain;
use crate::command::Command;
use crate::config::VehicleConfig;
use crate::dialect;
use crate::error::VehicleError;
use crate::event_loop::{InitManager, run_event_loop_with_init};
use crate::fence::FenceDomain;
use crate::geo::{GeoPoint3dMsl, try_latitude_e7, try_longitude_e7};
use crate::info::{InfoDomain, InfoHandle};
use crate::mission::{MissionDomain, MissionProtocolScope, send_domain_command};
use crate::modes::{CurrentMode, ModeDomain, ModesHandle, mode_number};
use crate::observation::{MetricHandle, MetricSample, ObservationHandle, ObservationSubscription};
use crate::params::ParamsDomain;
use crate::rally::RallyDomain;
use crate::raw::RawHandle;
use crate::state::{LinkState, StateChannels, VehicleState, create_channels};
use crate::support::{SupportDomain, SupportHandle};
use crate::telemetry::TelemetryHandle;
use std::sync::Arc;
use std::time::Instant;
use tokio::sync::{mpsc, oneshot};
use tokio_util::sync::CancellationToken;

pub use crate::ardupilot::ArduPilotHandle;
pub use crate::fence::FenceHandle;
pub use crate::mission::MissionHandle;
pub use crate::params::ParamsHandle;
pub use crate::rally::RallyHandle;
pub use crate::state::{AutopilotType, VehicleType};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct WireNormalizedGeo {
    latitude_e7: i32,
    longitude_e7: i32,
    altitude_mm: i32,
}

impl TryFrom<&GeoPoint3dMsl> for WireNormalizedGeo {
    type Error = VehicleError;

    fn try_from(point: &GeoPoint3dMsl) -> Result<Self, Self::Error> {
        Ok(Self {
            latitude_e7: try_latitude_e7(point.latitude_deg)?,
            longitude_e7: try_longitude_e7(point.longitude_deg)?,
            altitude_mm: quantize_meters_mm(point.altitude_msl_m),
        })
    }
}

impl WireNormalizedGeo {
    fn matches(self, observed: &GeoPoint3dMsl) -> bool {
        // observed comes from wire-decoded data — valid by construction.
        self.latitude_e7 == crate::geo::quantize_degrees_e7(observed.latitude_deg)
            && self.longitude_e7 == crate::geo::quantize_degrees_e7(observed.longitude_deg)
            && self.altitude_mm == quantize_meters_mm(observed.altitude_msl_m)
    }
}

fn quantize_meters_mm(value: f64) -> i32 {
    (value * 1000.0).round() as i32
}

#[derive(Clone)]
/// Root handle for a live MAVLink connection and all MAVKit domains.
pub struct Vehicle {
    pub(crate) inner: Arc<VehicleInner>,
}

pub(crate) struct VehicleInner {
    pub(crate) command_tx: mpsc::Sender<Command>,
    cancel: CancellationToken,
    pub(crate) stores: StateChannels,
    pub(crate) mission_protocol: MissionProtocolScope,
    pub(crate) mission: MissionDomain,
    pub(crate) params: ParamsDomain,
    pub(crate) fence: FenceDomain,
    pub(crate) rally: RallyDomain,
    pub(crate) ardupilot: ArduPilotDomain,
    pub(crate) info: InfoDomain,
    pub(crate) modes: ModeDomain,
    pub(crate) support: SupportDomain,
    pub(crate) _config: VehicleConfig,
}

impl Drop for VehicleInner {
    fn drop(&mut self) {
        self.cancel.cancel();
    }
}

impl VehicleInner {
    pub(crate) async fn set_mode(
        &self,
        custom_mode: u32,
        wait_for_observation: bool,
    ) -> Result<(), VehicleError> {
        send_domain_command(self.command_tx.clone(), |reply| Command::SetMode {
            custom_mode,
            reply,
        })
        .await?;

        if wait_for_observation {
            self.wait_for_mode_observation(custom_mode).await?;
        }

        Ok(())
    }

    pub(crate) async fn set_mode_by_name(
        &self,
        name: &str,
        wait_for_observation: bool,
    ) -> Result<(), VehicleError> {
        let custom_mode = self
            .resolve_mode_by_name(name)
            .ok_or_else(|| VehicleError::ModeNotAvailable(name.to_string()))?;

        self.set_mode(custom_mode, wait_for_observation).await
    }

    pub(crate) fn resolve_mode_by_name(&self, name: &str) -> Option<u32> {
        let requested = name.trim();
        if requested.is_empty() {
            return None;
        }

        let catalog = self.modes.catalog().latest().unwrap_or_default();
        if let Some(custom_mode) = catalog.into_iter().find_map(|mode| {
            mode.name
                .eq_ignore_ascii_case(requested)
                .then_some(mode.custom_mode)
        }) {
            return Some(custom_mode);
        }

        let identity = self.stores.vehicle_state.borrow().clone();
        mode_number(identity.autopilot, identity.vehicle_type, requested)
    }

    pub(crate) async fn wait_for_mode_observation(
        &self,
        custom_mode: u32,
    ) -> Result<(), VehicleError> {
        let current_mode = self.modes.current();
        if current_mode
            .latest()
            .is_some_and(|mode| mode.custom_mode == custom_mode)
        {
            return Ok(());
        }

        let mut subscription = current_mode.subscribe();
        let wait_for_match = async {
            while let Some(observed) = subscription.recv().await {
                if observed.custom_mode == custom_mode {
                    return Ok(());
                }
            }

            Err(VehicleError::Disconnected)
        };

        tokio::time::timeout(self._config.command_completion_timeout, wait_for_match)
            .await
            .map_err(|_| VehicleError::Timeout("waiting for mode change".into()))?
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// Snapshot identity from the first usable heartbeat.
///
/// This stays stable for a session unless the link switches to a different vehicle.
pub struct VehicleIdentity {
    pub system_id: u8,
    pub component_id: u8,
    pub autopilot: AutopilotType,
    pub vehicle_type: VehicleType,
}

trait DomainHandle<'a>: Sized {
    fn from_inner(inner: &'a VehicleInner) -> Self;
}

impl<'a> DomainHandle<'a> for MissionHandle<'a> {
    fn from_inner(inner: &'a VehicleInner) -> Self {
        MissionHandle::new(inner)
    }
}

impl<'a> DomainHandle<'a> for FenceHandle<'a> {
    fn from_inner(inner: &'a VehicleInner) -> Self {
        FenceHandle::new(inner)
    }
}

impl<'a> DomainHandle<'a> for RallyHandle<'a> {
    fn from_inner(inner: &'a VehicleInner) -> Self {
        RallyHandle::new(inner)
    }
}

impl<'a> DomainHandle<'a> for ArduPilotHandle<'a> {
    fn from_inner(inner: &'a VehicleInner) -> Self {
        ArduPilotHandle::new(inner)
    }
}

impl<'a> DomainHandle<'a> for ModesHandle<'a> {
    fn from_inner(inner: &'a VehicleInner) -> Self {
        Self::new(inner)
    }
}

impl<'a> DomainHandle<'a> for InfoHandle<'a> {
    fn from_inner(inner: &'a VehicleInner) -> Self {
        Self::new(inner)
    }
}

impl<'a> DomainHandle<'a> for SupportHandle<'a> {
    fn from_inner(inner: &'a VehicleInner) -> Self {
        SupportHandle::new(inner)
    }
}

impl<'a> DomainHandle<'a> for ParamsHandle<'a> {
    fn from_inner(inner: &'a VehicleInner) -> Self {
        ParamsHandle::new(inner)
    }
}

impl Vehicle {
    pub async fn connect(address: &str) -> Result<Self, VehicleError> {
        Self::connect_with_config(address, VehicleConfig::default()).await
    }

    pub async fn connect_udp(bind_addr: &str) -> Result<Self, VehicleError> {
        Self::connect(&format!("udpin:{bind_addr}")).await
    }

    pub async fn connect_tcp(addr: &str) -> Result<Self, VehicleError> {
        Self::connect(&format!("tcpin:{addr}")).await
    }

    pub async fn connect_serial(port: &str, baud: u32) -> Result<Self, VehicleError> {
        Self::connect(&format!("serial:{port}:{baud}")).await
    }

    pub async fn connect_with_config(
        address: &str,
        config: VehicleConfig,
    ) -> Result<Self, VehicleError> {
        let connection = mavlink::connect_async::<dialect::MavMessage>(address)
            .await
            .map_err(|err| VehicleError::ConnectionFailed(err.to_string()))?;

        Self::from_connection(connection, config).await
    }

    pub async fn from_connection(
        connection: Box<dyn mavlink::AsyncMavConnection<dialect::MavMessage> + Sync + Send>,
        config: VehicleConfig,
    ) -> Result<Self, VehicleError> {
        if config.command_buffer_size == 0 {
            return Err(VehicleError::InvalidParameter(
                "command_buffer_size must be at least 1".to_string(),
            ));
        }
        let (writers, stores) = create_channels();
        let cancel = CancellationToken::new();
        let (command_tx, command_rx) = mpsc::channel(config.command_buffer_size);
        let connect_timeout = config.connect_timeout;
        let init_manager = InitManager::new(config.clone());
        let mission_protocol = MissionProtocolScope::new();
        let mission = MissionDomain::new();
        let params = ParamsDomain::new();
        let fence = FenceDomain::new();
        let rally = RallyDomain::new();
        let ardupilot = ArduPilotDomain::new();
        let info = InfoDomain::new();
        let modes = ModeDomain::new();
        let support = SupportDomain::new();

        mission.start(&stores, cancel.clone());
        ardupilot.start(&stores, cancel.clone());
        info.start(&stores, &init_manager);
        modes.start(&stores, command_tx.clone());
        support.start(&stores, &init_manager);

        tokio::spawn(run_event_loop_with_init(
            connection,
            command_rx,
            writers,
            config.clone(),
            init_manager,
            cancel.clone(),
        ));

        let vehicle = Self {
            inner: Arc::new(VehicleInner {
                command_tx,
                cancel,
                stores,
                mission_protocol,
                mission,
                params,
                fence,
                rally,
                ardupilot,
                info,
                modes,
                support,
                _config: config,
            }),
        };

        let mut vehicle_state = vehicle.inner.stores.vehicle_state.clone();
        let mut link_state = vehicle.inner.stores.link_state.clone();
        let ready_wait = async {
            loop {
                match link_state.borrow().clone() {
                    LinkState::Connecting | LinkState::Connected => {}
                    LinkState::Disconnected => return Err(VehicleError::Disconnected),
                    LinkState::Error(message) => {
                        return Err(VehicleError::ConnectionFailed(message));
                    }
                }

                if vehicle_state.borrow().heartbeat_received {
                    return Ok::<(), VehicleError>(());
                }

                tokio::select! {
                    changed = vehicle_state.changed() => {
                        if changed.is_err() {
                            match link_state.borrow().clone() {
                                LinkState::Error(message) => {
                                    return Err(VehicleError::ConnectionFailed(message));
                                }
                                _ => return Err(VehicleError::Disconnected),
                            }
                        }
                    }
                    changed = link_state.changed() => {
                        if changed.is_err() {
                            return Err(VehicleError::Disconnected);
                        }

                        match link_state.borrow().clone() {
                            LinkState::Connecting | LinkState::Connected => {}
                            LinkState::Disconnected => return Err(VehicleError::Disconnected),
                            LinkState::Error(message) => {
                                return Err(VehicleError::ConnectionFailed(message));
                            }
                        }
                    }
                }
            }
        };

        tokio::select! {
            result = ready_wait => result?,
            _ = tokio::time::sleep(connect_timeout) => return Err(VehicleError::Timeout("connecting to vehicle".into())),
        }

        Ok(vehicle)
    }

    fn handle<'a, H>(&'a self) -> H
    where
        H: DomainHandle<'a>,
    {
        H::from_inner(self.inner.as_ref())
    }

    pub fn identity(&self) -> VehicleIdentity {
        let state: VehicleState = self.inner.stores.vehicle_state.borrow().clone();

        VehicleIdentity {
            system_id: state.system_id,
            component_id: state.component_id,
            autopilot: state.autopilot,
            vehicle_type: state.vehicle_type,
        }
    }

    pub fn info(&self) -> InfoHandle<'_> {
        self.handle()
    }

    pub fn support(&self) -> SupportHandle<'_> {
        self.handle()
    }

    pub fn available_modes(&self) -> ModesHandle<'_> {
        self.handle()
    }

    pub fn telemetry(&self) -> TelemetryHandle<'_> {
        TelemetryHandle::with_command_tx(
            &self.inner.stores.telemetry_handles,
            &self.inner.command_tx,
        )
    }

    pub fn mission(&self) -> MissionHandle<'_> {
        self.handle()
    }

    pub fn fence(&self) -> FenceHandle<'_> {
        self.handle()
    }

    pub fn rally(&self) -> RallyHandle<'_> {
        self.handle()
    }

    pub fn params(&self) -> ParamsHandle<'_> {
        self.handle()
    }

    pub fn raw(&self) -> RawHandle<'_> {
        RawHandle::new(self)
    }

    pub fn ardupilot(&self) -> ArduPilotHandle<'_> {
        self.handle()
    }

    /// Shortcut for `vehicle.available_modes().current()`.
    pub fn current_mode(&self) -> ObservationHandle<CurrentMode> {
        self.available_modes().current()
    }

    /// Shortcut for `vehicle.telemetry().home()`.
    pub fn home(&self) -> MetricHandle<GeoPoint3dMsl> {
        self.telemetry().home()
    }

    /// Shortcut for `vehicle.telemetry().origin()`.
    pub fn origin(&self) -> MetricHandle<GeoPoint3dMsl> {
        self.telemetry().origin()
    }

    pub async fn arm(&self, force: bool) -> Result<(), VehicleError> {
        self.send_command(|reply| Command::Arm { force, reply })
            .await
    }

    pub async fn disarm(&self, force: bool) -> Result<(), VehicleError> {
        self.send_command(|reply| Command::Disarm { force, reply })
            .await
    }

    pub async fn set_mode(
        &self,
        custom_mode: u32,
        wait_for_observation: bool,
    ) -> Result<(), VehicleError> {
        self.inner.set_mode(custom_mode, wait_for_observation).await
    }

    pub async fn set_mode_by_name(
        &self,
        name: &str,
        wait_for_observation: bool,
    ) -> Result<(), VehicleError> {
        self.inner
            .set_mode_by_name(name, wait_for_observation)
            .await
    }

    pub async fn set_home(&self, position: GeoPoint3dMsl) -> Result<(), VehicleError> {
        let lat_e7 = try_latitude_e7(position.latitude_deg)?;
        let lon_e7 = try_longitude_e7(position.longitude_deg)?;
        self.send_command(|reply| Command::RawCommandInt {
            payload: crate::command::RawCommandIntPayload {
                command: dialect::MavCmd::MAV_CMD_DO_SET_HOME,
                frame: dialect::MavFrame::MAV_FRAME_GLOBAL,
                current: 0,
                autocontinue: 0,
                params: [0.0, 0.0, 0.0, 0.0],
                x: lat_e7,
                y: lon_e7,
                // Altitude narrowed to f32 at wire boundary (MAVLink z field).
                z: position.altitude_msl_m as f32,
            },
            reply,
        })
        .await
        .map(|_| ())
    }

    #[allow(deprecated)]
    pub async fn set_home_current(&self) -> Result<(), VehicleError> {
        self.send_command(|reply| Command::Long {
            command: dialect::MavCmd::MAV_CMD_DO_SET_HOME,
            params: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            reply,
        })
        .await
    }

    pub async fn set_origin(&self, origin: GeoPoint3dMsl) -> Result<(), VehicleError> {
        let normalized = WireNormalizedGeo::try_from(&origin)?;
        let origin_handle = self.telemetry().origin();
        let mut subscription = origin_handle.subscribe();

        let sent_after = self
            .send_command(|reply| Command::SetOrigin {
                latitude: normalized.latitude_e7,
                longitude: normalized.longitude_e7,
                altitude: normalized.altitude_mm,
                reply,
            })
            .await?;

        self.wait_for_origin_observation(origin_handle, &mut subscription, normalized, sent_after)
            .await
    }

    pub async fn disconnect(&self) -> Result<(), VehicleError> {
        let mut link_state = self.inner.stores.link_state.clone();

        if matches!(
            link_state.borrow().clone(),
            LinkState::Disconnected | LinkState::Error(_)
        ) {
            return Ok(());
        }

        self.inner
            .command_tx
            .send(Command::Shutdown)
            .await
            .map_err(|_| VehicleError::Disconnected)?;

        loop {
            let current = link_state.borrow().clone();
            match current {
                LinkState::Disconnected | LinkState::Error(_) => return Ok(()),
                LinkState::Connecting | LinkState::Connected => {
                    link_state
                        .changed()
                        .await
                        .map_err(|_| VehicleError::Disconnected)?;
                }
            }
        }
    }

    async fn wait_for_origin_observation(
        &self,
        origin_handle: MetricHandle<GeoPoint3dMsl>,
        subscription: &mut ObservationSubscription<MetricSample<GeoPoint3dMsl>>,
        requested: WireNormalizedGeo,
        sent_after: Instant,
    ) -> Result<(), VehicleError> {
        if origin_handle.latest().is_some_and(|sample| {
            sample.received_at >= sent_after && requested.matches(&sample.value)
        }) {
            return Ok(());
        }

        let wait_for_match = async {
            while let Some(observed) = subscription.recv().await {
                if observed.received_at >= sent_after && requested.matches(&observed.value) {
                    return Ok(());
                }
            }

            Err(VehicleError::Disconnected)
        };

        tokio::time::timeout(
            self.inner._config.command_completion_timeout,
            wait_for_match,
        )
        .await
        .map_err(|_| VehicleError::Timeout("sending command".into()))?
    }

    pub(crate) async fn send_command<T>(
        &self,
        make: impl FnOnce(oneshot::Sender<Result<T, VehicleError>>) -> Command,
    ) -> Result<T, VehicleError> {
        let (tx, rx) = oneshot::channel();
        self.inner
            .command_tx
            .send(make(tx))
            .await
            .map_err(|_| VehicleError::Disconnected)?;

        rx.await.map_err(|_| VehicleError::Disconnected)?
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dialect::{self, MavModeFlag, MavState};
    use crate::geo::quantize_degrees_e7;
    use crate::test_support::{MockConnection, SentMessages};
    use mavlink::MavHeader;
    use std::sync::Arc;
    use std::time::Duration;
    use tokio::time::timeout;

    fn default_header() -> MavHeader {
        MavHeader {
            system_id: 1,
            component_id: 1,
            sequence: 0,
        }
    }

    fn heartbeat_msg(armed: bool, custom_mode: u32) -> dialect::MavMessage {
        let mut base_mode = MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        if armed {
            base_mode |= MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED;
        }

        dialect::MavMessage::HEARTBEAT(dialect::HEARTBEAT_DATA {
            custom_mode,
            mavtype: dialect::MavType::MAV_TYPE_QUADROTOR,
            autopilot: dialect::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode,
            system_status: MavState::MAV_STATE_STANDBY,
            mavlink_version: 3,
        })
    }

    fn fast_config() -> VehicleConfig {
        VehicleConfig {
            connect_timeout: Duration::from_millis(150),
            command_timeout: Duration::from_millis(50),
            command_completion_timeout: Duration::from_millis(150),
            auto_request_home: false,
            ..VehicleConfig::default()
        }
    }

    fn ack_msg(command: dialect::MavCmd, result: dialect::MavResult) -> dialect::MavMessage {
        dialect::MavMessage::COMMAND_ACK(dialect::COMMAND_ACK_DATA {
            command,
            result,
            progress: 0,
            result_param2: 0,
            target_system: 0,
            target_component: 0,
        })
    }

    fn gps_global_origin_msg(latitude: i32, longitude: i32, altitude: i32) -> dialect::MavMessage {
        dialect::MavMessage::GPS_GLOBAL_ORIGIN(dialect::GPS_GLOBAL_ORIGIN_DATA {
            latitude,
            longitude,
            altitude,
            time_usec: 0,
        })
    }

    fn available_modes_msg(
        number_modes: u8,
        mode_index: u8,
        custom_mode: u32,
        mode_name: &str,
    ) -> dialect::MavMessage {
        dialect::MavMessage::AVAILABLE_MODES(dialect::AVAILABLE_MODES_DATA {
            custom_mode,
            properties: dialect::MavModeProperty::empty(),
            number_modes,
            mode_index,
            standard_mode: dialect::MavStandardMode::MAV_STANDARD_MODE_NON_STANDARD,
            mode_name: mode_name.into(),
        })
    }

    async fn connect_mock_vehicle() -> (Vehicle, mpsc::Sender<(MavHeader, dialect::MavMessage)>) {
        let (vehicle, msg_tx, _sent) = connect_mock_vehicle_with_sent().await;
        (vehicle, msg_tx)
    }

    async fn connect_mock_vehicle_with_sent() -> (
        Vehicle,
        mpsc::Sender<(MavHeader, dialect::MavMessage)>,
        SentMessages,
    ) {
        let (msg_tx, msg_rx) = mpsc::channel(16);
        let (conn, sent) = MockConnection::new(msg_rx);
        let connect_task =
            tokio::spawn(
                async move { Vehicle::from_connection(Box::new(conn), fast_config()).await },
            );

        msg_tx
            .send((default_header(), heartbeat_msg(false, 7)))
            .await
            .expect("heartbeat should be delivered to the mock connection");

        let vehicle = timeout(Duration::from_millis(250), connect_task)
            .await
            .expect("connect should complete after first heartbeat")
            .expect("connect task should join")
            .expect("mock vehicle should connect");

        (vehicle, msg_tx, sent)
    }

    async fn wait_for_metric_disconnect<T: Clone + Send + Sync + 'static>(
        metric: crate::observation::MetricHandle<T>,
    ) -> Result<(), VehicleError> {
        metric.wait().await.map(|_| ())
    }

    fn test_vehicle_with_command_rx() -> (Vehicle, mpsc::Receiver<Command>) {
        let (_, stores) = create_channels();
        let cancel = CancellationToken::new();
        let (command_tx, command_rx) = mpsc::channel(1);

        (
            Vehicle {
                inner: Arc::new(VehicleInner {
                    command_tx,
                    cancel,
                    stores,
                    mission_protocol: MissionProtocolScope::new(),
                    mission: MissionDomain::new(),
                    params: ParamsDomain::new(),
                    fence: FenceDomain::new(),
                    rally: RallyDomain::new(),
                    ardupilot: ArduPilotDomain::new(),
                    info: InfoDomain::new(),
                    modes: ModeDomain::new(),
                    support: SupportDomain::new(),
                    _config: VehicleConfig::default(),
                }),
            },
            command_rx,
        )
    }

    fn dummy_vehicle() -> Vehicle {
        let (vehicle, _command_rx) = test_vehicle_with_command_rx();
        vehicle
    }

    #[test]
    fn send_sync_bounds() {
        fn assert_bounds<T: Clone + Send + Sync>() {}
        assert_bounds::<Vehicle>();
    }

    #[test]
    fn clone_shares_inner() {
        let vehicle = dummy_vehicle();
        let cloned = vehicle.clone();

        assert!(Arc::ptr_eq(&vehicle.inner, &cloned.inner));
    }

    #[test]
    fn identity_is_sync_value() {
        let vehicle = dummy_vehicle();
        let identity: VehicleIdentity = vehicle.identity();

        assert_eq!(identity.system_id, 0);
        assert_eq!(identity.component_id, 0);
        assert_eq!(identity.autopilot, AutopilotType::Unknown);
        assert_eq!(identity.vehicle_type, VehicleType::Unknown);
    }

    #[test]
    fn root_domain_accessors_compile() {
        let vehicle = dummy_vehicle();

        let _ = vehicle.info();
        let _ = vehicle.support();
        let _ = vehicle.available_modes();
        let _ = vehicle.telemetry();
        let _ = vehicle.mission();
        let _ = vehicle.fence();
        let _ = vehicle.rally();
        let _ = vehicle.params();
        let _ = vehicle.raw();
        let _ = vehicle.ardupilot();
    }

    #[tokio::test]
    async fn info_handle_defaults_to_sys_locator_until_metadata_arrives() {
        let (vehicle, msg_tx) = connect_mock_vehicle().await;

        assert_eq!(vehicle.info().best_unique_id(), None);
        assert_eq!(vehicle.info().best_display_id(), "sys:1/1");
        assert_eq!(
            timeout(
                Duration::from_millis(100),
                vehicle.info().persistent_identity().wait()
            )
            .await
            .expect("pending identity should be published")
            .expect("pending identity should be readable"),
            crate::PersistentIdentity::Pending {
                system_id: 1,
                component_id: 1,
            }
        );

        vehicle
            .disconnect()
            .await
            .expect("disconnect should succeed");
        drop(msg_tx);
    }

    #[tokio::test]
    async fn info_handle_uses_uid_precedence_and_upgrades_to_uid2() {
        let (vehicle, msg_tx) = connect_mock_vehicle().await;
        let uid = 0x0123_4567_89ab_cdef_u64;
        let uid2 = [
            0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
        ];

        msg_tx
            .send((
                default_header(),
                dialect::MavMessage::AUTOPILOT_VERSION(dialect::AUTOPILOT_VERSION_DATA {
                    uid,
                    flight_sw_version: 0x01020304,
                    os_sw_version: 0x05060708,
                    board_version: 77,
                    vendor_id: 42,
                    product_id: 99,
                    flight_custom_version: [0xde, 0xad, 0xbe, 0xef, 0xca, 0xfe, 0xba, 0xbe],
                    ..dialect::AUTOPILOT_VERSION_DATA::default()
                }),
            ))
            .await
            .expect("autopilot version should be delivered");

        let firmware = timeout(Duration::from_millis(250), vehicle.info().firmware().wait())
            .await
            .expect("firmware info should arrive")
            .expect("firmware info should be readable");
        assert_eq!(firmware.version.as_deref(), Some("1.2.3"));
        assert_eq!(firmware.os_version.as_deref(), Some("5.6.7"));
        assert_eq!(firmware.git_hash.as_deref(), Some("deadbeefcafebabe"));

        let hardware = timeout(Duration::from_millis(250), vehicle.info().hardware().wait())
            .await
            .expect("hardware info should arrive")
            .expect("hardware info should be readable");
        assert_eq!(hardware.board_vendor_id, Some(42));
        assert_eq!(hardware.board_product_id, Some(99));
        assert_eq!(hardware.board_version, Some(77));

        let unique_ids = timeout(
            Duration::from_millis(250),
            vehicle.info().unique_ids().wait(),
        )
        .await
        .expect("unique ids should arrive")
        .expect("unique ids should be readable");
        assert_eq!(unique_ids.uid, Some(uid));
        assert_eq!(
            vehicle.info().best_unique_id().as_deref(),
            Some("uid64:0123456789abcdef")
        );
        assert_eq!(vehicle.info().best_display_id(), "uid64:0123456789abcdef");
        assert_eq!(
            vehicle.info().persistent_identity().latest(),
            Some(crate::PersistentIdentity::Ready {
                canonical_id: "uid64:0123456789abcdef".into(),
                aliases: vec!["uid64:0123456789abcdef".into()],
            })
        );

        msg_tx
            .send((
                default_header(),
                dialect::MavMessage::AUTOPILOT_VERSION(dialect::AUTOPILOT_VERSION_DATA {
                    uid,
                    uid2,
                    flight_sw_version: 0x01020304,
                    os_sw_version: 0x05060708,
                    board_version: 77,
                    vendor_id: 42,
                    product_id: 99,
                    flight_custom_version: [0xde, 0xad, 0xbe, 0xef, 0xca, 0xfe, 0xba, 0xbe],
                    ..dialect::AUTOPILOT_VERSION_DATA::default()
                }),
            ))
            .await
            .expect("upgraded autopilot version should be delivered");

        timeout(Duration::from_millis(250), async {
            loop {
                if vehicle.info().best_display_id() == "mcu:00112233445566778899aabb000000000000" {
                    break;
                }
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
        })
        .await
        .expect("uid2 upgrade should be observed");

        assert_eq!(
            vehicle.info().best_unique_id().as_deref(),
            Some("mcu:00112233445566778899aabb000000000000")
        );
        assert_eq!(
            vehicle.info().persistent_identity().latest(),
            Some(crate::PersistentIdentity::Ready {
                canonical_id: "mcu:00112233445566778899aabb000000000000".into(),
                aliases: vec![
                    "mcu:00112233445566778899aabb000000000000".into(),
                    "uid64:0123456789abcdef".into(),
                ],
            })
        );

        vehicle
            .disconnect()
            .await
            .expect("disconnect should succeed");
    }

    #[tokio::test]
    async fn arm_ack_flow() {
        let (vehicle, msg_tx, sent) = connect_mock_vehicle_with_sent().await;

        let arm_task = {
            let vehicle = vehicle.clone();
            tokio::spawn(async move { vehicle.arm(false).await })
        };

        tokio::time::sleep(Duration::from_millis(10)).await;
        msg_tx
            .send((
                default_header(),
                ack_msg(
                    dialect::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
                    dialect::MavResult::MAV_RESULT_ACCEPTED,
                ),
            ))
            .await
            .expect("arm ack should be delivered");

        assert!(arm_task.await.expect("arm task should join").is_ok());

        let arm_sent = sent
            .lock()
            .expect("sent messages lock should not poison")
            .iter()
            .find_map(|(_, msg)| match msg {
                dialect::MavMessage::COMMAND_LONG(data)
                    if data.command == dialect::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM =>
                {
                    Some(data.clone())
                }
                _ => None,
            })
            .expect("arm command should be sent");

        assert_eq!(arm_sent.param1, 1.0);
        assert_eq!(arm_sent.param2, 0.0);

        let disarm_task = {
            let vehicle = vehicle.clone();
            tokio::spawn(async move { vehicle.disarm(false).await })
        };

        tokio::time::sleep(Duration::from_millis(10)).await;
        msg_tx
            .send((
                default_header(),
                ack_msg(
                    dialect::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
                    dialect::MavResult::MAV_RESULT_DENIED,
                ),
            ))
            .await
            .expect("disarm ack should be delivered");

        assert!(matches!(
            disarm_task.await.expect("disarm task should join"),
            Err(VehicleError::CommandRejected {
                command,
                result: crate::error::CommandResult::Denied,
        }) if command == dialect::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM as u16
        ));

        vehicle
            .disconnect()
            .await
            .expect("disconnect should succeed");
    }

    #[tokio::test]
    async fn set_mode_waits_for_observation() {
        let (vehicle, msg_tx, _) = connect_mock_vehicle_with_sent().await;

        let set_mode_ack_only = {
            let vehicle = vehicle.clone();
            tokio::spawn(async move { vehicle.set_mode(4, false).await })
        };

        tokio::time::sleep(Duration::from_millis(10)).await;
        msg_tx
            .send((
                default_header(),
                ack_msg(
                    dialect::MavCmd::MAV_CMD_DO_SET_MODE,
                    dialect::MavResult::MAV_RESULT_ACCEPTED,
                ),
            ))
            .await
            .expect("set mode ack should be delivered");

        assert!(
            set_mode_ack_only
                .await
                .expect("set mode ack-only task should join")
                .is_ok()
        );

        let set_mode_waiting = {
            let vehicle = vehicle.clone();
            tokio::spawn(async move { vehicle.set_mode(6, true).await })
        };

        tokio::time::sleep(Duration::from_millis(10)).await;
        msg_tx
            .send((
                default_header(),
                ack_msg(
                    dialect::MavCmd::MAV_CMD_DO_SET_MODE,
                    dialect::MavResult::MAV_RESULT_ACCEPTED,
                ),
            ))
            .await
            .expect("set mode wait ack should be delivered");

        tokio::time::sleep(Duration::from_millis(40)).await;
        assert!(
            !set_mode_waiting.is_finished(),
            "set_mode(..., true) should wait for observed mode after ACK"
        );

        msg_tx
            .send((default_header(), heartbeat_msg(false, 6)))
            .await
            .expect("confirming heartbeat should be delivered");

        assert!(
            set_mode_waiting
                .await
                .expect("set mode wait task should join")
                .is_ok()
        );

        let rejected = {
            let vehicle = vehicle.clone();
            tokio::spawn(async move { vehicle.set_mode(5, false).await })
        };

        tokio::time::sleep(Duration::from_millis(10)).await;
        msg_tx
            .send((
                default_header(),
                ack_msg(
                    dialect::MavCmd::MAV_CMD_DO_SET_MODE,
                    dialect::MavResult::MAV_RESULT_DENIED,
                ),
            ))
            .await
            .expect("rejected mode ack should be delivered");

        assert!(matches!(
            rejected.await.expect("rejected mode task should join"),
            Err(VehicleError::CommandRejected {
                command,
                result: crate::error::CommandResult::Denied,
        }) if command == dialect::MavCmd::MAV_CMD_DO_SET_MODE as u16
        ));

        vehicle
            .disconnect()
            .await
            .expect("disconnect should succeed");
    }

    #[tokio::test]
    async fn set_mode_by_name_resolve() {
        let (vehicle, msg_tx, sent) = connect_mock_vehicle_with_sent().await;

        let static_lookup = {
            let vehicle = vehicle.clone();
            tokio::spawn(async move { vehicle.set_mode_by_name("guided", false).await })
        };

        tokio::time::sleep(Duration::from_millis(10)).await;
        msg_tx
            .send((
                default_header(),
                ack_msg(
                    dialect::MavCmd::MAV_CMD_DO_SET_MODE,
                    dialect::MavResult::MAV_RESULT_ACCEPTED,
                ),
            ))
            .await
            .expect("static lookup ack should be delivered");

        assert!(
            static_lookup
                .await
                .expect("static lookup task should join")
                .is_ok()
        );

        let static_mode = sent
            .lock()
            .expect("sent messages lock should not poison")
            .iter()
            .rev()
            .find_map(|(_, msg)| match msg {
                dialect::MavMessage::COMMAND_LONG(data)
                    if data.command == dialect::MavCmd::MAV_CMD_DO_SET_MODE =>
                {
                    Some(data.param2)
                }
                _ => None,
            })
            .expect("static mode command should be sent") as u32;
        assert_eq!(static_mode, 4);

        msg_tx
            .send((default_header(), available_modes_msg(1, 1, 27, "AUTO_RTL")))
            .await
            .expect("available modes update should be delivered");
        tokio::time::sleep(Duration::from_millis(20)).await;

        let dynamic_lookup = {
            let vehicle = vehicle.clone();
            tokio::spawn(async move { vehicle.set_mode_by_name("AUTO_RTL", false).await })
        };

        tokio::time::sleep(Duration::from_millis(10)).await;
        msg_tx
            .send((
                default_header(),
                ack_msg(
                    dialect::MavCmd::MAV_CMD_DO_SET_MODE,
                    dialect::MavResult::MAV_RESULT_ACCEPTED,
                ),
            ))
            .await
            .expect("dynamic lookup ack should be delivered");

        assert!(
            dynamic_lookup
                .await
                .expect("dynamic lookup task should join")
                .is_ok()
        );

        let dynamic_mode = sent
            .lock()
            .expect("sent messages lock should not poison")
            .iter()
            .rev()
            .find_map(|(_, msg)| match msg {
                dialect::MavMessage::COMMAND_LONG(data)
                    if data.command == dialect::MavCmd::MAV_CMD_DO_SET_MODE =>
                {
                    Some(data.param2)
                }
                _ => None,
            })
            .expect("dynamic mode command should be sent") as u32;
        assert_eq!(dynamic_mode, 27);

        assert!(matches!(
            vehicle.set_mode_by_name("NOT_A_MODE", false).await,
            Err(VehicleError::ModeNotAvailable(name)) if name == "NOT_A_MODE"
        ));

        vehicle
            .disconnect()
            .await
            .expect("disconnect should succeed");
    }

    #[tokio::test]
    async fn set_home_ack_flow() {
        let (vehicle, msg_tx, sent) = connect_mock_vehicle_with_sent().await;
        let point = GeoPoint3dMsl {
            latitude_deg: 47.397742,
            longitude_deg: 8.545594,
            altitude_msl_m: 510.5,
        };

        let task = {
            let vehicle = vehicle.clone();
            let point = point.clone();
            tokio::spawn(async move { vehicle.set_home(point).await })
        };

        tokio::time::sleep(Duration::from_millis(10)).await;
        msg_tx
            .send((
                default_header(),
                ack_msg(
                    dialect::MavCmd::MAV_CMD_DO_SET_HOME,
                    dialect::MavResult::MAV_RESULT_ACCEPTED,
                ),
            ))
            .await
            .expect("set home ack should be delivered");

        assert!(task.await.expect("set home task should join").is_ok());

        let sent_home = sent
            .lock()
            .expect("sent messages lock should not poison")
            .iter()
            .find_map(|(_, msg)| match msg {
                dialect::MavMessage::COMMAND_INT(data)
                    if data.command == dialect::MavCmd::MAV_CMD_DO_SET_HOME =>
                {
                    Some(data.clone())
                }
                _ => None,
            })
            .expect("set home command should be sent as COMMAND_INT");

        assert_eq!(sent_home.param1, 0.0);
        assert_eq!(sent_home.x, quantize_degrees_e7(point.latitude_deg));
        assert_eq!(sent_home.y, quantize_degrees_e7(point.longitude_deg));
        assert_eq!(sent_home.z, point.altitude_msl_m as f32);

        vehicle
            .disconnect()
            .await
            .expect("disconnect should succeed");
    }

    #[allow(deprecated)]
    #[tokio::test]
    async fn set_home_current_ack_flow() {
        let (vehicle, msg_tx, sent) = connect_mock_vehicle_with_sent().await;

        let task = {
            let vehicle = vehicle.clone();
            tokio::spawn(async move { vehicle.set_home_current().await })
        };

        tokio::time::sleep(Duration::from_millis(10)).await;
        msg_tx
            .send((
                default_header(),
                ack_msg(
                    dialect::MavCmd::MAV_CMD_DO_SET_HOME,
                    dialect::MavResult::MAV_RESULT_ACCEPTED,
                ),
            ))
            .await
            .expect("set home current ack should be delivered");

        assert!(
            task.await
                .expect("set home current task should join")
                .is_ok()
        );

        let sent_home = sent
            .lock()
            .expect("sent messages lock should not poison")
            .iter()
            .find_map(|(_, msg)| match msg {
                dialect::MavMessage::COMMAND_LONG(data)
                    if data.command == dialect::MavCmd::MAV_CMD_DO_SET_HOME =>
                {
                    Some(data.clone())
                }
                _ => None,
            })
            .expect("set home current command should be sent");

        assert_eq!(sent_home.param1, 1.0);
        assert_eq!(sent_home.param5, 0.0);
        assert_eq!(sent_home.param6, 0.0);
        assert_eq!(sent_home.param7, 0.0);

        vehicle
            .disconnect()
            .await
            .expect("disconnect should succeed");
    }

    #[allow(deprecated)]
    #[tokio::test]
    async fn set_origin_observation() {
        let (vehicle, msg_tx, sent) = connect_mock_vehicle_with_sent().await;
        let requested = GeoPoint3dMsl {
            latitude_deg: 47.39812304,
            longitude_deg: 8.54632104,
            altitude_msl_m: 505.5004,
        };

        msg_tx
            .send((
                default_header(),
                gps_global_origin_msg(473_981_230, 85_463_210, 505_500),
            ))
            .await
            .expect("stale origin should be delivered");
        tokio::time::sleep(Duration::from_millis(20)).await;

        let task = {
            let vehicle = vehicle.clone();
            let requested = requested.clone();
            tokio::spawn(async move { vehicle.set_origin(requested).await })
        };

        tokio::time::sleep(Duration::from_millis(10)).await;

        let sent_origin = sent
            .lock()
            .expect("sent messages lock should not poison")
            .iter()
            .find_map(|(_, msg)| match msg {
                dialect::MavMessage::SET_GPS_GLOBAL_ORIGIN(data) => Some(data.clone()),
                _ => None,
            })
            .expect("set origin message should be sent");

        assert_eq!(sent_origin.target_system, 1);
        assert_eq!(sent_origin.latitude, 473_981_230);
        assert_eq!(sent_origin.longitude, 85_463_210);
        assert_eq!(sent_origin.altitude, 505_500);

        tokio::time::sleep(Duration::from_millis(40)).await;
        assert!(
            !task.is_finished(),
            "pre-existing matching origin should not confirm success"
        );

        msg_tx
            .send((
                default_header(),
                gps_global_origin_msg(470_000_000, 80_000_000, 400_000),
            ))
            .await
            .expect("non-matching origin should be delivered");
        tokio::time::sleep(Duration::from_millis(20)).await;
        assert!(
            !task.is_finished(),
            "non-matching origin should be ignored until timeout or match"
        );

        msg_tx
            .send((
                default_header(),
                gps_global_origin_msg(473_981_230, 85_463_210, 505_500),
            ))
            .await
            .expect("matching origin should be delivered");

        assert!(task.await.expect("set origin task should join").is_ok());

        vehicle
            .disconnect()
            .await
            .expect("disconnect should succeed");
    }

    #[tokio::test]
    async fn params_ops_fail_with_disconnected_without_event_loop() {
        let vehicle = dummy_vehicle();
        let download = vehicle
            .params()
            .download_all()
            .expect("download operation should start");

        assert!(matches!(
            download.wait().await,
            Err(VehicleError::Disconnected)
        ));
    }

    #[tokio::test]
    async fn disconnect_sends_shutdown_command() {
        let (vehicle, mut command_rx) = test_vehicle_with_command_rx();
        let disconnect_task = tokio::spawn(async move { vehicle.disconnect().await });

        let command = command_rx
            .recv()
            .await
            .expect("disconnect should send a shutdown command");

        assert!(matches!(command, Command::Shutdown));
        assert!(matches!(
            disconnect_task.await.expect("task should complete"),
            Err(VehicleError::Disconnected)
        ));
    }

    #[tokio::test]
    async fn connect_waits_for_first_heartbeat_and_populates_identity() {
        let (msg_tx, msg_rx) = mpsc::channel(16);
        let (conn, _sent) = MockConnection::new(msg_rx);

        let connect_task =
            tokio::spawn(
                async move { Vehicle::from_connection(Box::new(conn), fast_config()).await },
            );

        tokio::time::sleep(Duration::from_millis(20)).await;
        assert!(
            !connect_task.is_finished(),
            "connect should stay pending until the first heartbeat arrives"
        );

        msg_tx
            .send((default_header(), heartbeat_msg(true, 5)))
            .await
            .expect("heartbeat should be delivered");

        let vehicle = timeout(Duration::from_millis(250), connect_task)
            .await
            .expect("connect should finish after the heartbeat")
            .expect("connect task should join")
            .expect("connect should succeed");

        assert_eq!(
            vehicle.identity(),
            VehicleIdentity {
                system_id: 1,
                component_id: 1,
                autopilot: AutopilotType::ArduPilotMega,
                vehicle_type: VehicleType::Quadrotor,
            }
        );
        assert!(vehicle.available_modes().current().latest().is_some());

        vehicle
            .disconnect()
            .await
            .expect("disconnect should succeed");
    }

    #[tokio::test]
    async fn connect_reports_transport_failure_before_first_heartbeat() {
        let (msg_tx, msg_rx) = mpsc::channel(16);
        let (conn, _sent) = MockConnection::new(msg_rx);

        let connect_task =
            tokio::spawn(
                async move { Vehicle::from_connection(Box::new(conn), fast_config()).await },
            );

        drop(msg_tx);

        let result = timeout(Duration::from_millis(250), connect_task)
            .await
            .expect("connect should fail when the transport closes")
            .expect("connect task should join");

        let err = match result {
            Ok(_) => panic!("connect should not succeed without a heartbeat"),
            Err(err) => err,
        };

        assert!(
            matches!(err, VehicleError::ConnectionFailed(message) if message.contains("mock connection closed"))
        );
    }

    #[tokio::test]
    async fn connect_disconnect_waits_for_disconnected_state_and_pending_waits() {
        let (vehicle, msg_tx) = connect_mock_vehicle().await;
        let pending_wait = tokio::spawn(wait_for_metric_disconnect(vehicle.telemetry().home()));

        vehicle
            .disconnect()
            .await
            .expect("disconnect should wait for the lifecycle to finish closing");

        assert_eq!(
            *vehicle.inner.stores.link_state.borrow(),
            crate::state::LinkState::Disconnected,
            "disconnect should not return until link state is disconnected"
        );

        let wait_result = timeout(Duration::from_millis(100), pending_wait)
            .await
            .expect("pending metric wait should resolve promptly on disconnect")
            .expect("wait task should join");
        assert!(matches!(wait_result, Err(VehicleError::Disconnected)));

        drop(msg_tx);
    }
}
