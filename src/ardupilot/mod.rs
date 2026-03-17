mod calibration;
pub mod copter;
pub mod guided;
pub mod plane;
pub mod rover;
pub mod sub;
pub mod types;

pub use copter::ArduCopterHandle;
pub use guided::{
    ArduCopterGuidedHandle, ArduGuidedKind, ArduGuidedSession, ArduPlaneGuidedHandle,
    ArduPlaneKind, ArduPlaneVtolGuidedHandle, ArduRoverGuidedHandle, ArduSubGuidedHandle,
    GuidedSpecific, RelativeClimbTarget, SubGotoDepthTarget,
};
pub use plane::{ArduPlaneHandle, ArduPlaneVtolHandle};
pub use rover::ArduRoverHandle;
pub use sub::ArduSubHandle;

use crate::command::Command;
use crate::dialect;
use crate::error::VehicleError;
use crate::mission::send_domain_command;
use crate::observation::{ObservationHandle, ObservationWriter};
use crate::state::{StateChannels, VehicleType};
use crate::vehicle::VehicleInner;
use calibration::{
    MAV_CMD_DO_ACCEPT_MAG_CAL_ID, MAV_CMD_DO_CANCEL_MAG_CAL_ID, MAV_CMD_DO_START_MAG_CAL_ID,
};
use std::collections::BTreeMap;
use std::sync::Arc;
use tokio_util::sync::CancellationToken;

pub use types::{MagCalProgress, MagCalReport, MagCalStatus};

#[derive(Clone)]
pub(crate) struct ArduPilotDomain {
    inner: Arc<ArduPilotDomainInner>,
}

struct ArduPilotDomainInner {
    mag_cal_progress_writer: ObservationWriter<Vec<MagCalProgress>>,
    mag_cal_progress: ObservationHandle<Vec<MagCalProgress>>,
    mag_cal_report_writer: ObservationWriter<Vec<MagCalReport>>,
    mag_cal_report: ObservationHandle<Vec<MagCalReport>>,
    guided_lease: guided::GuidedLeaseScope,
}

impl ArduPilotDomain {
    pub(crate) fn new() -> Self {
        let (mag_cal_progress_writer, mag_cal_progress) = ObservationHandle::watch();
        let (mag_cal_report_writer, mag_cal_report) = ObservationHandle::watch();
        let _ = mag_cal_progress_writer.publish(Vec::new());
        let _ = mag_cal_report_writer.publish(Vec::new());

        Self {
            inner: Arc::new(ArduPilotDomainInner {
                mag_cal_progress_writer,
                mag_cal_progress,
                mag_cal_report_writer,
                mag_cal_report,
                guided_lease: guided::GuidedLeaseScope::new(),
            }),
        }
    }

    pub(crate) fn start(&self, stores: &StateChannels, cancel: CancellationToken) {
        let mut progress_rx = stores.mag_cal_progress.clone();
        let progress_writer = self.inner.mag_cal_progress_writer.clone();
        let progress_cancel = cancel.clone();
        tokio::spawn(async move {
            let mut by_compass = BTreeMap::<u8, MagCalProgress>::new();
            if let Some(progress) = progress_rx.borrow().clone() {
                by_compass.insert(progress.compass_id, progress);
                let _ = progress_writer.publish(by_compass.values().cloned().collect());
            }

            loop {
                tokio::select! {
                    _ = progress_cancel.cancelled() => break,
                    changed = progress_rx.changed() => {
                        if changed.is_err() {
                            break;
                        }

                        if let Some(progress) = progress_rx.borrow_and_update().clone() {
                            by_compass.insert(progress.compass_id, progress);
                            let _ = progress_writer.publish(by_compass.values().cloned().collect());
                        }
                    }
                }
            }
        });

        let mut report_rx = stores.mag_cal_report.clone();
        let report_writer = self.inner.mag_cal_report_writer.clone();
        let report_cancel = cancel.clone();
        tokio::spawn(async move {
            let mut by_compass = BTreeMap::<u8, MagCalReport>::new();
            if let Some(report) = report_rx.borrow().clone() {
                by_compass.insert(report.compass_id, report);
                let _ = report_writer.publish(by_compass.values().cloned().collect());
            }

            loop {
                tokio::select! {
                    _ = report_cancel.cancelled() => break,
                    changed = report_rx.changed() => {
                        if changed.is_err() {
                            break;
                        }

                        if let Some(report) = report_rx.borrow_and_update().clone() {
                            by_compass.insert(report.compass_id, report);
                            let _ = report_writer.publish(by_compass.values().cloned().collect());
                        }
                    }
                }
            }
        });
    }

    pub(crate) fn mag_cal_progress(&self) -> ObservationHandle<Vec<MagCalProgress>> {
        self.inner.mag_cal_progress.clone()
    }

    pub(crate) fn mag_cal_report(&self) -> ObservationHandle<Vec<MagCalReport>> {
        self.inner.mag_cal_report.clone()
    }

    pub(crate) fn begin_guided_lease(&self) -> Result<u64, VehicleError> {
        self.inner.guided_lease.acquire()
    }

    pub(crate) fn release_guided_lease(&self, lease_id: u64) {
        self.inner.guided_lease.release(lease_id);
    }

    pub(crate) fn guided_lease_scope(&self) -> guided::GuidedLeaseScope {
        self.inner.guided_lease.clone()
    }
}

/// ArduPilot-specific capability accessor.
pub struct ArduPilotHandle<'a> {
    inner: &'a VehicleInner,
}

impl<'a> ArduPilotHandle<'a> {
    pub(crate) fn new(inner: &'a VehicleInner) -> Self {
        Self { inner }
    }

    pub fn copter(&self) -> Option<ArduCopterHandle<'a>> {
        matches!(
            self.vehicle_type(),
            VehicleType::Quadrotor
                | VehicleType::Hexarotor
                | VehicleType::Octorotor
                | VehicleType::Tricopter
                | VehicleType::Coaxial
                | VehicleType::Helicopter
        )
        .then_some(ArduCopterHandle::new(self.inner))
    }

    pub fn plane(&self) -> Option<ArduPlaneHandle<'a>> {
        matches!(
            self.vehicle_type(),
            VehicleType::FixedWing | VehicleType::Vtol
        )
        .then_some(ArduPlaneHandle::new(self.inner))
    }

    pub fn rover(&self) -> Option<ArduRoverHandle<'a>> {
        matches!(self.vehicle_type(), VehicleType::GroundRover)
            .then_some(ArduRoverHandle::new(self.inner))
    }

    pub fn sub(&self) -> Option<ArduSubHandle<'a>> {
        matches!(self.vehicle_type(), VehicleType::Submarine)
            .then_some(ArduSubHandle::new(self.inner))
    }

    pub async fn guided(&self) -> Result<ArduGuidedSession, VehicleError> {
        let kind = self.guided_kind().ok_or_else(|| {
            VehicleError::Unsupported("guided session is unavailable for this vehicle type".into())
        })?;
        let guided_custom_mode = self
            .inner
            .resolve_mode_by_name("GUIDED")
            .ok_or_else(|| VehicleError::ModeNotAvailable("GUIDED".to_string()))?;
        let lease_id = self.inner.ardupilot.begin_guided_lease()?;

        let current_custom_mode = self
            .inner
            .modes
            .current()
            .latest()
            .map(|mode| mode.custom_mode);
        if current_custom_mode != Some(guided_custom_mode)
            && let Err(err) = self.inner.set_mode(guided_custom_mode, true).await
        {
            self.inner.ardupilot.release_guided_lease(lease_id);
            return Err(err);
        }

        Ok(ArduGuidedSession::new(
            self.inner,
            guided::GuidedSessionInit {
                lease_scope: self.inner.ardupilot.guided_lease_scope(),
                lease_id,
                kind,
                plane_kind: self.guided_plane_kind(),
                guided_mode_custom_mode: guided_custom_mode,
                command_tx: self.inner.command_tx.clone(),
                target_system: self.inner.stores.vehicle_state.borrow().system_id,
                target_component: self.inner.stores.vehicle_state.borrow().component_id,
            },
        ))
    }

    pub async fn request_prearm_checks(&self) -> Result<(), VehicleError> {
        self.send_long(dialect::MavCmd::MAV_CMD_RUN_PREARM_CHECKS, [0.0; 7])
            .await
    }

    pub async fn motor_test(
        &self,
        instance: u8,
        throttle_pct: f32,
        duration_s: u16,
    ) -> Result<(), VehicleError> {
        if !(1..=12).contains(&instance) {
            return Err(VehicleError::InvalidParameter(format!(
                "motor_test instance must be in 1..=12, got {instance}"
            )));
        }
        if !(0.0..=100.0).contains(&throttle_pct) {
            return Err(VehicleError::InvalidParameter(format!(
                "motor_test throttle_pct must be in 0..=100, got {throttle_pct}"
            )));
        }
        if !(1..=30).contains(&duration_s) {
            return Err(VehicleError::InvalidParameter(format!(
                "motor_test duration_s must be in 1..=30, got {duration_s}"
            )));
        }

        self.send_long(
            dialect::MavCmd::MAV_CMD_DO_MOTOR_TEST,
            [
                instance as f32,
                0.0,
                throttle_pct,
                duration_s as f32,
                1.0,
                0.0,
                0.0,
            ],
        )
        .await
    }

    pub async fn preflight_calibration(
        &self,
        gyro: bool,
        accel: bool,
        baro: bool,
        accel_trim: bool,
    ) -> Result<(), VehicleError> {
        let params = calibration::preflight_calibration_params(gyro, accel, baro, accel_trim)?;

        self.send_long(dialect::MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION, params)
            .await
    }

    pub async fn reboot(&self) -> Result<(), VehicleError> {
        self.send_long(
            dialect::MavCmd::MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        .await
    }

    pub async fn reboot_to_bootloader(&self) -> Result<(), VehicleError> {
        self.send_long(
            dialect::MavCmd::MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            [3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        .await
    }

    pub fn mag_cal_progress(&self) -> ObservationHandle<Vec<MagCalProgress>> {
        self.inner.ardupilot.mag_cal_progress()
    }

    pub fn mag_cal_report(&self) -> ObservationHandle<Vec<MagCalReport>> {
        self.inner.ardupilot.mag_cal_report()
    }

    pub async fn start_mag_cal(&self, compass_mask: u8) -> Result<(), VehicleError> {
        self.send_long_raw(
            MAV_CMD_DO_START_MAG_CAL_ID,
            [compass_mask as f32, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        .await
    }

    pub async fn accept_mag_cal(&self) -> Result<(), VehicleError> {
        self.send_long_raw(
            MAV_CMD_DO_ACCEPT_MAG_CAL_ID,
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        .await
    }

    pub async fn cancel_mag_cal(&self) -> Result<(), VehicleError> {
        self.send_long_raw(
            MAV_CMD_DO_CANCEL_MAG_CAL_ID,
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        .await
    }

    async fn send_long(
        &self,
        command: dialect::MavCmd,
        params: [f32; 7],
    ) -> Result<(), VehicleError> {
        send_domain_command(self.inner.command_tx.clone(), |reply| Command::Long {
            command,
            params,
            reply,
        })
        .await
    }

    async fn send_long_raw(&self, command_id: u16, params: [f32; 7]) -> Result<(), VehicleError> {
        send_domain_command(self.inner.command_tx.clone(), |reply| Command::LongRaw {
            command_id,
            params,
            reply,
        })
        .await
    }

    fn vehicle_type(&self) -> VehicleType {
        self.inner.stores.vehicle_state.borrow().vehicle_type
    }

    fn guided_kind(&self) -> Option<ArduGuidedKind> {
        match self.vehicle_type() {
            VehicleType::Quadrotor
            | VehicleType::Hexarotor
            | VehicleType::Octorotor
            | VehicleType::Tricopter
            | VehicleType::Coaxial
            | VehicleType::Helicopter => Some(ArduGuidedKind::Copter),
            VehicleType::FixedWing | VehicleType::Vtol => Some(ArduGuidedKind::Plane),
            VehicleType::GroundRover => Some(ArduGuidedKind::Rover),
            VehicleType::Submarine => Some(ArduGuidedKind::Sub),
            VehicleType::Unknown | VehicleType::Generic => None,
        }
    }

    fn guided_plane_kind(&self) -> Option<ArduPlaneKind> {
        match self.vehicle_type() {
            VehicleType::FixedWing => Some(ArduPlaneKind::FixedWing),
            VehicleType::Vtol => Some(ArduPlaneKind::Vtol),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::VehicleConfig;
    use crate::dialect::{self, MavModeFlag, MavState};
    use crate::error::VehicleError;
    use crate::state::{self, create_channels};
    use crate::vehicle::Vehicle;
    use mavlink::{
        AsyncMavConnection, MAVLinkMessageRaw, MAVLinkV2MessageRaw, MavHeader, MavlinkVersion,
    };
    use std::sync::{Arc, Mutex};
    use std::time::Duration;
    use tokio::sync::mpsc;
    use tokio::time::timeout;

    type SentMessages = Arc<Mutex<Vec<(MavHeader, dialect::MavMessage)>>>;

    struct MockConnection {
        recv_rx: tokio::sync::Mutex<mpsc::Receiver<(MavHeader, dialect::MavMessage)>>,
        sent: SentMessages,
    }

    impl MockConnection {
        fn new(rx: mpsc::Receiver<(MavHeader, dialect::MavMessage)>) -> (Self, SentMessages) {
            let sent = Arc::new(Mutex::new(Vec::new()));
            (
                Self {
                    recv_rx: tokio::sync::Mutex::new(rx),
                    sent: sent.clone(),
                },
                sent,
            )
        }
    }

    #[async_trait::async_trait]
    impl AsyncMavConnection<dialect::MavMessage> for MockConnection {
        async fn recv(
            &self,
        ) -> Result<(MavHeader, dialect::MavMessage), mavlink::error::MessageReadError> {
            let mut rx = self.recv_rx.lock().await;
            match rx.recv().await {
                Some(message) => Ok(message),
                None => Err(mavlink::error::MessageReadError::Io(std::io::Error::new(
                    std::io::ErrorKind::ConnectionReset,
                    "mock connection closed",
                ))),
            }
        }

        async fn recv_raw(&self) -> Result<MAVLinkMessageRaw, mavlink::error::MessageReadError> {
            let (header, message) = self.recv().await?;
            let mut raw = MAVLinkV2MessageRaw::new();
            raw.serialize_message(header, &message);
            Ok(MAVLinkMessageRaw::V2(raw))
        }

        async fn send(
            &self,
            header: &MavHeader,
            data: &dialect::MavMessage,
        ) -> Result<usize, mavlink::error::MessageWriteError> {
            self.sent.lock().unwrap().push((*header, data.clone()));
            Ok(0)
        }

        fn set_protocol_version(&mut self, _version: MavlinkVersion) {}

        fn protocol_version(&self) -> MavlinkVersion {
            MavlinkVersion::V2
        }

        fn set_allow_recv_any_version(&mut self, _allow: bool) {}

        fn allow_recv_any_version(&self) -> bool {
            true
        }
    }

    fn default_header() -> MavHeader {
        MavHeader {
            system_id: 1,
            component_id: 1,
            sequence: 0,
        }
    }

    fn heartbeat_msg_with_mode(mavtype: dialect::MavType, custom_mode: u32) -> dialect::MavMessage {
        dialect::MavMessage::HEARTBEAT(dialect::HEARTBEAT_DATA {
            custom_mode,
            mavtype,
            autopilot: dialect::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode: MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            system_status: MavState::MAV_STATE_STANDBY,
            mavlink_version: 3,
        })
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

    fn command_id_to_mav_cmd(command_id: u16) -> dialect::MavCmd {
        num_traits::FromPrimitive::from_u16(command_id)
            .expect("command id should be representable by mavkit::dialect::MavCmd")
    }

    fn ack_msg_by_id(command_id: u16, result: dialect::MavResult) -> dialect::MavMessage {
        ack_msg(command_id_to_mav_cmd(command_id), result)
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

    async fn connect_mock_vehicle_with_sent() -> (
        Vehicle,
        mpsc::Sender<(MavHeader, dialect::MavMessage)>,
        SentMessages,
    ) {
        connect_mock_vehicle_with_sent_for_type_and_mode(dialect::MavType::MAV_TYPE_QUADROTOR, 7)
            .await
    }

    async fn connect_mock_vehicle_with_sent_for_type(
        mavtype: dialect::MavType,
    ) -> (
        Vehicle,
        mpsc::Sender<(MavHeader, dialect::MavMessage)>,
        SentMessages,
    ) {
        connect_mock_vehicle_with_sent_for_type_and_mode(mavtype, 7).await
    }

    async fn connect_mock_vehicle_with_sent_for_type_and_mode(
        mavtype: dialect::MavType,
        custom_mode: u32,
    ) -> (
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
            .send((
                default_header(),
                heartbeat_msg_with_mode(mavtype, custom_mode),
            ))
            .await
            .expect("heartbeat should be delivered to the mock connection");

        let vehicle = timeout(Duration::from_millis(250), connect_task)
            .await
            .expect("connect should complete after first heartbeat")
            .expect("connect task should join")
            .expect("mock vehicle should connect");

        (vehicle, msg_tx, sent)
    }

    fn sent_mode_change_params(sent: &SentMessages) -> Vec<u32> {
        sent.lock()
            .expect("sent messages lock should not poison")
            .iter()
            .filter_map(|(_, msg)| match msg {
                dialect::MavMessage::COMMAND_LONG(data)
                    if data.command == dialect::MavCmd::MAV_CMD_DO_SET_MODE =>
                {
                    Some(data.param2 as u32)
                }
                _ => None,
            })
            .collect()
    }

    #[tokio::test]
    async fn class_handle_availability() {
        for mavtype in [
            dialect::MavType::MAV_TYPE_QUADROTOR,
            dialect::MavType::MAV_TYPE_TRICOPTER,
            dialect::MavType::MAV_TYPE_HELICOPTER,
        ] {
            let (vehicle, _msg_tx, _sent) = connect_mock_vehicle_with_sent_for_type(mavtype).await;
            let ardupilot = vehicle.ardupilot();

            assert!(
                ardupilot.copter().is_some(),
                "{mavtype:?} should expose copter()"
            );
            assert!(
                ardupilot.plane().is_none(),
                "{mavtype:?} should not expose plane()"
            );
            assert!(
                ardupilot.rover().is_none(),
                "{mavtype:?} should not expose rover()"
            );
            assert!(
                ardupilot.sub().is_none(),
                "{mavtype:?} should not expose sub()"
            );

            vehicle
                .disconnect()
                .await
                .expect("disconnect should succeed");
        }

        let (plane, _msg_tx, _sent) =
            connect_mock_vehicle_with_sent_for_type(dialect::MavType::MAV_TYPE_FIXED_WING).await;
        {
            let plane = plane
                .ardupilot()
                .plane()
                .expect("fixed wing heartbeat should expose plane()");
            assert!(
                plane.vtol().is_none(),
                "plain fixed wing should not expose plane().vtol()"
            );
        }
        plane.disconnect().await.expect("disconnect should succeed");

        let (quadplane, _msg_tx, _sent) =
            connect_mock_vehicle_with_sent_for_type(dialect::MavType::MAV_TYPE_VTOL_FIXEDROTOR)
                .await;
        {
            let quadplane = quadplane
                .ardupilot()
                .plane()
                .expect("vtol heartbeat should still expose plane()");
            assert!(
                quadplane.vtol().is_some(),
                "vtol heartbeat should expose plane().vtol() as a plane extension"
            );
        }
        quadplane
            .disconnect()
            .await
            .expect("disconnect should succeed");

        let (rover, _msg_tx, _sent) =
            connect_mock_vehicle_with_sent_for_type(dialect::MavType::MAV_TYPE_GROUND_ROVER).await;
        let ardupilot = rover.ardupilot();
        assert!(
            ardupilot.rover().is_some(),
            "ground rover heartbeat should expose rover()"
        );
        assert!(
            ardupilot.copter().is_none(),
            "ground rover heartbeat should not expose copter()"
        );
        assert!(
            ardupilot.plane().is_none(),
            "ground rover heartbeat should not expose plane()"
        );
        assert!(
            ardupilot.sub().is_none(),
            "ground rover heartbeat should not expose sub()"
        );
        rover.disconnect().await.expect("disconnect should succeed");

        let (sub, _msg_tx, _sent) =
            connect_mock_vehicle_with_sent_for_type(dialect::MavType::MAV_TYPE_SUBMARINE).await;
        let ardupilot = sub.ardupilot();
        assert!(
            ardupilot.sub().is_some(),
            "submarine heartbeat should expose sub()"
        );
        assert!(
            ardupilot.copter().is_none(),
            "submarine heartbeat should not expose copter()"
        );
        assert!(
            ardupilot.plane().is_none(),
            "submarine heartbeat should not expose plane()"
        );
        assert!(
            ardupilot.rover().is_none(),
            "submarine heartbeat should not expose rover()"
        );
        sub.disconnect().await.expect("disconnect should succeed");
    }

    #[tokio::test]
    async fn guided_exclusivity() {
        let (vehicle, _msg_tx, _sent) = connect_mock_vehicle_with_sent_for_type_and_mode(
            dialect::MavType::MAV_TYPE_QUADROTOR,
            4,
        )
        .await;

        let session = vehicle
            .ardupilot()
            .guided()
            .await
            .expect("first guided session should acquire the local lease");
        assert_eq!(session.kind(), ArduGuidedKind::Copter);
        assert!(matches!(session.specific(), GuidedSpecific::Copter(_)));

        let conflict = vehicle.ardupilot().guided().await;
        assert!(matches!(
            conflict,
            Err(VehicleError::OperationConflict {
                conflicting_domain,
                conflicting_op,
            }) if conflicting_domain == "ardupilot_guided" && conflicting_op == "session_active"
        ));

        session
            .close()
            .await
            .expect("explicit close should release the local lease");

        let reacquired = vehicle
            .ardupilot()
            .guided()
            .await
            .expect("guided lease should be acquirable again after close");
        reacquired
            .close()
            .await
            .expect("reacquired guided session should close cleanly");

        vehicle
            .disconnect()
            .await
            .expect("disconnect should succeed");
    }

    #[tokio::test]
    async fn guided_mode_asserting_acquisition() {
        let (vehicle, msg_tx, sent) = connect_mock_vehicle_with_sent().await;

        let guided_task = {
            let vehicle = vehicle.clone();
            tokio::spawn(async move { vehicle.ardupilot().guided().await })
        };

        tokio::time::sleep(Duration::from_millis(10)).await;
        assert_eq!(
            sent_mode_change_params(&sent),
            vec![4],
            "guided acquisition should request GUIDED mode when current mode is not guided"
        );

        msg_tx
            .send((
                default_header(),
                ack_msg(
                    dialect::MavCmd::MAV_CMD_DO_SET_MODE,
                    dialect::MavResult::MAV_RESULT_ACCEPTED,
                ),
            ))
            .await
            .expect("guided mode change ack should be delivered");

        tokio::time::sleep(Duration::from_millis(40)).await;
        assert!(
            !guided_task.is_finished(),
            "guided acquisition should wait for observed current mode after ACK"
        );

        msg_tx
            .send((
                default_header(),
                heartbeat_msg_with_mode(dialect::MavType::MAV_TYPE_QUADROTOR, 4),
            ))
            .await
            .expect("guiding heartbeat should be delivered");

        let session = guided_task
            .await
            .expect("guided task should join")
            .expect("guided acquisition should complete after mode observation");
        assert!(matches!(session.specific(), GuidedSpecific::Copter(_)));

        session
            .close()
            .await
            .expect("guided session should close cleanly");
        vehicle
            .disconnect()
            .await
            .expect("disconnect should succeed");
    }

    #[tokio::test]
    async fn guided_session_terminal_on_mode_loss() {
        let (vehicle, msg_tx, _sent) = connect_mock_vehicle_with_sent_for_type_and_mode(
            dialect::MavType::MAV_TYPE_QUADROTOR,
            4,
        )
        .await;

        let session = vehicle
            .ardupilot()
            .guided()
            .await
            .expect("guided session should acquire while already in guided mode");

        msg_tx
            .send((
                default_header(),
                heartbeat_msg_with_mode(dialect::MavType::MAV_TYPE_QUADROTOR, 6),
            ))
            .await
            .expect("mode-loss heartbeat should be delivered");

        timeout(Duration::from_millis(250), async {
            loop {
                if session.terminal_reason_for_test().is_some() {
                    break;
                }
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
        })
        .await
        .expect("guided session should become terminal after leaving guided mode");

        assert_eq!(session.terminal_reason_for_test(), Some("guided_mode_lost"));

        session
            .close()
            .await
            .expect("terminal guided session close should still release local lease");
        vehicle
            .disconnect()
            .await
            .expect("disconnect should succeed");
    }

    #[tokio::test]
    async fn guided_close_releases_lease_without_mode_restore() {
        let (vehicle, _msg_tx, sent) = connect_mock_vehicle_with_sent_for_type_and_mode(
            dialect::MavType::MAV_TYPE_QUADROTOR,
            4,
        )
        .await;

        let session = vehicle
            .ardupilot()
            .guided()
            .await
            .expect("guided session should acquire");

        assert!(matches!(
            vehicle.ardupilot().guided().await,
            Err(VehicleError::OperationConflict {
                conflicting_domain,
                conflicting_op,
            }) if conflicting_domain == "ardupilot_guided" && conflicting_op == "session_active"
        ));

        session
            .close()
            .await
            .expect("close should release only local lease state");

        let reacquired = vehicle
            .ardupilot()
            .guided()
            .await
            .expect("lease should be free after close");
        reacquired
            .close()
            .await
            .expect("reacquired guided session should close cleanly");

        assert!(
            sent_mode_change_params(&sent).is_empty(),
            "close must not restore prior mode or issue extra mode commands"
        );

        vehicle
            .disconnect()
            .await
            .expect("disconnect should succeed");
    }

    #[tokio::test]
    async fn copter_goto_command() {
        let (vehicle, _msg_tx, sent) = connect_mock_vehicle_with_sent_for_type_and_mode(
            dialect::MavType::MAV_TYPE_QUADROTOR,
            4,
        )
        .await;

        let session = vehicle
            .ardupilot()
            .guided()
            .await
            .expect("guided session should acquire while already in guided mode");
        let GuidedSpecific::Copter(copter) = session.specific() else {
            panic!("quadrotor guided session should expose copter handle");
        };

        copter
            .goto(crate::GeoPoint3dRelHome {
                latitude_deg: 47.397742,
                longitude_deg: 8.545594,
                relative_alt_m: 32.5,
            })
            .await
            .expect("guided goto should send set-position-target message");

        let goto_msg = sent
            .lock()
            .expect("sent messages lock should not poison")
            .iter()
            .find_map(|(_, msg)| match msg {
                dialect::MavMessage::SET_POSITION_TARGET_GLOBAL_INT(data) => Some(data.clone()),
                _ => None,
            })
            .expect("guided goto should send set_position_target_global_int");

        assert_eq!(
            goto_msg.coordinate_frame,
            dialect::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT
        );
        assert_eq!(goto_msg.lat_int, 473_977_420);
        assert_eq!(goto_msg.lon_int, 85_455_940);
        assert_eq!(goto_msg.alt, 32.5);

        session
            .close()
            .await
            .expect("guided session should close cleanly");
        vehicle
            .disconnect()
            .await
            .expect("disconnect should succeed");
    }

    #[tokio::test]
    async fn plane_guided_vtol_extension_availability() {
        let (plane, _msg_tx, _sent) = connect_mock_vehicle_with_sent_for_type_and_mode(
            dialect::MavType::MAV_TYPE_FIXED_WING,
            15,
        )
        .await;
        let plane_session = plane
            .ardupilot()
            .guided()
            .await
            .expect("fixed-wing guided session should acquire");
        let GuidedSpecific::Plane(fixed_wing) = plane_session.specific() else {
            panic!("fixed-wing guided session should expose plane handle");
        };
        assert_eq!(fixed_wing.kind(), ArduPlaneKind::FixedWing);
        assert!(fixed_wing.vtol().is_none());
        plane_session
            .close()
            .await
            .expect("fixed-wing guided session should close cleanly");
        plane.disconnect().await.expect("disconnect should succeed");

        let (quadplane, _msg_tx, _sent) = connect_mock_vehicle_with_sent_for_type_and_mode(
            dialect::MavType::MAV_TYPE_VTOL_FIXEDROTOR,
            15,
        )
        .await;
        let quadplane_session = quadplane
            .ardupilot()
            .guided()
            .await
            .expect("vtol plane guided session should acquire");
        {
            let GuidedSpecific::Plane(plane) = quadplane_session.specific() else {
                panic!("vtol plane guided session should expose plane handle");
            };
            assert_eq!(plane.kind(), ArduPlaneKind::Vtol);
            assert!(plane.vtol().is_some());
        }
        quadplane_session
            .close()
            .await
            .expect("vtol plane guided session should close cleanly");
        quadplane
            .disconnect()
            .await
            .expect("disconnect should succeed");
    }

    #[tokio::test]
    async fn vtol_takeoff_command_encoding() {
        let (vehicle, msg_tx, sent) = connect_mock_vehicle_with_sent_for_type_and_mode(
            dialect::MavType::MAV_TYPE_VTOL_FIXEDROTOR,
            15,
        )
        .await;

        let session = vehicle
            .ardupilot()
            .guided()
            .await
            .expect("vtol plane guided session should acquire");
        let task_session = session.clone();
        let takeoff_task = tokio::spawn(async move {
            let GuidedSpecific::Plane(plane) = task_session.specific() else {
                panic!("vtol plane guided session should expose plane handle");
            };
            plane
                .vtol()
                .expect("vtol plane should expose vtol guided extension")
                .takeoff(RelativeClimbTarget {
                    relative_climb_m: 12.5,
                })
                .await
        });

        tokio::time::sleep(Duration::from_millis(10)).await;
        msg_tx
            .send((
                default_header(),
                ack_msg(
                    dialect::MavCmd::MAV_CMD_NAV_TAKEOFF,
                    dialect::MavResult::MAV_RESULT_ACCEPTED,
                ),
            ))
            .await
            .expect("vtol takeoff ack should be delivered");

        assert!(
            takeoff_task
                .await
                .expect("vtol takeoff task should join")
                .is_ok()
        );

        let takeoff = sent
            .lock()
            .expect("sent messages lock should not poison")
            .iter()
            .find_map(|(_, msg)| match msg {
                dialect::MavMessage::COMMAND_INT(data)
                    if data.command == dialect::MavCmd::MAV_CMD_NAV_TAKEOFF =>
                {
                    Some(data.clone())
                }
                _ => None,
            })
            .expect("vtol takeoff should send command_int");

        assert_eq!(takeoff.frame, dialect::MavFrame::MAV_FRAME_LOCAL_OFFSET_NED);
        assert_eq!(takeoff.x, 0);
        assert_eq!(takeoff.y, 0);
        assert_eq!(takeoff.z, -12.5);

        session
            .close()
            .await
            .expect("guided session should close cleanly");
        vehicle
            .disconnect()
            .await
            .expect("disconnect should succeed");
    }

    #[tokio::test]
    async fn rover_drive_to() {
        let (vehicle, msg_tx, sent) = connect_mock_vehicle_with_sent_for_type_and_mode(
            dialect::MavType::MAV_TYPE_GROUND_ROVER,
            15,
        )
        .await;

        let session = vehicle
            .ardupilot()
            .guided()
            .await
            .expect("rover guided session should acquire");
        let task_session = session.clone();
        let drive_task = tokio::spawn(async move {
            let GuidedSpecific::Rover(rover) = task_session.specific() else {
                panic!("rover guided session should expose rover handle");
            };
            rover
                .drive_to(crate::GeoPoint2d {
                    latitude_deg: 47.397742,
                    longitude_deg: 8.545594,
                })
                .await
        });

        tokio::time::sleep(Duration::from_millis(10)).await;
        msg_tx
            .send((
                default_header(),
                ack_msg(
                    dialect::MavCmd::MAV_CMD_DO_REPOSITION,
                    dialect::MavResult::MAV_RESULT_ACCEPTED,
                ),
            ))
            .await
            .expect("rover reposition ack should be delivered");

        assert!(
            drive_task
                .await
                .expect("rover drive_to task should join")
                .is_ok()
        );

        let reposition = sent
            .lock()
            .expect("sent messages lock should not poison")
            .iter()
            .find_map(|(_, msg)| match msg {
                dialect::MavMessage::COMMAND_INT(data)
                    if data.command == dialect::MavCmd::MAV_CMD_DO_REPOSITION =>
                {
                    Some(data.clone())
                }
                _ => None,
            })
            .expect("rover drive_to should send command_int");

        assert_eq!(reposition.frame, dialect::MavFrame::MAV_FRAME_GLOBAL);
        assert_eq!(reposition.x, 473_977_420);
        assert_eq!(reposition.y, 85_455_940);
        assert_eq!(reposition.z, 0.0);
        assert_eq!(reposition.param2, 0.0);

        session
            .close()
            .await
            .expect("guided session should close cleanly");
        vehicle
            .disconnect()
            .await
            .expect("disconnect should succeed");
    }

    #[tokio::test]
    async fn sub_goto_depth_command_encoding() {
        let (vehicle, msg_tx, sent) = connect_mock_vehicle_with_sent_for_type_and_mode(
            dialect::MavType::MAV_TYPE_SUBMARINE,
            4,
        )
        .await;

        let session = vehicle
            .ardupilot()
            .guided()
            .await
            .expect("sub guided session should acquire");
        let task_session = session.clone();
        let goto_task = tokio::spawn(async move {
            let GuidedSpecific::Sub(sub) = task_session.specific() else {
                panic!("sub guided session should expose sub handle");
            };
            sub.goto_depth(SubGotoDepthTarget {
                point: crate::GeoPoint2d {
                    latitude_deg: 47.397742,
                    longitude_deg: 8.545594,
                },
                depth_m: 12.5,
            })
            .await
        });

        tokio::time::sleep(Duration::from_millis(10)).await;
        msg_tx
            .send((
                default_header(),
                ack_msg(
                    dialect::MavCmd::MAV_CMD_DO_REPOSITION,
                    dialect::MavResult::MAV_RESULT_ACCEPTED,
                ),
            ))
            .await
            .expect("sub reposition ack should be delivered");

        assert!(
            goto_task
                .await
                .expect("sub goto_depth task should join")
                .is_ok()
        );

        let reposition = sent
            .lock()
            .expect("sent messages lock should not poison")
            .iter()
            .find_map(|(_, msg)| match msg {
                dialect::MavMessage::COMMAND_INT(data)
                    if data.command == dialect::MavCmd::MAV_CMD_DO_REPOSITION =>
                {
                    Some(data.clone())
                }
                _ => None,
            })
            .expect("sub goto_depth should send command_int");

        assert_eq!(
            reposition.frame,
            dialect::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT
        );
        assert_eq!(reposition.x, 473_977_420);
        assert_eq!(reposition.y, 85_455_940);
        assert_eq!(reposition.z, -12.5);
        assert_eq!(reposition.param2, 0.0);

        session
            .close()
            .await
            .expect("guided session should close cleanly");
        vehicle
            .disconnect()
            .await
            .expect("disconnect should succeed");
    }

    #[tokio::test]
    async fn rover_guided_actions_fail_after_close() {
        let (vehicle, _msg_tx, _sent) = connect_mock_vehicle_with_sent_for_type_and_mode(
            dialect::MavType::MAV_TYPE_GROUND_ROVER,
            15,
        )
        .await;

        let session = vehicle
            .ardupilot()
            .guided()
            .await
            .expect("rover guided session should acquire");
        let closer = session.clone();
        let GuidedSpecific::Rover(rover) = session.specific() else {
            panic!("rover guided session should expose rover handle");
        };

        closer
            .close()
            .await
            .expect("closing a cloned guided session should release the shared lease");

        assert!(matches!(
            rover
                .drive_to(crate::GeoPoint2d {
                    latitude_deg: 47.397742,
                    longitude_deg: 8.545594,
                })
                .await,
            Err(VehicleError::OperationConflict {
                conflicting_domain,
                conflicting_op,
            }) if conflicting_domain == "ardupilot_guided" && conflicting_op == "session_closed"
        ));

        vehicle
            .disconnect()
            .await
            .expect("disconnect should succeed");
    }

    #[tokio::test]
    async fn sub_guided_actions_fail_after_terminal_mode_loss() {
        let (vehicle, msg_tx, _sent) = connect_mock_vehicle_with_sent_for_type_and_mode(
            dialect::MavType::MAV_TYPE_SUBMARINE,
            4,
        )
        .await;

        let session = vehicle
            .ardupilot()
            .guided()
            .await
            .expect("sub guided session should acquire");
        let GuidedSpecific::Sub(sub) = session.specific() else {
            panic!("sub guided session should expose sub handle");
        };

        msg_tx
            .send((
                default_header(),
                heartbeat_msg_with_mode(dialect::MavType::MAV_TYPE_SUBMARINE, 2),
            ))
            .await
            .expect("mode-loss heartbeat should be delivered");

        timeout(Duration::from_millis(250), async {
            loop {
                if session.terminal_reason_for_test().is_some() {
                    break;
                }
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
        })
        .await
        .expect("guided session should become terminal after mode loss");

        assert!(matches!(
            sub.goto_depth(SubGotoDepthTarget {
                point: crate::GeoPoint2d {
                    latitude_deg: 47.397742,
                    longitude_deg: 8.545594,
                },
                depth_m: 8.0,
            })
            .await,
            Err(VehicleError::OperationConflict {
                conflicting_domain,
                conflicting_op,
            }) if conflicting_domain == "ardupilot_guided" && conflicting_op == "session_terminal"
        ));

        session
            .close()
            .await
            .expect("terminal guided session should still close cleanly");
        vehicle
            .disconnect()
            .await
            .expect("disconnect should succeed");
    }

    #[tokio::test]
    async fn guided_actions_fail_after_close() {
        let (vehicle, _msg_tx, _sent) = connect_mock_vehicle_with_sent_for_type_and_mode(
            dialect::MavType::MAV_TYPE_QUADROTOR,
            4,
        )
        .await;

        let session = vehicle
            .ardupilot()
            .guided()
            .await
            .expect("guided session should acquire");
        let closer = session.clone();
        let GuidedSpecific::Copter(copter) = session.specific() else {
            panic!("quadrotor guided session should expose copter handle");
        };

        closer
            .close()
            .await
            .expect("closing a cloned guided session should release the shared lease");

        assert!(matches!(
            copter
                .goto(crate::GeoPoint3dRelHome {
                    latitude_deg: 47.397742,
                    longitude_deg: 8.545594,
                    relative_alt_m: 20.0,
                })
                .await,
            Err(VehicleError::OperationConflict {
                conflicting_domain,
                conflicting_op,
            }) if conflicting_domain == "ardupilot_guided" && conflicting_op == "session_closed"
        ));

        vehicle
            .disconnect()
            .await
            .expect("disconnect should succeed");
    }

    #[tokio::test]
    async fn guided_actions_fail_after_terminal_mode_loss() {
        let (vehicle, msg_tx, _sent) = connect_mock_vehicle_with_sent_for_type_and_mode(
            dialect::MavType::MAV_TYPE_QUADROTOR,
            4,
        )
        .await;

        let session = vehicle
            .ardupilot()
            .guided()
            .await
            .expect("guided session should acquire");
        let GuidedSpecific::Copter(copter) = session.specific() else {
            panic!("quadrotor guided session should expose copter handle");
        };

        msg_tx
            .send((
                default_header(),
                heartbeat_msg_with_mode(dialect::MavType::MAV_TYPE_QUADROTOR, 6),
            ))
            .await
            .expect("mode-loss heartbeat should be delivered");

        timeout(Duration::from_millis(250), async {
            loop {
                if session.terminal_reason_for_test().is_some() {
                    break;
                }
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
        })
        .await
        .expect("guided session should become terminal after mode loss");

        assert!(matches!(
            copter
                .goto(crate::GeoPoint3dRelHome {
                    latitude_deg: 47.397742,
                    longitude_deg: 8.545594,
                    relative_alt_m: 20.0,
                })
                .await,
            Err(VehicleError::OperationConflict {
                conflicting_domain,
                conflicting_op,
            }) if conflicting_domain == "ardupilot_guided" && conflicting_op == "session_terminal"
        ));

        session
            .close()
            .await
            .expect("terminal guided session should still close cleanly");
        vehicle
            .disconnect()
            .await
            .expect("disconnect should succeed");
    }

    #[tokio::test]
    async fn ardupilot_motor_test_rejects_invalid_ranges() {
        let (vehicle, _msg_tx, _sent) = connect_mock_vehicle_with_sent().await;

        assert!(matches!(
            vehicle.ardupilot().motor_test(0, 10.0, 5).await,
            Err(VehicleError::InvalidParameter(_))
        ));
        assert!(matches!(
            vehicle.ardupilot().motor_test(13, 10.0, 5).await,
            Err(VehicleError::InvalidParameter(_))
        ));
        assert!(matches!(
            vehicle.ardupilot().motor_test(1, -0.1, 5).await,
            Err(VehicleError::InvalidParameter(_))
        ));
        assert!(matches!(
            vehicle.ardupilot().motor_test(1, 100.1, 5).await,
            Err(VehicleError::InvalidParameter(_))
        ));
        assert!(matches!(
            vehicle.ardupilot().motor_test(1, 10.0, 0).await,
            Err(VehicleError::InvalidParameter(_))
        ));
        assert!(matches!(
            vehicle.ardupilot().motor_test(1, 10.0, 31).await,
            Err(VehicleError::InvalidParameter(_))
        ));

        vehicle
            .disconnect()
            .await
            .expect("disconnect should succeed");
    }

    #[tokio::test]
    async fn ardupilot_motor_test_sends_expected_command_long_shape() {
        let (vehicle, msg_tx, sent) = connect_mock_vehicle_with_sent().await;

        let motor_test_task = {
            let vehicle = vehicle.clone();
            tokio::spawn(async move { vehicle.ardupilot().motor_test(3, 42.5, 9).await })
        };

        tokio::time::sleep(Duration::from_millis(10)).await;
        msg_tx
            .send((
                default_header(),
                ack_msg(
                    dialect::MavCmd::MAV_CMD_DO_MOTOR_TEST,
                    dialect::MavResult::MAV_RESULT_ACCEPTED,
                ),
            ))
            .await
            .expect("motor test ack should be delivered");

        assert!(
            motor_test_task
                .await
                .expect("motor test task should join")
                .is_ok()
        );

        let motor_test = sent
            .lock()
            .expect("sent messages lock should not poison")
            .iter()
            .find_map(|(_, msg)| match msg {
                dialect::MavMessage::COMMAND_LONG(data)
                    if data.command == dialect::MavCmd::MAV_CMD_DO_MOTOR_TEST =>
                {
                    Some(data.clone())
                }
                _ => None,
            })
            .expect("motor test command should be sent");

        assert_eq!(motor_test.param1, 3.0);
        assert_eq!(motor_test.param2, 0.0);
        assert!((motor_test.param3 - 42.5).abs() < f32::EPSILON);
        assert_eq!(motor_test.param4, 9.0);
        assert_eq!(motor_test.param5, 1.0);
        assert_eq!(motor_test.param6, 0.0);
        assert_eq!(motor_test.param7, 0.0);

        vehicle
            .disconnect()
            .await
            .expect("disconnect should succeed");
    }

    #[tokio::test]
    async fn start_mag_cal_sends_and_accepts_ack() {
        let (vehicle, msg_tx, sent) = connect_mock_vehicle_with_sent().await;

        let start_task = {
            let vehicle = vehicle.clone();
            tokio::spawn(async move { vehicle.ardupilot().start_mag_cal(0b0000_0111).await })
        };

        tokio::time::sleep(Duration::from_millis(10)).await;
        msg_tx
            .send((
                default_header(),
                ack_msg_by_id(
                    MAV_CMD_DO_START_MAG_CAL_ID,
                    dialect::MavResult::MAV_RESULT_ACCEPTED,
                ),
            ))
            .await
            .expect("start_mag_cal ack should be delivered");

        assert!(
            start_task
                .await
                .expect("start_mag_cal task should join")
                .is_ok()
        );

        let command = sent
            .lock()
            .expect("sent messages lock should not poison")
            .iter()
            .find_map(|(_, msg)| match msg {
                dialect::MavMessage::COMMAND_LONG(data)
                    if data.command as u16 == MAV_CMD_DO_START_MAG_CAL_ID =>
                {
                    Some(data.clone())
                }
                _ => None,
            })
            .expect("start_mag_cal command should be sent");

        assert_eq!(command.param1, 7.0);
        assert_eq!(command.param2, 0.0);
        assert_eq!(command.param3, 0.0);
        assert_eq!(command.param4, 0.0);
        assert_eq!(command.param5, 0.0);
        assert_eq!(command.param6, 0.0);
        assert_eq!(command.param7, 0.0);

        vehicle
            .disconnect()
            .await
            .expect("disconnect should succeed");
    }

    #[tokio::test]
    async fn accept_mag_cal_sends_and_accepts_ack() {
        let (vehicle, msg_tx, sent) = connect_mock_vehicle_with_sent().await;

        let accept_task = {
            let vehicle = vehicle.clone();
            tokio::spawn(async move { vehicle.ardupilot().accept_mag_cal().await })
        };

        tokio::time::sleep(Duration::from_millis(10)).await;
        msg_tx
            .send((
                default_header(),
                ack_msg_by_id(
                    MAV_CMD_DO_ACCEPT_MAG_CAL_ID,
                    dialect::MavResult::MAV_RESULT_ACCEPTED,
                ),
            ))
            .await
            .expect("accept_mag_cal ack should be delivered");

        assert!(
            accept_task
                .await
                .expect("accept_mag_cal task should join")
                .is_ok()
        );

        let command = sent
            .lock()
            .expect("sent messages lock should not poison")
            .iter()
            .find_map(|(_, msg)| match msg {
                dialect::MavMessage::COMMAND_LONG(data)
                    if data.command as u16 == MAV_CMD_DO_ACCEPT_MAG_CAL_ID =>
                {
                    Some(data.clone())
                }
                _ => None,
            })
            .expect("accept_mag_cal command should be sent");

        assert_eq!(command.param1, 0.0);
        assert_eq!(command.param2, 0.0);
        assert_eq!(command.param3, 0.0);
        assert_eq!(command.param4, 0.0);
        assert_eq!(command.param5, 0.0);
        assert_eq!(command.param6, 0.0);
        assert_eq!(command.param7, 0.0);

        vehicle
            .disconnect()
            .await
            .expect("disconnect should succeed");
    }

    #[tokio::test]
    async fn cancel_mag_cal_sends_and_accepts_ack() {
        let (vehicle, msg_tx, sent) = connect_mock_vehicle_with_sent().await;

        let cancel_task = {
            let vehicle = vehicle.clone();
            tokio::spawn(async move { vehicle.ardupilot().cancel_mag_cal().await })
        };

        tokio::time::sleep(Duration::from_millis(10)).await;
        msg_tx
            .send((
                default_header(),
                ack_msg_by_id(
                    MAV_CMD_DO_CANCEL_MAG_CAL_ID,
                    dialect::MavResult::MAV_RESULT_ACCEPTED,
                ),
            ))
            .await
            .expect("cancel_mag_cal ack should be delivered");

        assert!(
            cancel_task
                .await
                .expect("cancel_mag_cal task should join")
                .is_ok()
        );

        let command = sent
            .lock()
            .expect("sent messages lock should not poison")
            .iter()
            .find_map(|(_, msg)| match msg {
                dialect::MavMessage::COMMAND_LONG(data)
                    if data.command as u16 == MAV_CMD_DO_CANCEL_MAG_CAL_ID =>
                {
                    Some(data.clone())
                }
                _ => None,
            })
            .expect("cancel_mag_cal command should be sent");

        assert_eq!(command.param1, 0.0);
        assert_eq!(command.param2, 0.0);
        assert_eq!(command.param3, 0.0);
        assert_eq!(command.param4, 0.0);
        assert_eq!(command.param5, 0.0);
        assert_eq!(command.param6, 0.0);
        assert_eq!(command.param7, 0.0);

        vehicle
            .disconnect()
            .await
            .expect("disconnect should succeed");
    }

    async fn preflight_calibration_command(
        vehicle: Vehicle,
        msg_tx: &mpsc::Sender<(MavHeader, dialect::MavMessage)>,
        sent: &SentMessages,
        gyro: bool,
        accel: bool,
        baro: bool,
        accel_trim: bool,
    ) -> dialect::COMMAND_LONG_DATA {
        let preflight_task = tokio::spawn(async move {
            vehicle
                .ardupilot()
                .preflight_calibration(gyro, accel, baro, accel_trim)
                .await
        });

        tokio::time::sleep(Duration::from_millis(10)).await;
        msg_tx
            .send((
                default_header(),
                ack_msg(
                    dialect::MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION,
                    dialect::MavResult::MAV_RESULT_ACCEPTED,
                ),
            ))
            .await
            .expect("preflight calibration ack should be delivered");

        assert!(
            preflight_task
                .await
                .expect("preflight calibration task should join")
                .is_ok()
        );

        sent.lock()
            .expect("sent messages lock should not poison")
            .iter()
            .rev()
            .find_map(|(_, msg)| match msg {
                dialect::MavMessage::COMMAND_LONG(data)
                    if data.command == dialect::MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION =>
                {
                    Some(data.clone())
                }
                _ => None,
            })
            .expect("preflight calibration command should be sent")
    }

    #[tokio::test]
    async fn preflight_cal_gyro() {
        let (vehicle, msg_tx, sent) = connect_mock_vehicle_with_sent().await;

        let command = preflight_calibration_command(
            vehicle.clone(),
            &msg_tx,
            &sent,
            true,
            false,
            false,
            false,
        )
        .await;

        assert_eq!(command.param1, 1.0);
        assert_eq!(command.param2, 0.0);
        assert_eq!(command.param3, 0.0);
        assert_eq!(command.param4, 0.0);
        assert_eq!(command.param5, 0.0);
        assert_eq!(command.param6, 0.0);
        assert_eq!(command.param7, 0.0);

        vehicle
            .disconnect()
            .await
            .expect("disconnect should succeed");
    }

    #[tokio::test]
    async fn preflight_cal_level_horizon_accel() {
        let (vehicle, msg_tx, sent) = connect_mock_vehicle_with_sent().await;

        let command = preflight_calibration_command(
            vehicle.clone(),
            &msg_tx,
            &sent,
            false,
            true,
            false,
            false,
        )
        .await;

        assert_eq!(command.param1, 0.0);
        assert_eq!(command.param2, 0.0);
        assert_eq!(command.param3, 0.0);
        assert_eq!(command.param4, 0.0);
        assert_eq!(command.param5, 1.0);
        assert_eq!(command.param6, 0.0);
        assert_eq!(command.param7, 0.0);

        vehicle
            .disconnect()
            .await
            .expect("disconnect should succeed");
    }

    #[tokio::test]
    async fn preflight_cal_accel_trim() {
        let (vehicle, msg_tx, sent) = connect_mock_vehicle_with_sent().await;

        let command = preflight_calibration_command(
            vehicle.clone(),
            &msg_tx,
            &sent,
            false,
            false,
            false,
            true,
        )
        .await;

        assert_eq!(command.param1, 0.0);
        assert_eq!(command.param2, 0.0);
        assert_eq!(command.param3, 0.0);
        assert_eq!(command.param4, 0.0);
        assert_eq!(command.param5, 2.0);
        assert_eq!(command.param6, 0.0);
        assert_eq!(command.param7, 0.0);

        vehicle
            .disconnect()
            .await
            .expect("disconnect should succeed");
    }

    #[tokio::test]
    async fn ardupilot_mag_cal_observations_aggregate_by_compass_id() {
        let (writers, channels) = create_channels();
        let cancel = tokio_util::sync::CancellationToken::new();
        let domain = ArduPilotDomain::new();
        domain.start(&channels, cancel.clone());

        assert_eq!(domain.mag_cal_progress().latest(), Some(Vec::new()));
        assert_eq!(domain.mag_cal_report().latest(), Some(Vec::new()));

        let mut progress_sub = domain.mag_cal_progress().subscribe();
        let mut report_sub = domain.mag_cal_report().subscribe();

        writers
            .mag_cal_progress
            .send_replace(Some(state::MagCalProgress {
                compass_id: 2,
                completion_pct: 45,
                status: state::MagCalStatus::RunningStepOne,
                attempt: 1,
            }));

        let first_progress = timeout(Duration::from_millis(250), async {
            loop {
                let observed = progress_sub.recv().await.expect("progress update expected");
                if observed.len() == 1 {
                    break observed;
                }
            }
        })
        .await
        .expect("progress aggregation should update");

        assert_eq!(
            first_progress
                .iter()
                .map(|item| item.compass_id)
                .collect::<Vec<_>>(),
            vec![2]
        );

        writers
            .mag_cal_progress
            .send_replace(Some(state::MagCalProgress {
                compass_id: 1,
                completion_pct: 80,
                status: state::MagCalStatus::RunningStepTwo,
                attempt: 2,
            }));

        let progress = timeout(Duration::from_millis(250), async {
            loop {
                let observed = progress_sub.recv().await.expect("progress update expected");
                if observed.len() == 2 {
                    break observed;
                }
            }
        })
        .await
        .expect("progress aggregation should update");

        assert_eq!(
            progress
                .iter()
                .map(|item| item.compass_id)
                .collect::<Vec<_>>(),
            vec![1, 2]
        );
        assert_eq!(progress[0].completion_pct, 80);
        assert_eq!(progress[1].completion_pct, 45);

        writers
            .mag_cal_report
            .send_replace(Some(state::MagCalReport {
                compass_id: 2,
                status: state::MagCalStatus::Success,
                fitness: 0.12,
                ofs_x: 1.0,
                ofs_y: 2.0,
                ofs_z: 3.0,
                autosaved: true,
            }));

        let reports = timeout(Duration::from_millis(250), async {
            loop {
                let observed = report_sub.recv().await.expect("report update expected");
                if observed.len() == 1 {
                    break observed;
                }
            }
        })
        .await
        .expect("report aggregation should update");

        assert_eq!(reports[0].compass_id, 2);
        assert_eq!(reports[0].status, state::MagCalStatus::Success);
        assert!(reports[0].autosaved);

        cancel.cancel();
    }
}
