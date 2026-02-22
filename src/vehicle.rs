use crate::command::Command;
use crate::config::VehicleConfig;
use crate::error::VehicleError;
use crate::event_loop::run_event_loop;
use crate::mission::{HomePosition, MissionHandle, TransferProgress};
use crate::params::{ParamProgress, ParamStore, ParamsHandle};
use crate::state::{
    create_channels, FlightMode, LinkState, MissionState, StateChannels, StatusMessage, Telemetry,
    VehicleIdentity, VehicleState,
};
use mavlink::common::{self, MavCmd};
use std::sync::Arc;
use tokio::sync::{mpsc, oneshot, watch};
use tokio_util::sync::CancellationToken;

/// Async MAVLink vehicle handle.
///
/// `Vehicle` is `Clone + Send + Sync`. Clones share the same connection.
/// When the last clone is dropped, the event loop is cancelled.
#[derive(Clone)]
pub struct Vehicle {
    pub(crate) inner: Arc<VehicleInner>,
}

pub(crate) struct VehicleInner {
    pub(crate) command_tx: mpsc::Sender<Command>,
    cancel: CancellationToken,
    channels: StateChannels,
    _config: VehicleConfig,
}

impl Drop for VehicleInner {
    fn drop(&mut self) {
        self.cancel.cancel();
    }
}

impl Vehicle {
    /// Connect using a mavlink address string (e.g. `udpin:0.0.0.0:14550`).
    /// Waits for the first HEARTBEAT before returning.
    pub async fn connect(address: &str) -> Result<Self, VehicleError> {
        Self::connect_with_config(address, VehicleConfig::default()).await
    }

    /// Connect via UDP. `bind_addr` is `host:port` to bind to (e.g. `0.0.0.0:14550`).
    pub async fn connect_udp(bind_addr: &str) -> Result<Self, VehicleError> {
        Self::connect(&format!("udpin:{bind_addr}")).await
    }

    /// Connect via TCP. `addr` is `host:port` to connect to.
    pub async fn connect_tcp(addr: &str) -> Result<Self, VehicleError> {
        Self::connect(&format!("tcpin:{addr}")).await
    }

    /// Connect via serial port.
    pub async fn connect_serial(port: &str, baud: u32) -> Result<Self, VehicleError> {
        Self::connect(&format!("serial:{port}:{baud}")).await
    }

    /// Connect with a custom `VehicleConfig`.
    pub async fn connect_with_config(
        address: &str,
        config: VehicleConfig,
    ) -> Result<Self, VehicleError> {
        let connection = mavlink::connect_async::<common::MavMessage>(address)
            .await
            .map_err(|err| VehicleError::ConnectionFailed(err.to_string()))?;

        Self::from_connection(connection, config).await
    }

    /// Create a `Vehicle` from a pre-built [`AsyncMavConnection`].
    ///
    /// This is the transport-agnostic entry point: any connection that
    /// implements `AsyncMavConnection<common::MavMessage>` can be used,
    /// including the [`StreamConnection`](crate::stream_connection::StreamConnection)
    /// adapter for BLE / SPP byte streams.
    ///
    /// Waits for the first HEARTBEAT before returning.
    pub async fn from_connection(
        connection: Box<dyn mavlink::AsyncMavConnection<common::MavMessage> + Sync + Send>,
        config: VehicleConfig,
    ) -> Result<Self, VehicleError> {
        let (writers, channels) = create_channels();
        let cancel = CancellationToken::new();
        let (command_tx, command_rx) = mpsc::channel(config.command_buffer_size);

        let loop_cancel = cancel.clone();
        let loop_config_timeout = config.connect_timeout;

        // Spawn the event loop
        let writers_for_loop = writers;
        tokio::spawn(run_event_loop(
            connection,
            command_rx,
            writers_for_loop,
            VehicleConfig {
                gcs_system_id: config.gcs_system_id,
                gcs_component_id: config.gcs_component_id,
                retry_policy: config.retry_policy,
                auto_request_home: config.auto_request_home,
                command_buffer_size: config.command_buffer_size,
                connect_timeout: config.connect_timeout,
            },
            loop_cancel,
        ));

        let vehicle = Vehicle {
            inner: Arc::new(VehicleInner {
                command_tx,
                cancel,
                channels,
                _config: config,
            }),
        };

        // Wait for first HEARTBEAT (indicated by vehicle_state becoming non-default)
        let mut vs_rx = vehicle.state();
        let heartbeat_wait = async {
            loop {
                vs_rx.changed().await.map_err(|_| VehicleError::Disconnected)?;
                let state = vs_rx.borrow().clone();
                // A heartbeat sets autopilot to something (at minimum Generic from target update)
                if state.custom_mode != 0 || state.armed || state.mode_name != "" {
                    return Ok::<(), VehicleError>(());
                }
            }
        };

        tokio::select! {
            result = heartbeat_wait => result?,
            _ = tokio::time::sleep(loop_config_timeout) => {
                return Err(VehicleError::Timeout);
            }
        }

        Ok(vehicle)
    }

    // --- Reactive state (watch channels) ---

    pub fn state(&self) -> watch::Receiver<VehicleState> {
        self.inner.channels.vehicle_state.clone()
    }

    pub fn telemetry(&self) -> watch::Receiver<Telemetry> {
        self.inner.channels.telemetry.clone()
    }

    pub fn home_position(&self) -> watch::Receiver<Option<HomePosition>> {
        self.inner.channels.home_position.clone()
    }

    pub fn mission_state(&self) -> watch::Receiver<MissionState> {
        self.inner.channels.mission_state.clone()
    }

    pub fn link_state(&self) -> watch::Receiver<LinkState> {
        self.inner.channels.link_state.clone()
    }

    pub fn mission_progress(&self) -> watch::Receiver<Option<TransferProgress>> {
        self.inner.channels.mission_progress.clone()
    }

    pub fn param_store(&self) -> watch::Receiver<ParamStore> {
        self.inner.channels.param_store.clone()
    }

    pub fn param_progress(&self) -> watch::Receiver<ParamProgress> {
        self.inner.channels.param_progress.clone()
    }

    pub fn statustext(&self) -> watch::Receiver<Option<StatusMessage>> {
        self.inner.channels.statustext.clone()
    }

    // --- Vehicle commands ---

    pub async fn arm(&self, force: bool) -> Result<(), VehicleError> {
        self.send_command(|reply| Command::Arm { force, reply }).await
    }

    pub async fn disarm(&self, force: bool) -> Result<(), VehicleError> {
        self.send_command(|reply| Command::Disarm { force, reply }).await
    }

    pub async fn set_mode(&self, custom_mode: u32) -> Result<(), VehicleError> {
        self.send_command(|reply| Command::SetMode { custom_mode, reply }).await
    }

    pub async fn set_mode_by_name(&self, name: &str) -> Result<(), VehicleError> {
        let state = self.inner.channels.vehicle_state.borrow().clone();
        let custom_mode = crate::modes::mode_number(state.autopilot, state.vehicle_type, name)
            .ok_or_else(|| VehicleError::ModeNotAvailable(name.to_string()))?;
        self.set_mode(custom_mode).await
    }

    pub async fn takeoff(&self, altitude_m: f32) -> Result<(), VehicleError> {
        self.command_long(
            MavCmd::MAV_CMD_NAV_TAKEOFF,
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, altitude_m],
        )
        .await
    }

    pub async fn goto(&self, lat_deg: f64, lon_deg: f64, alt_m: f32) -> Result<(), VehicleError> {
        let lat_e7 = (lat_deg * 1e7) as i32;
        let lon_e7 = (lon_deg * 1e7) as i32;
        self.send_command(|reply| Command::GuidedGoto {
            lat_e7,
            lon_e7,
            alt_m,
            reply,
        })
        .await
    }

    pub async fn command_long(
        &self,
        cmd: MavCmd,
        params: [f32; 7],
    ) -> Result<(), VehicleError> {
        self.send_command(|reply| Command::CommandLong {
            command: cmd,
            params,
            reply,
        })
        .await
    }

    /// MAV_CMD_PREFLIGHT_CALIBRATION. Set params to 1 to start that calibration.
    pub async fn preflight_calibration(
        &self,
        gyro: bool,
        accel: bool,
        radio_trim: bool,
    ) -> Result<(), VehicleError> {
        self.command_long(
            MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION,
            [
                if gyro { 1.0 } else { 0.0 },      // param1: gyro
                0.0,                                  // param2: magnetometer
                0.0,                                  // param3: ground pressure
                if radio_trim { 1.0 } else { 0.0 },  // param4: radio trim
                if accel { 1.0 } else { 0.0 },       // param5: accel
                0.0,                                  // param6: compass/motor
                0.0,                                  // param7: esc
            ],
        )
        .await
    }

    pub fn available_modes(&self) -> Vec<FlightMode> {
        let state = self.inner.channels.vehicle_state.borrow().clone();
        crate::modes::available_modes(state.autopilot, state.vehicle_type)
    }

    pub fn identity(&self) -> Option<VehicleIdentity> {
        let state = self.inner.channels.vehicle_state.borrow().clone();
        if state.mode_name.is_empty()
            && !state.armed
            && state.custom_mode == 0
        {
            return None;
        }
        Some(VehicleIdentity {
            system_id: 0, // Not directly exposed through watch channels
            component_id: 0,
            autopilot: state.autopilot,
            vehicle_type: state.vehicle_type,
        })
    }

    /// Mission sub-API.
    pub fn mission(&self) -> MissionHandle<'_> {
        MissionHandle::new(self)
    }

    /// Parameter sub-API.
    pub fn params(&self) -> ParamsHandle<'_> {
        ParamsHandle::new(self)
    }

    /// Gracefully disconnect from the vehicle.
    pub async fn disconnect(self) -> Result<(), VehicleError> {
        let _ = self.inner.command_tx.send(Command::Shutdown).await;
        Ok(())
    }

    // --- Internal helper ---

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
