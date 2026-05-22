use tokio::sync::{mpsc, oneshot, watch};

use crate::error::VehicleError;
use crate::geo::GeoPoint3dMsl;
use crate::mission::HomePosition;
use crate::{Vehicle, VehicleConfig, runtime};

use super::runtime::run_simulator;
use super::state::{
    ControlMessage, DEFAULT_HOME_ALT_M, DEFAULT_HOME_LAT_DEG, DEFAULT_HOME_LON_DEG,
    DEFAULT_TICK_HZ, DemoVehicleConfig, TeleportTarget, default_mode,
};
use super::transport::SimulatorConnection;

/// Entry point for the demo in-process simulator.
pub struct DemoVehicle;

/// Demo simulator vehicle family.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DemoProfile {
    /// ArduCopter-compatible demo profile.
    #[default]
    ArduCopter,
    /// ArduPlane-compatible fixed-wing demo profile.
    ArduPlane,
    /// ArduPlane-compatible quadplane VTOL demo profile.
    ArduQuadPlane,
}

/// Clock mode for the simulator tick loop.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DemoClock {
    /// Advance automatically on a wall-clock interval.
    #[default]
    RealTime,
    /// Advance only when [`DemoVehicleHandle::step`] is called.
    Manual,
}

/// Builder for configuring the demo simulator.
#[derive(Debug, Clone)]
pub struct DemoVehicleBuilder {
    profile: DemoProfile,
    clock: DemoClock,
    tick_hz: u32,
    home: HomePosition,
}

/// A cheap snapshot of the current simulator state.
#[derive(Debug, Clone, PartialEq)]
pub struct DemoVehicleSnapshot {
    pub time_boot_ms: u32,
    pub armed: bool,
    pub custom_mode: u32,
    pub home: HomePosition,
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    pub altitude_msl_m: f64,
    pub relative_alt_m: f64,
    pub roll_rad: f32,
    pub pitch_rad: f32,
    pub yaw_rad: f32,
    pub mission_current_wire_seq: u16,
    pub mission_total_wire_items: u16,
}

/// Handle for stepping or inspecting the running simulator.
#[derive(Clone)]
pub struct DemoVehicleHandle {
    pub(crate) control_tx: mpsc::Sender<ControlMessage>,
    pub(crate) snapshot_rx: watch::Receiver<DemoVehicleSnapshot>,
}

impl DemoVehicle {
    /// Start building a demo simulator instance.
    pub fn builder() -> DemoVehicleBuilder {
        DemoVehicleBuilder::default()
    }

    /// Start a simulator and connect MAVKit to it through an in-process MAVLink transport.
    pub async fn connect(
        config: VehicleConfig,
    ) -> Result<(Vehicle, DemoVehicleHandle), VehicleError> {
        Self::builder().connect(config).await
    }
}

impl Default for DemoVehicleBuilder {
    fn default() -> Self {
        Self {
            profile: DemoProfile::default(),
            clock: DemoClock::default(),
            tick_hz: DEFAULT_TICK_HZ,
            home: HomePosition {
                latitude_deg: DEFAULT_HOME_LAT_DEG,
                longitude_deg: DEFAULT_HOME_LON_DEG,
                altitude_m: DEFAULT_HOME_ALT_M,
            },
        }
    }
}

impl DemoVehicleBuilder {
    pub fn profile(mut self, profile: DemoProfile) -> Self {
        self.profile = profile;
        self
    }

    pub fn clock(mut self, clock: DemoClock) -> Self {
        self.clock = clock;
        self
    }

    pub fn tick_hz(mut self, tick_hz: u32) -> Self {
        self.tick_hz = tick_hz;
        self
    }

    pub fn home(mut self, home: HomePosition) -> Self {
        self.home = home;
        self
    }

    pub async fn connect(
        self,
        vehicle_config: VehicleConfig,
    ) -> Result<(Vehicle, DemoVehicleHandle), VehicleError> {
        let sim_config = self.build_config()?;
        let initial_snapshot = DemoVehicleSnapshot {
            time_boot_ms: 0,
            armed: false,
            custom_mode: default_mode(sim_config.profile),
            home: sim_config.home.clone(),
            latitude_deg: sim_config.home.latitude_deg,
            longitude_deg: sim_config.home.longitude_deg,
            altitude_msl_m: sim_config.home.altitude_m,
            relative_alt_m: 0.0,
            roll_rad: 0.0,
            pitch_rad: 0.0,
            yaw_rad: 0.0,
            mission_current_wire_seq: 0,
            mission_total_wire_items: 0,
        };

        let (connection, endpoints) = SimulatorConnection::new(128);
        let (control_tx, control_rx) = mpsc::channel(16);
        let (snapshot_tx, snapshot_rx) = watch::channel(initial_snapshot.clone());

        runtime::spawn(run_simulator(
            sim_config,
            endpoints,
            control_rx,
            snapshot_tx,
            initial_snapshot,
        ));

        let vehicle = match Vehicle::from_connection(Box::new(connection), vehicle_config).await {
            Ok(vehicle) => vehicle,
            Err(err) => {
                let _ = control_tx
                    .send(ControlMessage::Shutdown {
                        reply: oneshot::channel().0,
                    })
                    .await;
                return Err(err);
            }
        };

        Ok((
            vehicle,
            DemoVehicleHandle {
                control_tx,
                snapshot_rx,
            },
        ))
    }

    pub(crate) fn build_config(self) -> Result<DemoVehicleConfig, VehicleError> {
        if self.tick_hz == 0 {
            return Err(VehicleError::InvalidParameter(
                "demo simulator tick_hz must be at least 1".to_string(),
            ));
        }

        Ok(DemoVehicleConfig {
            profile: self.profile,
            clock: self.clock,
            tick_hz: self.tick_hz,
            home: self.home,
        })
    }
}

impl DemoVehicleHandle {
    pub fn snapshot(&self) -> DemoVehicleSnapshot {
        self.snapshot_rx.borrow().clone()
    }

    /// Move the demo simulator's vehicle state to an absolute MSL position immediately.
    ///
    /// This is a simulator control-plane operation rather than a MAVLink command: it updates the
    /// in-process backend directly and emits a telemetry burst so connected [`Vehicle`] handles can
    /// observe the new position without waiting for another tick. The active flight mode, armed
    /// state, home position, mission progress, and guided/mission target intent are preserved.
    pub async fn teleport_to(
        &self,
        position: GeoPoint3dMsl,
    ) -> Result<DemoVehicleSnapshot, VehicleError> {
        let target = TeleportTarget::try_from(position)?;
        let (reply_tx, reply_rx) = oneshot::channel();
        self.control_tx
            .send(ControlMessage::TeleportTo {
                target,
                reply: reply_tx,
            })
            .await
            .map_err(|_| VehicleError::Disconnected)?;
        reply_rx.await.map_err(|_| VehicleError::Disconnected)?
    }

    pub async fn step(&self) -> Result<DemoVehicleSnapshot, VehicleError> {
        let (reply_tx, reply_rx) = oneshot::channel();
        self.control_tx
            .send(ControlMessage::Step { reply: reply_tx })
            .await
            .map_err(|_| VehicleError::Disconnected)?;
        reply_rx.await.map_err(|_| VehicleError::Disconnected)?
    }

    pub async fn shutdown(&self) -> Result<(), VehicleError> {
        let (reply_tx, reply_rx) = oneshot::channel();
        self.control_tx
            .send(ControlMessage::Shutdown { reply: reply_tx })
            .await
            .map_err(|_| VehicleError::Disconnected)?;
        reply_rx.await.map_err(|_| VehicleError::Disconnected)?
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn builder_rejects_zero_tick_rate() {
        let error = DemoVehicle::builder()
            .tick_hz(0)
            .build_config()
            .unwrap_err();
        assert!(matches!(error, VehicleError::InvalidParameter(_)));
    }

    #[test]
    fn teleport_target_quantizes_position_to_wire_units() {
        let target = TeleportTarget::try_from(GeoPoint3dMsl {
            latitude_deg: 12.345_678_9,
            longitude_deg: -98.765_432_1,
            altitude_msl_m: 123.456_7,
        })
        .unwrap();

        assert_eq!(target.latitude_e7, 123_456_789);
        assert_eq!(target.longitude_e7, -987_654_321);
        assert_eq!(target.altitude_mm, 123_457);
    }

    #[test]
    fn teleport_target_rejects_invalid_position_values() {
        let error = TeleportTarget::try_from(GeoPoint3dMsl {
            latitude_deg: 91.0,
            longitude_deg: 0.0,
            altitude_msl_m: 0.0,
        })
        .unwrap_err();
        assert!(matches!(error, VehicleError::InvalidParameter(_)));

        let error = TeleportTarget::try_from(GeoPoint3dMsl {
            latitude_deg: 0.0,
            longitude_deg: 0.0,
            altitude_msl_m: f64::INFINITY,
        })
        .unwrap_err();
        assert!(matches!(error, VehicleError::InvalidParameter(_)));
    }
}
