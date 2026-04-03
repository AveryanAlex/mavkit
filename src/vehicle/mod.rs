use crate::ardupilot::ArduPilotDomain;
use crate::command::Command;
use crate::config::VehicleConfig;
use crate::error::VehicleError;
use crate::event_loop::InitManager;
use crate::fence::FenceDomain;
use crate::info::InfoDomain;
use crate::mission::{MissionDomain, MissionProtocolScope};
use crate::modes::ModeDomain;
use crate::params::ParamsDomain;
use crate::rally::RallyDomain;
use crate::state::StateChannels;
use crate::support::SupportDomain;
use std::sync::Arc;
use tokio::sync::{mpsc, oneshot};
use tokio_util::sync::CancellationToken;

mod accessors;
mod actions;
mod lifecycle;
mod rc_override;

#[cfg(test)]
mod tests;

pub use crate::ardupilot::ArduPilotHandle;
pub use crate::fence::FenceHandle;
pub use crate::mission::MissionHandle;
pub use crate::params::ParamsHandle;
pub use crate::rally::RallyHandle;
pub use crate::state::LinkState;
pub use crate::state::{AutopilotType, VehicleType};
pub use rc_override::{RcOverride, RcOverrideChannelValue};

/// Root handle for a live MAVLink connection and all MAVKit domains.
///
/// `Vehicle` is cheaply cloneable — every clone shares the same underlying connection and event
/// loop via an `Arc`. The event loop shuts down automatically when the last clone is dropped, or
/// explicitly via [`Vehicle::disconnect`].
///
/// # Lifecycle
/// 1. Call [`Vehicle::connect`] (or a typed variant) to open the transport and wait for the
///    first heartbeat.
/// 2. Use domain accessor methods (`vehicle.telemetry()`, `vehicle.mission()`, etc.) to get
///    short-lived handles that borrow `self`. Each handle exposes operations scoped to its domain.
/// 3. Call [`Vehicle::disconnect`] when finished, or simply drop all clones.
///
/// # Thread safety
/// `Vehicle` is `Clone + Send + Sync`. Clones may be sent to other tasks freely.
#[derive(Clone)]
pub struct Vehicle {
    pub(crate) inner: Arc<VehicleInner>,
}

pub(crate) struct VehicleInner {
    pub(crate) command_tx: mpsc::Sender<Command>,
    pub(crate) cancel: CancellationToken,
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
    pub(crate) fn new(
        command_tx: mpsc::Sender<Command>,
        cancel: CancellationToken,
        stores: StateChannels,
        config: VehicleConfig,
    ) -> Self {
        Self {
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
            _config: config,
        }
    }

    pub(crate) fn start_background_domains(&self, init_manager: &InitManager) {
        self.mission.start(&self.stores, self.cancel.clone());
        self.ardupilot.start(&self.stores, self.cancel.clone());
        self.info.start(&self.stores, init_manager);
        self.modes.start(&self.stores, self.command_tx.clone());
        self.support.start(&self.stores, init_manager);
    }
}

/// Identity snapshot populated from the first usable heartbeat.
///
/// Fields are stable for the lifetime of the connection — a reconnect produces a new
/// [`Vehicle`] with a fresh `VehicleIdentity`. Obtained via [`Vehicle::identity`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VehicleIdentity {
    /// MAVLink system ID of the connected vehicle (typically 1).
    pub system_id: u8,
    /// MAVLink component ID of the connected vehicle (typically 1 for the autopilot).
    pub component_id: u8,
    /// Autopilot firmware family (e.g. ArduPilot, PX4).
    pub autopilot: AutopilotType,
    /// Vehicle class (e.g. quadrotor, fixed-wing).
    pub vehicle_type: VehicleType,
}

impl Vehicle {
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
