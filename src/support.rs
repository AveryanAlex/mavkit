use crate::dialect::{self, MavProtocolCapability};
use crate::event_loop::{InitManager, InitSnapshot, InitState};
use crate::observation::{ObservationHandle, ObservationWriter};
use crate::runtime::{self, TaskHandle};
use crate::shared_state::recover_lock;
use crate::state::{AutopilotType, StateChannels, VehicleState};
use crate::types::SupportState;
use crate::vehicle::VehicleInner;
use std::sync::Arc;
use std::sync::Mutex;

/// Accessor for capability support observations derived from the vehicle's protocol capabilities.
///
/// Obtained from [`Vehicle::support`](crate::Vehicle::support). Capability flags are extracted
/// from the `AUTOPILOT_VERSION` message (`capabilities` bitmask). Observations remain unpublished
/// until there is evidence for the domain: `AUTOPILOT_VERSION` for capability support, and a
/// heartbeat/autopilot identity for ArduPilot support.
///
/// [`ObservationHandle::latest`](crate::ObservationHandle::latest) returns `None` before evidence
/// exists, [`SupportState::Supported`] or [`SupportState::Unsupported`] once the vehicle gives a
/// conclusive signal, and [`SupportState::Unknown`] only for terminal but inconclusive signals.
#[derive(Clone)]
pub struct SupportHandle<'a> {
    inner: &'a VehicleInner,
}

impl<'a> SupportHandle<'a> {
    pub(crate) fn new(inner: &'a VehicleInner) -> Self {
        Self { inner }
    }

    /// Whether the vehicle supports `COMMAND_INT` (`MAV_PROTOCOL_CAPABILITY_COMMAND_INT`).
    pub fn command_int(&self) -> ObservationHandle<SupportState> {
        self.seed_from_vehicle_state();
        self.inner.support.command_int()
    }

    /// Whether the vehicle supports MAVLink FTP (`MAV_PROTOCOL_CAPABILITY_FTP`).
    pub fn ftp(&self) -> ObservationHandle<SupportState> {
        self.seed_from_vehicle_state();
        self.inner.support.ftp()
    }

    /// Whether the vehicle supports terrain following (`MAV_PROTOCOL_CAPABILITY_TERRAIN`).
    pub fn terrain(&self) -> ObservationHandle<SupportState> {
        self.seed_from_vehicle_state();
        self.inner.support.terrain()
    }

    /// Whether the vehicle supports the geofence mission protocol
    /// (`MAV_PROTOCOL_CAPABILITY_MISSION_FENCE`).
    pub fn mission_fence(&self) -> ObservationHandle<SupportState> {
        self.seed_from_vehicle_state();
        self.inner.support.mission_fence()
    }

    /// Whether the vehicle supports the rally-point mission protocol
    /// (`MAV_PROTOCOL_CAPABILITY_MISSION_RALLY`).
    pub fn mission_rally(&self) -> ObservationHandle<SupportState> {
        self.seed_from_vehicle_state();
        self.inner.support.mission_rally()
    }

    /// Whether the connected autopilot is ArduPilot.
    ///
    /// Derived from the `autopilot` field in the heartbeat, not from capability bits. Returns
    /// [`SupportState::Unsupported`] for any non-ArduPilot
    /// autopilot once a heartbeat is received.
    pub fn ardupilot(&self) -> ObservationHandle<SupportState> {
        self.seed_from_vehicle_state();
        self.inner.support.ardupilot()
    }

    fn seed_from_vehicle_state(&self) {
        let state = self.inner.stores.vehicle_state.borrow().clone();
        self.inner.support.seed_from_vehicle_state(&state);
    }
}

#[derive(Clone)]
pub(crate) struct SupportDomain {
    inner: Arc<SupportDomainInner>,
}

struct SupportDomainInner {
    command_int_writer: ObservationWriter<SupportState>,
    command_int: ObservationHandle<SupportState>,
    ftp_writer: ObservationWriter<SupportState>,
    ftp: ObservationHandle<SupportState>,
    terrain_writer: ObservationWriter<SupportState>,
    terrain: ObservationHandle<SupportState>,
    mission_fence_writer: ObservationWriter<SupportState>,
    mission_fence: ObservationHandle<SupportState>,
    mission_rally_writer: ObservationWriter<SupportState>,
    mission_rally: ObservationHandle<SupportState>,
    ardupilot_writer: ObservationWriter<SupportState>,
    ardupilot: ObservationHandle<SupportState>,
    tracker: Mutex<SupportTracker>,
}

#[derive(Debug, Default)]
struct SupportTracker {
    vehicle_state: VehicleState,
    init_snapshot: InitSnapshot,
    command_int: Option<SupportState>,
    ftp: Option<SupportState>,
    terrain: Option<SupportState>,
    mission_fence: Option<SupportState>,
    mission_rally: Option<SupportState>,
    ardupilot: Option<SupportState>,
}

impl SupportDomain {
    pub(crate) fn new() -> Self {
        let (command_int_writer, command_int) = ObservationHandle::watch();
        let (ftp_writer, ftp) = ObservationHandle::watch();
        let (terrain_writer, terrain) = ObservationHandle::watch();
        let (mission_fence_writer, mission_fence) = ObservationHandle::watch();
        let (mission_rally_writer, mission_rally) = ObservationHandle::watch();
        let (ardupilot_writer, ardupilot) = ObservationHandle::watch();

        Self {
            inner: Arc::new(SupportDomainInner {
                command_int_writer,
                command_int,
                ftp_writer,
                ftp,
                terrain_writer,
                terrain,
                mission_fence_writer,
                mission_fence,
                mission_rally_writer,
                mission_rally,
                ardupilot_writer,
                ardupilot,
                tracker: Mutex::new(SupportTracker::default()),
            }),
        }
    }

    pub(crate) fn start(&self, stores: &StateChannels, init_manager: &InitManager) -> TaskHandle {
        let inner = self.inner.clone();
        let mut vehicle_state_rx = stores.vehicle_state.clone();
        let mut init_rx = init_manager.subscribe();

        inner.handle_vehicle_state(&vehicle_state_rx.borrow().clone());
        inner.handle_init_snapshot(&init_rx.borrow().clone());

        runtime::spawn(async move {
            loop {
                tokio::select! {
                    changed = vehicle_state_rx.changed() => {
                        if changed.is_err() {
                            break;
                        }
                        inner.handle_vehicle_state(&vehicle_state_rx.borrow_and_update().clone());
                    }
                    changed = init_rx.changed() => {
                        if changed.is_err() {
                            break;
                        }
                        inner.handle_init_snapshot(&init_rx.borrow_and_update().clone());
                    }
                }
            }
        })
    }

    pub(crate) fn command_int(&self) -> ObservationHandle<SupportState> {
        self.inner.command_int.clone()
    }

    pub(crate) fn ftp(&self) -> ObservationHandle<SupportState> {
        self.inner.ftp.clone()
    }

    pub(crate) fn terrain(&self) -> ObservationHandle<SupportState> {
        self.inner.terrain.clone()
    }

    pub(crate) fn mission_fence(&self) -> ObservationHandle<SupportState> {
        self.inner.mission_fence.clone()
    }

    pub(crate) fn mission_rally(&self) -> ObservationHandle<SupportState> {
        self.inner.mission_rally.clone()
    }

    pub(crate) fn ardupilot(&self) -> ObservationHandle<SupportState> {
        self.inner.ardupilot.clone()
    }

    pub(crate) fn close(&self) {
        self.inner.command_int_writer.close();
        self.inner.ftp_writer.close();
        self.inner.terrain_writer.close();
        self.inner.mission_fence_writer.close();
        self.inner.mission_rally_writer.close();
        self.inner.ardupilot_writer.close();
    }

    pub(crate) fn seed_from_vehicle_state(&self, state: &VehicleState) {
        self.inner.handle_vehicle_state(state);
    }

    #[cfg(test)]
    fn handle_vehicle_state(&self, state: &VehicleState) {
        self.inner.handle_vehicle_state(state);
    }

    #[cfg(test)]
    fn handle_init_snapshot(&self, snapshot: &InitSnapshot) {
        self.inner.handle_init_snapshot(snapshot);
    }
}

impl SupportDomainInner {
    fn handle_vehicle_state(&self, vehicle_state: &VehicleState) {
        self.update_tracker(|tracker| {
            tracker.vehicle_state = vehicle_state.clone();
        });
    }

    fn handle_init_snapshot(&self, init_snapshot: &InitSnapshot) {
        self.update_tracker(|tracker| {
            tracker.init_snapshot = init_snapshot.clone();
        });
    }

    fn update_tracker(&self, edit: impl FnOnce(&mut SupportTracker)) {
        let mut tracker = recover_lock(&self.tracker);
        edit(&mut tracker);
        self.recompute(&mut tracker);
    }

    fn recompute(&self, tracker: &mut SupportTracker) {
        publish_if_changed(
            &mut tracker.command_int,
            support_from_capability(
                &tracker.init_snapshot.autopilot_version,
                MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_COMMAND_INT,
            ),
            &self.command_int_writer,
        );
        publish_if_changed(
            &mut tracker.ftp,
            support_from_capability(
                &tracker.init_snapshot.autopilot_version,
                MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_FTP,
            ),
            &self.ftp_writer,
        );
        publish_if_changed(
            &mut tracker.terrain,
            support_from_capability(
                &tracker.init_snapshot.autopilot_version,
                MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_TERRAIN,
            ),
            &self.terrain_writer,
        );
        publish_if_changed(
            &mut tracker.mission_fence,
            support_from_capability(
                &tracker.init_snapshot.autopilot_version,
                MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_MISSION_FENCE,
            ),
            &self.mission_fence_writer,
        );
        publish_if_changed(
            &mut tracker.mission_rally,
            support_from_capability(
                &tracker.init_snapshot.autopilot_version,
                MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_MISSION_RALLY,
            ),
            &self.mission_rally_writer,
        );
        publish_if_changed(
            &mut tracker.ardupilot,
            ardupilot_support(&tracker.vehicle_state),
            &self.ardupilot_writer,
        );
    }
}

fn publish_if_changed(
    slot: &mut Option<SupportState>,
    next: Option<SupportState>,
    writer: &ObservationWriter<SupportState>,
) {
    let Some(next) = next else {
        return;
    };

    if slot.as_ref() == Some(&next) {
        return;
    }

    *slot = Some(next);
    let _ = writer.publish(next);
}

fn support_from_capability(
    state: &InitState<dialect::AUTOPILOT_VERSION_DATA>,
    capability: MavProtocolCapability,
) -> Option<SupportState> {
    match state {
        InitState::Available(version) if version.capabilities.contains(capability) => {
            Some(SupportState::Supported)
        }
        InitState::Available(_) => Some(SupportState::Unsupported),
        InitState::Unavailable { .. } => Some(SupportState::Unknown),
        InitState::Unknown | InitState::Requesting { .. } => None,
    }
}

fn ardupilot_support(vehicle_state: &VehicleState) -> Option<SupportState> {
    if !vehicle_state.heartbeat_received {
        return None;
    }

    Some(match vehicle_state.autopilot {
        AutopilotType::Unknown => SupportState::Unknown,
        AutopilotType::ArduPilotMega => SupportState::Supported,
        _ => SupportState::Unsupported,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dialect::{self, MavProtocolCapability};
    use crate::event_loop::{InitState, InitUnavailableReason};
    use crate::state::AutopilotType;
    use std::time::Duration;

    fn vehicle_state(autopilot: AutopilotType) -> VehicleState {
        VehicleState {
            heartbeat_received: true,
            autopilot,
            ..VehicleState::default()
        }
    }

    #[tokio::test]
    async fn support_observations_remain_unpublished_until_evidence_exists() {
        let domain = SupportDomain::new();
        let mut command_int = domain.command_int().subscribe();
        let mut ardupilot = domain.ardupilot().subscribe();

        assert_eq!(domain.command_int().latest(), None);
        assert_eq!(domain.ardupilot().latest(), None);

        domain.handle_vehicle_state(&VehicleState::default());
        domain.handle_init_snapshot(&InitSnapshot::default());

        assert!(
            tokio::time::timeout(Duration::from_millis(100), command_int.recv())
                .await
                .is_err(),
            "expected no command_int publish without AUTOPILOT_VERSION evidence"
        );
        assert!(
            tokio::time::timeout(Duration::from_millis(100), ardupilot.recv())
                .await
                .is_err(),
            "expected no ardupilot publish before heartbeat evidence"
        );
    }

    #[test]
    fn capability_bits_promote_only_matching_families() {
        let domain = SupportDomain::new();

        domain.handle_init_snapshot(&InitSnapshot {
            autopilot_version: InitState::Available(dialect::AUTOPILOT_VERSION_DATA {
                capabilities: MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_COMMAND_INT
                    | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_FTP,
                ..dialect::AUTOPILOT_VERSION_DATA::default()
            }),
            ..InitSnapshot::default()
        });

        assert_eq!(domain.command_int().latest(), Some(SupportState::Supported));
        assert_eq!(domain.ftp().latest(), Some(SupportState::Supported));
        assert_eq!(domain.terrain().latest(), Some(SupportState::Unsupported));
        assert_eq!(
            domain.mission_fence().latest(),
            Some(SupportState::Unsupported)
        );
        assert_eq!(
            domain.mission_rally().latest(),
            Some(SupportState::Unsupported)
        );
        assert_eq!(domain.ardupilot().latest(), None);
    }

    #[test]
    fn absent_capability_bit_becomes_unsupported_when_autopilot_version_is_known() {
        let domain = SupportDomain::new();

        domain.handle_init_snapshot(&InitSnapshot {
            autopilot_version: InitState::Available(dialect::AUTOPILOT_VERSION_DATA {
                capabilities: MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_FTP,
                ..dialect::AUTOPILOT_VERSION_DATA::default()
            }),
            ..InitSnapshot::default()
        });

        assert_eq!(
            domain.command_int().latest(),
            Some(SupportState::Unsupported)
        );
    }

    #[test]
    fn terminal_inconclusive_autopilot_version_yields_unknown() {
        let domain = SupportDomain::new();

        domain.handle_init_snapshot(&InitSnapshot {
            autopilot_version: InitState::Unavailable {
                reason: InitUnavailableReason::SilenceBudgetExhausted,
            },
            ..InitSnapshot::default()
        });

        assert_eq!(domain.command_int().latest(), Some(SupportState::Unknown));
        assert_eq!(domain.ftp().latest(), Some(SupportState::Unknown));
        assert_eq!(domain.terrain().latest(), Some(SupportState::Unknown));
        assert_eq!(domain.mission_fence().latest(), Some(SupportState::Unknown));
        assert_eq!(domain.mission_rally().latest(), Some(SupportState::Unknown));
    }

    #[test]
    fn ardupilot_support_remains_unpublished_before_heartbeat_then_resolves() {
        let domain = SupportDomain::new();

        domain.handle_vehicle_state(&VehicleState::default());
        assert_eq!(domain.ardupilot().latest(), None);

        domain.handle_vehicle_state(&vehicle_state(AutopilotType::ArduPilotMega));
        assert_eq!(domain.ardupilot().latest(), Some(SupportState::Supported));

        domain.handle_vehicle_state(&vehicle_state(AutopilotType::Px4));
        assert_eq!(domain.ardupilot().latest(), Some(SupportState::Unsupported));
    }
}
