use crate::dialect::{self, MavProtocolCapability};
use crate::event_loop::{InitManager, InitSnapshot, InitState};
use crate::observation::{ObservationHandle, ObservationWriter};
use crate::shared_state::recover_lock;
use crate::state::{AutopilotType, StateChannels, VehicleState};
use crate::types::SupportState;
use crate::vehicle::VehicleInner;
use std::sync::Arc;
use std::sync::Mutex;
use tokio::task::JoinHandle;

/// Accessor for capability support observations derived from the vehicle's protocol capabilities.
///
/// Obtained from [`Vehicle::support`](crate::Vehicle::support). Capability flags are extracted
/// from the `AUTOPILOT_VERSION` message (`capabilities` bitmask). Each observation starts as
/// [`SupportState::Unknown`] and is updated once the init
/// sequence completes or the first heartbeat arrives.
///
/// All observation handles have their initial value seeded from the current vehicle state, so
/// callers get a non-stale result even before the async loop pushes an update.
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

impl SupportTracker {
    fn seeded_unknowns() -> Self {
        Self {
            command_int: Some(SupportState::Unknown),
            ftp: Some(SupportState::Unknown),
            terrain: Some(SupportState::Unknown),
            mission_fence: Some(SupportState::Unknown),
            mission_rally: Some(SupportState::Unknown),
            ardupilot: Some(SupportState::Unknown),
            ..Self::default()
        }
    }
}

impl SupportDomain {
    pub(crate) fn new() -> Self {
        let (command_int_writer, command_int) = ObservationHandle::watch();
        let (ftp_writer, ftp) = ObservationHandle::watch();
        let (terrain_writer, terrain) = ObservationHandle::watch();
        let (mission_fence_writer, mission_fence) = ObservationHandle::watch();
        let (mission_rally_writer, mission_rally) = ObservationHandle::watch();
        let (ardupilot_writer, ardupilot) = ObservationHandle::watch();

        let domain = Self {
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
                tracker: Mutex::new(SupportTracker::seeded_unknowns()),
            }),
        };

        domain.publish_unknowns();
        domain
    }

    pub(crate) fn start(
        &self,
        stores: &StateChannels,
        init_manager: &InitManager,
    ) -> JoinHandle<()> {
        let inner = self.inner.clone();
        let mut vehicle_state_rx = stores.vehicle_state.clone();
        let mut init_rx = init_manager.subscribe();

        inner.handle_vehicle_state(&vehicle_state_rx.borrow().clone());
        inner.handle_init_snapshot(&init_rx.borrow().clone());

        tokio::spawn(async move {
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

    fn publish_unknowns(&self) {
        let _ = self.inner.command_int_writer.publish(SupportState::Unknown);
        let _ = self.inner.ftp_writer.publish(SupportState::Unknown);
        let _ = self.inner.terrain_writer.publish(SupportState::Unknown);
        let _ = self
            .inner
            .mission_fence_writer
            .publish(SupportState::Unknown);
        let _ = self
            .inner
            .mission_rally_writer
            .publish(SupportState::Unknown);
        let _ = self.inner.ardupilot_writer.publish(SupportState::Unknown);
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
        let capabilities = capabilities_from_init_state(&tracker.init_snapshot.autopilot_version);

        publish_if_changed(
            &mut tracker.command_int,
            support_from_capability(
                capabilities,
                MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_COMMAND_INT,
            ),
            &self.command_int_writer,
        );
        publish_if_changed(
            &mut tracker.ftp,
            support_from_capability(
                capabilities,
                MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_FTP,
            ),
            &self.ftp_writer,
        );
        publish_if_changed(
            &mut tracker.terrain,
            support_from_capability(
                capabilities,
                MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_TERRAIN,
            ),
            &self.terrain_writer,
        );
        publish_if_changed(
            &mut tracker.mission_fence,
            support_from_capability(
                capabilities,
                MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_MISSION_FENCE,
            ),
            &self.mission_fence_writer,
        );
        publish_if_changed(
            &mut tracker.mission_rally,
            support_from_capability(
                capabilities,
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
    next: SupportState,
    writer: &ObservationWriter<SupportState>,
) {
    if slot.as_ref() == Some(&next) {
        return;
    }

    *slot = Some(next);
    let _ = writer.publish(next);
}

fn capabilities_from_init_state(
    state: &InitState<dialect::AUTOPILOT_VERSION_DATA>,
) -> Option<MavProtocolCapability> {
    match state {
        InitState::Available(version) => Some(version.capabilities),
        InitState::Unknown | InitState::Requesting { .. } | InitState::Unavailable { .. } => None,
    }
}

fn support_from_capability(
    capabilities: Option<MavProtocolCapability>,
    capability: MavProtocolCapability,
) -> SupportState {
    match capabilities {
        Some(bits) if bits.contains(capability) => SupportState::Supported,
        Some(_) | None => SupportState::Unknown,
    }
}

fn ardupilot_support(vehicle_state: &VehicleState) -> SupportState {
    if !vehicle_state.heartbeat_received {
        return SupportState::Unknown;
    }

    match vehicle_state.autopilot {
        AutopilotType::Unknown => SupportState::Unknown,
        AutopilotType::ArduPilotMega => SupportState::Supported,
        _ => SupportState::Unsupported,
    }
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
    async fn seeded_unknown_does_not_republish_when_inputs_still_unknown() {
        let domain = SupportDomain::new();
        let mut command_int = domain.command_int().subscribe();
        let mut ardupilot = domain.ardupilot().subscribe();

        assert_eq!(
            tokio::time::timeout(Duration::from_millis(100), command_int.recv())
                .await
                .expect("initial command_int state should be published")
                .expect("command_int subscription should remain open"),
            SupportState::Unknown,
        );
        assert_eq!(
            tokio::time::timeout(Duration::from_millis(100), ardupilot.recv())
                .await
                .expect("initial ardupilot state should be published")
                .expect("ardupilot subscription should remain open"),
            SupportState::Unknown,
        );

        domain.handle_vehicle_state(&VehicleState::default());
        domain.handle_init_snapshot(&InitSnapshot::default());

        assert!(
            tokio::time::timeout(Duration::from_millis(100), command_int.recv())
                .await
                .is_err(),
            "expected no duplicate Unknown publish for command_int"
        );
        assert!(
            tokio::time::timeout(Duration::from_millis(100), ardupilot.recv())
                .await
                .is_err(),
            "expected no duplicate Unknown publish for ardupilot"
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
        assert_eq!(domain.terrain().latest(), Some(SupportState::Unknown));
        assert_eq!(domain.mission_fence().latest(), Some(SupportState::Unknown));
        assert_eq!(domain.mission_rally().latest(), Some(SupportState::Unknown));
        assert_eq!(domain.ardupilot().latest(), Some(SupportState::Unknown));
    }

    #[test]
    fn bounded_autopilot_version_silence_keeps_capability_domains_unknown() {
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
    fn ardupilot_support_uses_identity_signal_not_capability_bits() {
        let domain = SupportDomain::new();

        domain.handle_vehicle_state(&vehicle_state(AutopilotType::ArduPilotMega));
        assert_eq!(domain.ardupilot().latest(), Some(SupportState::Supported));

        domain.handle_vehicle_state(&vehicle_state(AutopilotType::Px4));
        assert_eq!(domain.ardupilot().latest(), Some(SupportState::Unsupported));
    }
}
