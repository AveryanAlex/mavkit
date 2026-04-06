use std::time::Duration;

use crate::command::Command;
use crate::error::VehicleError;
use crate::mission::{MissionType, WireMissionPlan, run_domain_operation};
use crate::observation::{ObservationHandle, ObservationSubscription};
use crate::protocol_scope::{MissionProtocolScope, OperationReservation};
use crate::shared_state::SharedState;
use crate::types::{StoredPlanOperationKind, SupportState, SyncState};
use crate::vehicle::VehicleInner;
use std::sync::Arc;

pub(crate) trait StoredPlanState:
    Clone + Default + PartialEq + Send + Sync + 'static
{
    type Plan;
    type OperationKind;

    fn set_active_op(&mut self, kind: Option<Self::OperationKind>);
    fn set_plan(&mut self, plan: Self::Plan);
    fn set_sync(&mut self, sync: SyncState);
    fn cleared_plan() -> Self::Plan;
}

#[derive(Clone)]
pub(crate) struct StoredPlanDomain<S: StoredPlanState> {
    inner: Arc<StoredPlanDomainInner<S>>,
}

struct StoredPlanDomainInner<S: StoredPlanState> {
    state: SharedState<S>,
}

impl<S: StoredPlanState> StoredPlanDomain<S> {
    pub(crate) fn new() -> Self {
        Self {
            inner: Arc::new(StoredPlanDomainInner {
                state: SharedState::new(S::default()),
            }),
        }
    }

    pub(crate) fn state(&self) -> ObservationHandle<S> {
        self.inner.state.handle()
    }

    pub(crate) fn close(&self) {
        self.inner.state.close();
    }

    pub(crate) fn begin_operation(
        &self,
        scope: &MissionProtocolScope,
        domain: &'static str,
        kind: S::OperationKind,
        op_name: &'static str,
    ) -> Result<OperationReservation, VehicleError> {
        let reservation = scope.begin_operation(domain, op_name)?;
        self.update_state(|state| {
            state.set_active_op(Some(kind));
        });
        Ok(reservation)
    }

    pub(crate) fn finish_operation(&self, scope: &MissionProtocolScope, op_id: u64) {
        scope.finish_operation(op_id);
        self.update_state(|state| {
            state.set_active_op(None);
        });
    }

    pub(crate) fn note_operation_error(&self) {
        self.update_state(|state| {
            state.set_sync(SyncState::PossiblyStale);
        });
    }

    pub(crate) fn note_upload_success(&self, plan: S::Plan) {
        self.update_state(|state| {
            state.set_plan(plan);
            state.set_sync(SyncState::Current);
        });
    }

    pub(crate) fn note_download_success(&self, plan: S::Plan) {
        self.update_state(|state| {
            state.set_plan(plan);
            state.set_sync(SyncState::Current);
        });
    }

    pub(crate) fn note_clear_success(&self) {
        self.update_state(|state| {
            state.set_plan(S::cleared_plan());
            state.set_sync(SyncState::Current);
        });
    }

    pub(crate) fn update_state(&self, edit: impl FnOnce(&mut S)) {
        let _ = self.inner.state.update(edit);
    }
}

/// Domain-specific wiring that `StoredPlanHandle` needs but cannot infer from `StoredPlanState`.
///
/// Implement this on the state type (e.g. `FenceState`, `RallyState`) to get a fully-working
/// `StoredPlanHandle<S>` for free.  The handle's `upload`, `download`, `clear`, `support`,
/// and all observation accessors are derived entirely from this trait.
pub(crate) trait StoredPlanAccess:
    StoredPlanState<OperationKind = StoredPlanOperationKind>
where
    Self::Plan: Clone + Send + 'static,
{
    /// Human-readable domain name forwarded to `begin_operation` (e.g. `"fence"`, `"rally"`).
    const DOMAIN_NAME: &'static str;

    /// The MAVLink `MissionType` used for download and clear wire commands.
    const MISSION_TYPE: MissionType;

    /// Binds `S` to the correct domain field on `VehicleInner` (e.g. `inner.fence`
    /// for `FenceState`, `inner.rally` for `RallyState`).
    fn domain(inner: &VehicleInner) -> &StoredPlanDomain<Self>;

    /// Seeds support state from the current vehicle observation, then returns the
    /// domain-specific support handle.
    fn support(inner: &VehicleInner) -> ObservationHandle<SupportState>;

    /// Converts a typed plan into the wire format for upload.
    fn plan_to_wire(plan: &Self::Plan) -> Result<WireMissionPlan, VehicleError>;

    /// Converts a downloaded wire plan back into the typed plan.
    fn plan_from_wire(plan: WireMissionPlan) -> Result<Self::Plan, VehicleError>;
}

/// Generic handle shared by `FenceHandle` and `RallyHandle`.
///
/// All stored-plan domains (fence, rally) share identical observation + transfer semantics, so
/// the implementation lives here and the concrete handles forward to it.  This keeps the public
/// API surface concrete and stable while avoiding code duplication.
///
/// # Conflict model
///
/// The fence and rally domains share the same `MissionProtocolScope` as the mission domain.
/// ArduPilot's MAVLink mission protocol permits only one transfer at a time across all domains,
/// so starting a transfer while another is active (for any domain) returns
/// [`VehicleError::OperationConflict`] immediately.
pub(crate) struct StoredPlanHandle<'a, S: StoredPlanAccess>
where
    S::Plan: Clone + Send + 'static,
{
    inner: &'a VehicleInner,
    _marker: std::marker::PhantomData<S>,
}

impl<'a, S: StoredPlanAccess> StoredPlanHandle<'a, S>
where
    S::Plan: Clone + Send + 'static,
{
    pub(crate) fn new(inner: &'a VehicleInner) -> Self {
        Self {
            inner,
            _marker: std::marker::PhantomData,
        }
    }

    /// Returns the capability-support observation for this domain.
    ///
    /// Seeds derived state from the current vehicle identity before returning so the caller
    /// gets a non-stale initial value even before the event loop has had a chance to push
    /// an update.
    pub fn support(&self) -> ObservationHandle<SupportState> {
        S::support(self.inner)
    }

    /// Returns the most recently observed state, or `None` if no update has arrived yet.
    pub fn latest(&self) -> Option<S> {
        S::domain(self.inner).state().latest()
    }

    /// Returns the current cached state immediately when one is already available; otherwise waits
    /// for the first state observed after the call.
    ///
    /// Returns the default state if the vehicle disconnects before any state becomes available.
    pub async fn wait(&self) -> S {
        S::domain(self.inner)
            .state()
            .wait()
            .await
            .unwrap_or_default()
    }

    /// Like [`wait`](Self::wait), but returns the current cached state immediately when one is
    /// already available.
    ///
    /// If no state is cached yet, this waits for the first observed state and returns
    /// [`VehicleError::Timeout`] if it does not arrive within `timeout`. Returns
    /// [`VehicleError::Disconnected`] if the vehicle disconnects before any state becomes
    /// available.
    pub async fn wait_timeout(&self, timeout: Duration) -> Result<S, VehicleError> {
        S::domain(self.inner).state().wait_timeout(timeout).await
    }

    /// Subscribes to an async stream of state updates.
    pub fn subscribe(&self) -> ObservationSubscription<S> {
        S::domain(self.inner).state().subscribe()
    }

    /// Begins a plan upload.  Returns a handle that can be awaited for the operation result.
    ///
    /// Fails immediately if another mission-protocol operation is already active.
    pub fn upload(
        &self,
        plan: S::Plan,
    ) -> Result<crate::mission::operations::MissionOperationHandle<()>, VehicleError> {
        let wire_plan = S::plan_to_wire(&plan)?;
        let domain = S::domain(self.inner).clone();
        let reservation = domain.begin_operation(
            &self.inner.mission_protocol,
            S::DOMAIN_NAME,
            StoredPlanOperationKind::Upload,
            "upload",
        )?;
        let protocol = self.inner.mission_protocol.clone();
        let op_id = reservation.id;

        Ok(run_domain_operation(
            self.inner.command_tx.clone(),
            self.inner.stores.mission_progress.clone(),
            reservation,
            |reply, cancel| Command::MissionUpload {
                plan: wire_plan,
                reply,
                cancel,
            },
            move |result, _| {
                match &result {
                    Ok(()) => domain.note_upload_success(plan),
                    Err(_) => domain.note_operation_error(),
                }
                domain.finish_operation(&protocol, op_id);
                result
            },
        ))
    }

    /// Begins a plan download from the vehicle.  Returns a handle that resolves to the plan.
    ///
    /// Fails immediately if another mission-protocol operation is already active.
    pub fn download(
        &self,
    ) -> Result<crate::mission::operations::MissionOperationHandle<S::Plan>, VehicleError> {
        let domain = S::domain(self.inner).clone();
        let reservation = domain.begin_operation(
            &self.inner.mission_protocol,
            S::DOMAIN_NAME,
            StoredPlanOperationKind::Download,
            "download",
        )?;
        let protocol = self.inner.mission_protocol.clone();
        let op_id = reservation.id;

        Ok(run_domain_operation(
            self.inner.command_tx.clone(),
            self.inner.stores.mission_progress.clone(),
            reservation,
            |reply, cancel| Command::MissionDownload {
                mission_type: S::MISSION_TYPE,
                reply,
                cancel,
            },
            move |result, _| {
                let r = match result {
                    Ok(wire) => match S::plan_from_wire(wire) {
                        Ok(typed_plan) => {
                            let result = Ok(typed_plan.clone());
                            domain.note_download_success(typed_plan);
                            result
                        }
                        Err(err) => {
                            domain.note_operation_error();
                            Err(err)
                        }
                    },
                    Err(err) => {
                        domain.note_operation_error();
                        Err(err)
                    }
                };
                domain.finish_operation(&protocol, op_id);
                r
            },
        ))
    }

    /// Clears all stored items for this domain on the vehicle.
    ///
    /// Fails immediately if another mission-protocol operation is already active.
    pub fn clear(
        &self,
    ) -> Result<crate::mission::operations::MissionOperationHandle<()>, VehicleError> {
        let domain = S::domain(self.inner).clone();
        let reservation = domain.begin_operation(
            &self.inner.mission_protocol,
            S::DOMAIN_NAME,
            StoredPlanOperationKind::Clear,
            "clear",
        )?;
        let protocol = self.inner.mission_protocol.clone();
        let op_id = reservation.id;

        Ok(run_domain_operation(
            self.inner.command_tx.clone(),
            self.inner.stores.mission_progress.clone(),
            reservation,
            |reply, cancel| Command::MissionClear {
                mission_type: S::MISSION_TYPE,
                reply,
                cancel,
            },
            move |result, _| {
                match &result {
                    Ok(()) => domain.note_clear_success(),
                    Err(_) => domain.note_operation_error(),
                }
                domain.finish_operation(&protocol, op_id);
                result
            },
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    #[derive(Clone, Debug, Default, PartialEq)]
    struct TestState {
        plan: Option<Vec<String>>,
        sync: SyncState,
        active_op: Option<&'static str>,
    }

    impl StoredPlanState for TestState {
        type Plan = Vec<String>;
        type OperationKind = &'static str;

        fn set_active_op(&mut self, kind: Option<Self::OperationKind>) {
            self.active_op = kind;
        }

        fn set_plan(&mut self, plan: Self::Plan) {
            self.plan = Some(plan);
        }

        fn set_sync(&mut self, sync: SyncState) {
            self.sync = sync;
        }

        fn cleared_plan() -> Self::Plan {
            Vec::new()
        }
    }

    #[test]
    fn begin_operation_succeeds_and_sets_active_op() {
        let domain = StoredPlanDomain::<TestState>::new();
        let scope = MissionProtocolScope::new();

        let reservation = domain
            .begin_operation(&scope, "mission", "upload", "upload")
            .unwrap();

        let state = domain.state().latest().unwrap();
        assert_eq!(state.active_op, Some("upload"));

        domain.finish_operation(&scope, reservation.id);
    }

    #[test]
    fn finish_operation_clears_active_op() {
        let domain = StoredPlanDomain::<TestState>::new();
        let scope = MissionProtocolScope::new();

        let reservation = domain
            .begin_operation(&scope, "mission", "download", "download")
            .unwrap();
        domain.finish_operation(&scope, reservation.id);

        let state = domain.state().latest().unwrap();
        assert_eq!(state.active_op, None);
    }

    #[test]
    fn begin_operation_conflicts_when_scope_is_held_by_other_domain() {
        let scope = MissionProtocolScope::new();
        let mission_domain = StoredPlanDomain::<TestState>::new();
        let fence_domain = StoredPlanDomain::<TestState>::new();

        let reservation = mission_domain
            .begin_operation(&scope, "mission", "upload", "upload")
            .unwrap();

        let err = match fence_domain.begin_operation(&scope, "fence", "download", "download") {
            Ok(_) => panic!("expected operation conflict"),
            Err(err) => err,
        };

        assert!(matches!(
            err,
            VehicleError::OperationConflict {
                conflicting_domain,
                conflicting_op,
            } if conflicting_domain == "mission" && conflicting_op == "upload"
        ));

        mission_domain.finish_operation(&scope, reservation.id);
    }

    #[test]
    fn note_upload_success_sets_plan_and_sync_current() {
        let domain = StoredPlanDomain::<TestState>::new();

        let plan = vec!["A".to_string(), "B".to_string()];
        domain.note_upload_success(plan.clone());

        let state = domain.state().latest().unwrap();
        assert_eq!(state.plan, Some(plan));
        assert_eq!(state.sync, SyncState::Current);
    }

    #[test]
    fn note_download_success_sets_plan_and_sync_current() {
        let domain = StoredPlanDomain::<TestState>::new();

        let plan = vec!["D1".to_string()];
        domain.note_download_success(plan.clone());

        let state = domain.state().latest().unwrap();
        assert_eq!(state.plan, Some(plan));
        assert_eq!(state.sync, SyncState::Current);
    }

    #[test]
    fn note_clear_success_sets_cleared_plan_and_sync_current() {
        let domain = StoredPlanDomain::<TestState>::new();

        domain.note_upload_success(vec!["before-clear".to_string()]);
        domain.note_clear_success();

        let state = domain.state().latest().unwrap();
        assert_eq!(state.plan, Some(Vec::new()));
        assert_eq!(state.sync, SyncState::Current);
    }

    #[test]
    fn note_operation_error_sets_sync_possibly_stale() {
        let domain = StoredPlanDomain::<TestState>::new();

        domain.note_upload_success(vec!["fresh".to_string()]);
        domain.note_operation_error();

        let state = domain.state().latest().unwrap();
        assert_eq!(state.sync, SyncState::PossiblyStale);
    }

    #[tokio::test]
    async fn update_state_dedup_does_not_publish_when_unchanged() {
        let domain = StoredPlanDomain::<TestState>::new();
        let mut sub = domain.state().subscribe();

        let initial = tokio::time::timeout(Duration::from_millis(100), sub.recv())
            .await
            .unwrap()
            .unwrap();
        assert_eq!(initial, TestState::default());

        domain.update_state(|_state| {});

        let no_change = tokio::time::timeout(Duration::from_millis(100), sub.recv()).await;
        assert!(
            no_change.is_err(),
            "expected no publish for unchanged state"
        );
    }

    #[tokio::test]
    async fn update_state_publishes_when_state_changes() {
        let domain = StoredPlanDomain::<TestState>::new();
        let mut sub = domain.state().subscribe();

        let _ = tokio::time::timeout(Duration::from_millis(100), sub.recv())
            .await
            .unwrap()
            .unwrap();

        domain.update_state(|state| {
            state.sync = SyncState::Current;
        });

        let changed = tokio::time::timeout(Duration::from_millis(100), sub.recv())
            .await
            .unwrap()
            .unwrap();
        assert_eq!(changed.sync, SyncState::Current);
    }
}
