use crate::error::VehicleError;
use crate::mission::{MissionProtocolScope, OperationReservation};
use crate::observation::{ObservationHandle, ObservationWriter};
use crate::types::SyncState;
use std::sync::{Arc, Mutex};

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
    state_writer: ObservationWriter<S>,
    state: ObservationHandle<S>,
    latest_state: Mutex<S>,
}

impl<S: StoredPlanState> StoredPlanDomain<S> {
    pub(crate) fn new() -> Self {
        let (state_writer, state) = ObservationHandle::watch();
        let latest = S::default();
        let _ = state_writer.publish(latest.clone());

        Self {
            inner: Arc::new(StoredPlanDomainInner {
                state_writer,
                state,
                latest_state: Mutex::new(latest),
            }),
        }
    }

    pub(crate) fn state(&self) -> ObservationHandle<S> {
        self.inner.state.clone()
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
        let mut latest = self.inner.latest_state.lock().unwrap();
        let mut next = latest.clone();
        edit(&mut next);
        if *latest != next {
            *latest = next.clone();
            let _ = self.inner.state_writer.publish(next);
        }
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
