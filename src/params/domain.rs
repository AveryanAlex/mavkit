use super::types::{ParamState, ParamStore, ParamWriteResult};
use crate::error::VehicleError;
use crate::observation::ObservationHandle;
use crate::protocol_scope::{MissionProtocolScope, OperationReservation};
use crate::shared_state::SharedState;
use crate::types::{ParamOperationKind, SyncState};
use std::sync::Arc;

#[derive(Clone)]
pub(crate) struct ParamsDomain {
    inner: Arc<ParamsDomainInner>,
}

struct ParamsDomainInner {
    state: SharedState<ParamState>,
}

impl ParamsDomain {
    pub(crate) fn new() -> Self {
        Self {
            inner: Arc::new(ParamsDomainInner {
                state: SharedState::new(ParamState::default()),
            }),
        }
    }

    pub(crate) fn state(&self) -> ObservationHandle<ParamState> {
        self.inner.state.handle()
    }

    pub(crate) fn close(&self) {
        self.inner.state.close();
    }

    pub(crate) fn begin_operation(
        &self,
        scope: &MissionProtocolScope,
        kind: ParamOperationKind,
        op_name: &'static str,
    ) -> Result<OperationReservation, VehicleError> {
        let reservation = scope.begin_operation("params", op_name)?;
        self.update_state(|state| {
            state.active_op = Some(kind);
        });
        Ok(reservation)
    }

    pub(crate) fn finish_operation(&self, scope: &MissionProtocolScope, op_id: u64) {
        scope.finish_operation(op_id);
        self.update_state(|state| {
            state.active_op = None;
        });
    }

    pub(crate) fn note_download_success(&self, store: ParamStore) {
        self.update_state(|state| {
            state.store = Some(store);
            state.sync = SyncState::Current;
        });
    }

    pub(crate) fn note_write_result(&self, store: ParamStore, result: &ParamWriteResult) {
        self.update_state(|state| {
            state.store = Some(store);
            state.sync = if result.success {
                SyncState::Current
            } else {
                SyncState::PossiblyStale
            };
        });
    }

    pub(crate) fn note_batch_result(&self, store: ParamStore, results: &[ParamWriteResult]) {
        let all_ok = results.iter().all(|result| result.success);
        self.update_state(|state| {
            state.store = Some(store);
            state.sync = if all_ok {
                SyncState::Current
            } else {
                SyncState::PossiblyStale
            };
        });
    }

    pub(crate) fn note_operation_error(&self) {
        self.update_state(|state| {
            state.sync = SyncState::PossiblyStale;
        });
    }

    fn update_state(&self, edit: impl FnOnce(&mut ParamState)) {
        let _ = self.inner.state.update(edit);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;
    use std::time::Duration;

    fn sample_store() -> ParamStore {
        let mut params = HashMap::new();
        params.insert(
            "ARMING_CHECK".to_string(),
            super::super::types::Param {
                name: "ARMING_CHECK".to_string(),
                value: 1.0,
                param_type: super::super::types::ParamType::Int32,
                index: 0,
            },
        );

        ParamStore {
            params,
            expected_count: 1,
        }
    }

    fn write_result(
        name: &str,
        success: bool,
        requested_value: f32,
        confirmed_value: f32,
    ) -> ParamWriteResult {
        ParamWriteResult {
            name: name.to_string(),
            requested_value,
            confirmed_value,
            success,
        }
    }

    #[tokio::test]
    async fn update_state_dedup_does_not_publish_when_unchanged() {
        let domain = ParamsDomain::new();
        let mut sub = domain.state().subscribe();

        let initial = tokio::time::timeout(Duration::from_millis(100), sub.recv())
            .await
            .unwrap()
            .unwrap();
        assert_eq!(initial, ParamState::default());

        domain.update_state(|_state| {});

        let no_change = tokio::time::timeout(Duration::from_millis(100), sub.recv()).await;
        assert!(
            no_change.is_err(),
            "expected no publish for unchanged state"
        );
    }

    #[tokio::test]
    async fn note_batch_result_marks_possibly_stale_when_any_write_fails() {
        let domain = ParamsDomain::new();
        let mut sub = domain.state().subscribe();
        let store = sample_store();

        let initial = tokio::time::timeout(Duration::from_millis(100), sub.recv())
            .await
            .unwrap()
            .unwrap();
        assert_eq!(initial, ParamState::default());

        domain.note_download_success(store.clone());

        let downloaded = tokio::time::timeout(Duration::from_millis(100), sub.recv())
            .await
            .unwrap()
            .unwrap();
        assert_eq!(downloaded.store, Some(store.clone()));
        assert_eq!(downloaded.sync, SyncState::Current);

        let results = vec![
            write_result("ARMING_CHECK", true, 1.0, 1.0),
            write_result("ATC_RAT_PIT_P", false, 0.15, 0.12),
        ];
        domain.note_batch_result(store.clone(), &results);

        let stale = tokio::time::timeout(Duration::from_millis(100), sub.recv())
            .await
            .unwrap()
            .unwrap();
        assert_eq!(stale.store, Some(store));
        assert_eq!(stale.sync, SyncState::PossiblyStale);
    }
}
