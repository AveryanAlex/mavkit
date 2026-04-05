use super::types::{ParamState, ParamStore, ParamWriteResult};
use crate::error::VehicleError;
use crate::observation::{ObservationHandle, ObservationWriter};
use crate::protocol_scope::{MissionProtocolScope, OperationReservation};
use crate::types::{ParamOperationKind, SyncState};
use std::sync::{Arc, Mutex};

#[derive(Clone)]
pub(crate) struct ParamsDomain {
    inner: Arc<ParamsDomainInner>,
}

struct ParamsDomainInner {
    state_writer: ObservationWriter<ParamState>,
    state: ObservationHandle<ParamState>,
    latest_state: Mutex<ParamState>,
}

impl ParamsDomain {
    pub(crate) fn new() -> Self {
        let (state_writer, state) = ObservationHandle::watch();
        let latest = ParamState::default();
        let _ = state_writer.publish(latest.clone());

        Self {
            inner: Arc::new(ParamsDomainInner {
                state_writer,
                state,
                latest_state: Mutex::new(latest),
            }),
        }
    }

    pub(crate) fn state(&self) -> ObservationHandle<ParamState> {
        self.inner.state.clone()
    }

    pub(crate) fn close(&self) {
        self.inner.state_writer.close();
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
        let mut latest = self.inner.latest_state.lock().unwrap();
        let mut next = latest.clone();
        edit(&mut next);
        if *latest != next {
            *latest = next.clone();
            let _ = self.inner.state_writer.publish(next);
        }
    }
}
