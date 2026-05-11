use super::types::{MissionPlan, MissionState};
use crate::error::VehicleError;
use crate::observation::ObservationHandle;
use crate::protocol_scope::{MissionProtocolScope, OperationReservation};
use crate::runtime::{self, TaskHandle};
use crate::state::StateChannels;
use crate::stored_plan::StoredPlanDomain;
use crate::types::MissionOperationKind;
use tokio_util::sync::CancellationToken;

#[derive(Clone)]
pub(crate) struct MissionDomain {
    inner: StoredPlanDomain<MissionState>,
}

impl MissionDomain {
    pub(crate) fn new() -> Self {
        Self {
            inner: StoredPlanDomain::new(),
        }
    }

    pub(crate) fn start(&self, stores: &StateChannels, cancel: CancellationToken) -> TaskHandle {
        self.update_current_index(stores.mission_state.borrow().current_seq);
        let mut mission_state_rx = stores.mission_state.clone();
        let domain = self.clone();

        runtime::spawn(async move {
            loop {
                tokio::select! {
                    _ = cancel.cancelled() => break,
                    changed = mission_state_rx.changed() => {
                        if changed.is_err() {
                            break;
                        }
                        let current = mission_state_rx.borrow_and_update().current_seq;
                        domain.update_current_index(current);
                    }
                }
            }
        })
    }

    pub(crate) fn state(&self) -> ObservationHandle<MissionState> {
        self.inner.state()
    }

    pub(crate) fn close(&self) {
        self.inner.close();
    }

    pub(crate) fn begin_operation(
        &self,
        scope: &MissionProtocolScope,
        kind: MissionOperationKind,
        op_name: &'static str,
    ) -> Result<OperationReservation, VehicleError> {
        self.inner.begin_operation(scope, "mission", kind, op_name)
    }

    pub(crate) fn finish_operation(&self, scope: &MissionProtocolScope, op_id: u64) {
        self.inner.finish_operation(scope, op_id);
    }

    pub(crate) fn note_operation_error(&self) {
        self.inner.note_operation_error();
    }

    pub(crate) fn note_upload_success(&self, plan: MissionPlan) {
        self.inner.note_upload_success(plan);
    }

    pub(crate) fn note_download_success(&self, plan: MissionPlan) {
        self.inner.note_download_success(plan);
    }

    pub(crate) fn note_clear_success(&self) {
        self.inner.note_clear_success();
    }

    fn update_current_index(&self, current_index: Option<u16>) {
        self.update_state(|state| {
            state.current_index = current_index;
        });
    }

    fn update_state(&self, edit: impl FnOnce(&mut MissionState)) {
        self.inner.update_state(edit);
    }
}

#[cfg(test)]
mod tests {
    use super::MissionDomain;
    use crate::error::VehicleError;
    use crate::protocol_scope::MissionProtocolScope;
    use crate::types::MissionOperationKind;

    #[test]
    fn operation_conflict() {
        let scope = MissionProtocolScope::new();
        let domain = MissionDomain::new();
        let first = domain
            .begin_operation(&scope, MissionOperationKind::Upload, "upload")
            .unwrap();
        let conflict =
            match domain.begin_operation(&scope, MissionOperationKind::Download, "download") {
                Ok(_) => panic!("expected operation conflict"),
                Err(err) => err,
            };

        assert!(matches!(
            conflict,
            VehicleError::OperationConflict {
                conflicting_domain,
                conflicting_op,
            } if conflicting_domain == "mission" && conflicting_op == "upload"
        ));

        domain.finish_operation(&scope, first.id);
    }
}
