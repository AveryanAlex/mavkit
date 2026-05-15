use crate::error::VehicleError;
use crate::shared_state::recover_lock;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use tokio_util::sync::CancellationToken;

#[derive(Clone)]
pub(crate) struct MissionProtocolScope {
    inner: Arc<MissionProtocolScopeInner>,
}

struct MissionProtocolScopeInner {
    vehicle_cancel: CancellationToken,
    active_operation: Mutex<Option<ActiveOperation>>,
    operation_id: AtomicU64,
}

struct ActiveOperation {
    id: u64,
    domain: &'static str,
    op_name: &'static str,
}

#[derive(Debug)]
pub(crate) struct OperationReservation {
    pub(crate) id: u64,
    pub(crate) cancel: CancellationToken,
}

impl MissionProtocolScope {
    pub(crate) fn new(vehicle_cancel: CancellationToken) -> Self {
        Self {
            inner: Arc::new(MissionProtocolScopeInner {
                vehicle_cancel,
                active_operation: Mutex::new(None),
                operation_id: AtomicU64::new(1),
            }),
        }
    }

    #[cfg(test)]
    pub(crate) fn detached_for_test() -> Self {
        Self::new(CancellationToken::new())
    }

    pub(crate) fn begin_operation(
        &self,
        domain: &'static str,
        op_name: &'static str,
    ) -> Result<OperationReservation, VehicleError> {
        let mut active = recover_lock(&self.inner.active_operation);
        if let Some(conflict) = active.as_ref() {
            return Err(VehicleError::OperationConflict {
                conflicting_domain: conflict.domain.to_string(),
                conflicting_op: conflict.op_name.to_string(),
            });
        }

        let id = self.inner.operation_id.fetch_add(1, Ordering::Relaxed);
        let cancel = self.inner.vehicle_cancel.child_token();
        *active = Some(ActiveOperation {
            id,
            domain,
            op_name,
        });

        Ok(OperationReservation { id, cancel })
    }

    pub(crate) fn finish_operation(&self, op_id: u64) -> bool {
        let mut active = recover_lock(&self.inner.active_operation);
        if active
            .as_ref()
            .is_some_and(|active_op| active_op.id == op_id)
        {
            *active = None;
            return true;
        }

        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn finish_operation_clears_only_matching_reservation() {
        let scope = MissionProtocolScope::detached_for_test();
        let reservation = scope
            .begin_operation("mission", "upload")
            .expect("first reservation should succeed");

        let conflict = scope
            .begin_operation("params", "download_all")
            .expect_err("second reservation should conflict while mission upload is active");
        assert!(matches!(
            conflict,
            VehicleError::OperationConflict {
                conflicting_domain,
                conflicting_op,
            } if conflicting_domain == "mission" && conflicting_op == "upload"
        ));

        assert!(!scope.finish_operation(reservation.id + 1));

        let stale_id_conflict = scope
            .begin_operation("params", "download_all")
            .expect_err("stale finish should not clear the active mission reservation");
        assert!(matches!(
            stale_id_conflict,
            VehicleError::OperationConflict {
                conflicting_domain,
                conflicting_op,
            } if conflicting_domain == "mission" && conflicting_op == "upload"
        ));

        assert!(scope.finish_operation(reservation.id));

        let next = scope
            .begin_operation("params", "download_all")
            .expect("matching finish should clear the reservation");
        assert_eq!(next.id, reservation.id + 1);
    }

    #[test]
    fn vehicle_cancellation_cancels_active_operation_token() {
        let vehicle_cancel = CancellationToken::new();
        let scope = MissionProtocolScope::new(vehicle_cancel.clone());
        let reservation = scope
            .begin_operation("mission", "download")
            .expect("operation should start");

        assert!(!reservation.cancel.is_cancelled());

        vehicle_cancel.cancel();

        assert!(reservation.cancel.is_cancelled());
    }
}
