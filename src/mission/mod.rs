pub mod commands;
pub mod operations;
pub mod transfer;
pub mod types;
pub mod validation;
pub mod wire;

#[cfg(test)]
pub(crate) mod test_support;

pub use commands::{MissionCommand, RawMissionCommand};
pub use operations::{MissionClearOp, MissionDownloadOp, MissionUploadOp, MissionVerifyOp};
pub use transfer::{
    MissionTransferMachine, RetryPolicy, TransferDirection, TransferError, TransferEvent,
    TransferPhase, TransferProgress,
};
pub(crate) use types::WireMissionPlan;
pub use types::{
    HomePosition, IssueSeverity, MissionFrame, MissionIssue, MissionItem, MissionPlan,
    MissionState, MissionType,
};
pub use validation::{CompareTolerance, normalize_for_compare, plans_equivalent, validate_plan};
pub use wire::{mission_items_for_upload, mission_plan_from_download};

use crate::command::Command;
use crate::error::VehicleError;
use crate::observation::{ObservationHandle, ObservationSubscription, ObservationWriter};
use crate::operation::run_with_progress_bridge;
use crate::state::StateChannels;
use crate::stored_plan::StoredPlanDomain;
use crate::types::{MissionOperationKind, MissionOperationProgress};
use crate::vehicle::VehicleInner;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::sync::{mpsc, oneshot};
use tokio::task::JoinHandle;
use tokio_util::sync::CancellationToken;

#[derive(Clone)]
pub(crate) struct MissionDomain {
    inner: StoredPlanDomain<MissionState>,
}

#[derive(Clone)]
pub(crate) struct MissionProtocolScope {
    inner: Arc<MissionProtocolScopeInner>,
}

struct MissionProtocolScopeInner {
    active_operation: Mutex<Option<ActiveOperation>>,
    operation_id: AtomicU64,
}

struct ActiveOperation {
    id: u64,
    domain: &'static str,
    op_name: &'static str,
}

pub(crate) struct OperationReservation {
    pub(crate) id: u64,
    pub(crate) cancel: CancellationToken,
}

impl MissionProtocolScope {
    pub(crate) fn new() -> Self {
        Self {
            inner: Arc::new(MissionProtocolScopeInner {
                active_operation: Mutex::new(None),
                operation_id: AtomicU64::new(1),
            }),
        }
    }

    pub(crate) fn begin_operation(
        &self,
        domain: &'static str,
        op_name: &'static str,
    ) -> Result<OperationReservation, VehicleError> {
        let mut active = self.inner.active_operation.lock().unwrap();
        if let Some(conflict) = active.as_ref() {
            return Err(VehicleError::OperationConflict {
                conflicting_domain: conflict.domain.to_string(),
                conflicting_op: conflict.op_name.to_string(),
            });
        }

        let id = self.inner.operation_id.fetch_add(1, Ordering::Relaxed);
        let cancel = CancellationToken::new();
        *active = Some(ActiveOperation {
            id,
            domain,
            op_name,
        });

        Ok(OperationReservation { id, cancel })
    }

    pub(crate) fn finish_operation(&self, op_id: u64) {
        let mut active = self.inner.active_operation.lock().unwrap();
        if active
            .as_ref()
            .is_some_and(|active_op| active_op.id == op_id)
        {
            *active = None;
        }
    }
}

impl MissionDomain {
    pub(crate) fn new() -> Self {
        Self {
            inner: StoredPlanDomain::new(),
        }
    }

    pub(crate) fn start(
        &self,
        stores: &StateChannels,
        cancel: CancellationToken,
    ) -> JoinHandle<()> {
        self.update_current_index(stores.mission_state.borrow().current_seq);
        let mut mission_state_rx = stores.mission_state.clone();
        let domain = self.clone();

        tokio::spawn(async move {
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

    fn note_operation_error(&self) {
        self.inner.note_operation_error();
    }

    fn note_upload_success(&self, plan: MissionPlan) {
        self.inner.note_upload_success(plan);
    }

    fn note_download_success(&self, plan: MissionPlan) {
        self.inner.note_download_success(plan);
    }

    fn note_clear_success(&self) {
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

fn map_transfer_progress(progress: TransferProgress) -> MissionOperationProgress {
    match progress.phase {
        TransferPhase::Idle => MissionOperationProgress::RequestCount,
        TransferPhase::RequestCount => MissionOperationProgress::RequestCount,
        TransferPhase::TransferItems => match progress.direction {
            TransferDirection::Upload => MissionOperationProgress::SendingItem {
                current: progress.completed_items,
                total: progress.total_items,
            },
            TransferDirection::Download => MissionOperationProgress::ReceivingItem {
                current: progress.completed_items,
                total: progress.total_items,
            },
        },
        TransferPhase::AwaitAck => MissionOperationProgress::AwaitingAck,
        TransferPhase::Completed => MissionOperationProgress::Completed,
        TransferPhase::Failed => MissionOperationProgress::Failed,
        TransferPhase::Cancelled => MissionOperationProgress::Cancelled,
    }
}

pub(crate) async fn send_domain_command<T>(
    command_tx: mpsc::Sender<Command>,
    make: impl FnOnce(oneshot::Sender<Result<T, VehicleError>>) -> Command,
) -> Result<T, VehicleError> {
    let (tx, rx) = oneshot::channel();
    command_tx
        .send(make(tx))
        .await
        .map_err(|_| VehicleError::Disconnected)?;

    rx.await.map_err(|_| VehicleError::Disconnected)?
}

fn verify_plans_match(expected: &MissionPlan, observed: &MissionPlan) -> bool {
    let lhs = normalize_for_compare(expected);
    let rhs = normalize_for_compare(observed);
    plans_equivalent(&lhs, &rhs, CompareTolerance::default())
}

/// Run a domain operation with the shared progress-bridge + cancel + finalize lifecycle.
///
/// `make_command` receives the reply sender **and** the per-operation cancel token.
/// The token is embedded in the `Command` variant so the event-loop handler can
/// select on it alongside the vehicle-wide cancel.
pub(crate) fn run_domain_operation<C, T>(
    command_tx: mpsc::Sender<Command>,
    mission_progress_rx: tokio::sync::watch::Receiver<Option<TransferProgress>>,
    reservation: OperationReservation,
    make_command: impl FnOnce(oneshot::Sender<Result<C, VehicleError>>, CancellationToken) -> Command
    + Send
    + 'static,
    on_result: impl FnOnce(
        Result<C, VehicleError>,
        &ObservationWriter<MissionOperationProgress>,
    ) -> Result<T, VehicleError>
    + Send
    + 'static,
) -> operations::MissionOperationHandle<T>
where
    C: Send + 'static,
    T: Send + 'static,
{
    let (progress_writer, progress) = ObservationHandle::watch();
    let _ = progress_writer.publish(MissionOperationProgress::RequestCount);
    let (result_tx, result_rx) = oneshot::channel();

    let op =
        operations::MissionOperationHandle::new(progress, result_rx, reservation.cancel.clone());

    let cancel = reservation.cancel;

    tokio::spawn(async move {
        let command_result = run_with_progress_bridge(
            mission_progress_rx,
            progress_writer.clone(),
            cancel.clone(),
            map_transfer_progress,
            send_domain_command(command_tx, |reply| make_command(reply, cancel.clone())),
        )
        .await;

        let result = on_result(command_result, &progress_writer);

        let final_phase = match &result {
            Ok(_) => MissionOperationProgress::Completed,
            Err(VehicleError::Cancelled) => MissionOperationProgress::Cancelled,
            Err(_) => MissionOperationProgress::Failed,
        };
        let _ = progress_writer.publish(final_phase);

        let _ = result_tx.send(result);
    });

    op
}

/// Accessor for mission state and mission transfer operations.
///
/// Obtained from [`Vehicle::mission`](crate::Vehicle::mission). The handle is borrowed from the
/// vehicle and is valid until the owning borrow ends.
///
/// # Mission protocol exclusivity
///
/// ArduPilot's MAVLink mission protocol is single-threaded on the vehicle side — only one
/// transfer can run at a time across all domains (mission, fence, rally, params). Starting a
/// second transfer while one is active returns [`VehicleError::OperationConflict`] immediately.
pub struct MissionHandle<'a> {
    inner: &'a VehicleInner,
}

impl<'a> MissionHandle<'a> {
    pub(crate) fn new(inner: &'a VehicleInner) -> Self {
        Self { inner }
    }

    /// Returns the most recently observed mission state, or `None` if no update has arrived yet.
    pub fn latest(&self) -> Option<MissionState> {
        self.inner.mission.state().latest()
    }

    /// Waits for the next mission state update and returns it.
    ///
    /// Returns the default state if the vehicle disconnects before an update arrives.
    pub async fn wait(&self) -> MissionState {
        self.inner.mission.state().wait().await.unwrap_or_default()
    }

    /// Like [`wait`](Self::wait), but returns [`VehicleError::Timeout`] if no state update
    /// arrives within `timeout`.
    pub async fn wait_timeout(&self, timeout: Duration) -> Result<MissionState, VehicleError> {
        self.inner.mission.state().wait_timeout(timeout).await
    }

    /// Subscribes to an async stream of mission state updates.
    pub fn subscribe(&self) -> ObservationSubscription<MissionState> {
        self.inner.mission.state().subscribe()
    }

    /// Begins uploading a mission plan to the vehicle.
    ///
    /// The plan is validated before the transfer starts — any validation error with
    /// [`IssueSeverity::Error`] returns immediately as [`VehicleError::InvalidMissionItem`] or
    /// [`VehicleError::InvalidMissionPlan`].
    ///
    /// Returns a [`MissionUploadOp`] handle that can be awaited for the final result, observed for
    /// progress, or cancelled. The operation completes only when the vehicle acknowledges the full
    /// transfer. On success, the local plan cache is updated; on failure or cancellation it is
    /// marked as [`SyncState::PossiblyStale`](crate::types::SyncState::PossiblyStale).
    ///
    /// Returns [`VehicleError::OperationConflict`] immediately if a transfer is already active.
    pub fn upload(&self, plan: MissionPlan) -> Result<MissionUploadOp, VehicleError> {
        let issues = validate_plan(&plan);
        if let Some(issue) = issues.iter().find(|i| i.severity == IssueSeverity::Error) {
            return if let Some(seq) = issue.seq {
                Err(VehicleError::InvalidMissionItem {
                    index: usize::from(seq),
                    reason: crate::error::MissionValidationReason::Other(format!(
                        "{}: {}",
                        issue.code, issue.message
                    )),
                })
            } else {
                Err(VehicleError::InvalidMissionPlan(format!(
                    "{}: {}",
                    issue.code, issue.message
                )))
            };
        }

        let wire_plan = WireMissionPlan {
            mission_type: MissionType::Mission,
            items: plan.items.clone(),
        };

        let domain = self.inner.mission.clone();
        let reservation = domain.begin_operation(
            &self.inner.mission_protocol,
            MissionOperationKind::Upload,
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

    /// Begins downloading the vehicle's active mission plan.
    ///
    /// Returns a [`MissionDownloadOp`] handle. On success, the local plan cache is updated and
    /// the resolved `MissionPlan` is the operation result. On failure or cancellation the cache is
    /// marked as [`SyncState::PossiblyStale`](crate::types::SyncState::PossiblyStale).
    ///
    /// Returns [`VehicleError::OperationConflict`] immediately if a transfer is already active.
    pub fn download(&self) -> Result<MissionDownloadOp, VehicleError> {
        let domain = self.inner.mission.clone();
        let reservation = domain.begin_operation(
            &self.inner.mission_protocol,
            MissionOperationKind::Download,
            "download",
        )?;
        let protocol = self.inner.mission_protocol.clone();
        let op_id = reservation.id;

        Ok(run_domain_operation(
            self.inner.command_tx.clone(),
            self.inner.stores.mission_progress.clone(),
            reservation,
            |reply, cancel| Command::MissionDownload {
                mission_type: MissionType::Mission,
                reply,
                cancel,
            },
            move |result, _| {
                let r = match result {
                    Ok(wire_plan) => {
                        let plan = MissionPlan {
                            items: wire_plan.items,
                        };
                        domain.note_download_success(plan.clone());
                        Ok(plan)
                    }
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

    /// Clears the vehicle's mission storage.
    ///
    /// Returns a [`MissionClearOp`] handle. On success, the local plan cache is set to an empty
    /// plan and marked current. On failure or cancellation it is marked as
    /// [`SyncState::PossiblyStale`](crate::types::SyncState::PossiblyStale).
    ///
    /// Returns [`VehicleError::OperationConflict`] immediately if a transfer is already active.
    pub fn clear(&self) -> Result<MissionClearOp, VehicleError> {
        let domain = self.inner.mission.clone();
        let reservation = domain.begin_operation(
            &self.inner.mission_protocol,
            MissionOperationKind::Clear,
            "clear",
        )?;
        let protocol = self.inner.mission_protocol.clone();
        let op_id = reservation.id;

        Ok(run_domain_operation(
            self.inner.command_tx.clone(),
            self.inner.stores.mission_progress.clone(),
            reservation,
            |reply, cancel| Command::MissionClear {
                mission_type: MissionType::Mission,
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

    /// Downloads the vehicle's active mission and compares it to `plan`.
    ///
    /// The resolved `bool` is `true` if the downloaded plan is equivalent to the supplied one
    /// (within the default floating-point tolerance used by [`plans_equivalent`]), `false`
    /// otherwise.  The local cache is updated with the downloaded plan regardless of whether
    /// the comparison succeeds.
    ///
    /// Returns [`VehicleError::OperationConflict`] immediately if a transfer is already active.
    pub fn verify(&self, plan: MissionPlan) -> Result<MissionVerifyOp, VehicleError> {
        let domain = self.inner.mission.clone();
        let reservation = domain.begin_operation(
            &self.inner.mission_protocol,
            MissionOperationKind::Verify,
            "verify",
        )?;
        let protocol = self.inner.mission_protocol.clone();
        let op_id = reservation.id;

        Ok(run_domain_operation(
            self.inner.command_tx.clone(),
            self.inner.stores.mission_progress.clone(),
            reservation,
            |reply, cancel| Command::MissionDownload {
                mission_type: MissionType::Mission,
                reply,
                cancel,
            },
            move |result, pw| {
                let r = match result {
                    Ok(wire_plan) => {
                        let downloaded = MissionPlan {
                            items: wire_plan.items,
                        };
                        let _ = pw.publish(MissionOperationProgress::Verifying);
                        let matched = verify_plans_match(&plan, &downloaded);
                        domain.note_download_success(downloaded);
                        Ok(matched)
                    }
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

    /// Jumps to mission item `index` and waits until the vehicle reports that item as current.
    ///
    /// First sends `MAV_CMD_DO_SET_MISSION_CURRENT`, then polls the MAVLink `MISSION_CURRENT`
    /// stream until the vehicle echoes back the expected sequence number. Returns
    /// [`VehicleError::Timeout`] if the echo does not arrive within
    /// `VehicleConfig::command_completion_timeout`.
    pub async fn set_current(&self, index: u16) -> Result<(), VehicleError> {
        send_domain_command(self.inner.command_tx.clone(), |reply| {
            Command::MissionSetCurrent { seq: index, reply }
        })
        .await?;

        self.wait_for_current_index(index).await
    }

    async fn wait_for_current_index(&self, index: u16) -> Result<(), VehicleError> {
        let mut mission_state_rx = self.inner.stores.mission_state.clone();
        if mission_state_rx.borrow().current_seq == Some(index) {
            return Ok(());
        }

        let wait_for_match = async {
            loop {
                mission_state_rx
                    .changed()
                    .await
                    .map_err(|_| VehicleError::Disconnected)?;
                if mission_state_rx.borrow_and_update().current_seq == Some(index) {
                    return Ok(());
                }
            }
        };

        tokio::time::timeout(
            self.inner._config.command_completion_timeout,
            wait_for_match,
        )
        .await
        .map_err(|_| VehicleError::Timeout("mission domain command".into()))?
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn upload_progress() {
        let progress = TransferProgress {
            direction: TransferDirection::Upload,
            mission_type: MissionType::Mission,
            phase: TransferPhase::TransferItems,
            completed_items: 1,
            total_items: 3,
            retries_used: 0,
        };

        assert_eq!(
            map_transfer_progress(progress),
            MissionOperationProgress::SendingItem {
                current: 1,
                total: 3,
            }
        );
    }

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
