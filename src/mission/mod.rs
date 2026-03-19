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
use crate::state::StateChannels;
use crate::stored_plan::StoredPlanDomain;
use crate::types::{MissionOperationKind, MissionOperationProgress};
use crate::vehicle::VehicleInner;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::sync::{mpsc, oneshot};
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

    pub(crate) fn start(&self, stores: &StateChannels, cancel: CancellationToken) {
        self.update_current_index(stores.mission_state.borrow().current_seq);
        let mut mission_state_rx = stores.mission_state.clone();
        let domain = self.clone();

        tokio::spawn(async move {
            let mission_state_task = tokio::spawn(async move {
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
            });

            if let Err(err) = mission_state_task.await
                && err.is_panic()
            {
                tracing::error!("mission current-seq watcher task panicked: {err}");
            }
        });
    }

    pub(crate) fn state(&self) -> ObservationHandle<MissionState> {
        self.inner.state()
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

pub(crate) fn spawn_transfer_progress_bridge(
    mut mission_progress_rx: tokio::sync::watch::Receiver<Option<TransferProgress>>,
    progress_writer: ObservationWriter<MissionOperationProgress>,
    stop: CancellationToken,
) -> tokio::task::JoinHandle<()> {
    tokio::spawn(async move {
        let _ = mission_progress_rx.borrow_and_update();

        loop {
            tokio::select! {
                _ = stop.cancelled() => break,
                changed = mission_progress_rx.changed() => {
                    if changed.is_err() {
                        break;
                    }
                    if let Some(progress) = mission_progress_rx.borrow_and_update().clone() {
                        let _ = progress_writer.publish(map_transfer_progress(progress));
                    }
                }
            }
        }
    })
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
/// The `make_command` closure builds the `Command` to send. The `on_result` closure
/// receives the command outcome and the progress writer, handles domain state updates
/// (note_*_success / note_operation_error / finish_operation), and returns the final
/// result to deliver to the caller.
pub(crate) fn run_domain_operation<C, T>(
    command_tx: mpsc::Sender<Command>,
    mission_progress_rx: tokio::sync::watch::Receiver<Option<TransferProgress>>,
    reservation: OperationReservation,
    make_command: impl FnOnce(oneshot::Sender<Result<C, VehicleError>>) -> Command + Send + 'static,
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

    let op = operations::MissionOperationHandle::new(
        progress,
        result_rx,
        reservation.cancel.clone(),
        command_tx.clone(),
    );

    let cancel = reservation.cancel;

    tokio::spawn(async move {
        let progress_stop = CancellationToken::new();
        let progress_task = spawn_transfer_progress_bridge(
            mission_progress_rx,
            progress_writer.clone(),
            progress_stop.clone(),
        );

        let command_result = tokio::select! {
            _ = cancel.cancelled() => Err(VehicleError::Cancelled),
            result = send_domain_command(command_tx, make_command) => result,
        };

        progress_stop.cancel();
        let _ = progress_task.await;

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
pub struct MissionHandle<'a> {
    inner: &'a VehicleInner,
}

impl<'a> MissionHandle<'a> {
    pub(crate) fn new(inner: &'a VehicleInner) -> Self {
        Self { inner }
    }

    pub fn latest(&self) -> Option<MissionState> {
        self.inner.mission.state().latest()
    }

    pub async fn wait(&self) -> MissionState {
        self.inner.mission.state().wait().await.unwrap_or_default()
    }

    pub async fn wait_timeout(&self, timeout: Duration) -> Result<MissionState, VehicleError> {
        self.inner.mission.state().wait_timeout(timeout).await
    }

    pub fn subscribe(&self) -> ObservationSubscription<MissionState> {
        self.inner.mission.state().subscribe()
    }

    pub fn upload(&self, plan: MissionPlan) -> Result<MissionUploadOp, VehicleError> {
        let issues = validate_plan(&plan);
        if let Some(issue) = issues.iter().find(|i| i.severity == IssueSeverity::Error) {
            return Err(VehicleError::MissionValidation(format!(
                "{}: {}",
                issue.code, issue.message
            )));
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
            |reply| Command::MissionUpload {
                plan: wire_plan,
                reply,
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
            |reply| Command::MissionDownload {
                mission_type: MissionType::Mission,
                reply,
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
            |reply| Command::MissionClear {
                mission_type: MissionType::Mission,
                reply,
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
            |reply| Command::MissionDownload {
                mission_type: MissionType::Mission,
                reply,
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
