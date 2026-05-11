use crate::command::Command;
use crate::error::VehicleError;
use crate::mission::operations;
use crate::mission::transfer::{TransferDirection, TransferPhase, TransferProgress};
use crate::observation::{ObservationHandle, ObservationWriter};
use crate::operation::{run_with_progress_bridge, send_domain_command};
use crate::protocol_scope::OperationReservation;
use crate::runtime;
use crate::types::MissionOperationProgress;
use tokio::sync::{mpsc, oneshot};
use tokio_util::sync::CancellationToken;

pub(crate) fn map_transfer_progress(progress: TransferProgress) -> MissionOperationProgress {
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

    runtime::spawn(async move {
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

#[cfg(test)]
mod tests {
    use super::map_transfer_progress;
    use crate::mission::MissionType;
    use crate::mission::transfer::{TransferDirection, TransferPhase, TransferProgress};
    use crate::types::MissionOperationProgress;

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
}
