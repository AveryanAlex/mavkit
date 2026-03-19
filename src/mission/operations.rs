use std::time::Duration;

use crate::command::Command;
use crate::error::VehicleError;
use crate::observation::{ObservationHandle, ObservationSubscription};
use crate::types::MissionOperationProgress;
use tokio::sync::{Mutex, mpsc, oneshot};
use tokio_util::sync::CancellationToken;

/// Handle for an in-flight mission-domain operation.
pub struct MissionOperationHandle<T: Send + 'static> {
    progress: ObservationHandle<MissionOperationProgress>,
    result_rx: Mutex<Option<oneshot::Receiver<Result<T, VehicleError>>>>,
    cancel: CancellationToken,
    command_tx: mpsc::Sender<Command>,
}

impl<T: Send + 'static> MissionOperationHandle<T> {
    pub(crate) fn new(
        progress: ObservationHandle<MissionOperationProgress>,
        result_rx: oneshot::Receiver<Result<T, VehicleError>>,
        cancel: CancellationToken,
        command_tx: mpsc::Sender<Command>,
    ) -> Self {
        Self {
            progress,
            result_rx: Mutex::new(Some(result_rx)),
            cancel,
            command_tx,
        }
    }

    pub fn latest(&self) -> Option<MissionOperationProgress> {
        self.progress.latest()
    }

    pub fn subscribe(&self) -> ObservationSubscription<MissionOperationProgress> {
        self.progress.subscribe()
    }

    pub async fn wait(&self) -> Result<T, VehicleError> {
        let receiver = {
            let mut guard = self.result_rx.lock().await;
            guard.take().ok_or_else(|| {
                VehicleError::Unsupported("operation result already consumed".to_string())
            })?
        };

        receiver.await.map_err(|_| VehicleError::Disconnected)?
    }

    /// Like [`wait`](Self::wait), but returns [`VehicleError::Timeout`] if the
    /// operation does not complete within `timeout`.
    pub async fn wait_timeout(&self, timeout: Duration) -> Result<T, VehicleError> {
        let receiver = {
            let mut guard = self.result_rx.lock().await;
            guard.take().ok_or_else(|| {
                VehicleError::Unsupported("operation result already consumed".to_string())
            })?
        };

        tokio::time::timeout(timeout, receiver)
            .await
            .map_err(|_| VehicleError::Timeout("operation wait".into()))?
            .map_err(|_| VehicleError::Disconnected)?
    }

    pub fn cancel(&self) {
        self.cancel.cancel();
        let _ = self.command_tx.try_send(Command::MissionCancelTransfer);
    }
}

/// Handle for a mission upload operation.
pub type MissionUploadOp = MissionOperationHandle<()>;
/// Handle for a mission download operation.
pub type MissionDownloadOp = MissionOperationHandle<crate::mission::MissionPlan>;
/// Handle for a mission clear operation.
pub type MissionClearOp = MissionOperationHandle<()>;
/// Handle for a mission verify operation.
pub type MissionVerifyOp = MissionOperationHandle<bool>;
