use std::time::Duration;

use crate::error::VehicleError;
use crate::observation::{ObservationHandle, ObservationSubscription};
use crate::types::ParamOperationProgress;
use tokio::sync::{Mutex, oneshot};
use tokio_util::sync::CancellationToken;

/// Handle for an in-flight parameter-domain operation.
pub struct ParamOperationHandle<T: Send + 'static> {
    progress: ObservationHandle<ParamOperationProgress>,
    result_rx: Mutex<Option<oneshot::Receiver<Result<T, VehicleError>>>>,
    cancel: CancellationToken,
}

impl<T: Send + 'static> ParamOperationHandle<T> {
    pub(crate) fn new(
        progress: ObservationHandle<ParamOperationProgress>,
        result_rx: oneshot::Receiver<Result<T, VehicleError>>,
        cancel: CancellationToken,
    ) -> Self {
        Self {
            progress,
            result_rx: Mutex::new(Some(result_rx)),
            cancel,
        }
    }

    pub fn latest(&self) -> Option<ParamOperationProgress> {
        self.progress.latest()
    }

    pub fn subscribe(&self) -> ObservationSubscription<ParamOperationProgress> {
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
            .map_err(|_| VehicleError::Timeout("param operation wait".into()))?
            .map_err(|_| VehicleError::Disconnected)?
    }

    pub fn cancel(&self) {
        self.cancel.cancel();
    }
}

impl<T: Send + 'static> Drop for ParamOperationHandle<T> {
    fn drop(&mut self) {
        self.cancel.cancel();
    }
}

/// Handle for a parameter download-all operation.
pub type ParamDownloadOp = ParamOperationHandle<crate::params::ParamStore>;
/// Handle for a parameter write-batch operation.
pub type ParamWriteBatchOp = ParamOperationHandle<Vec<crate::params::ParamWriteResult>>;
