use std::time::Duration;

use crate::error::VehicleError;
use crate::observation::{ObservationHandle, ObservationSubscription};
use crate::operation::OperationHandle;
use crate::types::ParamOperationProgress;
use tokio::sync::oneshot;
use tokio_util::sync::CancellationToken;

/// Handle for an in-flight parameter-domain operation.
pub struct ParamOperationHandle<T: Send + 'static> {
    core: OperationHandle<T, ParamOperationProgress>,
}

impl<T: Send + 'static> ParamOperationHandle<T> {
    pub(crate) fn new(
        progress: ObservationHandle<ParamOperationProgress>,
        result_rx: oneshot::Receiver<Result<T, VehicleError>>,
        cancel: CancellationToken,
    ) -> Self {
        Self {
            core: OperationHandle::new(progress, result_rx, cancel),
        }
    }

    pub fn latest(&self) -> Option<ParamOperationProgress> {
        self.core.latest()
    }

    pub fn subscribe(&self) -> ObservationSubscription<ParamOperationProgress> {
        self.core.subscribe()
    }

    pub async fn wait(&self) -> Result<T, VehicleError> {
        self.core.wait().await
    }

    /// Like [`wait`](Self::wait), but returns [`VehicleError::Timeout`] if the
    /// operation does not complete within `timeout`.
    pub async fn wait_timeout(&self, timeout: Duration) -> Result<T, VehicleError> {
        self.core
            .wait_timeout(timeout, "param operation wait")
            .await
    }

    pub fn cancel(&self) {
        self.core.cancel();
    }
}

/// Handle for a parameter download-all operation.
pub type ParamDownloadOp = ParamOperationHandle<crate::params::ParamStore>;
/// Handle for a parameter write-batch operation.
pub type ParamWriteBatchOp = ParamOperationHandle<Vec<crate::params::ParamWriteResult>>;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::observation::ObservationHandle;

    #[tokio::test]
    async fn param_op_cancels_on_drop() {
        let cancel = CancellationToken::new();
        let (_writer, progress_handle) = ObservationHandle::<ParamOperationProgress>::watch();
        let (_tx, rx) = oneshot::channel::<Result<(), VehicleError>>();
        let handle = ParamOperationHandle::new(progress_handle, rx, cancel.clone());
        drop(handle);
        assert!(cancel.is_cancelled());
    }
}
