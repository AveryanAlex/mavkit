use std::time::Duration;

use crate::error::VehicleError;
use crate::observation::{ObservationHandle, ObservationSubscription};
use crate::operation::OperationHandle;
use crate::types::MissionOperationProgress;
use tokio::sync::oneshot;
use tokio_util::sync::CancellationToken;

/// Handle for an in-flight mission-domain operation.
///
/// Cancellation is cooperative: calling [`cancel`](Self::cancel) or dropping
/// the handle signals the per-operation `CancellationToken` that is embedded
/// in the event-loop command. The handler selects on that token alongside the
/// vehicle-wide cancel, so it exits promptly.
pub struct MissionOperationHandle<T: Send + 'static> {
    core: OperationHandle<T, MissionOperationProgress>,
}

impl<T: Send + 'static> MissionOperationHandle<T> {
    pub(crate) fn new(
        progress: ObservationHandle<MissionOperationProgress>,
        result_rx: oneshot::Receiver<Result<T, VehicleError>>,
        cancel: CancellationToken,
    ) -> Self {
        Self {
            core: OperationHandle::new(progress, result_rx, cancel),
        }
    }

    pub fn cancel_token(&self) -> CancellationToken {
        self.core.cancel_token()
    }

    pub fn latest(&self) -> Option<MissionOperationProgress> {
        self.core.latest()
    }

    pub fn subscribe(&self) -> ObservationSubscription<MissionOperationProgress> {
        self.core.subscribe()
    }

    pub async fn wait(&self) -> Result<T, VehicleError> {
        self.core.wait().await
    }

    /// Like [`wait`](Self::wait), but returns [`VehicleError::Timeout`] if the
    /// operation does not complete within `timeout`.
    pub async fn wait_timeout(&self, timeout: Duration) -> Result<T, VehicleError> {
        self.core.wait_timeout(timeout, "operation wait").await
    }

    pub fn cancel(&self) {
        self.core.cancel();
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::observation::ObservationHandle;

    #[tokio::test]
    async fn mission_op_cancels_on_drop() {
        let cancel = CancellationToken::new();
        let (_writer, progress_handle) = ObservationHandle::<MissionOperationProgress>::watch();
        let (_tx, rx) = oneshot::channel::<Result<(), VehicleError>>();
        let handle = MissionOperationHandle::new(progress_handle, rx, cancel.clone());
        drop(handle);
        assert!(cancel.is_cancelled());
    }
}
