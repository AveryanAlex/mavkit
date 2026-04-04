use std::future::Future;
use std::time::Duration;

use crate::error::VehicleError;
use crate::observation::{ObservationHandle, ObservationSubscription, ObservationWriter};
use tokio::sync::{Mutex, oneshot, watch};
use tokio::task::JoinHandle;
use tokio_util::sync::CancellationToken;

pub(crate) struct OperationHandle<T: Send + 'static, P: Clone + Send + Sync + 'static> {
    progress: ObservationHandle<P>,
    result_rx: Mutex<Option<oneshot::Receiver<Result<T, VehicleError>>>>,
    cancel: CancellationToken,
}

impl<T: Send + 'static, P: Clone + Send + Sync + 'static> OperationHandle<T, P> {
    pub(crate) fn new(
        progress: ObservationHandle<P>,
        result_rx: oneshot::Receiver<Result<T, VehicleError>>,
        cancel: CancellationToken,
    ) -> Self {
        Self {
            progress,
            result_rx: Mutex::new(Some(result_rx)),
            cancel,
        }
    }

    pub(crate) fn latest(&self) -> Option<P> {
        self.progress.latest()
    }

    pub(crate) fn subscribe(&self) -> ObservationSubscription<P> {
        self.progress.subscribe()
    }

    pub(crate) async fn wait(&self) -> Result<T, VehicleError> {
        let receiver = {
            let mut guard = self.result_rx.lock().await;
            guard.take().ok_or_else(|| {
                VehicleError::Unsupported("operation result already consumed".to_string())
            })?
        };

        receiver.await.map_err(|_| VehicleError::Disconnected)?
    }

    pub(crate) async fn wait_timeout(
        &self,
        timeout: Duration,
        timeout_context: &'static str,
    ) -> Result<T, VehicleError> {
        let receiver = {
            let mut guard = self.result_rx.lock().await;
            guard.take().ok_or_else(|| {
                VehicleError::Unsupported("operation result already consumed".to_string())
            })?
        };

        tokio::time::timeout(timeout, receiver)
            .await
            .map_err(|_| VehicleError::Timeout(timeout_context.into()))?
            .map_err(|_| VehicleError::Disconnected)?
    }

    pub(crate) fn cancel(&self) {
        self.cancel.cancel();
    }

    pub(crate) fn cancel_token(&self) -> CancellationToken {
        self.cancel.clone()
    }
}

impl<T: Send + 'static, P: Clone + Send + Sync + 'static> Drop for OperationHandle<T, P> {
    fn drop(&mut self) {
        self.cancel.cancel();
    }
}

pub(crate) fn spawn_watch_progress_bridge<S, P>(
    mut progress_rx: watch::Receiver<Option<S>>,
    progress_writer: ObservationWriter<P>,
    stop: CancellationToken,
    map: impl Fn(S) -> P + Send + 'static,
) -> JoinHandle<()>
where
    S: Clone + Send + Sync + 'static,
    P: Clone + Send + Sync + 'static,
{
    tokio::spawn(async move {
        let _ = progress_rx.borrow_and_update();

        loop {
            tokio::select! {
                _ = stop.cancelled() => break,
                changed = progress_rx.changed() => {
                    if changed.is_err() {
                        break;
                    }
                    if let Some(progress) = progress_rx.borrow_and_update().clone() {
                        let _ = progress_writer.publish(map(progress));
                    }
                }
            }
        }
    })
}

pub(crate) async fn run_with_progress_bridge<S, P, C, F>(
    progress_rx: watch::Receiver<Option<S>>,
    progress_writer: ObservationWriter<P>,
    cancel: CancellationToken,
    map_progress: impl Fn(S) -> P + Send + 'static,
    command: F,
) -> Result<C, VehicleError>
where
    S: Clone + Send + Sync + 'static,
    P: Clone + Send + Sync + 'static,
    C: Send + 'static,
    F: Future<Output = Result<C, VehicleError>>,
{
    let progress_stop = CancellationToken::new();
    let progress_task = spawn_watch_progress_bridge(
        progress_rx,
        progress_writer,
        progress_stop.clone(),
        map_progress,
    );

    let command_result = tokio::select! {
        _ = cancel.cancelled() => Err(VehicleError::Cancelled),
        result = command => result,
    };

    progress_stop.cancel();
    let _ = progress_task.await;

    command_result
}
