#![allow(dead_code)]
// This foundation module lands before the planned runtime call-site migration tasks.

use crate::time::Instant;
use std::future::Future;
use std::time::Duration;

#[cfg(target_arch = "wasm32")]
use futures::future::{AbortHandle, Abortable, Either, select};
#[cfg(target_arch = "wasm32")]
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct TimeoutError;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum TaskJoinError {
    Panic,
}

pub(crate) struct TaskHandle {
    #[cfg(not(target_arch = "wasm32"))]
    inner: tokio::task::JoinHandle<()>,
    #[cfg(target_arch = "wasm32")]
    abort_handle: AbortHandle,
    #[cfg(target_arch = "wasm32")]
    completion_rx: tokio::sync::oneshot::Receiver<()>,
    #[cfg(target_arch = "wasm32")]
    finished: Arc<AtomicBool>,
}

#[cfg(not(target_arch = "wasm32"))]
pub(crate) fn spawn<F>(future: F) -> TaskHandle
where
    F: Future<Output = ()> + Send + 'static,
{
    TaskHandle {
        inner: tokio::spawn(future),
    }
}

#[cfg(target_arch = "wasm32")]
pub(crate) fn spawn<F>(future: F) -> TaskHandle
where
    F: Future<Output = ()> + 'static,
{
    let (abort_handle, abort_registration) = AbortHandle::new_pair();
    let (completion_tx, completion_rx) = tokio::sync::oneshot::channel();
    let finished = Arc::new(AtomicBool::new(false));
    let finished_flag = Arc::clone(&finished);

    wasm_bindgen_futures::spawn_local(async move {
        let abortable = Abortable::new(future, abort_registration);
        let _ = abortable.await;
        finished_flag.store(true, Ordering::SeqCst);
        let _ = completion_tx.send(());
    });

    TaskHandle {
        abort_handle,
        completion_rx,
        finished,
    }
}

#[cfg(not(target_arch = "wasm32"))]
pub(crate) async fn sleep(duration: Duration) {
    tokio::time::sleep(duration).await;
}

#[cfg(target_arch = "wasm32")]
pub(crate) async fn sleep(duration: Duration) {
    let millis = duration.as_millis().min(u32::MAX as u128) as u32;
    gloo_timers::future::TimeoutFuture::new(millis).await;
}

#[cfg(not(target_arch = "wasm32"))]
pub(crate) async fn sleep_until(deadline: Instant) {
    tokio::time::sleep_until(deadline).await;
}

#[cfg(target_arch = "wasm32")]
pub(crate) async fn sleep_until(deadline: Instant) {
    let now = Instant::now();
    if deadline > now {
        sleep(deadline.duration_since(now)).await;
    }
}

#[cfg(not(target_arch = "wasm32"))]
pub(crate) async fn timeout<F, T>(duration: Duration, future: F) -> Result<T, TimeoutError>
where
    F: Future<Output = T>,
{
    tokio::time::timeout(duration, future)
        .await
        .map_err(|_| TimeoutError)
}

#[cfg(target_arch = "wasm32")]
pub(crate) async fn timeout<F, T>(duration: Duration, future: F) -> Result<T, TimeoutError>
where
    F: Future<Output = T>,
{
    let timeout_future = sleep(duration);
    futures::pin_mut!(future);
    futures::pin_mut!(timeout_future);

    match select(future, timeout_future).await {
        Either::Left((value, _)) => Ok(value),
        Either::Right(((), _)) => Err(TimeoutError),
    }
}

impl TaskHandle {
    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) fn abort(&self) {
        self.inner.abort();
    }

    #[cfg(target_arch = "wasm32")]
    pub(crate) fn abort(&self) {
        self.abort_handle.abort();
    }

    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) fn is_finished(&self) -> bool {
        self.inner.is_finished()
    }

    #[cfg(target_arch = "wasm32")]
    pub(crate) fn is_finished(&self) -> bool {
        self.finished.load(Ordering::SeqCst)
    }

    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) async fn join(self) -> Result<(), TaskJoinError> {
        match self.inner.await {
            Ok(()) => Ok(()),
            Err(err) if err.is_panic() => Err(TaskJoinError::Panic),
            Err(_) => Ok(()),
        }
    }

    #[cfg(target_arch = "wasm32")]
    pub(crate) async fn join(self) -> Result<(), TaskJoinError> {
        let _ = self.completion_rx.await;
        Ok(())
    }
}

pub(crate) struct TaskGroup {
    tasks: Vec<TaskHandle>,
}

impl TaskGroup {
    pub(crate) fn new() -> Self {
        Self { tasks: Vec::new() }
    }

    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) fn spawn<F>(&mut self, future: F)
    where
        F: Future<Output = ()> + Send + 'static,
    {
        self.tasks.push(spawn(future));
    }

    #[cfg(target_arch = "wasm32")]
    pub(crate) fn spawn<F>(&mut self, future: F)
    where
        F: Future<Output = ()> + 'static,
    {
        self.tasks.push(spawn(future));
    }

    pub(crate) async fn reap_finished(&mut self) -> Vec<TaskJoinError> {
        let mut errors = Vec::new();
        let mut index = 0;
        while index < self.tasks.len() {
            if self.tasks[index].is_finished() {
                let task = self.tasks.remove(index);
                if let Err(err) = task.join().await {
                    errors.push(err);
                }
            } else {
                index += 1;
            }
        }
        errors
    }

    pub(crate) fn abort_all(&self) {
        for task in &self.tasks {
            task.abort();
        }
    }

    pub(crate) async fn join_all(self) -> Vec<TaskJoinError> {
        let mut errors = Vec::new();
        for task in self.tasks {
            if let Err(err) = task.join().await {
                errors.push(err);
            }
        }
        errors
    }

    pub(crate) fn is_empty(&self) -> bool {
        self.tasks.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    };
    use std::time::Duration;
    use tokio::sync::oneshot;
    #[cfg(not(target_arch = "wasm32"))]
    use tokio::sync::{Barrier, Notify};

    #[tokio::test]
    async fn timeout_returns_error_when_sleep_wins() {
        let result = timeout(Duration::from_millis(1), async {
            sleep(Duration::from_millis(50)).await;
            42_u8
        })
        .await;

        assert_eq!(result, Err(TimeoutError));
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn native_instant_alias_matches_tokio_clock() {
        fn accept_tokio_instant(_instant: tokio::time::Instant) {}

        accept_tokio_instant(Instant::now());
    }

    #[tokio::test]
    async fn task_group_aborts_and_joins_retained_tasks() {
        let finished = Arc::new(AtomicBool::new(false));
        let marker = finished.clone();
        let (_tx, rx) = oneshot::channel::<()>();

        let mut group = TaskGroup::new();
        group.spawn(async move {
            let _ = rx.await;
            marker.store(true, Ordering::SeqCst);
        });

        group.abort_all();
        let errors = group.join_all().await;

        assert!(errors.is_empty());
        assert!(!finished.load(Ordering::SeqCst));
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[tokio::test]
    async fn task_group_preserves_concurrent_execution() {
        let barrier = Arc::new(Barrier::new(3));
        let completion = Arc::new(Notify::new());
        let completed = Arc::new(std::sync::atomic::AtomicUsize::new(0));

        let mut group = TaskGroup::new();
        for _ in 0..2 {
            let barrier = barrier.clone();
            let completion = completion.clone();
            let completed = completed.clone();
            group.spawn(async move {
                barrier.wait().await;
                if completed.fetch_add(1, Ordering::SeqCst) + 1 == 2 {
                    completion.notify_one();
                }
            });
        }

        barrier.wait().await;
        completion.notified().await;

        assert_eq!(completed.load(Ordering::SeqCst), 2);
        assert!(group.join_all().await.is_empty());
    }
}
