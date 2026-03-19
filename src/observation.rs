use crate::error::VehicleError;
use crate::telemetry::{TelemetryMessageKind, VehicleTimestamp};
use std::pin::Pin;
use std::sync::{Arc, Mutex};
use std::task::{Context, Poll};
use std::time::{Duration, Instant};
use tokio::sync::{broadcast, watch};
use tokio_stream::wrappers::errors::BroadcastStreamRecvError;
use tokio_stream::wrappers::{BroadcastStream, WatchStream};
use tokio_stream::{Stream, StreamExt};

#[derive(Clone, Debug, PartialEq, Eq)]
/// Observation-level support status used by metric and message handles.
pub enum SupportState {
    Unknown,
    Supported,
    Unsupported,
}

#[derive(Clone, Debug, PartialEq)]
/// Value plus provenance metadata for coalescing metric streams.
pub struct MetricSample<T> {
    pub value: T,
    pub source: TelemetryMessageKind,
    pub vehicle_time: Option<VehicleTimestamp>,
    pub received_at: Instant,
}

#[derive(Clone, Debug, PartialEq)]
/// Value plus timestamps for event-like telemetry messages.
pub struct MessageSample<M> {
    pub value: M,
    pub received_at: Instant,
    pub vehicle_time: Option<VehicleTimestamp>,
}

struct WatchStore<T: Clone + Send + Sync + 'static> {
    tx: watch::Sender<Option<T>>,
}

struct BroadcastStore<T: Clone + Send + Sync + 'static> {
    tx: broadcast::Sender<T>,
    latest: Arc<Mutex<Option<T>>>,
}

enum ObservationBacking<T: Clone + Send + Sync + 'static> {
    Watch(WatchStore<T>),
    Broadcast(BroadcastStore<T>),
}

#[derive(Clone)]
/// Generic observation reader over either watch-style or broadcast backing.
pub struct ObservationHandle<T: Clone + Send + Sync + 'static> {
    backing: Arc<ObservationBacking<T>>,
    close_rx: watch::Receiver<()>,
}

#[derive(Clone)]
/// Generic observation writer paired with an [`ObservationHandle`].
pub struct ObservationWriter<T: Clone + Send + Sync + 'static> {
    backing: Arc<ObservationBacking<T>>,
    _close_tx: Arc<watch::Sender<()>>,
}

enum SubscriptionBacking<T: Clone + Send + Sync + 'static> {
    Watch { stream: WatchStream<Option<T>> },
    Broadcast { stream: BroadcastStream<T> },
}

/// Async stream subscription created from an [`ObservationHandle`].
pub struct ObservationSubscription<T: Clone + Send + Sync + 'static> {
    backing: SubscriptionBacking<T>,
    close_stream: WatchStream<()>,
    disconnect_emitted: bool,
}

impl<T: Clone + Send + Sync + 'static> ObservationHandle<T> {
    pub fn watch() -> (ObservationWriter<T>, Self) {
        let (tx, _) = watch::channel(None::<T>);
        let (close_tx, close_rx) = watch::channel(());
        let backing = Arc::new(ObservationBacking::Watch(WatchStore { tx }));

        (
            ObservationWriter {
                backing: backing.clone(),
                _close_tx: Arc::new(close_tx),
            },
            Self { backing, close_rx },
        )
    }

    pub fn broadcast(capacity: usize) -> (ObservationWriter<T>, Self) {
        let (tx, _) = broadcast::channel(capacity.max(1));
        let (close_tx, close_rx) = watch::channel(());
        let backing = Arc::new(ObservationBacking::Broadcast(BroadcastStore {
            tx,
            latest: Arc::new(Mutex::new(None)),
        }));

        (
            ObservationWriter {
                backing: backing.clone(),
                _close_tx: Arc::new(close_tx),
            },
            Self { backing, close_rx },
        )
    }

    pub fn latest(&self) -> Option<T> {
        match self.backing.as_ref() {
            ObservationBacking::Watch(store) => store.tx.subscribe().borrow().clone(),
            ObservationBacking::Broadcast(store) => {
                store.latest.lock().ok().and_then(|v| v.clone())
            }
        }
    }

    pub async fn wait(&self) -> Result<T, VehicleError> {
        if let Some(value) = self.latest() {
            return Ok(value);
        }

        match self.backing.as_ref() {
            ObservationBacking::Watch(store) => {
                let mut value_rx = store.tx.subscribe();
                let mut close_rx = self.close_rx.clone();

                if let Some(value) = value_rx.borrow().clone() {
                    return Ok(value);
                }

                loop {
                    tokio::select! {
                        changed = value_rx.changed() => {
                            changed.map_err(|_| VehicleError::Disconnected)?;
                            if let Some(value) = value_rx.borrow_and_update().clone() {
                                return Ok(value);
                            }
                        }
                        _ = close_rx.changed() => {
                            return Err(VehicleError::Disconnected);
                        }
                    }
                }
            }
            ObservationBacking::Broadcast(store) => {
                let mut events_rx = store.tx.subscribe();
                let mut close_rx = self.close_rx.clone();

                if let Some(value) = self.latest() {
                    return Ok(value);
                }

                loop {
                    tokio::select! {
                        recv = events_rx.recv() => {
                            match recv {
                                Ok(value) => return Ok(value),
                                Err(broadcast::error::RecvError::Lagged(_)) => continue,
                                Err(broadcast::error::RecvError::Closed) => return Err(VehicleError::Disconnected),
                            }
                        }
                        _ = close_rx.changed() => {
                            return Err(VehicleError::Disconnected);
                        }
                    }
                }
            }
        }
    }

    pub async fn wait_timeout(&self, timeout: Duration) -> Result<T, VehicleError> {
        if let Some(value) = self.latest() {
            return Ok(value);
        }

        match tokio::time::timeout(timeout, self.wait()).await {
            Ok(result) => result,
            Err(_) => Err(VehicleError::Timeout("observation wait".into())),
        }
    }

    pub fn subscribe(&self) -> ObservationSubscription<T> {
        match self.backing.as_ref() {
            ObservationBacking::Watch(store) => {
                let value_rx = store.tx.subscribe();
                let close_stream = WatchStream::from_changes(self.close_rx.clone());
                ObservationSubscription {
                    backing: SubscriptionBacking::Watch {
                        stream: WatchStream::new(value_rx),
                    },
                    close_stream,
                    disconnect_emitted: false,
                }
            }
            ObservationBacking::Broadcast(store) => ObservationSubscription {
                backing: SubscriptionBacking::Broadcast {
                    stream: BroadcastStream::new(store.tx.subscribe()),
                },
                close_stream: WatchStream::from_changes(self.close_rx.clone()),
                disconnect_emitted: false,
            },
        }
    }
}

impl<T: Clone + Send + Sync + 'static> ObservationWriter<T> {
    pub fn publish(&self, value: T) -> Result<(), VehicleError> {
        match self.backing.as_ref() {
            ObservationBacking::Watch(store) => {
                store.tx.send_replace(Some(value));
                Ok(())
            }
            ObservationBacking::Broadcast(store) => {
                if let Ok(mut latest) = store.latest.lock() {
                    *latest = Some(value.clone());
                }
                let _ = store.tx.send(value);
                Ok(())
            }
        }
    }
}

impl<T: Clone + Send + Sync + 'static> ObservationSubscription<T> {
    pub async fn recv(&mut self) -> Option<T> {
        self.next().await
    }
}

impl<T: Clone + Send + Sync + 'static> Stream for ObservationSubscription<T> {
    type Item = T;

    fn poll_next(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
        let this = self.get_mut();

        if this.disconnect_emitted {
            return Poll::Ready(None);
        }

        match Pin::new(&mut this.close_stream).poll_next(cx) {
            Poll::Ready(None) => {
                this.disconnect_emitted = true;
                return Poll::Ready(None);
            }
            Poll::Ready(Some(_)) => {}
            Poll::Pending => {}
        }

        match &mut this.backing {
            SubscriptionBacking::Watch { stream } => loop {
                match Pin::new(&mut *stream).poll_next(cx) {
                    Poll::Ready(Some(Some(value))) => return Poll::Ready(Some(value)),
                    Poll::Ready(Some(None)) => continue,
                    Poll::Ready(None) => {
                        this.disconnect_emitted = true;
                        return Poll::Ready(None);
                    }
                    Poll::Pending => return Poll::Pending,
                }
            },
            SubscriptionBacking::Broadcast { stream } => loop {
                match Pin::new(&mut *stream).poll_next(cx) {
                    Poll::Ready(Some(Ok(value))) => return Poll::Ready(Some(value)),
                    Poll::Ready(Some(Err(BroadcastStreamRecvError::Lagged(_)))) => {
                        continue;
                    }
                    Poll::Ready(None) => {
                        this.disconnect_emitted = true;
                        return Poll::Ready(None);
                    }
                    Poll::Pending => return Poll::Pending,
                }
            },
        }
    }
}

#[derive(Clone)]
/// Typed metric handle with support metadata and wait/subscribe helpers.
pub struct MetricHandle<T: Clone + Send + Sync + 'static> {
    observation: ObservationHandle<MetricSample<T>>,
    support: ObservationHandle<SupportState>,
}

impl<T: Clone + Send + Sync + 'static> MetricHandle<T> {
    pub fn new(
        observation: ObservationHandle<MetricSample<T>>,
        support: ObservationHandle<SupportState>,
    ) -> Self {
        Self {
            observation,
            support,
        }
    }

    pub fn latest(&self) -> Option<MetricSample<T>> {
        self.observation.latest()
    }

    pub async fn wait(&self) -> Result<MetricSample<T>, VehicleError> {
        self.observation.wait().await
    }

    pub async fn wait_timeout(&self, timeout: Duration) -> Result<MetricSample<T>, VehicleError> {
        self.observation.wait_timeout(timeout).await
    }

    pub fn subscribe(&self) -> ObservationSubscription<MetricSample<T>> {
        self.observation.subscribe()
    }

    pub fn support(&self) -> ObservationHandle<SupportState> {
        self.support.clone()
    }
}

#[derive(Clone)]
/// Typed message handle with support metadata and request/stream helpers.
pub struct MessageHandle<M: Clone + Send + Sync + 'static> {
    observation: ObservationHandle<MessageSample<M>>,
    support: ObservationHandle<SupportState>,
}

impl<M: Clone + Send + Sync + 'static> MessageHandle<M> {
    pub fn new(
        observation: ObservationHandle<MessageSample<M>>,
        support: ObservationHandle<SupportState>,
    ) -> Self {
        Self {
            observation,
            support,
        }
    }

    pub fn latest(&self) -> Option<MessageSample<M>> {
        self.observation.latest()
    }

    pub async fn wait(&self) -> Result<MessageSample<M>, VehicleError> {
        self.observation.wait().await
    }

    pub async fn wait_timeout(&self, timeout: Duration) -> Result<MessageSample<M>, VehicleError> {
        self.observation.wait_timeout(timeout).await
    }

    pub fn subscribe(&self) -> ObservationSubscription<MessageSample<M>> {
        self.observation.subscribe()
    }

    pub fn support(&self) -> ObservationHandle<SupportState> {
        self.support.clone()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tokio_stream::Stream;

    fn watch_fixture() -> (ObservationWriter<u32>, ObservationHandle<u32>) {
        ObservationHandle::<u32>::watch()
    }

    fn broadcast_fixture(
        capacity: usize,
    ) -> (ObservationWriter<String>, ObservationHandle<String>) {
        ObservationHandle::<String>::broadcast(capacity)
    }

    #[tokio::test]
    async fn watch_lifecycle() {
        let (writer, handle) = watch_fixture();
        assert_eq!(handle.latest(), None);

        writer.publish(42).unwrap();
        assert_eq!(handle.latest(), Some(42));

        writer.publish(99).unwrap();
        assert_eq!(handle.latest(), Some(99));
    }

    #[tokio::test]
    async fn wait_disconnect() {
        let (writer, handle) = watch_fixture();
        let task = tokio::spawn(async move { handle.wait().await });

        tokio::time::sleep(Duration::from_millis(20)).await;
        drop(writer);

        let result = task.await.unwrap();
        assert!(matches!(result, Err(VehicleError::Disconnected)));
    }

    #[tokio::test]
    async fn wait_timeout() {
        let (_writer, handle) = watch_fixture();
        let result = handle.wait_timeout(Duration::from_millis(20)).await;
        assert!(matches!(result, Err(VehicleError::Timeout(_))));
    }

    #[tokio::test]
    async fn wait_returns_current_value_immediately() {
        let (writer, handle) = watch_fixture();
        writer.publish(7).unwrap();
        assert_eq!(handle.wait().await.unwrap(), 7);
    }

    #[tokio::test]
    async fn subscribe_watch_emits_current_then_updates() {
        let (writer, handle) = watch_fixture();
        writer.publish(10).unwrap();

        let mut sub = handle.subscribe();
        assert_eq!(sub.recv().await.unwrap(), 10);

        writer.publish(11).unwrap();
        assert_eq!(sub.recv().await.unwrap(), 11);
    }

    #[test]
    fn subscribe_exposes_stream_item_t() {
        fn assert_stream<S: Stream<Item = u32>>(_stream: S) {}

        let (_writer, handle) = watch_fixture();
        assert_stream(handle.subscribe());
    }

    #[tokio::test]
    async fn shared_store() {
        let (writer, handle_a) = watch_fixture();
        let handle_b = handle_a.clone();

        writer.publish(123).unwrap();

        assert_eq!(handle_a.latest(), Some(123));
        assert_eq!(handle_b.latest(), Some(123));
    }

    #[tokio::test]
    async fn broadcast_no_coalesce() {
        let (writer, handle) = broadcast_fixture(16);
        let mut sub = handle.subscribe();

        writer.publish("msg1".to_string()).unwrap();
        writer.publish("msg2".to_string()).unwrap();
        writer.publish("msg3".to_string()).unwrap();

        assert_eq!(sub.recv().await.unwrap(), "msg1");
        assert_eq!(sub.recv().await.unwrap(), "msg2");
        assert_eq!(sub.recv().await.unwrap(), "msg3");
    }

    #[tokio::test]
    async fn broadcast_subscribe_only_future_events() {
        let (writer, handle) = broadcast_fixture(16);

        writer.publish("before".to_string()).unwrap();
        let mut sub = handle.subscribe();
        writer.publish("after".to_string()).unwrap();

        assert_eq!(sub.recv().await.unwrap(), "after");
    }

    #[tokio::test]
    async fn broadcast_wait_returns_latest_immediately() {
        let (writer, handle) = broadcast_fixture(16);

        writer.publish("latest".to_string()).unwrap();
        assert_eq!(handle.wait().await.unwrap(), "latest");
    }

    #[tokio::test]
    async fn broadcast_lag_skips_to_retained_values() {
        let (writer, handle) = broadcast_fixture(2);
        let mut sub = handle.subscribe();

        writer.publish("m1".to_string()).unwrap();
        writer.publish("m2".to_string()).unwrap();
        writer.publish("m3".to_string()).unwrap();
        writer.publish("m4".to_string()).unwrap();

        assert_eq!(sub.recv().await.unwrap(), "m3");
        assert_eq!(sub.recv().await.unwrap(), "m4");
    }
}
