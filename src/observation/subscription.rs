use std::pin::Pin;
use std::task::{Context, Poll};
use tokio::sync::{broadcast, watch};
use tokio_stream::wrappers::errors::BroadcastStreamRecvError;
use tokio_stream::wrappers::{BroadcastStream, WatchStream};
use tokio_stream::{Stream, StreamExt};

pub(super) enum SubscriptionBacking<T: Clone + Send + Sync + 'static> {
    Closed,
    Watch { stream: WatchStream<Option<T>> },
    Broadcast { stream: BroadcastStream<T> },
}

/// A `Stream<Item = T>` created from an [`crate::ObservationHandle`].
///
/// The stream terminates (yields `None`) when the originating [`crate::ObservationWriter`] is
/// dropped. This signals vehicle disconnection and lets callers break out of
/// `while let Some(value) = sub.recv().await` loops cleanly.
///
/// For watch-backed observations the stream replays the current value on creation and
/// then emits each subsequent change. For broadcast-backed observations only events
/// published *after* subscription creation are delivered; earlier events are not
/// replayed. If the channel is lagged (the receiver falls too far behind the sender)
/// intermediate events are silently dropped and the stream continues from the next
/// available item.
pub struct ObservationSubscription<T: Clone + Send + Sync + 'static> {
    pub(super) backing: SubscriptionBacking<T>,
    close_stream: WatchStream<bool>,
    disconnect_emitted: bool,
}

impl<T: Clone + Send + Sync + 'static> ObservationSubscription<T> {
    pub(super) fn watch(
        value_rx: watch::Receiver<Option<T>>,
        close_rx: watch::Receiver<bool>,
    ) -> Self {
        if *close_rx.borrow() {
            return Self::closed();
        }

        Self {
            backing: SubscriptionBacking::Watch {
                stream: WatchStream::new(value_rx),
            },
            close_stream: WatchStream::new(close_rx),
            disconnect_emitted: false,
        }
    }

    pub(super) fn broadcast(
        events_rx: broadcast::Receiver<T>,
        close_rx: watch::Receiver<bool>,
    ) -> Self {
        if *close_rx.borrow() {
            return Self::closed();
        }

        Self {
            backing: SubscriptionBacking::Broadcast {
                stream: BroadcastStream::new(events_rx),
            },
            close_stream: WatchStream::new(close_rx),
            disconnect_emitted: false,
        }
    }

    pub async fn recv(&mut self) -> Option<T> {
        self.next().await
    }

    fn closed() -> Self {
        let (_close_tx, close_rx) = watch::channel(true);

        Self {
            backing: SubscriptionBacking::Closed,
            close_stream: WatchStream::new(close_rx),
            disconnect_emitted: true,
        }
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
            Poll::Ready(Some(closed)) => {
                if closed {
                    this.disconnect_emitted = true;
                    return Poll::Ready(None);
                }
            }
            Poll::Pending => {}
        }

        match &mut this.backing {
            SubscriptionBacking::Closed => {
                this.disconnect_emitted = true;
                Poll::Ready(None)
            }
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
