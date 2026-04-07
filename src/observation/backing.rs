//! Internal storage for observation handles and writers.
//!
//! Watch-backed observations keep the latest coalesced value in the Tokio watch channel.
//! Broadcast-backed observations pair the channel with a recovered mutex cache so callers can
//! read the most recent event immediately while subscriptions remain future-only.

use std::sync::{Arc, Mutex, MutexGuard};
use tokio::sync::{broadcast, watch};

pub(super) struct WatchStore<T: Clone + Send + Sync + 'static> {
    pub(super) tx: watch::Sender<Option<T>>,
}

pub(super) struct BroadcastStore<T: Clone + Send + Sync + 'static> {
    pub(super) tx: broadcast::Sender<T>,
    latest: Arc<Mutex<Option<T>>>,
}

impl<T: Clone + Send + Sync + 'static> BroadcastStore<T> {
    pub(super) fn new(tx: broadcast::Sender<T>) -> Self {
        Self {
            tx,
            latest: Arc::new(Mutex::new(None)),
        }
    }

    pub(super) fn latest(&self) -> Option<T> {
        self.latest_guard().clone()
    }

    pub(super) fn set_latest(&self, value: Option<T>) {
        *self.latest_guard() = value;
    }

    fn latest_guard(&self) -> MutexGuard<'_, Option<T>> {
        match self.latest.lock() {
            Ok(guard) => guard,
            Err(poisoned) => poisoned.into_inner(),
        }
    }
}

pub(super) enum ObservationBacking<T: Clone + Send + Sync + 'static> {
    Watch(WatchStore<T>),
    Broadcast(BroadcastStore<T>),
}

impl<T: Clone + Send + Sync + 'static> ObservationBacking<T> {
    pub(super) fn latest(&self) -> Option<T> {
        match self {
            Self::Watch(store) => store.tx.subscribe().borrow().clone(),
            Self::Broadcast(store) => store.latest(),
        }
    }
}

pub(super) struct CloseNotifier {
    close_tx: watch::Sender<bool>,
}

impl CloseNotifier {
    pub(super) fn new() -> (Arc<Self>, watch::Receiver<bool>) {
        let (close_tx, close_rx) = watch::channel(false);
        (Arc::new(Self { close_tx }), close_rx)
    }

    pub(super) fn close(&self) {
        let _ = self.close_tx.send(true);
    }
}

impl Drop for CloseNotifier {
    fn drop(&mut self) {
        let _ = self.close_tx.send(true);
    }
}
