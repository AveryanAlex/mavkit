//! Reactive observation primitives shared across the crate.
//!
//! This root owns the public handle, writer, and stream types used for vehicle state,
//! telemetry, and discrete event delivery. The sibling files keep the low-level backing
//! stores and typed wrappers separate so the exported [`crate::ObservationHandle`],
//! [`crate::ObservationWriter`], and [`crate::ObservationSubscription`] paths stay stable.

mod backing;
mod subscription;
#[cfg(test)]
mod tests;
mod typed;

use crate::error::VehicleError;
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::{broadcast, watch};

use backing::{BroadcastStore, CloseNotifier, ObservationBacking, WatchStore};

pub use crate::types::SupportState;
pub use subscription::ObservationSubscription;
pub use typed::{MessageHandle, MessageSample, MetricHandle, MetricSample};

/// Read side of a reactive observation channel.
///
/// Observations are the primary mechanism for telemetry, vehicle state, and domain
/// updates. Each handle wraps one of two backing strategies:
///
/// - **Watch** — coalesces updates; [`latest()`] always returns the most recent value
///   (which may be stale if the vehicle is silent). Callers that need every change
///   should use [`subscribe()`] or [`wait()`] rather than polling [`latest()`].
///
/// - **Broadcast** — preserves every event; no coalescing occurs. [`latest()`]
///   returns the value of the last published event, but [`subscribe()`] only receives
///   events published *after* the subscription was created. Use this for status-text
///   messages and other discrete events where history matters.
///
/// [`wait()`] blocks until the first value is available (or returns immediately if one
/// already exists). It returns [`VehicleError::Disconnected`] when the writer is
/// dropped, signalling vehicle disconnection.
///
/// [`subscribe()`] returns an [`ObservationSubscription`] that implements `Stream<Item = T>`.
/// The stream terminates (yields `None`) when the writer is dropped.
///
/// [`latest()`]: ObservationHandle::latest
/// [`wait()`]: ObservationHandle::wait
/// [`subscribe()`]: ObservationHandle::subscribe
#[derive(Clone)]
pub struct ObservationHandle<T: Clone + Send + Sync + 'static> {
    backing: Arc<ObservationBacking<T>>,
    close_rx: watch::Receiver<bool>,
}

/// Generic observation writer paired with an [`ObservationHandle`].
#[derive(Clone)]
pub struct ObservationWriter<T: Clone + Send + Sync + 'static> {
    backing: Arc<ObservationBacking<T>>,
    close_notifier: Arc<CloseNotifier>,
}

impl<T: Clone + Send + Sync + 'static> ObservationHandle<T> {
    /// Create a coalescing (watch-backed) observation pair.
    ///
    /// Use for metrics that should always expose the most recent value — telemetry
    /// readings, vehicle state, etc. Only the latest published value is retained;
    /// rapid updates between reads are collapsed into the last one.
    pub fn watch() -> (ObservationWriter<T>, Self) {
        let (tx, _) = watch::channel(None::<T>);
        let (close_notifier, close_rx) = CloseNotifier::new();
        let backing = Arc::new(ObservationBacking::Watch(WatchStore { tx }));

        (
            ObservationWriter {
                backing: backing.clone(),
                close_notifier,
            },
            Self { backing, close_rx },
        )
    }

    /// Create a broadcast-backed observation pair.
    ///
    /// Use for discrete events where every occurrence matters — status-text messages,
    /// calibration reports, etc. All active subscriptions receive every event
    /// independently. `capacity` is the broadcast channel buffer size; slow receivers
    /// that fall behind by more than `capacity` items will skip the intermediate events.
    pub fn broadcast(capacity: usize) -> (ObservationWriter<T>, Self) {
        let (tx, _) = broadcast::channel(capacity.max(1));
        let (close_notifier, close_rx) = CloseNotifier::new();
        let backing = Arc::new(ObservationBacking::Broadcast(BroadcastStore::new(tx)));

        (
            ObservationWriter {
                backing: backing.clone(),
                close_notifier,
            },
            Self { backing, close_rx },
        )
    }

    pub fn latest(&self) -> Option<T> {
        self.backing.latest()
    }

    pub async fn wait(&self) -> Result<T, VehicleError> {
        match self.backing.as_ref() {
            ObservationBacking::Watch(store) => {
                let mut value_rx = store.tx.subscribe();
                let mut close_rx = self.close_rx.clone();

                if let Some(value) = Self::latest_if_open(&close_rx, || value_rx.borrow().clone())?
                {
                    return Ok(value);
                }

                loop {
                    tokio::select! {
                        changed = value_rx.changed() => {
                            changed.map_err(|_| VehicleError::Disconnected)?;
                            if let Some(value) = Self::latest_if_open(&close_rx, || value_rx.borrow_and_update().clone())? {
                                return Ok(value);
                            }
                        }
                        changed = close_rx.changed() => {
                            if changed.is_err() || *close_rx.borrow_and_update() {
                                return Err(VehicleError::Disconnected);
                            }
                        }
                    }
                }
            }
            ObservationBacking::Broadcast(store) => {
                let mut events_rx = store.tx.subscribe();
                let mut close_rx = self.close_rx.clone();

                if let Some(value) = Self::latest_if_open(&close_rx, || store.latest())? {
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
                        changed = close_rx.changed() => {
                            if changed.is_err() || *close_rx.borrow_and_update() {
                                return Err(VehicleError::Disconnected);
                            }
                        }
                    }
                }
            }
        }
    }

    pub async fn wait_timeout(&self, timeout: Duration) -> Result<T, VehicleError> {
        match tokio::time::timeout(timeout, self.wait()).await {
            Ok(result) => result,
            Err(_) => Err(VehicleError::Timeout("observation wait".into())),
        }
    }

    pub fn subscribe(&self) -> ObservationSubscription<T> {
        match self.backing.as_ref() {
            ObservationBacking::Watch(store) => {
                ObservationSubscription::watch(store.tx.subscribe(), self.close_rx.clone())
            }
            ObservationBacking::Broadcast(store) => {
                ObservationSubscription::broadcast(store.tx.subscribe(), self.close_rx.clone())
            }
        }
    }

    fn latest_if_open<F>(
        close_rx: &watch::Receiver<bool>,
        latest: F,
    ) -> Result<Option<T>, VehicleError>
    where
        F: FnOnce() -> Option<T>,
    {
        if *close_rx.borrow() {
            return Err(VehicleError::Disconnected);
        }

        let latest = latest();

        if *close_rx.borrow() {
            return Err(VehicleError::Disconnected);
        }

        Ok(latest)
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
                store.set_latest(Some(value.clone()));
                let _ = store.tx.send(value);
                Ok(())
            }
        }
    }

    pub(crate) fn clear(&self) {
        match self.backing.as_ref() {
            ObservationBacking::Watch(store) => {
                store.tx.send_replace(None);
            }
            ObservationBacking::Broadcast(store) => {
                store.set_latest(None);
            }
        }
    }

    pub(crate) fn close(&self) {
        self.close_notifier.close();
    }
}
