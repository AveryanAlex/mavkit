//! Small crate-private state caches built on top of observations.
//!
//! Domains that need a deduped last-known snapshot keep it here and publish changes through the
//! paired observation handles, reusing the crate's poison-recovery policy instead of
//! reimplementing lock handling at each call site.

use crate::observation::{ObservationHandle, ObservationWriter};
use std::sync::{Mutex, MutexGuard};

/// Recovers a mutex guard even when the lock was poisoned by a prior panic.
///
/// Internal state trackers and caches should keep making progress after test or task panics, so
/// the explicit lock policy across shared-core domains is to recover with `into_inner()` instead
/// of propagating panics through unrelated observation paths.
pub(crate) fn recover_lock<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    match mutex.lock() {
        Ok(guard) => guard,
        Err(poisoned) => poisoned.into_inner(),
    }
}

/// Small crate-private helper for watch-backed cached domain state.
///
/// Poisoned mutex state is recovered with `into_inner()` instead of panicking: this cache is a
/// last-known snapshot used to dedupe publishes, so future observation updates should keep making
/// progress even if a previous updater panicked while holding the lock.
pub(crate) struct SharedState<T>
where
    T: Clone + PartialEq + Send + Sync + 'static,
{
    writer: ObservationWriter<T>,
    handle: ObservationHandle<T>,
    latest: Mutex<T>,
}

impl<T> SharedState<T>
where
    T: Clone + PartialEq + Send + Sync + 'static,
{
    pub(crate) fn new(initial: T) -> Self {
        let (writer, handle) = ObservationHandle::watch();
        let _ = writer.publish(initial.clone());

        Self {
            writer,
            handle,
            latest: Mutex::new(initial),
        }
    }

    pub(crate) fn handle(&self) -> ObservationHandle<T> {
        self.handle.clone()
    }

    pub(crate) fn close(&self) {
        self.writer.close();
    }

    pub(crate) fn update(&self, edit: impl FnOnce(&mut T)) -> bool {
        let mut latest = self.latest_guard();
        let mut next = latest.clone();
        edit(&mut next);

        if *latest == next {
            return false;
        }

        *latest = next.clone();
        let _ = self.writer.publish(next);
        true
    }

    fn latest_guard(&self) -> MutexGuard<'_, T> {
        recover_lock(&self.latest)
    }
}
