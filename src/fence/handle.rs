use std::time::Duration;

use crate::error::VehicleError;
use crate::mission::operations::MissionOperationHandle;
use crate::observation::{ObservationHandle, ObservationSubscription};
use crate::stored_plan::StoredPlanHandle;
use crate::types::SupportState;
use crate::vehicle::VehicleInner;

use super::plan::FencePlan;
use super::state::FenceState;

/// Handle for a fence upload operation.
pub type FenceUploadOp = MissionOperationHandle<()>;
/// Handle for a fence download operation.
pub type FenceDownloadOp = MissionOperationHandle<FencePlan>;
/// Handle for a fence clear operation.
pub type FenceClearOp = MissionOperationHandle<()>;

/// Accessor for geofence state and transfer operations.
///
/// Obtained from [`Vehicle::fence`](crate::Vehicle::fence). Fence items pass through the
/// MAVLink mission protocol unchanged — no home insertion or extraction is performed (unlike the
/// mission domain).
///
/// # Conflict model
///
/// Fence transfers share the same `MissionProtocolScope`
/// as the mission and rally domains.  Starting a fence transfer while any other domain transfer
/// is active returns [`VehicleError::OperationConflict`] immediately.
pub struct FenceHandle<'a> {
    handle: StoredPlanHandle<'a, FenceState>,
}

impl<'a> FenceHandle<'a> {
    pub(crate) fn new(inner: &'a VehicleInner) -> Self {
        Self {
            handle: StoredPlanHandle::new(inner),
        }
    }

    /// Returns a capability-support observation for the fence domain.
    ///
    /// The initial value is seeded from the current vehicle identity so callers get a non-stale
    /// result even before the event loop has pushed an update.
    pub fn support(&self) -> ObservationHandle<SupportState> {
        self.handle.support()
    }

    /// Returns the most recently observed fence state, or `None` if no update has arrived yet.
    pub fn latest(&self) -> Option<FenceState> {
        self.handle.latest()
    }

    /// Returns the current cached fence state immediately when one is already available;
    /// otherwise waits for the first fence state observed after the call.
    ///
    /// Returns the default state if the vehicle disconnects before any fence state becomes
    /// available.
    pub async fn wait(&self) -> FenceState {
        self.handle.wait().await
    }

    /// Like [`wait`](Self::wait), but returns the current cached fence state immediately when one
    /// is already available.
    ///
    /// If no fence state is cached yet, this waits for the first observed fence state and returns
    /// [`VehicleError::Timeout`] if it does not arrive within `timeout`. Returns
    /// [`VehicleError::Disconnected`] if the vehicle disconnects before any fence state becomes
    /// available.
    pub async fn wait_timeout(&self, timeout: Duration) -> Result<FenceState, VehicleError> {
        self.handle.wait_timeout(timeout).await
    }

    /// Subscribes to an async stream of fence state updates.
    pub fn subscribe(&self) -> ObservationSubscription<FenceState> {
        self.handle.subscribe()
    }

    /// Begins uploading a fence plan to the vehicle.
    ///
    /// The plan is serialised to MAVLink mission items using the fence-specific wire encoding
    /// (polygon vertex counts, circle radii, inclusion groups). Returns a [`FenceUploadOp`] handle.
    /// On success, the local cache is updated; on failure or cancellation it is marked as
    /// [`SyncState::PossiblyStale`](crate::types::SyncState::PossiblyStale).
    ///
    /// Returns [`VehicleError::OperationConflict`] immediately if any transfer is already active.
    pub fn upload(&self, plan: FencePlan) -> Result<FenceUploadOp, VehicleError> {
        self.handle.upload(plan)
    }

    /// Begins downloading the vehicle's active geofence plan.
    ///
    /// Returns a [`FenceDownloadOp`] handle. On success, the wire items are decoded back into a
    /// typed [`FencePlan`] and the local cache is updated.
    ///
    /// Returns [`VehicleError::OperationConflict`] immediately if any transfer is already active.
    pub fn download(&self) -> Result<FenceDownloadOp, VehicleError> {
        self.handle.download()
    }

    /// Clears the vehicle's geofence storage.
    ///
    /// Returns a [`FenceClearOp`] handle. On success, the local cache is set to an empty
    /// [`FencePlan`] and marked current.
    ///
    /// Returns [`VehicleError::OperationConflict`] immediately if any transfer is already active.
    pub fn clear(&self) -> Result<FenceClearOp, VehicleError> {
        self.handle.clear()
    }
}
