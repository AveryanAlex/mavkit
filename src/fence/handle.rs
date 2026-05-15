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
    /// The observation remains unpublished until `AUTOPILOT_VERSION` provides capability evidence
    /// or init reaches a terminal inconclusive state.
    pub fn support(&self) -> ObservationHandle<SupportState> {
        self.handle.support()
    }

    /// Returns the current cached fence state.
    ///
    /// The fence domain publishes its default cache immediately; default `plan`/`sync` values mean
    /// no vehicle-confirmed fence plan is cached yet.
    pub fn latest(&self) -> Option<FenceState> {
        self.handle.latest()
    }

    /// Returns the current cached fence state immediately.
    ///
    /// The default cache represents no vehicle-confirmed fence data yet.
    pub async fn wait(&self) -> FenceState {
        self.handle.wait().await
    }

    /// Like [`wait`](Self::wait), but returns the current cached fence state immediately when one
    /// is already available.
    ///
    /// Because the fence domain publishes a default cache at construction, this normally returns
    /// immediately. [`VehicleError::Timeout`] or [`VehicleError::Disconnected`] are only expected
    /// if the observation channel has no cached value due to future semantic changes or teardown.
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
