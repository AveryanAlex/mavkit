pub mod transfer;
pub mod types;
pub mod validation;
pub mod wire;

#[cfg(test)]
pub(crate) mod test_support;

pub use transfer::{
    MissionTransferMachine, RetryPolicy, TransferDirection, TransferError, TransferEvent,
    TransferPhase, TransferProgress,
};
pub use types::{
    HomePosition, IssueSeverity, MissionFrame, MissionIssue, MissionItem, MissionPlan, MissionType,
};
pub use validation::{CompareTolerance, normalize_for_compare, plans_equivalent, validate_plan};
pub use wire::{items_for_wire_upload, plan_from_wire_download};

use crate::Vehicle;
use crate::error::VehicleError;

/// Handle to mission operations on a `Vehicle`.
pub struct MissionHandle<'a> {
    vehicle: &'a Vehicle,
}

impl<'a> MissionHandle<'a> {
    pub(crate) fn new(vehicle: &'a Vehicle) -> Self {
        Self { vehicle }
    }

    /// Upload a mission plan to the vehicle.
    pub async fn upload(&self, plan: MissionPlan) -> Result<(), VehicleError> {
        self.vehicle
            .send_command(|reply| crate::command::Command::MissionUpload { plan, reply })
            .await
    }

    /// Download the current mission plan from the vehicle.
    pub async fn download(&self, mission_type: MissionType) -> Result<MissionPlan, VehicleError> {
        self.vehicle
            .send_command(|reply| crate::command::Command::MissionDownload {
                mission_type,
                reply,
            })
            .await
    }

    /// Clear the mission of the given type on the vehicle.
    pub async fn clear(&self, mission_type: MissionType) -> Result<(), VehicleError> {
        self.vehicle
            .send_command(|reply| crate::command::Command::MissionClear {
                mission_type,
                reply,
            })
            .await
    }

    /// Upload a plan, download it back, and check equivalence.
    pub async fn verify_roundtrip(&self, plan: MissionPlan) -> Result<bool, VehicleError> {
        self.upload(plan.clone()).await?;
        let readback = self.download(plan.mission_type).await?;
        let mut lhs = normalize_for_compare(&plan);
        let mut rhs = normalize_for_compare(&readback);
        // Autopilot may overwrite home position; compare items only
        lhs.home = None;
        rhs.home = None;
        Ok(plans_equivalent(&lhs, &rhs, CompareTolerance::default()))
    }

    /// Set the current mission item sequence number on the vehicle.
    pub async fn set_current(&self, seq: u16) -> Result<(), VehicleError> {
        self.vehicle
            .send_command(|reply| crate::command::Command::MissionSetCurrent { seq, reply })
            .await
    }

    /// Cancel any in-progress mission transfer.
    pub fn cancel_transfer(&self) {
        let _ = self
            .vehicle
            .inner
            .command_tx
            .try_send(crate::command::Command::MissionCancelTransfer);
    }
}
