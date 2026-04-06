use super::operations::{MissionClearOp, MissionDownloadOp, MissionUploadOp, MissionVerifyOp};
use super::runner::run_domain_operation;
use super::types::{IssueSeverity, MissionPlan, MissionState, MissionType, WireMissionPlan};
use super::validation::{CompareTolerance, normalize_for_compare, plans_equivalent, validate_plan};
use crate::command::Command;
use crate::error::VehicleError;
use crate::observation::{ObservationSubscription, ObservationWriter};
use crate::operation::send_domain_command;
use crate::types::{MissionOperationKind, MissionOperationProgress};
use crate::vehicle::VehicleInner;
use std::time::Duration;

fn verify_plans_match(expected: &MissionPlan, observed: &MissionPlan) -> bool {
    let lhs = normalize_for_compare(expected);
    let rhs = normalize_for_compare(observed);
    plans_equivalent(&lhs, &rhs, CompareTolerance::default())
}

/// Accessor for mission state and mission transfer operations.
///
/// Obtained from [`Vehicle::mission`](crate::Vehicle::mission). The handle is borrowed from the
/// vehicle and is valid until the owning borrow ends.
///
/// # Mission protocol exclusivity
///
/// ArduPilot's MAVLink mission protocol is single-threaded on the vehicle side — only one
/// transfer can run at a time across all domains (mission, fence, rally, params). Starting a
/// second transfer while one is active returns [`VehicleError::OperationConflict`] immediately.
pub struct MissionHandle<'a> {
    inner: &'a VehicleInner,
}

impl<'a> MissionHandle<'a> {
    pub(crate) fn new(inner: &'a VehicleInner) -> Self {
        Self { inner }
    }

    /// Returns the most recently observed mission state, or `None` if no update has arrived yet.
    pub fn latest(&self) -> Option<MissionState> {
        self.inner.mission.state().latest()
    }

    /// Returns the current cached mission state immediately when one is already available;
    /// otherwise waits for the first mission state observed after the call.
    ///
    /// Returns the default state if the vehicle disconnects before any mission state becomes
    /// available.
    pub async fn wait(&self) -> MissionState {
        self.inner.mission.state().wait().await.unwrap_or_default()
    }

    /// Like [`wait`](Self::wait), but returns the current cached mission state immediately when
    /// one is already available.
    ///
    /// If no mission state is cached yet, this waits for the first observed mission state and
    /// returns [`VehicleError::Timeout`] if it does not arrive within `timeout`. Returns
    /// [`VehicleError::Disconnected`] if the vehicle disconnects before any mission state becomes
    /// available.
    pub async fn wait_timeout(&self, timeout: Duration) -> Result<MissionState, VehicleError> {
        self.inner.mission.state().wait_timeout(timeout).await
    }

    /// Subscribes to an async stream of mission state updates.
    pub fn subscribe(&self) -> ObservationSubscription<MissionState> {
        self.inner.mission.state().subscribe()
    }

    /// Begins uploading a mission plan to the vehicle.
    ///
    /// The plan is validated before the transfer starts — any validation error with
    /// [`IssueSeverity::Error`] returns immediately as [`VehicleError::InvalidMissionItem`] or
    /// [`VehicleError::InvalidMissionPlan`].
    ///
    /// Returns a [`MissionUploadOp`] handle that can be awaited for the final result, observed for
    /// progress, or cancelled. The operation completes only when the vehicle acknowledges the full
    /// transfer. On success, the local plan cache is updated; on failure or cancellation it is
    /// marked as [`SyncState::PossiblyStale`](crate::types::SyncState::PossiblyStale).
    ///
    /// Returns [`VehicleError::OperationConflict`] immediately if a transfer is already active.
    pub fn upload(&self, plan: MissionPlan) -> Result<MissionUploadOp, VehicleError> {
        let issues = validate_plan(&plan);
        if let Some(issue) = issues.iter().find(|i| i.severity == IssueSeverity::Error) {
            return if let Some(seq) = issue.seq {
                Err(VehicleError::InvalidMissionItem {
                    index: usize::from(seq),
                    reason: crate::error::MissionValidationReason::Other(format!(
                        "{}: {}",
                        issue.code, issue.message
                    )),
                })
            } else {
                Err(VehicleError::InvalidMissionPlan(format!(
                    "{}: {}",
                    issue.code, issue.message
                )))
            };
        }

        let wire_plan = WireMissionPlan {
            mission_type: MissionType::Mission,
            items: plan.items.clone(),
        };

        let domain = self.inner.mission.clone();
        let reservation = domain.begin_operation(
            &self.inner.mission_protocol,
            MissionOperationKind::Upload,
            "upload",
        )?;
        let protocol = self.inner.mission_protocol.clone();
        let op_id = reservation.id;

        Ok(run_domain_operation(
            self.inner.command_tx.clone(),
            self.inner.stores.mission_progress.clone(),
            reservation,
            |reply, cancel| Command::MissionUpload {
                plan: wire_plan,
                reply,
                cancel,
            },
            move |result, _| {
                match &result {
                    Ok(()) => domain.note_upload_success(plan),
                    Err(_) => domain.note_operation_error(),
                }
                domain.finish_operation(&protocol, op_id);
                result
            },
        ))
    }

    /// Begins downloading the vehicle's active mission plan.
    ///
    /// Returns a [`MissionDownloadOp`] handle. On success, the local plan cache is updated and
    /// the resolved `MissionPlan` is the operation result. On failure or cancellation the cache is
    /// marked as [`SyncState::PossiblyStale`](crate::types::SyncState::PossiblyStale).
    ///
    /// Returns [`VehicleError::OperationConflict`] immediately if a transfer is already active.
    pub fn download(&self) -> Result<MissionDownloadOp, VehicleError> {
        let domain = self.inner.mission.clone();
        let reservation = domain.begin_operation(
            &self.inner.mission_protocol,
            MissionOperationKind::Download,
            "download",
        )?;
        let protocol = self.inner.mission_protocol.clone();
        let op_id = reservation.id;

        Ok(run_domain_operation(
            self.inner.command_tx.clone(),
            self.inner.stores.mission_progress.clone(),
            reservation,
            |reply, cancel| Command::MissionDownload {
                mission_type: MissionType::Mission,
                reply,
                cancel,
            },
            move |result, _| {
                let r = match result {
                    Ok(wire_plan) => {
                        let plan = MissionPlan {
                            items: wire_plan.items,
                        };
                        domain.note_download_success(plan.clone());
                        Ok(plan)
                    }
                    Err(err) => {
                        domain.note_operation_error();
                        Err(err)
                    }
                };
                domain.finish_operation(&protocol, op_id);
                r
            },
        ))
    }

    /// Clears the vehicle's mission storage.
    ///
    /// Returns a [`MissionClearOp`] handle. On success, the local plan cache is set to an empty
    /// plan and marked current. On failure or cancellation it is marked as
    /// [`SyncState::PossiblyStale`](crate::types::SyncState::PossiblyStale).
    ///
    /// Returns [`VehicleError::OperationConflict`] immediately if a transfer is already active.
    pub fn clear(&self) -> Result<MissionClearOp, VehicleError> {
        let domain = self.inner.mission.clone();
        let reservation = domain.begin_operation(
            &self.inner.mission_protocol,
            MissionOperationKind::Clear,
            "clear",
        )?;
        let protocol = self.inner.mission_protocol.clone();
        let op_id = reservation.id;

        Ok(run_domain_operation(
            self.inner.command_tx.clone(),
            self.inner.stores.mission_progress.clone(),
            reservation,
            |reply, cancel| Command::MissionClear {
                mission_type: MissionType::Mission,
                reply,
                cancel,
            },
            move |result, _| {
                match &result {
                    Ok(()) => domain.note_clear_success(),
                    Err(_) => domain.note_operation_error(),
                }
                domain.finish_operation(&protocol, op_id);
                result
            },
        ))
    }

    /// Downloads the vehicle's active mission and compares it to `plan`.
    ///
    /// The resolved `bool` is `true` if the downloaded plan is equivalent to the supplied one
    /// (within the default floating-point tolerance used by [`plans_equivalent`]), `false`
    /// otherwise.  The local cache is updated with the downloaded plan regardless of whether
    /// the comparison succeeds.
    ///
    /// Returns [`VehicleError::OperationConflict`] immediately if a transfer is already active.
    pub fn verify(&self, plan: MissionPlan) -> Result<MissionVerifyOp, VehicleError> {
        let domain = self.inner.mission.clone();
        let reservation = domain.begin_operation(
            &self.inner.mission_protocol,
            MissionOperationKind::Verify,
            "verify",
        )?;
        let protocol = self.inner.mission_protocol.clone();
        let op_id = reservation.id;

        Ok(run_domain_operation(
            self.inner.command_tx.clone(),
            self.inner.stores.mission_progress.clone(),
            reservation,
            |reply, cancel| Command::MissionDownload {
                mission_type: MissionType::Mission,
                reply,
                cancel,
            },
            move |result, pw: &ObservationWriter<MissionOperationProgress>| {
                let r = match result {
                    Ok(wire_plan) => {
                        let downloaded = MissionPlan {
                            items: wire_plan.items,
                        };
                        let _ = pw.publish(MissionOperationProgress::Verifying);
                        let matched = verify_plans_match(&plan, &downloaded);
                        domain.note_download_success(downloaded);
                        Ok(matched)
                    }
                    Err(err) => {
                        domain.note_operation_error();
                        Err(err)
                    }
                };
                domain.finish_operation(&protocol, op_id);
                r
            },
        ))
    }

    /// Jumps to mission item `index` and waits until the vehicle reports that item as current.
    ///
    /// First sends `MAV_CMD_DO_SET_MISSION_CURRENT`, then polls the MAVLink `MISSION_CURRENT`
    /// stream until the vehicle echoes back the expected sequence number. Returns
    /// [`VehicleError::Timeout`] if the echo does not arrive within
    /// `VehicleConfig::command_completion_timeout`.
    pub async fn set_current(&self, index: u16) -> Result<(), VehicleError> {
        send_domain_command(self.inner.command_tx.clone(), |reply| {
            Command::MissionSetCurrent { seq: index, reply }
        })
        .await?;

        self.wait_for_current_index(index).await
    }

    async fn wait_for_current_index(&self, index: u16) -> Result<(), VehicleError> {
        let mut mission_state_rx = self.inner.stores.mission_state.clone();
        if mission_state_rx.borrow().current_seq == Some(index) {
            return Ok(());
        }

        let wait_for_match = async {
            loop {
                mission_state_rx
                    .changed()
                    .await
                    .map_err(|_| VehicleError::Disconnected)?;
                if mission_state_rx.borrow_and_update().current_seq == Some(index) {
                    return Ok(());
                }
            }
        };

        tokio::time::timeout(
            self.inner._config.command_completion_timeout,
            wait_for_match,
        )
        .await
        .map_err(|_| VehicleError::Timeout("mission domain command".into()))?
    }
}
