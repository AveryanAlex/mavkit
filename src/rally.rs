use std::time::Duration;

use crate::command::Command;
use crate::error::VehicleError;
use crate::geo::{
    GeoPoint3d, GeoPoint3dMsl, GeoPoint3dRelHome, GeoPoint3dTerrain, try_latitude_e7,
    try_longitude_e7,
};
use crate::mission::commands::MissionFrame as WireMissionFrame;
use crate::mission::operations::MissionOperationHandle;
use crate::mission::{
    MissionCommand, MissionItem, MissionProtocolScope, MissionType, OperationReservation,
    RawMissionCommand, WireMissionPlan, run_domain_operation,
};
use crate::observation::{ObservationHandle, ObservationSubscription};
use crate::stored_plan::{StoredPlanDomain, StoredPlanState};
use crate::types::{StoredPlanOperationKind, SupportState, SyncState};
use crate::vehicle::VehicleInner;

const MAV_CMD_NAV_RALLY_POINT: u16 = 5100;

#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
/// Rally-point plan used by ArduPilot rally storage.
pub struct RallyPlan {
    pub points: Vec<GeoPoint3d>,
}

#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
/// Cached rally-domain state plus sync and active-operation markers.
pub struct RallyState {
    pub plan: Option<RallyPlan>,
    pub sync: SyncState,
    pub active_op: Option<StoredPlanOperationKind>,
}

/// Handle for a rally upload operation.
pub type RallyUploadOp = MissionOperationHandle<()>;
/// Handle for a rally download operation.
pub type RallyDownloadOp = MissionOperationHandle<RallyPlan>;
/// Handle for a rally clear operation.
pub type RallyClearOp = MissionOperationHandle<()>;

#[derive(Clone)]
pub(crate) struct RallyDomain {
    inner: StoredPlanDomain<RallyState>,
}

impl StoredPlanState for RallyState {
    type Plan = RallyPlan;
    type OperationKind = StoredPlanOperationKind;

    fn set_active_op(&mut self, kind: Option<Self::OperationKind>) {
        self.active_op = kind;
    }

    fn set_plan(&mut self, plan: Self::Plan) {
        self.plan = Some(plan);
    }

    fn set_sync(&mut self, sync: SyncState) {
        self.sync = sync;
    }

    fn cleared_plan() -> Self::Plan {
        RallyPlan { points: Vec::new() }
    }
}

impl RallyDomain {
    pub(crate) fn new() -> Self {
        Self {
            inner: StoredPlanDomain::new(),
        }
    }

    pub(crate) fn state(&self) -> ObservationHandle<RallyState> {
        self.inner.state()
    }

    pub(crate) fn begin_operation(
        &self,
        scope: &MissionProtocolScope,
        kind: StoredPlanOperationKind,
        op_name: &'static str,
    ) -> Result<OperationReservation, VehicleError> {
        self.inner.begin_operation(scope, "rally", kind, op_name)
    }

    pub(crate) fn finish_operation(&self, scope: &MissionProtocolScope, op_id: u64) {
        self.inner.finish_operation(scope, op_id);
    }

    fn note_operation_error(&self) {
        self.inner.note_operation_error();
    }

    fn note_upload_success(&self, plan: RallyPlan) {
        self.inner.note_upload_success(plan);
    }

    fn note_download_success(&self, plan: RallyPlan) {
        self.inner.note_download_success(plan);
    }

    fn note_clear_success(&self) {
        self.inner.note_clear_success();
    }
}

/// Accessor for rally state and transfer operations.
pub struct RallyHandle<'a> {
    inner: &'a VehicleInner,
}

impl<'a> RallyHandle<'a> {
    pub(crate) fn new(inner: &'a VehicleInner) -> Self {
        Self { inner }
    }

    pub fn support(&self) -> ObservationHandle<SupportState> {
        let vehicle_state = self.inner.stores.vehicle_state.borrow().clone();
        self.inner.support.seed_from_vehicle_state(&vehicle_state);
        self.inner.support.mission_rally()
    }

    pub fn latest(&self) -> Option<RallyState> {
        self.inner.rally.state().latest()
    }

    pub async fn wait(&self) -> RallyState {
        self.inner.rally.state().wait().await.unwrap_or_default()
    }

    /// Like [`wait`](Self::wait), but returns [`VehicleError::Timeout`] if no
    /// state update arrives within `timeout`.
    pub async fn wait_timeout(&self, timeout: Duration) -> Result<RallyState, VehicleError> {
        self.inner.rally.state().wait_timeout(timeout).await
    }

    pub fn subscribe(&self) -> ObservationSubscription<RallyState> {
        self.inner.rally.state().subscribe()
    }

    pub fn upload(&self, plan: RallyPlan) -> Result<RallyUploadOp, VehicleError> {
        let wire_plan = mission_plan_from_rally_plan(&plan)?;
        let domain = self.inner.rally.clone();
        let reservation = domain.begin_operation(
            &self.inner.mission_protocol,
            StoredPlanOperationKind::Upload,
            "upload",
        )?;
        let protocol = self.inner.mission_protocol.clone();
        let op_id = reservation.id;

        Ok(run_domain_operation(
            self.inner.command_tx.clone(),
            self.inner.stores.mission_progress.clone(),
            reservation,
            |reply| Command::MissionUpload {
                plan: wire_plan,
                reply,
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

    pub fn download(&self) -> Result<RallyDownloadOp, VehicleError> {
        let domain = self.inner.rally.clone();
        let reservation = domain.begin_operation(
            &self.inner.mission_protocol,
            StoredPlanOperationKind::Download,
            "download",
        )?;
        let protocol = self.inner.mission_protocol.clone();
        let op_id = reservation.id;

        Ok(run_domain_operation(
            self.inner.command_tx.clone(),
            self.inner.stores.mission_progress.clone(),
            reservation,
            |reply| Command::MissionDownload {
                mission_type: MissionType::Rally,
                reply,
            },
            move |result, _| {
                let r = match result {
                    Ok(plan) => match rally_plan_from_mission_plan(plan) {
                        Ok(rally_plan) => {
                            domain.note_download_success(rally_plan.clone());
                            Ok(rally_plan)
                        }
                        Err(err) => {
                            domain.note_operation_error();
                            Err(err)
                        }
                    },
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

    pub fn clear(&self) -> Result<RallyClearOp, VehicleError> {
        let domain = self.inner.rally.clone();
        let reservation = domain.begin_operation(
            &self.inner.mission_protocol,
            StoredPlanOperationKind::Clear,
            "clear",
        )?;
        let protocol = self.inner.mission_protocol.clone();
        let op_id = reservation.id;

        Ok(run_domain_operation(
            self.inner.command_tx.clone(),
            self.inner.stores.mission_progress.clone(),
            reservation,
            |reply| Command::MissionClear {
                mission_type: MissionType::Rally,
                reply,
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
}

fn mission_plan_from_rally_plan(plan: &RallyPlan) -> Result<WireMissionPlan, VehicleError> {
    if plan.points.len() > u16::MAX as usize {
        return Err(VehicleError::InvalidParameter(format!(
            "rally plan has {} points, exceeding the {} item protocol limit",
            plan.points.len(),
            u16::MAX
        )));
    }

    let items: Vec<MissionItem> = plan
        .points
        .iter()
        .map(rally_item)
        .collect::<Result<_, _>>()?;

    Ok(WireMissionPlan {
        mission_type: MissionType::Rally,
        items,
    })
}

fn rally_plan_from_mission_plan(plan: WireMissionPlan) -> Result<RallyPlan, VehicleError> {
    if plan.mission_type != MissionType::Rally {
        return Err(VehicleError::InvalidParameter(
            "rally operations expect MissionType::Rally plan".to_string(),
        ));
    }

    let mut points = Vec::with_capacity(plan.items.len());
    for item in plan.items {
        points.push(rally_point_from_item(item)?);
    }

    Ok(RallyPlan { points })
}

fn rally_item(point: &GeoPoint3d) -> Result<MissionItem, VehicleError> {
    let (frame, x, y, z) = point_to_wire(point)?;

    Ok(MissionItem {
        command: MissionCommand::Other(RawMissionCommand {
            command: MAV_CMD_NAV_RALLY_POINT,
            frame,
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            x,
            y,
            z,
        }),
        current: false,
        autocontinue: true,
    })
}

fn point_to_wire(point: &GeoPoint3d) -> Result<(WireMissionFrame, i32, i32, f32), VehicleError> {
    match point {
        GeoPoint3d::Msl(point) => Ok((
            WireMissionFrame::Global,
            try_latitude_e7(point.latitude_deg)?,
            try_longitude_e7(point.longitude_deg)?,
            point.altitude_msl_m as f32,
        )),
        GeoPoint3d::RelHome(point) => Ok((
            WireMissionFrame::GlobalRelativeAlt,
            try_latitude_e7(point.latitude_deg)?,
            try_longitude_e7(point.longitude_deg)?,
            point.relative_alt_m as f32,
        )),
        GeoPoint3d::Terrain(point) => Ok((
            WireMissionFrame::GlobalTerrainAlt,
            try_latitude_e7(point.latitude_deg)?,
            try_longitude_e7(point.longitude_deg)?,
            point.altitude_terrain_m as f32,
        )),
    }
}

fn rally_point_from_item(item: MissionItem) -> Result<GeoPoint3d, VehicleError> {
    let (command, frame, _params, x, y, z) = item.command.into_wire();
    if command != MAV_CMD_NAV_RALLY_POINT {
        return Err(rally_decode_error(&format!(
            "unsupported rally mission command {command}"
        )));
    }

    decode_point3d(frame, x, y, z)
}

fn decode_point3d(
    frame: WireMissionFrame,
    x: i32,
    y: i32,
    z: f32,
) -> Result<GeoPoint3d, VehicleError> {
    let latitude_deg = f64::from(x) / 1e7;
    let longitude_deg = f64::from(y) / 1e7;

    match frame {
        WireMissionFrame::Global => Ok(GeoPoint3d::Msl(GeoPoint3dMsl {
            latitude_deg,
            longitude_deg,
            altitude_msl_m: f64::from(z),
        })),
        WireMissionFrame::GlobalRelativeAlt => Ok(GeoPoint3d::RelHome(GeoPoint3dRelHome {
            latitude_deg,
            longitude_deg,
            relative_alt_m: f64::from(z),
        })),
        WireMissionFrame::GlobalTerrainAlt => Ok(GeoPoint3d::Terrain(GeoPoint3dTerrain {
            latitude_deg,
            longitude_deg,
            altitude_terrain_m: f64::from(z),
        })),
        other => Err(rally_decode_error(&format!(
            "unsupported rally frame {:?}",
            other
        ))),
    }
}

fn rally_decode_error(detail: &str) -> VehicleError {
    VehicleError::TransferFailed {
        domain: "rally".to_string(),
        phase: "decode".to_string(),
        detail: detail.to_string(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mission::MissionDomain;

    fn sample_plan() -> RallyPlan {
        RallyPlan {
            points: vec![
                GeoPoint3d::RelHome(GeoPoint3dRelHome {
                    latitude_deg: 47.401,
                    longitude_deg: 8.541,
                    relative_alt_m: 55.0,
                }),
                GeoPoint3d::Msl(GeoPoint3dMsl {
                    latitude_deg: 47.402,
                    longitude_deg: 8.542,
                    altitude_msl_m: 510.5,
                }),
                GeoPoint3d::Terrain(GeoPoint3dTerrain {
                    latitude_deg: 47.403,
                    longitude_deg: 8.543,
                    altitude_terrain_m: 23.25,
                }),
            ],
        }
    }

    #[test]
    fn wire_roundtrip() {
        let plan = sample_plan();
        let wire_plan = mission_plan_from_rally_plan(&plan).unwrap();

        assert_eq!(wire_plan.mission_type, MissionType::Rally);
        assert_eq!(wire_plan.items.len(), 3);

        let (first_cmd, first_frame, _, _, _, first_z) =
            wire_plan.items[0].command.clone().into_wire();
        assert_eq!(first_cmd, MAV_CMD_NAV_RALLY_POINT);
        assert_eq!(first_frame, WireMissionFrame::GlobalRelativeAlt);
        assert_eq!(first_z, 55.0);

        let (_, second_frame, _, _, _, second_z) = wire_plan.items[1].command.clone().into_wire();
        assert_eq!(second_frame, WireMissionFrame::Global);
        assert_eq!(second_z, 510.5);

        let (_, third_frame, _, _, _, third_z) = wire_plan.items[2].command.clone().into_wire();
        assert_eq!(third_frame, WireMissionFrame::GlobalTerrainAlt);
        assert_eq!(third_z, 23.25);

        let roundtrip =
            rally_plan_from_mission_plan(wire_plan).expect("rally download should decode");
        assert_eq!(roundtrip, plan);
    }

    #[test]
    fn mission_upload_conflicts_with_rally_download() {
        let mission_protocol = MissionProtocolScope::new();
        let mission = MissionDomain::new();
        let rally = RallyDomain::new();

        let upload = mission
            .begin_operation(
                &mission_protocol,
                crate::types::MissionOperationKind::Upload,
                "upload",
            )
            .expect("mission upload should start first");

        let conflict = match rally.begin_operation(
            &mission_protocol,
            StoredPlanOperationKind::Download,
            "download",
        ) {
            Ok(_) => panic!("rally download should conflict while mission upload is active"),
            Err(err) => err,
        };

        assert!(matches!(
            conflict,
            VehicleError::OperationConflict {
                conflicting_domain,
                conflicting_op,
            } if conflicting_domain == "mission" && conflicting_op == "upload"
        ));

        mission.finish_operation(&mission_protocol, upload.id);
    }
}
