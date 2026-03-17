use crate::command::Command;
use crate::error::VehicleError;
use crate::geo::{GeoPoint3d, GeoPoint3dMsl, GeoPoint3dRelHome, GeoPoint3dTerrain};
use crate::mission::commands::MissionFrame as WireMissionFrame;
use crate::mission::operations::MissionOperationHandle;
use crate::mission::{
    MissionCommand, MissionItem, MissionPlan, MissionProtocolScope, MissionType,
    OperationReservation, RawMissionCommand, send_domain_command, spawn_transfer_progress_bridge,
};
use crate::observation::{ObservationHandle, ObservationSubscription, ObservationWriter};
use crate::types::{MissionOperationProgress, StoredPlanOperationKind, SupportState, SyncState};
use crate::vehicle::VehicleInner;
use std::sync::{Arc, Mutex};
use tokio::sync::oneshot;
use tokio_util::sync::CancellationToken;

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
    inner: Arc<RallyDomainInner>,
}

struct RallyDomainInner {
    state_writer: ObservationWriter<RallyState>,
    state: ObservationHandle<RallyState>,
    latest_state: Mutex<RallyState>,
}

impl RallyDomain {
    pub(crate) fn new() -> Self {
        let (state_writer, state) = ObservationHandle::watch();
        let latest = RallyState::default();
        let _ = state_writer.publish(latest.clone());

        Self {
            inner: Arc::new(RallyDomainInner {
                state_writer,
                state,
                latest_state: Mutex::new(latest),
            }),
        }
    }

    pub(crate) fn state(&self) -> ObservationHandle<RallyState> {
        self.inner.state.clone()
    }

    pub(crate) fn begin_operation(
        &self,
        scope: &MissionProtocolScope,
        kind: StoredPlanOperationKind,
        op_name: &'static str,
    ) -> Result<OperationReservation, VehicleError> {
        let reservation = scope.begin_operation("rally", op_name)?;
        self.update_state(|state| {
            state.active_op = Some(kind);
        });
        Ok(reservation)
    }

    pub(crate) fn finish_operation(&self, scope: &MissionProtocolScope, op_id: u64) {
        scope.finish_operation(op_id);
        self.update_state(|state| {
            state.active_op = None;
        });
    }

    fn note_operation_error(&self) {
        self.update_state(|state| {
            state.sync = SyncState::PossiblyStale;
        });
    }

    fn note_upload_success(&self, plan: RallyPlan) {
        self.update_state(|state| {
            state.plan = Some(plan);
            state.sync = SyncState::Current;
        });
    }

    fn note_download_success(&self, plan: RallyPlan) {
        self.update_state(|state| {
            state.plan = Some(plan);
            state.sync = SyncState::Current;
        });
    }

    fn note_clear_success(&self) {
        self.update_state(|state| {
            state.plan = Some(RallyPlan { points: Vec::new() });
            state.sync = SyncState::Current;
        });
    }

    fn update_state(&self, edit: impl FnOnce(&mut RallyState)) {
        let mut latest = self.inner.latest_state.lock().unwrap();
        let mut next = latest.clone();
        edit(&mut next);
        if *latest != next {
            *latest = next.clone();
            let _ = self.inner.state_writer.publish(next);
        }
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

    pub fn subscribe(&self) -> ObservationSubscription<RallyState> {
        self.inner.rally.state().subscribe()
    }

    pub fn upload(&self, plan: RallyPlan) -> Result<RallyUploadOp, VehicleError> {
        let wire_plan = mission_plan_from_rally_plan(&plan);
        let domain = self.inner.rally.clone();
        let reservation = domain.begin_operation(
            &self.inner.mission_protocol,
            StoredPlanOperationKind::Upload,
            "upload",
        )?;
        let (progress_writer, progress) = ObservationHandle::watch();
        let _ = progress_writer.publish(MissionOperationProgress::RequestCount);
        let (result_tx, result_rx) = oneshot::channel();

        let op = MissionOperationHandle::new(
            progress,
            result_rx,
            reservation.cancel.clone(),
            self.inner.command_tx.clone(),
        );

        let command_tx = self.inner.command_tx.clone();
        let mission_progress_rx = self.inner.stores.mission_progress.clone();
        let mission_protocol = self.inner.mission_protocol.clone();
        let op_id = reservation.id;
        let cancel = reservation.cancel;
        let uploaded_plan = plan.clone();

        tokio::spawn(async move {
            let progress_stop = CancellationToken::new();
            let progress_task = spawn_transfer_progress_bridge(
                mission_progress_rx,
                progress_writer.clone(),
                progress_stop.clone(),
            );

            let result = tokio::select! {
                _ = cancel.cancelled() => Err(VehicleError::Cancelled),
                command_result = send_domain_command(command_tx, |reply| Command::MissionUpload {
                    plan: wire_plan,
                    reply,
                }) => command_result,
            };

            progress_stop.cancel();
            let _ = progress_task.await;

            let final_phase = match &result {
                Ok(()) => MissionOperationProgress::Completed,
                Err(VehicleError::Cancelled) => MissionOperationProgress::Cancelled,
                Err(_) => MissionOperationProgress::Failed,
            };
            let _ = progress_writer.publish(final_phase);

            match &result {
                Ok(()) => domain.note_upload_success(uploaded_plan),
                Err(_) => domain.note_operation_error(),
            }

            domain.finish_operation(&mission_protocol, op_id);
            let _ = result_tx.send(result);
        });

        Ok(op)
    }

    pub fn download(&self) -> Result<RallyDownloadOp, VehicleError> {
        let domain = self.inner.rally.clone();
        let reservation = domain.begin_operation(
            &self.inner.mission_protocol,
            StoredPlanOperationKind::Download,
            "download",
        )?;
        let (progress_writer, progress) = ObservationHandle::watch();
        let _ = progress_writer.publish(MissionOperationProgress::RequestCount);
        let (result_tx, result_rx) = oneshot::channel();

        let op = MissionOperationHandle::new(
            progress,
            result_rx,
            reservation.cancel.clone(),
            self.inner.command_tx.clone(),
        );

        let command_tx = self.inner.command_tx.clone();
        let mission_progress_rx = self.inner.stores.mission_progress.clone();
        let mission_protocol = self.inner.mission_protocol.clone();
        let op_id = reservation.id;
        let cancel = reservation.cancel;

        tokio::spawn(async move {
            let progress_stop = CancellationToken::new();
            let progress_task = spawn_transfer_progress_bridge(
                mission_progress_rx,
                progress_writer.clone(),
                progress_stop.clone(),
            );

            let command_result = tokio::select! {
                _ = cancel.cancelled() => Err(VehicleError::Cancelled),
                result = send_domain_command(command_tx, |reply| Command::MissionDownload {
                    mission_type: MissionType::Rally,
                    reply,
                }) => result,
            };

            progress_stop.cancel();
            let _ = progress_task.await;

            let result = match command_result {
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

            let final_phase = match &result {
                Ok(_) => MissionOperationProgress::Completed,
                Err(VehicleError::Cancelled) => MissionOperationProgress::Cancelled,
                Err(_) => MissionOperationProgress::Failed,
            };
            let _ = progress_writer.publish(final_phase);

            domain.finish_operation(&mission_protocol, op_id);
            let _ = result_tx.send(result);
        });

        Ok(op)
    }

    pub fn clear(&self) -> Result<RallyClearOp, VehicleError> {
        let domain = self.inner.rally.clone();
        let reservation = domain.begin_operation(
            &self.inner.mission_protocol,
            StoredPlanOperationKind::Clear,
            "clear",
        )?;
        let (progress_writer, progress) = ObservationHandle::watch();
        let _ = progress_writer.publish(MissionOperationProgress::RequestCount);
        let (result_tx, result_rx) = oneshot::channel();

        let op = MissionOperationHandle::new(
            progress,
            result_rx,
            reservation.cancel.clone(),
            self.inner.command_tx.clone(),
        );

        let command_tx = self.inner.command_tx.clone();
        let mission_progress_rx = self.inner.stores.mission_progress.clone();
        let mission_protocol = self.inner.mission_protocol.clone();
        let op_id = reservation.id;
        let cancel = reservation.cancel;

        tokio::spawn(async move {
            let progress_stop = CancellationToken::new();
            let progress_task = spawn_transfer_progress_bridge(
                mission_progress_rx,
                progress_writer.clone(),
                progress_stop.clone(),
            );

            let result = tokio::select! {
                _ = cancel.cancelled() => Err(VehicleError::Cancelled),
                command_result = send_domain_command(command_tx, |reply| Command::MissionClear {
                    mission_type: MissionType::Rally,
                    reply,
                }) => command_result,
            };

            progress_stop.cancel();
            let _ = progress_task.await;

            let final_phase = match &result {
                Ok(()) => MissionOperationProgress::Completed,
                Err(VehicleError::Cancelled) => MissionOperationProgress::Cancelled,
                Err(_) => MissionOperationProgress::Failed,
            };
            let _ = progress_writer.publish(final_phase);

            match &result {
                Ok(()) => domain.note_clear_success(),
                Err(_) => domain.note_operation_error(),
            }

            domain.finish_operation(&mission_protocol, op_id);
            let _ = result_tx.send(result);
        });

        Ok(op)
    }
}

fn mission_plan_from_rally_plan(plan: &RallyPlan) -> MissionPlan {
    let items = plan
        .points
        .iter()
        .enumerate()
        .map(|(index, point)| rally_item(index as u16, point))
        .collect();

    MissionPlan {
        mission_type: MissionType::Rally,
        items,
    }
}

fn rally_plan_from_mission_plan(plan: MissionPlan) -> Result<RallyPlan, VehicleError> {
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

fn rally_item(seq: u16, point: &GeoPoint3d) -> MissionItem {
    let (frame, x, y, z) = point_to_wire(point);

    MissionItem {
        seq,
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
    }
}

fn point_to_wire(point: &GeoPoint3d) -> (WireMissionFrame, i32, i32, f32) {
    match point {
        GeoPoint3d::Msl(point) => (
            WireMissionFrame::Global,
            quantize_degrees_e7(point.latitude_deg),
            quantize_degrees_e7(point.longitude_deg),
            point.altitude_msl_m as f32,
        ),
        GeoPoint3d::RelHome(point) => (
            WireMissionFrame::GlobalRelativeAlt,
            quantize_degrees_e7(point.latitude_deg),
            quantize_degrees_e7(point.longitude_deg),
            point.relative_alt_m as f32,
        ),
        GeoPoint3d::Terrain(point) => (
            WireMissionFrame::GlobalTerrainAlt,
            quantize_degrees_e7(point.latitude_deg),
            quantize_degrees_e7(point.longitude_deg),
            point.altitude_terrain_m as f32,
        ),
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
    let latitude_deg = x as f64 / 1e7;
    let longitude_deg = y as f64 / 1e7;

    match frame {
        WireMissionFrame::Global => Ok(GeoPoint3d::Msl(GeoPoint3dMsl {
            latitude_deg,
            longitude_deg,
            altitude_msl_m: z as f64,
        })),
        WireMissionFrame::GlobalRelativeAlt => Ok(GeoPoint3d::RelHome(GeoPoint3dRelHome {
            latitude_deg,
            longitude_deg,
            relative_alt_m: z as f64,
        })),
        WireMissionFrame::GlobalTerrainAlt => Ok(GeoPoint3d::Terrain(GeoPoint3dTerrain {
            latitude_deg,
            longitude_deg,
            altitude_terrain_m: z as f64,
        })),
        other => Err(rally_decode_error(&format!(
            "unsupported rally frame {:?}",
            other
        ))),
    }
}

fn quantize_degrees_e7(value: f64) -> i32 {
    (value * 1e7).round() as i32
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
        let wire_plan = mission_plan_from_rally_plan(&plan);

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
