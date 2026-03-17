use crate::command::Command;
use crate::error::VehicleError;
use crate::geo::GeoPoint2d;
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

const MAV_CMD_NAV_FENCE_RETURN_POINT: u16 = 5000;
const MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION: u16 = 5001;
const MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION: u16 = 5002;
const MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION: u16 = 5003;
const MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION: u16 = 5004;

#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
/// Full geofence plan with optional return point and region list.
pub struct FencePlan {
    pub return_point: Option<GeoPoint2d>,
    pub regions: Vec<FenceRegion>,
}

#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
/// Polygon region that marks space where operation is allowed.
pub struct FenceInclusionPolygon {
    pub vertices: Vec<GeoPoint2d>,
    pub inclusion_group: u8,
}

#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
/// Polygon region that marks space where operation is forbidden.
pub struct FenceExclusionPolygon {
    pub vertices: Vec<GeoPoint2d>,
}

#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
/// Circular inclusion region.
pub struct FenceInclusionCircle {
    pub center: GeoPoint2d,
    pub radius_m: f32,
    pub inclusion_group: u8,
}

#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
/// Circular exclusion region.
pub struct FenceExclusionCircle {
    pub center: GeoPoint2d,
    pub radius_m: f32,
}

#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
/// Geofence region union used in upload and download plans.
pub enum FenceRegion {
    InclusionPolygon(FenceInclusionPolygon),
    ExclusionPolygon(FenceExclusionPolygon),
    InclusionCircle(FenceInclusionCircle),
    ExclusionCircle(FenceExclusionCircle),
}

impl From<FenceInclusionPolygon> for FenceRegion {
    fn from(value: FenceInclusionPolygon) -> Self {
        Self::InclusionPolygon(value)
    }
}

impl From<FenceExclusionPolygon> for FenceRegion {
    fn from(value: FenceExclusionPolygon) -> Self {
        Self::ExclusionPolygon(value)
    }
}

impl From<FenceInclusionCircle> for FenceRegion {
    fn from(value: FenceInclusionCircle) -> Self {
        Self::InclusionCircle(value)
    }
}

impl From<FenceExclusionCircle> for FenceRegion {
    fn from(value: FenceExclusionCircle) -> Self {
        Self::ExclusionCircle(value)
    }
}

#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
/// Cached fence-domain state plus sync and active-operation markers.
pub struct FenceState {
    pub plan: Option<FencePlan>,
    pub sync: SyncState,
    pub active_op: Option<StoredPlanOperationKind>,
}

/// Handle for a fence upload operation.
pub type FenceUploadOp = MissionOperationHandle<()>;
/// Handle for a fence download operation.
pub type FenceDownloadOp = MissionOperationHandle<FencePlan>;
/// Handle for a fence clear operation.
pub type FenceClearOp = MissionOperationHandle<()>;

#[derive(Clone)]
pub(crate) struct FenceDomain {
    inner: Arc<FenceDomainInner>,
}

struct FenceDomainInner {
    state_writer: ObservationWriter<FenceState>,
    state: ObservationHandle<FenceState>,
    latest_state: Mutex<FenceState>,
}

impl FenceDomain {
    pub(crate) fn new() -> Self {
        let (state_writer, state) = ObservationHandle::watch();
        let latest = FenceState::default();
        let _ = state_writer.publish(latest.clone());

        Self {
            inner: Arc::new(FenceDomainInner {
                state_writer,
                state,
                latest_state: Mutex::new(latest),
            }),
        }
    }

    pub(crate) fn state(&self) -> ObservationHandle<FenceState> {
        self.inner.state.clone()
    }

    pub(crate) fn begin_operation(
        &self,
        scope: &MissionProtocolScope,
        kind: StoredPlanOperationKind,
        op_name: &'static str,
    ) -> Result<OperationReservation, VehicleError> {
        let reservation = scope.begin_operation("fence", op_name)?;
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

    fn note_upload_success(&self, plan: FencePlan) {
        self.update_state(|state| {
            state.plan = Some(plan);
            state.sync = SyncState::Current;
        });
    }

    fn note_download_success(&self, plan: FencePlan) {
        self.update_state(|state| {
            state.plan = Some(plan);
            state.sync = SyncState::Current;
        });
    }

    fn note_clear_success(&self) {
        self.update_state(|state| {
            state.plan = Some(FencePlan {
                return_point: None,
                regions: Vec::new(),
            });
            state.sync = SyncState::Current;
        });
    }

    fn update_state(&self, edit: impl FnOnce(&mut FenceState)) {
        let mut latest = self.inner.latest_state.lock().unwrap();
        let mut next = latest.clone();
        edit(&mut next);
        if *latest != next {
            *latest = next.clone();
            let _ = self.inner.state_writer.publish(next);
        }
    }
}

/// Accessor for fence state and transfer operations.
pub struct FenceHandle<'a> {
    inner: &'a VehicleInner,
}

impl<'a> FenceHandle<'a> {
    pub(crate) fn new(inner: &'a VehicleInner) -> Self {
        Self { inner }
    }

    pub fn support(&self) -> ObservationHandle<SupportState> {
        let vehicle_state = self.inner.stores.vehicle_state.borrow().clone();
        self.inner.support.seed_from_vehicle_state(&vehicle_state);
        self.inner.support.mission_fence()
    }

    pub fn latest(&self) -> Option<FenceState> {
        self.inner.fence.state().latest()
    }

    pub async fn wait(&self) -> FenceState {
        self.inner.fence.state().wait().await.unwrap_or_default()
    }

    pub fn subscribe(&self) -> ObservationSubscription<FenceState> {
        self.inner.fence.state().subscribe()
    }

    pub fn upload(&self, plan: FencePlan) -> Result<FenceUploadOp, VehicleError> {
        let wire_plan = mission_plan_from_fence_plan(&plan)?;
        let domain = self.inner.fence.clone();
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

    pub fn download(&self) -> Result<FenceDownloadOp, VehicleError> {
        let domain = self.inner.fence.clone();
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
                    mission_type: MissionType::Fence,
                    reply,
                }) => result,
            };

            progress_stop.cancel();
            let _ = progress_task.await;

            let result = match command_result {
                Ok(plan) => match fence_plan_from_mission_plan(plan) {
                    Ok(fence_plan) => {
                        domain.note_download_success(fence_plan.clone());
                        Ok(fence_plan)
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

    pub fn clear(&self) -> Result<FenceClearOp, VehicleError> {
        let domain = self.inner.fence.clone();
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
                    mission_type: MissionType::Fence,
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

fn mission_plan_from_fence_plan(plan: &FencePlan) -> Result<MissionPlan, VehicleError> {
    let mut items = Vec::new();

    if let Some(return_point) = &plan.return_point {
        items.push(fence_item(
            0,
            MAV_CMD_NAV_FENCE_RETURN_POINT,
            0.0,
            0.0,
            return_point,
        ));
    }

    for region in &plan.regions {
        match region {
            FenceRegion::InclusionPolygon(region) => {
                validate_polygon("inclusion polygon", &region.vertices)?;
                append_polygon_items(
                    &mut items,
                    MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                    &region.vertices,
                    region.inclusion_group,
                );
            }
            FenceRegion::ExclusionPolygon(region) => {
                validate_polygon("exclusion polygon", &region.vertices)?;
                append_polygon_items(
                    &mut items,
                    MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
                    &region.vertices,
                    0,
                );
            }
            FenceRegion::InclusionCircle(region) => {
                validate_radius("inclusion circle", region.radius_m)?;
                items.push(fence_item(
                    items.len() as u16,
                    MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION,
                    region.radius_m,
                    f32::from(region.inclusion_group),
                    &region.center,
                ));
            }
            FenceRegion::ExclusionCircle(region) => {
                validate_radius("exclusion circle", region.radius_m)?;
                items.push(fence_item(
                    items.len() as u16,
                    MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
                    region.radius_m,
                    0.0,
                    &region.center,
                ));
            }
        }
    }

    Ok(MissionPlan {
        mission_type: MissionType::Fence,
        items,
    })
}

fn fence_plan_from_mission_plan(plan: MissionPlan) -> Result<FencePlan, VehicleError> {
    if plan.mission_type != MissionType::Fence {
        return Err(VehicleError::InvalidParameter(
            "fence operations expect MissionType::Fence plan".to_string(),
        ));
    }

    fence_plan_from_items(&plan.items)
}

fn fence_plan_from_items(items: &[MissionItem]) -> Result<FencePlan, VehicleError> {
    let mut return_point = None;
    let mut regions = Vec::new();
    let mut index = 0;

    while index < items.len() {
        let item = &items[index];
        let (command, frame, params, x, y, _z) = item.command.clone().into_wire();
        let point = decode_point2d(frame, x, y)?;

        match command {
            MAV_CMD_NAV_FENCE_RETURN_POINT => {
                if return_point.is_some() {
                    return Err(fence_decode_error("duplicate fence return point"));
                }
                return_point = Some(point);
                index += 1;
            }
            MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION => {
                validate_radius("downloaded inclusion circle", params[0])?;
                regions.push(
                    FenceInclusionCircle {
                        center: point,
                        radius_m: params[0],
                        inclusion_group: decode_u8_param(params[1], "inclusion_group")?,
                    }
                    .into(),
                );
                index += 1;
            }
            MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION => {
                validate_radius("downloaded exclusion circle", params[0])?;
                regions.push(
                    FenceExclusionCircle {
                        center: point,
                        radius_m: params[0],
                    }
                    .into(),
                );
                index += 1;
            }
            MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION => {
                let (vertices, consumed) = collect_polygon(
                    &items[index..],
                    MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                    Some(decode_u8_param(params[1], "inclusion_group")?),
                )?;
                regions.push(
                    FenceInclusionPolygon {
                        vertices,
                        inclusion_group: decode_u8_param(params[1], "inclusion_group")?,
                    }
                    .into(),
                );
                index += consumed;
            }
            MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION => {
                let (vertices, consumed) = collect_polygon(
                    &items[index..],
                    MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
                    None,
                )?;
                regions.push(FenceExclusionPolygon { vertices }.into());
                index += consumed;
            }
            other => {
                return Err(fence_decode_error(&format!(
                    "unsupported fence mission command {other}"
                )));
            }
        }
    }

    Ok(FencePlan {
        return_point,
        regions,
    })
}

fn collect_polygon(
    items: &[MissionItem],
    expected_command: u16,
    expected_group: Option<u8>,
) -> Result<(Vec<GeoPoint2d>, usize), VehicleError> {
    let first = items
        .first()
        .ok_or_else(|| fence_decode_error("missing polygon fence item"))?;
    let (_, _, params, _, _, _) = first.command.clone().into_wire();
    let vertex_count = decode_vertex_count(params[0])?;

    if items.len() < vertex_count {
        return Err(fence_decode_error("polygon vertex run ended early"));
    }

    let mut vertices = Vec::with_capacity(vertex_count);
    for item in items.iter().take(vertex_count) {
        let (command, frame, params, x, y, _z) = item.command.clone().into_wire();
        if command != expected_command {
            return Err(fence_decode_error("polygon vertices must be sequential"));
        }
        if decode_vertex_count(params[0])? != vertex_count {
            return Err(fence_decode_error(
                "polygon vertices must agree on vertex count",
            ));
        }
        if let Some(group) = expected_group
            && decode_u8_param(params[1], "inclusion_group")? != group
        {
            return Err(fence_decode_error(
                "polygon inclusion vertices must agree on inclusion_group",
            ));
        }
        vertices.push(decode_point2d(frame, x, y)?);
    }

    Ok((vertices, vertex_count))
}

fn append_polygon_items(
    items: &mut Vec<MissionItem>,
    command: u16,
    vertices: &[GeoPoint2d],
    param2: u8,
) {
    let vertex_count = vertices.len() as f32;
    for vertex in vertices {
        items.push(fence_item(
            items.len() as u16,
            command,
            vertex_count,
            f32::from(param2),
            vertex,
        ));
    }
}

fn fence_item(seq: u16, command: u16, param1: f32, param2: f32, point: &GeoPoint2d) -> MissionItem {
    MissionItem {
        seq,
        command: MissionCommand::Other(RawMissionCommand {
            command,
            frame: WireMissionFrame::Global,
            param1,
            param2,
            param3: 0.0,
            param4: 0.0,
            x: quantize_degrees_e7(point.latitude_deg),
            y: quantize_degrees_e7(point.longitude_deg),
            z: 0.0,
        }),
        current: false,
        autocontinue: true,
    }
}

fn quantize_degrees_e7(value: f64) -> i32 {
    (value * 1e7).round() as i32
}

fn decode_point2d(frame: WireMissionFrame, x: i32, y: i32) -> Result<GeoPoint2d, VehicleError> {
    match frame {
        WireMissionFrame::Global
        | WireMissionFrame::GlobalRelativeAlt
        | WireMissionFrame::GlobalTerrainAlt => Ok(GeoPoint2d {
            latitude_deg: x as f64 / 1e7,
            longitude_deg: y as f64 / 1e7,
        }),
        other => Err(fence_decode_error(&format!(
            "unsupported fence frame {:?}",
            other
        ))),
    }
}

fn validate_polygon(kind: &str, vertices: &[GeoPoint2d]) -> Result<(), VehicleError> {
    if vertices.len() < 3 {
        return Err(VehicleError::InvalidParameter(format!(
            "{kind} requires at least 3 vertices"
        )));
    }
    Ok(())
}

fn validate_radius(kind: &str, radius_m: f32) -> Result<(), VehicleError> {
    if !radius_m.is_finite() || radius_m <= 0.0 {
        return Err(VehicleError::InvalidParameter(format!(
            "{kind} radius must be positive and finite"
        )));
    }
    Ok(())
}

fn decode_vertex_count(value: f32) -> Result<usize, VehicleError> {
    if !value.is_finite() || value < 3.0 || value.fract() != 0.0 {
        return Err(fence_decode_error("invalid polygon vertex count"));
    }
    Ok(value as usize)
}

fn decode_u8_param(value: f32, label: &str) -> Result<u8, VehicleError> {
    if !value.is_finite() || !(0.0..=255.0).contains(&value) || value.fract() != 0.0 {
        return Err(fence_decode_error(&format!("invalid {label}")));
    }
    Ok(value as u8)
}

fn fence_decode_error(detail: &str) -> VehicleError {
    VehicleError::TransferFailed {
        domain: "fence".to_string(),
        phase: "decode".to_string(),
        detail: detail.to_string(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mission::MissionDomain;

    fn point(latitude_deg: f64, longitude_deg: f64) -> GeoPoint2d {
        GeoPoint2d {
            latitude_deg,
            longitude_deg,
        }
    }

    fn sample_plan() -> FencePlan {
        FencePlan {
            return_point: Some(point(47.40, 8.54)),
            regions: vec![
                FenceInclusionPolygon {
                    vertices: vec![
                        point(47.401, 8.541),
                        point(47.402, 8.542),
                        point(47.403, 8.543),
                    ],
                    inclusion_group: 7,
                }
                .into(),
                FenceExclusionPolygon {
                    vertices: vec![
                        point(47.411, 8.551),
                        point(47.412, 8.552),
                        point(47.413, 8.553),
                        point(47.414, 8.554),
                    ],
                }
                .into(),
                FenceInclusionCircle {
                    center: point(47.421, 8.561),
                    radius_m: 120.0,
                    inclusion_group: 3,
                }
                .into(),
                FenceExclusionCircle {
                    center: point(47.431, 8.571),
                    radius_m: 35.0,
                }
                .into(),
            ],
        }
    }

    #[test]
    fn wire_roundtrip() {
        let plan = sample_plan();
        let wire_plan = mission_plan_from_fence_plan(&plan).expect("fence upload should flatten");

        assert_eq!(wire_plan.mission_type, MissionType::Fence);
        assert_eq!(wire_plan.items.len(), 10);

        let (return_cmd, _, _, _, _, _) = wire_plan.items[0].command.clone().into_wire();
        assert_eq!(return_cmd, MAV_CMD_NAV_FENCE_RETURN_POINT);

        let (_, _, inclusion_params, _, _, _) = wire_plan.items[1].command.clone().into_wire();
        assert_eq!(inclusion_params[0], 3.0);
        assert_eq!(inclusion_params[1], 7.0);

        let (_, _, circle_params, _, _, _) = wire_plan.items[8].command.clone().into_wire();
        assert_eq!(circle_params[0], 120.0);
        assert_eq!(circle_params[1], 3.0);

        let roundtrip =
            fence_plan_from_mission_plan(wire_plan).expect("fence download should regroup");
        assert_eq!(roundtrip, plan);
    }

    #[test]
    fn mission_upload_conflicts_with_fence_download() {
        let mission_protocol = MissionProtocolScope::new();
        let mission = MissionDomain::new();
        let fence = FenceDomain::new();

        let upload = mission
            .begin_operation(
                &mission_protocol,
                crate::types::MissionOperationKind::Upload,
                "upload",
            )
            .expect("mission upload should start first");

        let conflict = match fence.begin_operation(
            &mission_protocol,
            StoredPlanOperationKind::Download,
            "download",
        ) {
            Ok(_) => panic!("fence download should conflict while mission upload is active"),
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
