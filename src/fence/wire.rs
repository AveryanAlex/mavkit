use crate::error::VehicleError;
use crate::geo::{GeoPoint2d, try_latitude_e7, try_longitude_e7};
use crate::mission::commands::MissionFrame as WireMissionFrame;
use crate::mission::{
    MissionCommand, MissionItem, MissionType, RawMissionCommand, WireMissionPlan,
};

use super::plan::{
    FenceExclusionCircle, FenceExclusionPolygon, FenceInclusionCircle, FenceInclusionPolygon,
    FencePlan, FenceRegion,
};

const MAV_CMD_NAV_FENCE_RETURN_POINT: u16 = 5000;
const MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION: u16 = 5001;
const MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION: u16 = 5002;
const MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION: u16 = 5003;
const MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION: u16 = 5004;

pub(super) fn mission_plan_from_fence_plan(
    plan: &FencePlan,
) -> Result<WireMissionPlan, VehicleError> {
    let mut items = Vec::new();

    if let Some(return_point) = &plan.return_point {
        items.push(fence_item(
            MAV_CMD_NAV_FENCE_RETURN_POINT,
            0.0,
            0.0,
            return_point,
        )?);
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
                )?;
            }
            FenceRegion::ExclusionPolygon(region) => {
                validate_polygon("exclusion polygon", &region.vertices)?;
                append_polygon_items(
                    &mut items,
                    MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
                    &region.vertices,
                    0,
                )?;
            }
            FenceRegion::InclusionCircle(region) => {
                validate_radius("inclusion circle", region.radius_m)?;
                items.push(fence_item(
                    MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION,
                    region.radius_m,
                    f32::from(region.inclusion_group),
                    &region.center,
                )?);
            }
            FenceRegion::ExclusionCircle(region) => {
                validate_radius("exclusion circle", region.radius_m)?;
                items.push(fence_item(
                    MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
                    region.radius_m,
                    0.0,
                    &region.center,
                )?);
            }
        }
    }

    if items.len() > u16::MAX as usize {
        return Err(VehicleError::InvalidParameter(format!(
            "fence plan produces {} items, exceeding the {} item protocol limit",
            items.len(),
            u16::MAX
        )));
    }

    Ok(WireMissionPlan {
        mission_type: MissionType::Fence,
        items,
    })
}

pub(super) fn fence_plan_from_mission_plan(
    plan: WireMissionPlan,
) -> Result<FencePlan, VehicleError> {
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
) -> Result<(), VehicleError> {
    let vertex_count = vertices.len() as f32;
    for vertex in vertices {
        items.push(fence_item(
            command,
            vertex_count,
            f32::from(param2),
            vertex,
        )?);
    }
    Ok(())
}

fn fence_item(
    command: u16,
    param1: f32,
    param2: f32,
    point: &GeoPoint2d,
) -> Result<MissionItem, VehicleError> {
    Ok(MissionItem {
        command: MissionCommand::Other(RawMissionCommand {
            command,
            frame: WireMissionFrame::Global,
            param1,
            param2,
            param3: 0.0,
            param4: 0.0,
            x: try_latitude_e7(point.latitude_deg)?,
            y: try_longitude_e7(point.longitude_deg)?,
            z: 0.0,
        }),
        autocontinue: true,
    })
}

fn decode_point2d(frame: WireMissionFrame, x: i32, y: i32) -> Result<GeoPoint2d, VehicleError> {
    match frame {
        WireMissionFrame::Global
        | WireMissionFrame::GlobalRelativeAlt
        | WireMissionFrame::GlobalTerrainAlt => Ok(GeoPoint2d {
            latitude_deg: f64::from(x) / 1e7,
            longitude_deg: f64::from(y) / 1e7,
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
    use crate::geo::GeoPoint2d;
    use crate::mission::MissionType;

    use super::{
        FenceExclusionCircle, FenceExclusionPolygon, FenceInclusionCircle, FenceInclusionPolygon,
        FencePlan, FenceRegion, MAV_CMD_NAV_FENCE_RETURN_POINT, fence_plan_from_mission_plan,
        mission_plan_from_fence_plan,
    };

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
                FenceRegion::InclusionPolygon(FenceInclusionPolygon {
                    vertices: vec![
                        point(47.401, 8.541),
                        point(47.402, 8.542),
                        point(47.403, 8.543),
                    ],
                    inclusion_group: 7,
                }),
                FenceRegion::ExclusionPolygon(FenceExclusionPolygon {
                    vertices: vec![
                        point(47.411, 8.551),
                        point(47.412, 8.552),
                        point(47.413, 8.553),
                        point(47.414, 8.554),
                    ],
                }),
                FenceRegion::InclusionCircle(FenceInclusionCircle {
                    center: point(47.421, 8.561),
                    radius_m: 120.0,
                    inclusion_group: 3,
                }),
                FenceRegion::ExclusionCircle(FenceExclusionCircle {
                    center: point(47.431, 8.571),
                    radius_m: 35.0,
                }),
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
}
