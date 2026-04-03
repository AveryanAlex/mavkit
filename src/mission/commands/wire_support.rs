use super::MissionFrame;
use crate::geo::{
    GeoPoint3d, GeoPoint3dMsl, GeoPoint3dRelHome, GeoPoint3dTerrain, quantize_degrees_e7,
};

pub(super) fn position_to_wire(position: GeoPoint3d) -> (MissionFrame, i32, i32, f32) {
    match position {
        GeoPoint3d::Msl(point) => (
            MissionFrame::Global,
            quantize_degrees_e7(point.latitude_deg),
            quantize_degrees_e7(point.longitude_deg),
            point.altitude_msl_m as f32,
        ),
        GeoPoint3d::RelHome(point) => (
            MissionFrame::GlobalRelativeAlt,
            quantize_degrees_e7(point.latitude_deg),
            quantize_degrees_e7(point.longitude_deg),
            point.relative_alt_m as f32,
        ),
        GeoPoint3d::Terrain(point) => (
            MissionFrame::GlobalTerrainAlt,
            quantize_degrees_e7(point.latitude_deg),
            quantize_degrees_e7(point.longitude_deg),
            point.altitude_terrain_m as f32,
        ),
    }
}

pub(super) fn position_from_wire(frame: MissionFrame, x: i32, y: i32, z: f32) -> GeoPoint3d {
    let latitude_deg = f64::from(x) / 1e7;
    let longitude_deg = f64::from(y) / 1e7;

    match frame {
        MissionFrame::Global => GeoPoint3d::Msl(GeoPoint3dMsl {
            latitude_deg,
            longitude_deg,
            altitude_msl_m: f64::from(z),
        }),
        MissionFrame::GlobalRelativeAlt => GeoPoint3d::RelHome(GeoPoint3dRelHome {
            latitude_deg,
            longitude_deg,
            relative_alt_m: f64::from(z),
        }),
        MissionFrame::GlobalTerrainAlt => GeoPoint3d::Terrain(GeoPoint3dTerrain {
            latitude_deg,
            longitude_deg,
            altitude_terrain_m: f64::from(z),
        }),
        MissionFrame::Mission | MissionFrame::Other(_) => GeoPoint3d::Msl(GeoPoint3dMsl {
            latitude_deg,
            longitude_deg,
            altitude_msl_m: f64::from(z),
        }),
    }
}

pub(super) fn position_command_to_wire(
    position: GeoPoint3d,
    params: [f32; 4],
) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    let (frame, x, y, z) = position_to_wire(position);
    (frame, params, x, y, z)
}

pub(super) fn mission_command_to_wire(
    params: [f32; 4],
    x: i32,
    y: i32,
    z: f32,
) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    (MissionFrame::Mission, params, x, y, z)
}

pub(super) fn unit_command_to_wire() -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire([0.0, 0.0, 0.0, 0.0], 0, 0, 0.0)
}

pub(super) fn bool_to_param(value: bool) -> f32 {
    if value { 1.0 } else { 0.0 }
}

pub(super) fn bool_from_param(value: f32) -> bool {
    value > 0.5
}

pub(super) fn u8_from_param(value: f32) -> u8 {
    value.round() as u8
}

pub(super) fn i8_from_param(value: f32) -> i8 {
    value.round() as i8
}

pub(super) fn u16_from_param(value: f32) -> u16 {
    value.round() as u16
}

pub(super) fn u32_from_param(value: f32) -> u32 {
    value.round() as u32
}

pub(super) fn empty_unit_from_wire(
    _frame: MissionFrame,
    _params: [f32; 4],
    _x: i32,
    _y: i32,
    _z: f32,
) {
}
