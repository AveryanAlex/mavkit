use crate::geo::{GeoPoint2d, GeoPoint3dMsl, GeoPoint3dRelHome};
use crate::state::GpsFixType;
use serde::{Deserialize, Serialize};

/// Global position sourced from `GLOBAL_POSITION_INT`.
///
/// `altitude_msl_m` is the WGS-84 mean-sea-level altitude in metres.
/// `relative_alt_m` is the altitude above the home position set at arming time.
/// Both altitudes can be negative (e.g. flying below home elevation).
#[cfg_attr(feature = "typescript", derive(specta::Type))]
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GlobalPosition {
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    pub altitude_msl_m: f64,
    pub relative_alt_m: f64,
}

/// Euler-angle attitude decoded from `ATTITUDE` messages.
///
/// All angles are in degrees. `yaw_deg` is in the range `(-180, 180]`, where 0 is
/// north. Positive roll is right-bank; positive pitch is nose-up.
#[cfg_attr(feature = "typescript", derive(specta::Type))]
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct EulerAttitude {
    pub roll_deg: f64,
    pub pitch_deg: f64,
    pub yaw_deg: f64,
}

/// GPS signal quality sourced from `GPS_RAW_INT`.
///
/// `satellites` and `hdop` are `None` when the autopilot reports the sentinel values
/// `255` (satellites unknown) or `65535` (HDOP unknown), respectively.
/// Lower `hdop` values indicate better horizontal position accuracy (< 2.0 is good,
/// > 5.0 is poor).
#[cfg_attr(feature = "typescript", derive(specta::Type))]
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GpsQuality {
    pub fix_type: GpsFixType,
    pub satellites: Option<u8>,
    pub hdop: Option<f64>,
}

/// Per-cell battery voltages sourced from `BATTERY_STATUS`.
///
/// The vector length varies by vehicle; cells not physically present are omitted.
/// Voltages are in volts. A typical LiPo cell ranges from ~3.0 V (empty) to ~4.2 V
/// (fully charged).
#[cfg_attr(feature = "typescript", derive(specta::Type))]
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CellVoltages {
    pub voltages_v: Vec<f64>,
}

/// Distance and bearing to the active waypoint, sourced from `NAV_CONTROLLER_OUTPUT`.
///
/// `bearing_deg` is the ground-track bearing to the waypoint in degrees `[0, 360)`.
/// Only meaningful when a mission is active; values are unspecified when the vehicle
/// is loitering or has no active waypoint.
#[cfg_attr(feature = "typescript", derive(specta::Type))]
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct WaypointProgress {
    pub distance_m: f64,
    pub bearing_deg: f64,
}

/// Lateral guidance state sourced from `NAV_CONTROLLER_OUTPUT`.
///
/// `bearing_deg` is the desired heading the autopilot is commanding `[0, 360)`.
/// `cross_track_error_m` is the signed perpendicular distance from the desired track
/// line; positive values mean the vehicle is to the right of the track.
#[cfg_attr(feature = "typescript", derive(specta::Type))]
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GuidanceState {
    pub bearing_deg: f64,
    pub cross_track_error_m: f64,
}

/// Terrain clearance sourced from `TERRAIN_REPORT`.
///
/// `terrain_height_m` is the terrain elevation above mean sea level at the vehicle's
/// current position. `height_above_terrain_m` is the vehicle's altitude above that
/// terrain — the value the autopilot uses for terrain-following. Only available when
/// the autopilot has terrain data loaded for the current location.
#[cfg_attr(feature = "typescript", derive(specta::Type))]
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TerrainClearance {
    pub terrain_height_m: f64,
    pub height_above_terrain_m: f64,
}

impl From<GlobalPosition> for GeoPoint2d {
    fn from(value: GlobalPosition) -> Self {
        Self {
            latitude_deg: value.latitude_deg,
            longitude_deg: value.longitude_deg,
        }
    }
}

impl From<GlobalPosition> for GeoPoint3dMsl {
    fn from(value: GlobalPosition) -> Self {
        Self {
            latitude_deg: value.latitude_deg,
            longitude_deg: value.longitude_deg,
            altitude_msl_m: value.altitude_msl_m,
        }
    }
}

impl From<GlobalPosition> for GeoPoint3dRelHome {
    fn from(value: GlobalPosition) -> Self {
        Self {
            latitude_deg: value.latitude_deg,
            longitude_deg: value.longitude_deg,
            relative_alt_m: value.relative_alt_m,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn global_position_to_geo_point_conversions() {
        let global = GlobalPosition {
            latitude_deg: 47.397742,
            longitude_deg: 8.545594,
            altitude_msl_m: 510.0,
            relative_alt_m: 50.0,
        };

        let point_2d: GeoPoint2d = global.clone().into();
        assert_eq!(point_2d.latitude_deg, 47.397742);
        assert_eq!(point_2d.longitude_deg, 8.545594);

        let point_msl: GeoPoint3dMsl = global.clone().into();
        assert_eq!(point_msl.latitude_deg, 47.397742);
        assert_eq!(point_msl.longitude_deg, 8.545594);
        assert_eq!(point_msl.altitude_msl_m, 510.0);

        let point_rel_home: GeoPoint3dRelHome = global.into();
        assert_eq!(point_rel_home.latitude_deg, 47.397742);
        assert_eq!(point_rel_home.longitude_deg, 8.545594);
        assert_eq!(point_rel_home.relative_alt_m, 50.0);
    }
}
