use crate::error::VehicleError;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
/// WGS84 latitude and longitude in decimal degrees.
pub struct GeoPoint2d {
    pub latitude_deg: f64,
    pub longitude_deg: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
/// WGS84 position with altitude above mean sea level.
pub struct GeoPoint3dMsl {
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    pub altitude_msl_m: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
/// WGS84 position with altitude relative to home.
pub struct GeoPoint3dRelHome {
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    pub relative_alt_m: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
/// WGS84 position with terrain-referenced altitude.
pub struct GeoPoint3dTerrain {
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    pub altitude_terrain_m: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
/// Tagged 3D point that preserves the altitude reference frame.
pub enum GeoPoint3d {
    Msl(GeoPoint3dMsl),
    RelHome(GeoPoint3dRelHome),
    Terrain(GeoPoint3dTerrain),
}

impl GeoPoint3d {
    pub fn msl(latitude_deg: f64, longitude_deg: f64, altitude_msl_m: f64) -> Self {
        Self::Msl(GeoPoint3dMsl {
            latitude_deg,
            longitude_deg,
            altitude_msl_m,
        })
    }

    pub fn rel_home(latitude_deg: f64, longitude_deg: f64, relative_alt_m: f64) -> Self {
        Self::RelHome(GeoPoint3dRelHome {
            latitude_deg,
            longitude_deg,
            relative_alt_m,
        })
    }

    pub fn terrain(latitude_deg: f64, longitude_deg: f64, altitude_terrain_m: f64) -> Self {
        Self::Terrain(GeoPoint3dTerrain {
            latitude_deg,
            longitude_deg,
            altitude_terrain_m,
        })
    }
}

impl From<GeoPoint3dMsl> for GeoPoint3d {
    fn from(value: GeoPoint3dMsl) -> Self {
        Self::Msl(value)
    }
}

impl From<GeoPoint3dRelHome> for GeoPoint3d {
    fn from(value: GeoPoint3dRelHome) -> Self {
        Self::RelHome(value)
    }
}

impl From<GeoPoint3dTerrain> for GeoPoint3d {
    fn from(value: GeoPoint3dTerrain) -> Self {
        Self::Terrain(value)
    }
}

impl From<GeoPoint3dMsl> for GeoPoint2d {
    fn from(value: GeoPoint3dMsl) -> Self {
        Self {
            latitude_deg: value.latitude_deg,
            longitude_deg: value.longitude_deg,
        }
    }
}

impl From<GeoPoint3dRelHome> for GeoPoint2d {
    fn from(value: GeoPoint3dRelHome) -> Self {
        Self {
            latitude_deg: value.latitude_deg,
            longitude_deg: value.longitude_deg,
        }
    }
}

impl From<GeoPoint3dTerrain> for GeoPoint2d {
    fn from(value: GeoPoint3dTerrain) -> Self {
        Self {
            latitude_deg: value.latitude_deg,
            longitude_deg: value.longitude_deg,
        }
    }
}

impl From<GeoPoint3d> for GeoPoint2d {
    fn from(value: GeoPoint3d) -> Self {
        match value {
            GeoPoint3d::Msl(p) => Self {
                latitude_deg: p.latitude_deg,
                longitude_deg: p.longitude_deg,
            },
            GeoPoint3d::RelHome(p) => Self {
                latitude_deg: p.latitude_deg,
                longitude_deg: p.longitude_deg,
            },
            GeoPoint3d::Terrain(p) => Self {
                latitude_deg: p.latitude_deg,
                longitude_deg: p.longitude_deg,
            },
        }
    }
}

pub(crate) fn quantize_degrees_e7(value: f64) -> i32 {
    debug_assert!(
        value.is_finite(),
        "quantize_degrees_e7: value must be finite, got {value}"
    );
    let scaled = value * 1e7;
    debug_assert!(
        (i32::MIN as f64..=i32::MAX as f64).contains(&scaled.round()),
        "quantize_degrees_e7: {value}° overflows i32 degE7 range"
    );
    scaled.round() as i32
}

/// Validate and quantize a latitude (±90°) to degE7.
pub(crate) fn try_latitude_e7(value: f64) -> Result<i32, VehicleError> {
    validate_coordinate(value, -90.0, 90.0, "latitude")
}

/// Validate and quantize a longitude (±180°) to degE7.
pub(crate) fn try_longitude_e7(value: f64) -> Result<i32, VehicleError> {
    validate_coordinate(value, -180.0, 180.0, "longitude")
}

fn validate_coordinate(
    value: f64,
    min_deg: f64,
    max_deg: f64,
    name: &str,
) -> Result<i32, VehicleError> {
    if !value.is_finite() {
        return Err(VehicleError::InvalidParameter(format!(
            "{name} must be finite, got {value}"
        )));
    }
    if !(min_deg..=max_deg).contains(&value) {
        return Err(VehicleError::InvalidParameter(format!(
            "{name} must be in [{min_deg}, {max_deg}], got {value}"
        )));
    }
    Ok(quantize_degrees_e7(value))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn geo_to_2d_projections() {
        let msl = GeoPoint3dMsl {
            latitude_deg: 47.397,
            longitude_deg: 8.545,
            altitude_msl_m: 500.0,
        };
        let rel_home = GeoPoint3dRelHome {
            latitude_deg: 47.398,
            longitude_deg: 8.546,
            relative_alt_m: 120.0,
        };
        let terrain = GeoPoint3dTerrain {
            latitude_deg: 47.399,
            longitude_deg: 8.547,
            altitude_terrain_m: 15.0,
        };

        assert_eq!(
            GeoPoint2d::from(msl.clone()),
            GeoPoint2d {
                latitude_deg: 47.397,
                longitude_deg: 8.545,
            }
        );
        assert_eq!(
            GeoPoint2d::from(rel_home.clone()),
            GeoPoint2d {
                latitude_deg: 47.398,
                longitude_deg: 8.546,
            }
        );
        assert_eq!(
            GeoPoint2d::from(terrain.clone()),
            GeoPoint2d {
                latitude_deg: 47.399,
                longitude_deg: 8.547,
            }
        );

        assert_eq!(
            GeoPoint2d::from(GeoPoint3d::Msl(msl)),
            GeoPoint2d {
                latitude_deg: 47.397,
                longitude_deg: 8.545,
            }
        );
        assert_eq!(
            GeoPoint2d::from(GeoPoint3d::RelHome(rel_home)),
            GeoPoint2d {
                latitude_deg: 47.398,
                longitude_deg: 8.546,
            }
        );
        assert_eq!(
            GeoPoint2d::from(GeoPoint3d::Terrain(terrain)),
            GeoPoint2d {
                latitude_deg: 47.399,
                longitude_deg: 8.547,
            }
        );
    }

    #[test]
    fn serde_roundtrip() {
        let msl = GeoPoint3dMsl {
            latitude_deg: 47.397742,
            longitude_deg: 8.545594,
            altitude_msl_m: 488.0,
        };
        let json = serde_json::to_string(&msl).unwrap();
        let restored: GeoPoint3dMsl = serde_json::from_str(&json).unwrap();
        assert_eq!(msl, restored);

        let point2d = GeoPoint2d {
            latitude_deg: -33.8688,
            longitude_deg: 151.2093,
        };
        let json = serde_json::to_string(&point2d).unwrap();
        let restored: GeoPoint2d = serde_json::from_str(&json).unwrap();
        assert_eq!(point2d, restored);

        let rel = GeoPoint3dRelHome {
            latitude_deg: 0.0,
            longitude_deg: 0.0,
            relative_alt_m: 100.0,
        };
        let json = serde_json::to_string(&rel).unwrap();
        let restored: GeoPoint3dRelHome = serde_json::from_str(&json).unwrap();
        assert_eq!(rel, restored);

        let terrain = GeoPoint3dTerrain {
            latitude_deg: 51.5074,
            longitude_deg: -0.1278,
            altitude_terrain_m: 15.0,
        };
        let json = serde_json::to_string(&terrain).unwrap();
        let restored: GeoPoint3dTerrain = serde_json::from_str(&json).unwrap();
        assert_eq!(terrain, restored);
    }

    #[test]
    fn try_latitude_e7_boundaries() {
        assert!(try_latitude_e7(90.0).is_ok());
        assert!(try_latitude_e7(-90.0).is_ok());
        assert!(try_latitude_e7(0.0).is_ok());
        assert!(try_latitude_e7(90.0001).is_err());
        assert!(try_latitude_e7(-90.0001).is_err());
        assert!(try_latitude_e7(f64::NAN).is_err());
        assert!(try_latitude_e7(f64::INFINITY).is_err());
        assert!(try_latitude_e7(f64::NEG_INFINITY).is_err());
    }

    #[test]
    fn try_longitude_e7_boundaries() {
        assert!(try_longitude_e7(180.0).is_ok());
        assert!(try_longitude_e7(-180.0).is_ok());
        assert!(try_longitude_e7(0.0).is_ok());
        assert!(try_longitude_e7(180.0001).is_err());
        assert!(try_longitude_e7(-180.0001).is_err());
        assert!(try_longitude_e7(f64::NAN).is_err());
        assert!(try_longitude_e7(f64::INFINITY).is_err());
        assert!(try_longitude_e7(f64::NEG_INFINITY).is_err());
    }

    #[test]
    fn try_latitude_e7_quantization() {
        assert_eq!(try_latitude_e7(90.0).unwrap(), 900_000_000);
        assert_eq!(try_latitude_e7(-90.0).unwrap(), -900_000_000);
        assert_eq!(try_latitude_e7(47.397742).unwrap(), 473_977_420);
    }
}
