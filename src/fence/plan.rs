use crate::geo::GeoPoint2d;

/// Full geofence plan with optional return point and region list.
#[cfg_attr(feature = "typescript", derive(specta::Type))]
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct FencePlan {
    pub return_point: Option<GeoPoint2d>,
    pub regions: Vec<FenceRegion>,
}

/// Polygon region that marks space where operation is allowed.
#[cfg_attr(feature = "typescript", derive(specta::Type))]
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct FenceInclusionPolygon {
    pub vertices: Vec<GeoPoint2d>,
    pub inclusion_group: u8,
}

/// Polygon region that marks space where operation is forbidden.
#[cfg_attr(feature = "typescript", derive(specta::Type))]
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct FenceExclusionPolygon {
    pub vertices: Vec<GeoPoint2d>,
}

/// Circular inclusion region.
#[cfg_attr(feature = "typescript", derive(specta::Type))]
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct FenceInclusionCircle {
    pub center: GeoPoint2d,
    pub radius_m: f32,
    pub inclusion_group: u8,
}

/// Circular exclusion region.
#[cfg_attr(feature = "typescript", derive(specta::Type))]
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct FenceExclusionCircle {
    pub center: GeoPoint2d,
    pub radius_m: f32,
}

/// Geofence region union used in upload and download plans.
#[cfg_attr(feature = "typescript", derive(specta::Type))]
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
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
