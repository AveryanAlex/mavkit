use pyo3::prelude::*;

use crate::enums::PyGpsFixType;

pub(super) fn convert_gps_fix_type(value: mavkit::GpsFixType) -> PyGpsFixType {
    match value {
        mavkit::GpsFixType::NoFix => PyGpsFixType::NoFix,
        mavkit::GpsFixType::Fix2d => PyGpsFixType::Fix2d,
        mavkit::GpsFixType::Fix3d => PyGpsFixType::Fix3d,
        mavkit::GpsFixType::Dgps => PyGpsFixType::Dgps,
        mavkit::GpsFixType::RtkFloat => PyGpsFixType::RtkFloat,
        mavkit::GpsFixType::RtkFixed => PyGpsFixType::RtkFixed,
    }
}

#[pyclass(name = "GlobalPosition", frozen, skip_from_py_object)]
#[derive(Clone)]
pub(crate) struct PyGlobalPosition {
    latitude_deg: f64,
    longitude_deg: f64,
    altitude_msl_m: f64,
    relative_alt_m: f64,
}

impl From<mavkit::GlobalPosition> for PyGlobalPosition {
    fn from(value: mavkit::GlobalPosition) -> Self {
        Self {
            latitude_deg: value.latitude_deg,
            longitude_deg: value.longitude_deg,
            altitude_msl_m: value.altitude_msl_m,
            relative_alt_m: value.relative_alt_m,
        }
    }
}

#[pymethods]
impl PyGlobalPosition {
    #[new]
    #[pyo3(signature = (*, latitude_deg, longitude_deg, altitude_msl_m, relative_alt_m))]
    fn new(
        latitude_deg: f64,
        longitude_deg: f64,
        altitude_msl_m: f64,
        relative_alt_m: f64,
    ) -> Self {
        Self {
            latitude_deg,
            longitude_deg,
            altitude_msl_m,
            relative_alt_m,
        }
    }

    #[getter]
    fn latitude_deg(&self) -> f64 {
        self.latitude_deg
    }

    #[getter]
    fn longitude_deg(&self) -> f64 {
        self.longitude_deg
    }

    #[getter]
    fn altitude_msl_m(&self) -> f64 {
        self.altitude_msl_m
    }

    #[getter]
    fn relative_alt_m(&self) -> f64 {
        self.relative_alt_m
    }
}

#[pyclass(name = "EulerAttitude", frozen, skip_from_py_object)]
#[derive(Clone)]
pub(crate) struct PyEulerAttitude {
    roll_deg: f64,
    pitch_deg: f64,
    yaw_deg: f64,
}

impl From<mavkit::EulerAttitude> for PyEulerAttitude {
    fn from(value: mavkit::EulerAttitude) -> Self {
        Self {
            roll_deg: value.roll_deg,
            pitch_deg: value.pitch_deg,
            yaw_deg: value.yaw_deg,
        }
    }
}

#[pymethods]
impl PyEulerAttitude {
    #[new]
    fn new(roll_deg: f64, pitch_deg: f64, yaw_deg: f64) -> Self {
        Self {
            roll_deg,
            pitch_deg,
            yaw_deg,
        }
    }

    #[getter]
    fn roll_deg(&self) -> f64 {
        self.roll_deg
    }

    #[getter]
    fn pitch_deg(&self) -> f64 {
        self.pitch_deg
    }

    #[getter]
    fn yaw_deg(&self) -> f64 {
        self.yaw_deg
    }
}

#[pyclass(name = "GpsQuality", frozen, skip_from_py_object)]
#[derive(Clone)]
pub(crate) struct PyGpsQuality {
    fix_type: PyGpsFixType,
    satellites: Option<u8>,
    hdop: Option<f64>,
}

impl From<mavkit::GpsQuality> for PyGpsQuality {
    fn from(value: mavkit::GpsQuality) -> Self {
        let mavkit::GpsQuality {
            fix_type,
            satellites,
            hdop,
        } = value;
        Self {
            fix_type: convert_gps_fix_type(fix_type),
            satellites,
            hdop,
        }
    }
}

#[pymethods]
impl PyGpsQuality {
    #[new]
    fn new(fix_type: PyGpsFixType, satellites: Option<u8>, hdop: Option<f64>) -> Self {
        Self {
            fix_type,
            satellites,
            hdop,
        }
    }

    #[getter]
    fn fix_type(&self) -> PyGpsFixType {
        self.fix_type
    }

    #[getter]
    fn satellites(&self) -> Option<u8> {
        self.satellites
    }

    #[getter]
    fn hdop(&self) -> Option<f64> {
        self.hdop
    }
}

#[pyclass(name = "CellVoltages", frozen, skip_from_py_object)]
#[derive(Clone)]
pub(crate) struct PyCellVoltages {
    voltages_v: Vec<f64>,
}

impl From<mavkit::CellVoltages> for PyCellVoltages {
    fn from(value: mavkit::CellVoltages) -> Self {
        Self {
            voltages_v: value.voltages_v,
        }
    }
}

#[pymethods]
impl PyCellVoltages {
    #[new]
    fn new(voltages_v: Vec<f64>) -> Self {
        Self { voltages_v }
    }

    #[getter]
    fn voltages_v(&self) -> Vec<f64> {
        self.voltages_v.clone()
    }
}

#[pyclass(name = "WaypointProgress", frozen, skip_from_py_object)]
#[derive(Clone)]
pub(crate) struct PyWaypointProgress {
    distance_m: f64,
    bearing_deg: f64,
}

impl From<mavkit::WaypointProgress> for PyWaypointProgress {
    fn from(value: mavkit::WaypointProgress) -> Self {
        Self {
            distance_m: value.distance_m,
            bearing_deg: value.bearing_deg,
        }
    }
}

#[pymethods]
impl PyWaypointProgress {
    #[new]
    fn new(distance_m: f64, bearing_deg: f64) -> Self {
        Self {
            distance_m,
            bearing_deg,
        }
    }

    #[getter]
    fn distance_m(&self) -> f64 {
        self.distance_m
    }

    #[getter]
    fn bearing_deg(&self) -> f64 {
        self.bearing_deg
    }
}

#[pyclass(name = "GuidanceState", frozen, skip_from_py_object)]
#[derive(Clone)]
pub(crate) struct PyGuidanceState {
    bearing_deg: f64,
    cross_track_error_m: f64,
}

impl From<mavkit::GuidanceState> for PyGuidanceState {
    fn from(value: mavkit::GuidanceState) -> Self {
        Self {
            bearing_deg: value.bearing_deg,
            cross_track_error_m: value.cross_track_error_m,
        }
    }
}

#[pymethods]
impl PyGuidanceState {
    #[new]
    fn new(bearing_deg: f64, cross_track_error_m: f64) -> Self {
        Self {
            bearing_deg,
            cross_track_error_m,
        }
    }

    #[getter]
    fn bearing_deg(&self) -> f64 {
        self.bearing_deg
    }

    #[getter]
    fn cross_track_error_m(&self) -> f64 {
        self.cross_track_error_m
    }
}

#[pyclass(name = "TerrainClearance", frozen, skip_from_py_object)]
#[derive(Clone)]
pub(crate) struct PyTerrainClearance {
    terrain_height_m: f64,
    height_above_terrain_m: f64,
}

impl From<mavkit::TerrainClearance> for PyTerrainClearance {
    fn from(value: mavkit::TerrainClearance) -> Self {
        Self {
            terrain_height_m: value.terrain_height_m,
            height_above_terrain_m: value.height_above_terrain_m,
        }
    }
}

#[pymethods]
impl PyTerrainClearance {
    #[new]
    fn new(terrain_height_m: f64, height_above_terrain_m: f64) -> Self {
        Self {
            terrain_height_m,
            height_above_terrain_m,
        }
    }

    #[getter]
    fn terrain_height_m(&self) -> f64 {
        self.terrain_height_m
    }

    #[getter]
    fn height_above_terrain_m(&self) -> f64 {
        self.height_above_terrain_m
    }
}
