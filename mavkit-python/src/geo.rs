use pyo3::exceptions::PyTypeError;
use pyo3::prelude::*;

use crate::enums::PyMissionFrame;

// ---------------------------------------------------------------------------
// GeoPoint2d
// ---------------------------------------------------------------------------

#[pyclass(name = "GeoPoint2d", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyGeoPoint2d {
    inner: mavkit::GeoPoint2d,
}

impl PyGeoPoint2d {
    pub(crate) fn from_inner(inner: mavkit::GeoPoint2d) -> Self {
        Self { inner }
    }

    pub(crate) fn into_inner(&self) -> mavkit::GeoPoint2d {
        self.inner.clone()
    }
}

/// Extract a `mavkit::GeoPoint2d` from an arbitrary Python object.
///
/// Returns a `TypeError` if the object is not a `GeoPoint2d`.
pub(crate) fn geo_point2d_from_py(point: &Bound<'_, PyAny>) -> PyResult<mavkit::GeoPoint2d> {
    if let Ok(point) = point.extract::<PyRef<'_, PyGeoPoint2d>>() {
        Ok(point.into_inner())
    } else {
        Err(PyTypeError::new_err("expected GeoPoint2d"))
    }
}

#[pymethods]
impl PyGeoPoint2d {
    #[new]
    #[pyo3(signature = (*, latitude_deg, longitude_deg))]
    fn new(latitude_deg: f64, longitude_deg: f64) -> Self {
        Self {
            inner: mavkit::GeoPoint2d {
                latitude_deg,
                longitude_deg,
            },
        }
    }

    #[getter]
    fn latitude_deg(&self) -> f64 {
        self.inner.latitude_deg
    }

    #[getter]
    fn longitude_deg(&self) -> f64 {
        self.inner.longitude_deg
    }
}

// ---------------------------------------------------------------------------
// GeoPoint3dMsl
// ---------------------------------------------------------------------------

#[pyclass(name = "GeoPoint3dMsl", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyGeoPoint3dMsl {
    latitude_deg: f64,
    longitude_deg: f64,
    altitude_msl_m: f64,
}

impl From<mavkit::GeoPoint3dMsl> for PyGeoPoint3dMsl {
    fn from(value: mavkit::GeoPoint3dMsl) -> Self {
        Self {
            latitude_deg: value.latitude_deg,
            longitude_deg: value.longitude_deg,
            altitude_msl_m: value.altitude_msl_m,
        }
    }
}

impl PyGeoPoint3dMsl {
    pub(crate) fn into_inner(&self) -> mavkit::GeoPoint3dMsl {
        mavkit::GeoPoint3dMsl {
            latitude_deg: self.latitude_deg,
            longitude_deg: self.longitude_deg,
            altitude_msl_m: self.altitude_msl_m,
        }
    }
}

#[pymethods]
impl PyGeoPoint3dMsl {
    #[new]
    #[pyo3(signature = (*, latitude_deg, longitude_deg, altitude_msl_m))]
    fn new(latitude_deg: f64, longitude_deg: f64, altitude_msl_m: f64) -> Self {
        Self {
            latitude_deg,
            longitude_deg,
            altitude_msl_m,
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
}

// ---------------------------------------------------------------------------
// GeoPoint3dRelHome
// ---------------------------------------------------------------------------

#[pyclass(name = "GeoPoint3dRelHome", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyGeoPoint3dRelHome {
    inner: mavkit::GeoPoint3dRelHome,
}

impl PyGeoPoint3dRelHome {
    pub(crate) fn from_inner(inner: mavkit::GeoPoint3dRelHome) -> Self {
        Self { inner }
    }

    pub(crate) fn into_inner(&self) -> mavkit::GeoPoint3dRelHome {
        self.inner.clone()
    }
}

#[pymethods]
impl PyGeoPoint3dRelHome {
    #[new]
    #[pyo3(signature = (*, latitude_deg, longitude_deg, relative_alt_m))]
    fn new(latitude_deg: f64, longitude_deg: f64, relative_alt_m: f64) -> Self {
        Self {
            inner: mavkit::GeoPoint3dRelHome {
                latitude_deg,
                longitude_deg,
                relative_alt_m,
            },
        }
    }

    #[getter]
    fn latitude_deg(&self) -> f64 {
        self.inner.latitude_deg
    }

    #[getter]
    fn longitude_deg(&self) -> f64 {
        self.inner.longitude_deg
    }

    #[getter]
    fn relative_alt_m(&self) -> f64 {
        self.inner.relative_alt_m
    }
}

// ---------------------------------------------------------------------------
// GeoPoint3dTerrain
// ---------------------------------------------------------------------------

#[pyclass(name = "GeoPoint3dTerrain", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyGeoPoint3dTerrain {
    inner: mavkit::GeoPoint3dTerrain,
}

impl PyGeoPoint3dTerrain {
    pub(crate) fn from_inner(inner: mavkit::GeoPoint3dTerrain) -> Self {
        Self { inner }
    }

    pub(crate) fn into_inner(&self) -> mavkit::GeoPoint3dTerrain {
        self.inner.clone()
    }
}

#[pymethods]
impl PyGeoPoint3dTerrain {
    #[new]
    #[pyo3(signature = (*, latitude_deg, longitude_deg, altitude_terrain_m))]
    fn new(latitude_deg: f64, longitude_deg: f64, altitude_terrain_m: f64) -> Self {
        Self {
            inner: mavkit::GeoPoint3dTerrain {
                latitude_deg,
                longitude_deg,
                altitude_terrain_m,
            },
        }
    }

    #[getter]
    fn latitude_deg(&self) -> f64 {
        self.inner.latitude_deg
    }

    #[getter]
    fn longitude_deg(&self) -> f64 {
        self.inner.longitude_deg
    }

    #[getter]
    fn altitude_terrain_m(&self) -> f64 {
        self.inner.altitude_terrain_m
    }
}

// ---------------------------------------------------------------------------
// GeoPoint3d (union / discriminated wrapper used by mission items)
// ---------------------------------------------------------------------------

/// Decompose a `GeoPoint3d` into its Python frame enum and raw coordinate
/// components. Used by mission command wrappers to expose `frame`,
/// `latitude_deg`, `longitude_deg`, and `altitude_m` getters uniformly.
pub(crate) fn position_components(
    position: &mavkit::GeoPoint3d,
) -> (PyMissionFrame, f64, f64, f32) {
    match position {
        mavkit::GeoPoint3d::Msl(point) => (
            PyMissionFrame::GlobalInt,
            point.latitude_deg,
            point.longitude_deg,
            point.altitude_msl_m as f32,
        ),
        mavkit::GeoPoint3d::RelHome(point) => (
            PyMissionFrame::GlobalRelativeAltInt,
            point.latitude_deg,
            point.longitude_deg,
            point.relative_alt_m as f32,
        ),
        mavkit::GeoPoint3d::Terrain(point) => (
            PyMissionFrame::GlobalTerrainAltInt,
            point.latitude_deg,
            point.longitude_deg,
            point.altitude_terrain_m as f32,
        ),
    }
}

#[pyclass(name = "GeoPoint3d", frozen, from_py_object)]
#[derive(Clone)]
pub struct PyGeoPoint3d {
    pub(crate) inner: mavkit::GeoPoint3d,
}

#[pymethods]
impl PyGeoPoint3d {
    #[staticmethod]
    #[pyo3(signature = (*, latitude_deg, longitude_deg, altitude_msl_m))]
    fn msl(latitude_deg: f64, longitude_deg: f64, altitude_msl_m: f64) -> Self {
        Self {
            inner: mavkit::GeoPoint3d::Msl(mavkit::GeoPoint3dMsl {
                latitude_deg,
                longitude_deg,
                altitude_msl_m,
            }),
        }
    }

    #[staticmethod]
    #[pyo3(signature = (*, latitude_deg, longitude_deg, relative_alt_m))]
    fn rel_home(latitude_deg: f64, longitude_deg: f64, relative_alt_m: f64) -> Self {
        Self {
            inner: mavkit::GeoPoint3d::RelHome(mavkit::GeoPoint3dRelHome {
                latitude_deg,
                longitude_deg,
                relative_alt_m,
            }),
        }
    }

    #[staticmethod]
    #[pyo3(signature = (*, latitude_deg, longitude_deg, altitude_terrain_m))]
    fn terrain(latitude_deg: f64, longitude_deg: f64, altitude_terrain_m: f64) -> Self {
        Self {
            inner: mavkit::GeoPoint3d::Terrain(mavkit::GeoPoint3dTerrain {
                latitude_deg,
                longitude_deg,
                altitude_terrain_m,
            }),
        }
    }

    #[getter]
    fn frame(&self) -> PyMissionFrame {
        position_components(&self.inner).0
    }

    #[getter]
    fn latitude_deg(&self) -> f64 {
        position_components(&self.inner).1
    }

    #[getter]
    fn longitude_deg(&self) -> f64 {
        position_components(&self.inner).2
    }

    #[getter]
    fn altitude_m(&self) -> f32 {
        position_components(&self.inner).3
    }
}
