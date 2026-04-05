use pyo3::exceptions::PyTypeError;
use pyo3::prelude::*;

use crate::geo::{
    PyGeoPoint2d, PyGeoPoint3dMsl, PyGeoPoint3dRelHome, PyGeoPoint3dTerrain, geo_point2d_from_py,
};

#[pyclass(name = "FenceInclusionPolygon", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyFenceInclusionPolygon {
    inner: mavkit::FenceInclusionPolygon,
}

impl PyFenceInclusionPolygon {
    pub(crate) fn from_inner(inner: mavkit::FenceInclusionPolygon) -> Self {
        Self { inner }
    }
}

#[pymethods]
impl PyFenceInclusionPolygon {
    #[new]
    #[pyo3(signature = (*, vertices, inclusion_group))]
    fn new(py: Python<'_>, vertices: Vec<Py<PyAny>>, inclusion_group: u8) -> PyResult<Self> {
        Ok(Self {
            inner: mavkit::FenceInclusionPolygon {
                vertices: vertices
                    .into_iter()
                    .map(|vertex| geo_point2d_from_py(vertex.bind(py)))
                    .collect::<PyResult<Vec<_>>>()?,
                inclusion_group,
            },
        })
    }

    #[getter]
    fn vertices(&self) -> Vec<PyGeoPoint2d> {
        self.inner
            .vertices
            .iter()
            .cloned()
            .map(PyGeoPoint2d::from_inner)
            .collect()
    }

    #[getter]
    fn inclusion_group(&self) -> u8 {
        self.inner.inclusion_group
    }
}

#[pyclass(name = "FenceExclusionPolygon", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyFenceExclusionPolygon {
    inner: mavkit::FenceExclusionPolygon,
}

impl PyFenceExclusionPolygon {
    pub(crate) fn from_inner(inner: mavkit::FenceExclusionPolygon) -> Self {
        Self { inner }
    }
}

#[pymethods]
impl PyFenceExclusionPolygon {
    #[new]
    #[pyo3(signature = (*, vertices))]
    fn new(py: Python<'_>, vertices: Vec<Py<PyAny>>) -> PyResult<Self> {
        Ok(Self {
            inner: mavkit::FenceExclusionPolygon {
                vertices: vertices
                    .into_iter()
                    .map(|vertex| geo_point2d_from_py(vertex.bind(py)))
                    .collect::<PyResult<Vec<_>>>()?,
            },
        })
    }

    #[getter]
    fn vertices(&self) -> Vec<PyGeoPoint2d> {
        self.inner
            .vertices
            .iter()
            .cloned()
            .map(PyGeoPoint2d::from_inner)
            .collect()
    }
}

#[pyclass(name = "FenceInclusionCircle", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyFenceInclusionCircle {
    inner: mavkit::FenceInclusionCircle,
}

impl PyFenceInclusionCircle {
    pub(crate) fn from_inner(inner: mavkit::FenceInclusionCircle) -> Self {
        Self { inner }
    }
}

#[pymethods]
impl PyFenceInclusionCircle {
    #[new]
    #[pyo3(signature = (*, center, radius_m, inclusion_group))]
    fn new(
        py: Python<'_>,
        center: Py<PyAny>,
        radius_m: f32,
        inclusion_group: u8,
    ) -> PyResult<Self> {
        Ok(Self {
            inner: mavkit::FenceInclusionCircle {
                center: geo_point2d_from_py(center.bind(py))?,
                radius_m,
                inclusion_group,
            },
        })
    }

    #[getter]
    fn center(&self) -> PyGeoPoint2d {
        PyGeoPoint2d::from_inner(self.inner.center.clone())
    }

    #[getter]
    fn radius_m(&self) -> f32 {
        self.inner.radius_m
    }

    #[getter]
    fn inclusion_group(&self) -> u8 {
        self.inner.inclusion_group
    }
}

#[pyclass(name = "FenceExclusionCircle", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyFenceExclusionCircle {
    inner: mavkit::FenceExclusionCircle,
}

impl PyFenceExclusionCircle {
    pub(crate) fn from_inner(inner: mavkit::FenceExclusionCircle) -> Self {
        Self { inner }
    }
}

#[pymethods]
impl PyFenceExclusionCircle {
    #[new]
    #[pyo3(signature = (*, center, radius_m))]
    fn new(py: Python<'_>, center: Py<PyAny>, radius_m: f32) -> PyResult<Self> {
        Ok(Self {
            inner: mavkit::FenceExclusionCircle {
                center: geo_point2d_from_py(center.bind(py))?,
                radius_m,
            },
        })
    }

    #[getter]
    fn center(&self) -> PyGeoPoint2d {
        PyGeoPoint2d::from_inner(self.inner.center.clone())
    }

    #[getter]
    fn radius_m(&self) -> f32 {
        self.inner.radius_m
    }
}

fn fence_region_from_py(region: &Bound<'_, PyAny>) -> PyResult<mavkit::FenceRegion> {
    if let Ok(region) = region.extract::<PyRef<'_, PyFenceInclusionPolygon>>() {
        return Ok(region.inner.clone().into());
    }
    if let Ok(region) = region.extract::<PyRef<'_, PyFenceExclusionPolygon>>() {
        return Ok(region.inner.clone().into());
    }
    if let Ok(region) = region.extract::<PyRef<'_, PyFenceInclusionCircle>>() {
        return Ok(region.inner.clone().into());
    }
    if let Ok(region) = region.extract::<PyRef<'_, PyFenceExclusionCircle>>() {
        return Ok(region.inner.clone().into());
    }
    Err(PyTypeError::new_err(
        "expected FenceInclusionPolygon, FenceExclusionPolygon, FenceInclusionCircle, or FenceExclusionCircle",
    ))
}

fn fence_region_into_py(py: Python<'_>, region: mavkit::FenceRegion) -> PyResult<Py<PyAny>> {
    match region {
        mavkit::FenceRegion::InclusionPolygon(region) => {
            Ok(Py::new(py, PyFenceInclusionPolygon::from_inner(region))?.into_any())
        }
        mavkit::FenceRegion::ExclusionPolygon(region) => {
            Ok(Py::new(py, PyFenceExclusionPolygon::from_inner(region))?.into_any())
        }
        mavkit::FenceRegion::InclusionCircle(region) => {
            Ok(Py::new(py, PyFenceInclusionCircle::from_inner(region))?.into_any())
        }
        mavkit::FenceRegion::ExclusionCircle(region) => {
            Ok(Py::new(py, PyFenceExclusionCircle::from_inner(region))?.into_any())
        }
    }
}

#[pyclass(name = "FencePlan", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyFencePlan {
    pub(crate) inner: mavkit::FencePlan,
}

impl PyFencePlan {
    pub(crate) fn from_inner(inner: mavkit::FencePlan) -> Self {
        Self { inner }
    }
}

#[pymethods]
impl PyFencePlan {
    #[new]
    #[pyo3(signature = (*, return_point=None, regions))]
    fn new(
        py: Python<'_>,
        return_point: Option<Py<PyAny>>,
        regions: Vec<Py<PyAny>>,
    ) -> PyResult<Self> {
        let regions = regions
            .into_iter()
            .map(|region| fence_region_from_py(region.bind(py)))
            .collect::<PyResult<Vec<_>>>()?;
        Ok(Self {
            inner: mavkit::FencePlan {
                return_point: return_point
                    .map(|point| geo_point2d_from_py(point.bind(py)))
                    .transpose()?,
                regions,
            },
        })
    }

    #[getter]
    fn return_point(&self) -> Option<PyGeoPoint2d> {
        self.inner
            .return_point
            .clone()
            .map(PyGeoPoint2d::from_inner)
    }

    #[getter]
    fn regions(&self, py: Python<'_>) -> PyResult<Vec<Py<PyAny>>> {
        self.inner
            .regions
            .iter()
            .cloned()
            .map(|region| fence_region_into_py(py, region))
            .collect()
    }
}

fn rally_point_from_py(point: &Bound<'_, PyAny>) -> PyResult<mavkit::GeoPoint3d> {
    if let Ok(point) = point.extract::<PyRef<'_, PyGeoPoint3dMsl>>() {
        return Ok(mavkit::GeoPoint3d::Msl(point.to_inner()));
    }
    if let Ok(point) = point.extract::<PyRef<'_, PyGeoPoint3dRelHome>>() {
        return Ok(mavkit::GeoPoint3d::RelHome(point.to_inner()));
    }
    if let Ok(point) = point.extract::<PyRef<'_, PyGeoPoint3dTerrain>>() {
        return Ok(mavkit::GeoPoint3d::Terrain(point.to_inner()));
    }
    Err(PyTypeError::new_err(
        "expected GeoPoint3dMsl, GeoPoint3dRelHome, or GeoPoint3dTerrain",
    ))
}

fn rally_point_into_py(py: Python<'_>, point: mavkit::GeoPoint3d) -> PyResult<Py<PyAny>> {
    match point {
        mavkit::GeoPoint3d::Msl(point) => Ok(Py::new(py, PyGeoPoint3dMsl::from(point))?.into_any()),
        mavkit::GeoPoint3d::RelHome(point) => {
            Ok(Py::new(py, PyGeoPoint3dRelHome::from_inner(point))?.into_any())
        }
        mavkit::GeoPoint3d::Terrain(point) => {
            Ok(Py::new(py, PyGeoPoint3dTerrain::from_inner(point))?.into_any())
        }
    }
}

#[pyclass(name = "RallyPlan", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyRallyPlan {
    pub(crate) inner: mavkit::RallyPlan,
}

impl PyRallyPlan {
    pub(crate) fn from_inner(inner: mavkit::RallyPlan) -> Self {
        Self { inner }
    }
}

#[pymethods]
impl PyRallyPlan {
    #[new]
    #[pyo3(signature = (*, points))]
    fn new(py: Python<'_>, points: Vec<Py<PyAny>>) -> PyResult<Self> {
        Ok(Self {
            inner: mavkit::RallyPlan {
                points: points
                    .into_iter()
                    .map(|point| rally_point_from_py(point.bind(py)))
                    .collect::<PyResult<Vec<_>>>()?,
            },
        })
    }

    #[getter]
    fn points(&self, py: Python<'_>) -> PyResult<Vec<Py<PyAny>>> {
        self.inner
            .points
            .iter()
            .cloned()
            .map(|point| rally_point_into_py(py, point))
            .collect()
    }
}
