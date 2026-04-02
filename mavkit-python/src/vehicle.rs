#[cfg(feature = "test-support")]
use mavkit::dialect;
#[cfg(feature = "test-support")]
use mavlink::{
    AsyncMavConnection, MAVLinkMessageRaw, MAVLinkV2MessageRaw, MavHeader, MavlinkVersion,
};
use pyo3::exceptions::PyTypeError;
#[cfg(feature = "test-support")]
use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use std::sync::Arc;
#[cfg(feature = "test-support")]
use std::sync::Mutex;
#[cfg(feature = "test-support")]
use std::time::Duration;
#[cfg(feature = "test-support")]
use tokio::sync::mpsc;
#[cfg(feature = "test-support")]
use tokio::time::timeout;

use crate::ardupilot::PyArduPilotHandle;
use crate::config::PyVehicleConfig;
use crate::enums::{PyAutopilotType, PyVehicleType};
use crate::error::{duration_from_secs, to_py_err};
use crate::geo::{
    PyGeoPoint2d, PyGeoPoint3dMsl, PyGeoPoint3dRelHome, PyGeoPoint3dTerrain, geo_point2d_from_py,
};
use crate::info::PyInfoHandle;
use crate::link::PyLinkHandle;
use crate::macros::py_subscription;
use crate::mission::PyMissionPlan;
use crate::modes::{PyCurrentModeHandle, PyModesHandle};
use crate::params::{PyParamsHandle, PySyncState};
use crate::raw_message::{PyRawMessage, PyRawMessageStream};
use crate::support::{PySupportHandle, PySupportStateHandle};
use crate::telemetry::{PyMetricHandle, PyTelemetryHandle};

fn geo_point_msl(
    latitude_deg: f64,
    longitude_deg: f64,
    altitude_msl_m: f64,
) -> mavkit::GeoPoint3dMsl {
    mavkit::GeoPoint3dMsl {
        latitude_deg,
        longitude_deg,
        altitude_msl_m,
    }
}

pub(crate) fn vehicle_label(vehicle: &mavkit::Vehicle) -> String {
    let identity = vehicle.identity();
    format!("sys={}, comp={}", identity.system_id, identity.component_id)
}

fn mission_progress_phase_name(progress: &mavkit::MissionOperationProgress) -> &'static str {
    match progress {
        mavkit::MissionOperationProgress::RequestCount => "request_count",
        mavkit::MissionOperationProgress::SendingItem { .. } => "sending_item",
        mavkit::MissionOperationProgress::ReceivingItem { .. } => "receiving_item",
        mavkit::MissionOperationProgress::AwaitingAck => "awaiting_ack",
        mavkit::MissionOperationProgress::Verifying => "verifying",
        mavkit::MissionOperationProgress::Completed => "completed",
        mavkit::MissionOperationProgress::Failed => "failed",
        mavkit::MissionOperationProgress::Cancelled => "cancelled",
    }
}

fn stored_plan_operation_name(kind: mavkit::StoredPlanOperationKind) -> &'static str {
    match kind {
        mavkit::StoredPlanOperationKind::Upload => "upload",
        mavkit::StoredPlanOperationKind::Download => "download",
        mavkit::StoredPlanOperationKind::Clear => "clear",
    }
}

#[pyclass(name = "FenceInclusionPolygon", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyFenceInclusionPolygon {
    inner: mavkit::FenceInclusionPolygon,
}

impl PyFenceInclusionPolygon {
    fn from_inner(inner: mavkit::FenceInclusionPolygon) -> Self {
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
    fn from_inner(inner: mavkit::FenceExclusionPolygon) -> Self {
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
    fn from_inner(inner: mavkit::FenceInclusionCircle) -> Self {
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
    fn from_inner(inner: mavkit::FenceExclusionCircle) -> Self {
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
    inner: mavkit::FencePlan,
}

impl PyFencePlan {
    fn from_inner(inner: mavkit::FencePlan) -> Self {
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
        return Ok(mavkit::GeoPoint3d::Msl(point.into_inner()));
    }
    if let Ok(point) = point.extract::<PyRef<'_, PyGeoPoint3dRelHome>>() {
        return Ok(mavkit::GeoPoint3d::RelHome(point.into_inner()));
    }
    if let Ok(point) = point.extract::<PyRef<'_, PyGeoPoint3dTerrain>>() {
        return Ok(mavkit::GeoPoint3d::Terrain(point.into_inner()));
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
    inner: mavkit::RallyPlan,
}

impl PyRallyPlan {
    fn from_inner(inner: mavkit::RallyPlan) -> Self {
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

#[pyclass(name = "FenceState", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyFenceState {
    inner: mavkit::FenceState,
}

impl From<mavkit::FenceState> for PyFenceState {
    fn from(inner: mavkit::FenceState) -> Self {
        Self { inner }
    }
}

#[pymethods]
impl PyFenceState {
    #[getter]
    fn plan(&self) -> Option<PyFencePlan> {
        self.inner.plan.clone().map(PyFencePlan::from_inner)
    }

    #[getter]
    fn sync(&self) -> PySyncState {
        self.inner.sync.into()
    }

    #[getter]
    fn active_op(&self) -> Option<String> {
        self.inner
            .active_op
            .map(|op| stored_plan_operation_name(op).to_string())
    }
}

#[pyclass(name = "RallyState", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyRallyState {
    inner: mavkit::RallyState,
}

impl From<mavkit::RallyState> for PyRallyState {
    fn from(inner: mavkit::RallyState) -> Self {
        Self { inner }
    }
}

#[pymethods]
impl PyRallyState {
    #[getter]
    fn plan(&self) -> Option<PyRallyPlan> {
        self.inner.plan.clone().map(PyRallyPlan::from_inner)
    }

    #[getter]
    fn sync(&self) -> PySyncState {
        self.inner.sync.into()
    }

    #[getter]
    fn active_op(&self) -> Option<String> {
        self.inner
            .active_op
            .map(|op| stored_plan_operation_name(op).to_string())
    }
}

#[pyclass(name = "MissionState", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyMissionState {
    pub(crate) inner: mavkit::mission::MissionState,
}

impl From<mavkit::mission::MissionState> for PyMissionState {
    fn from(inner: mavkit::mission::MissionState) -> Self {
        Self { inner }
    }
}

#[pymethods]
impl PyMissionState {
    #[getter]
    fn plan(&self) -> Option<PyMissionPlan> {
        self.inner.plan.clone().map(|inner| PyMissionPlan { inner })
    }

    #[getter]
    fn current_index(&self) -> Option<u16> {
        self.inner.current_index
    }

    #[getter]
    fn current_seq(&self) -> Option<u16> {
        self.current_index()
    }

    #[getter]
    fn total_items(&self) -> u16 {
        self.inner
            .plan
            .as_ref()
            .map_or(0, |plan| plan.items.len() as u16)
    }

    #[getter]
    fn sync(&self) -> PySyncState {
        self.inner.sync.into()
    }

    #[getter]
    fn active_op(&self) -> Option<String> {
        self.inner.active_op.map(|op| {
            match op {
                mavkit::MissionOperationKind::Upload => "upload",
                mavkit::MissionOperationKind::Download => "download",
                mavkit::MissionOperationKind::Clear => "clear",
                mavkit::MissionOperationKind::Verify => "verify",
            }
            .to_string()
        })
    }

    fn __repr__(&self) -> String {
        format!(
            "MissionState(current_index={:?}, sync={:?}, active_op={:?}, has_plan={})",
            self.inner.current_index,
            self.inner.sync,
            self.inner.active_op,
            self.inner.plan.is_some()
        )
    }
}

#[pyclass(name = "VehicleIdentity", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyVehicleIdentity {
    pub(crate) inner: mavkit::VehicleIdentity,
}

#[pymethods]
impl PyVehicleIdentity {
    #[getter]
    fn system_id(&self) -> u8 {
        self.inner.system_id
    }

    #[getter]
    fn component_id(&self) -> u8 {
        self.inner.component_id
    }

    #[getter]
    fn autopilot(&self) -> PyAutopilotType {
        self.inner.autopilot.into()
    }

    #[getter]
    fn vehicle_type(&self) -> PyVehicleType {
        self.inner.vehicle_type.into()
    }

    fn __repr__(&self) -> String {
        format!(
            "VehicleIdentity(sys={}, comp={}, autopilot={:?}, type={:?})",
            self.inner.system_id,
            self.inner.component_id,
            self.inner.autopilot,
            self.inner.vehicle_type
        )
    }
}

py_subscription!(
    PyMissionStateSubscription,
    mavkit::mission::MissionState,
    PyMissionState,
    "MissionStateSubscription",
    "mission-state subscription closed"
);

py_subscription!(
    PyFenceStateSubscription,
    mavkit::FenceState,
    PyFenceState,
    "FenceStateSubscription",
    "fence-state subscription closed"
);

py_subscription!(
    PyRallyStateSubscription,
    mavkit::RallyState,
    PyRallyState,
    "RallyStateSubscription",
    "rally-state subscription closed"
);

#[pyclass(name = "MissionOperationProgress", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyMissionOperationProgress {
    inner: mavkit::MissionOperationProgress,
}

impl From<mavkit::MissionOperationProgress> for PyMissionOperationProgress {
    fn from(inner: mavkit::MissionOperationProgress) -> Self {
        Self { inner }
    }
}

#[pymethods]
impl PyMissionOperationProgress {
    #[getter]
    fn phase(&self) -> &str {
        mission_progress_phase_name(&self.inner)
    }

    #[getter]
    fn current(&self) -> u16 {
        match self.inner {
            mavkit::MissionOperationProgress::SendingItem { current, .. }
            | mavkit::MissionOperationProgress::ReceivingItem { current, .. } => current,
            _ => 0,
        }
    }

    #[getter]
    fn total(&self) -> u16 {
        match self.inner {
            mavkit::MissionOperationProgress::SendingItem { total, .. }
            | mavkit::MissionOperationProgress::ReceivingItem { total, .. } => total,
            _ => 0,
        }
    }
}

py_subscription!(
    PyMissionProgressSubscription,
    mavkit::MissionOperationProgress,
    PyMissionOperationProgress,
    "MissionProgressSubscription",
    "mission-progress subscription closed"
);

macro_rules! define_progress_op {
    ($rust_name:ident, $py_name:literal, $inner:ty, $map:expr) => {
        #[pyclass(name = $py_name, frozen, skip_from_py_object)]
        #[derive(Clone)]
        pub struct $rust_name {
            inner: Arc<$inner>,
        }

        #[pymethods]
        impl $rust_name {
            fn latest(&self) -> Option<PyMissionOperationProgress> {
                self.inner
                    .latest()
                    .map(|inner| PyMissionOperationProgress { inner })
            }

            fn subscribe(&self) -> PyMissionProgressSubscription {
                PyMissionProgressSubscription {
                    inner: Arc::new(tokio::sync::Mutex::new(self.inner.subscribe())),
                }
            }

            fn cancel(&self) {
                self.inner.cancel();
            }

            fn wait<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
                let inner = self.inner.clone();
                let map = $map;
                pyo3_async_runtimes::tokio::future_into_py(py, async move {
                    let value = inner.wait().await.map_err(to_py_err)?;
                    map(value)
                })
            }

            fn wait_timeout<'py>(
                &self,
                py: Python<'py>,
                timeout_secs: f64,
            ) -> PyResult<Bound<'py, PyAny>> {
                let inner = self.inner.clone();
                let timeout = duration_from_secs(timeout_secs)?;
                let map = $map;
                pyo3_async_runtimes::tokio::future_into_py(py, async move {
                    let value = inner.wait_timeout(timeout).await.map_err(to_py_err)?;
                    map(value)
                })
            }
        }
    };
}

define_progress_op!(
    PyMissionUploadOp,
    "MissionUploadOp",
    mavkit::MissionUploadOp,
    |_value| Ok(())
);
define_progress_op!(
    PyMissionDownloadOp,
    "MissionDownloadOp",
    mavkit::MissionDownloadOp,
    |value| Ok(PyMissionPlan { inner: value })
);
define_progress_op!(
    PyMissionClearOp,
    "MissionClearOp",
    mavkit::MissionClearOp,
    |_value| Ok(())
);
define_progress_op!(
    PyMissionVerifyOp,
    "MissionVerifyOp",
    mavkit::MissionVerifyOp,
    |value| Ok(value)
);
define_progress_op!(
    PyFenceUploadOp,
    "FenceUploadOp",
    mavkit::FenceUploadOp,
    |_value| Ok(())
);
define_progress_op!(
    PyFenceDownloadOp,
    "FenceDownloadOp",
    mavkit::FenceDownloadOp,
    |value| Ok(PyFencePlan::from_inner(value))
);
define_progress_op!(
    PyFenceClearOp,
    "FenceClearOp",
    mavkit::FenceClearOp,
    |_value| Ok(())
);
define_progress_op!(
    PyRallyUploadOp,
    "RallyUploadOp",
    mavkit::RallyUploadOp,
    |_value| Ok(())
);
define_progress_op!(
    PyRallyDownloadOp,
    "RallyDownloadOp",
    mavkit::RallyDownloadOp,
    |value| Ok(PyRallyPlan::from_inner(value))
);
define_progress_op!(
    PyRallyClearOp,
    "RallyClearOp",
    mavkit::RallyClearOp,
    |_value| Ok(())
);

#[pyclass(name = "CommandAck", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyCommandAck {
    inner: mavkit::CommandAck,
}

#[pymethods]
impl PyCommandAck {
    #[getter]
    fn command(&self) -> u16 {
        self.inner.command
    }

    #[getter]
    fn result(&self) -> u8 {
        self.inner.result
    }

    #[getter]
    fn progress(&self) -> Option<u8> {
        self.inner.progress
    }

    #[getter]
    fn result_param2(&self) -> Option<i32> {
        self.inner.result_param2
    }
}

#[pyclass(name = "MissionHandle", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyMissionHandle {
    inner: mavkit::Vehicle,
}

impl PyMissionHandle {
    fn new(inner: mavkit::Vehicle) -> Self {
        Self { inner }
    }
}

#[pymethods]
impl PyMissionHandle {
    fn latest(&self) -> Option<PyMissionState> {
        self.inner
            .mission()
            .latest()
            .map(|inner| PyMissionState { inner })
    }

    fn wait<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let inner = vehicle.mission().wait().await;
            Ok(PyMissionState { inner })
        })
    }

    fn wait_timeout<'py>(&self, py: Python<'py>, timeout_secs: f64) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        let timeout = duration_from_secs(timeout_secs)?;
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let inner = vehicle
                .mission()
                .wait_timeout(timeout)
                .await
                .map_err(to_py_err)?;
            Ok(PyMissionState { inner })
        })
    }

    fn subscribe(&self) -> PyMissionStateSubscription {
        PyMissionStateSubscription {
            inner: Arc::new(tokio::sync::Mutex::new(self.inner.mission().subscribe())),
        }
    }

    fn upload(&self, plan: &PyMissionPlan) -> PyResult<PyMissionUploadOp> {
        let vehicle = self.inner.clone();
        let plan = plan.inner.clone();
        let op = pyo3_async_runtimes::tokio::get_runtime()
            .block_on(async move { vehicle.mission().upload(plan) })
            .map_err(to_py_err)?;
        Ok(PyMissionUploadOp {
            inner: Arc::new(op),
        })
    }

    fn download(&self) -> PyResult<PyMissionDownloadOp> {
        let vehicle = self.inner.clone();
        let op = pyo3_async_runtimes::tokio::get_runtime()
            .block_on(async move { vehicle.mission().download() })
            .map_err(to_py_err)?;
        Ok(PyMissionDownloadOp {
            inner: Arc::new(op),
        })
    }

    fn clear(&self) -> PyResult<PyMissionClearOp> {
        let vehicle = self.inner.clone();
        let op = pyo3_async_runtimes::tokio::get_runtime()
            .block_on(async move { vehicle.mission().clear() })
            .map_err(to_py_err)?;
        Ok(PyMissionClearOp {
            inner: Arc::new(op),
        })
    }

    fn verify(&self, plan: &PyMissionPlan) -> PyResult<PyMissionVerifyOp> {
        let vehicle = self.inner.clone();
        let plan = plan.inner.clone();
        let op = pyo3_async_runtimes::tokio::get_runtime()
            .block_on(async move { vehicle.mission().verify(plan) })
            .map_err(to_py_err)?;
        Ok(PyMissionVerifyOp {
            inner: Arc::new(op),
        })
    }

    fn set_current<'py>(&self, py: Python<'py>, index: u16) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            vehicle
                .mission()
                .set_current(index)
                .await
                .map_err(to_py_err)?;
            Ok(())
        })
    }

    fn __repr__(&self) -> String {
        format!("MissionHandle({})", vehicle_label(&self.inner))
    }
}

#[pyclass(name = "FenceHandle", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyFenceHandle {
    inner: mavkit::Vehicle,
}

impl PyFenceHandle {
    fn new(inner: mavkit::Vehicle) -> Self {
        Self { inner }
    }
}

#[pymethods]
impl PyFenceHandle {
    fn support(&self) -> PySupportStateHandle {
        PySupportStateHandle::new(self.inner.fence().support())
    }

    fn latest(&self) -> Option<PyFenceState> {
        self.inner
            .fence()
            .latest()
            .map(|inner| PyFenceState { inner })
    }

    fn wait<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let inner = vehicle.fence().wait().await;
            Ok(PyFenceState { inner })
        })
    }

    fn wait_timeout<'py>(&self, py: Python<'py>, timeout_secs: f64) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        let timeout = duration_from_secs(timeout_secs)?;
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let inner = vehicle
                .fence()
                .wait_timeout(timeout)
                .await
                .map_err(to_py_err)?;
            Ok(PyFenceState { inner })
        })
    }

    fn subscribe(&self) -> PyFenceStateSubscription {
        PyFenceStateSubscription {
            inner: Arc::new(tokio::sync::Mutex::new(self.inner.fence().subscribe())),
        }
    }

    fn upload(&self, plan: &PyFencePlan) -> PyResult<PyFenceUploadOp> {
        let vehicle = self.inner.clone();
        let plan = plan.inner.clone();
        let op = pyo3_async_runtimes::tokio::get_runtime()
            .block_on(async move { vehicle.fence().upload(plan) })
            .map_err(to_py_err)?;
        Ok(PyFenceUploadOp {
            inner: Arc::new(op),
        })
    }

    fn download(&self) -> PyResult<PyFenceDownloadOp> {
        let vehicle = self.inner.clone();
        let op = pyo3_async_runtimes::tokio::get_runtime()
            .block_on(async move { vehicle.fence().download() })
            .map_err(to_py_err)?;
        Ok(PyFenceDownloadOp {
            inner: Arc::new(op),
        })
    }

    fn clear(&self) -> PyResult<PyFenceClearOp> {
        let vehicle = self.inner.clone();
        let op = pyo3_async_runtimes::tokio::get_runtime()
            .block_on(async move { vehicle.fence().clear() })
            .map_err(to_py_err)?;
        Ok(PyFenceClearOp {
            inner: Arc::new(op),
        })
    }

    fn __repr__(&self) -> String {
        format!("FenceHandle({})", vehicle_label(&self.inner))
    }
}

#[pyclass(name = "RallyHandle", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyRallyHandle {
    inner: mavkit::Vehicle,
}

impl PyRallyHandle {
    fn new(inner: mavkit::Vehicle) -> Self {
        Self { inner }
    }
}

#[pymethods]
impl PyRallyHandle {
    fn support(&self) -> PySupportStateHandle {
        PySupportStateHandle::new(self.inner.rally().support())
    }

    fn latest(&self) -> Option<PyRallyState> {
        self.inner
            .rally()
            .latest()
            .map(|inner| PyRallyState { inner })
    }

    fn wait<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let inner = vehicle.rally().wait().await;
            Ok(PyRallyState { inner })
        })
    }

    fn wait_timeout<'py>(&self, py: Python<'py>, timeout_secs: f64) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        let timeout = duration_from_secs(timeout_secs)?;
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let inner = vehicle
                .rally()
                .wait_timeout(timeout)
                .await
                .map_err(to_py_err)?;
            Ok(PyRallyState { inner })
        })
    }

    fn subscribe(&self) -> PyRallyStateSubscription {
        PyRallyStateSubscription {
            inner: Arc::new(tokio::sync::Mutex::new(self.inner.rally().subscribe())),
        }
    }

    fn upload(&self, plan: &PyRallyPlan) -> PyResult<PyRallyUploadOp> {
        let vehicle = self.inner.clone();
        let plan = plan.inner.clone();
        let op = pyo3_async_runtimes::tokio::get_runtime()
            .block_on(async move { vehicle.rally().upload(plan) })
            .map_err(to_py_err)?;
        Ok(PyRallyUploadOp {
            inner: Arc::new(op),
        })
    }

    fn download(&self) -> PyResult<PyRallyDownloadOp> {
        let vehicle = self.inner.clone();
        let op = pyo3_async_runtimes::tokio::get_runtime()
            .block_on(async move { vehicle.rally().download() })
            .map_err(to_py_err)?;
        Ok(PyRallyDownloadOp {
            inner: Arc::new(op),
        })
    }

    fn clear(&self) -> PyResult<PyRallyClearOp> {
        let vehicle = self.inner.clone();
        let op = pyo3_async_runtimes::tokio::get_runtime()
            .block_on(async move { vehicle.rally().clear() })
            .map_err(to_py_err)?;
        Ok(PyRallyClearOp {
            inner: Arc::new(op),
        })
    }

    fn __repr__(&self) -> String {
        format!("RallyHandle({})", vehicle_label(&self.inner))
    }
}

#[pyclass(name = "RawHandle", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyRawHandle {
    inner: mavkit::Vehicle,
}

impl PyRawHandle {
    fn new(inner: mavkit::Vehicle) -> Self {
        Self { inner }
    }
}

#[pymethods]
impl PyRawHandle {
    fn command_long<'py>(
        &self,
        py: Python<'py>,
        command: u16,
        params: [f32; 7],
    ) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let inner = vehicle
                .raw()
                .command_long(command, params)
                .await
                .map_err(to_py_err)?;
            Ok(PyCommandAck { inner })
        })
    }

    #[allow(clippy::too_many_arguments)]
    fn command_int<'py>(
        &self,
        py: Python<'py>,
        command: u16,
        frame: u8,
        current: u8,
        autocontinue: u8,
        params: [f32; 4],
        x: i32,
        y: i32,
        z: f32,
    ) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let inner = vehicle
                .raw()
                .command_int(command, frame, current, autocontinue, params, x, y, z)
                .await
                .map_err(to_py_err)?;
            Ok(PyCommandAck { inner })
        })
    }

    fn request_message<'py>(
        &self,
        py: Python<'py>,
        message_id: u32,
        timeout_secs: f64,
    ) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        let timeout = duration_from_secs(timeout_secs)?;
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let message = vehicle
                .raw()
                .request_message(message_id, timeout)
                .await
                .map_err(to_py_err)?;
            PyRawMessage::from_raw_message(message)
        })
    }

    fn set_message_interval<'py>(
        &self,
        py: Python<'py>,
        message_id: u32,
        interval_us: i32,
    ) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            vehicle
                .raw()
                .set_message_interval(message_id, interval_us)
                .await
                .map_err(to_py_err)?;
            Ok(())
        })
    }

    fn send<'py>(&self, py: Python<'py>, message: &PyRawMessage) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        let message = message.to_rust();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            vehicle.raw().send(message).await.map_err(to_py_err)?;
            Ok(())
        })
    }

    fn subscribe(&self) -> PyRawMessageStream {
        PyRawMessageStream::from_subscription(Box::pin(self.inner.raw().subscribe()))
    }

    fn __repr__(&self) -> String {
        format!("RawHandle({})", vehicle_label(&self.inner))
    }
}

#[cfg(feature = "test-support")]
type TestSentMessages = Arc<Mutex<Vec<(MavHeader, dialect::MavMessage)>>>;

#[cfg(feature = "test-support")]
struct MockConnection {
    recv_rx: tokio::sync::Mutex<mpsc::Receiver<(MavHeader, dialect::MavMessage)>>,
    sent: TestSentMessages,
}

#[cfg(feature = "test-support")]
impl MockConnection {
    fn new(rx: mpsc::Receiver<(MavHeader, dialect::MavMessage)>) -> (Self, TestSentMessages) {
        let sent = Arc::new(Mutex::new(Vec::new()));
        (
            Self {
                recv_rx: tokio::sync::Mutex::new(rx),
                sent: sent.clone(),
            },
            sent,
        )
    }
}

#[cfg(feature = "test-support")]
impl AsyncMavConnection<dialect::MavMessage> for MockConnection {
    fn recv<'life0, 'async_trait>(
        &'life0 self,
    ) -> std::pin::Pin<
        Box<
            dyn std::future::Future<
                    Output = Result<
                        (MavHeader, dialect::MavMessage),
                        mavlink::error::MessageReadError,
                    >,
                > + Send
                + 'async_trait,
        >,
    >
    where
        'life0: 'async_trait,
        Self: 'async_trait,
    {
        Box::pin(async move {
            let mut rx = self.recv_rx.lock().await;
            match rx.recv().await {
                Some(message) => Ok(message),
                None => Err(mavlink::error::MessageReadError::Io(std::io::Error::new(
                    std::io::ErrorKind::ConnectionReset,
                    "mock connection closed",
                ))),
            }
        })
    }

    fn recv_raw<'life0, 'async_trait>(
        &'life0 self,
    ) -> std::pin::Pin<
        Box<
            dyn std::future::Future<
                    Output = Result<mavlink::MAVLinkMessageRaw, mavlink::error::MessageReadError>,
                > + Send
                + 'async_trait,
        >,
    >
    where
        'life0: 'async_trait,
        Self: 'async_trait,
    {
        Box::pin(async move {
            let (header, message) = self.recv().await?;
            let mut raw = MAVLinkV2MessageRaw::new();
            raw.serialize_message(header, &message);
            Ok(MAVLinkMessageRaw::V2(raw))
        })
    }

    fn send<'life0, 'life1, 'life2, 'async_trait>(
        &'life0 self,
        header: &'life1 MavHeader,
        data: &'life2 dialect::MavMessage,
    ) -> std::pin::Pin<
        Box<
            dyn std::future::Future<Output = Result<usize, mavlink::error::MessageWriteError>>
                + Send
                + 'async_trait,
        >,
    >
    where
        'life0: 'async_trait,
        'life1: 'async_trait,
        'life2: 'async_trait,
        Self: 'async_trait,
    {
        let header = *header;
        let data = data.clone();
        Box::pin(async move {
            self.sent.lock().unwrap().push((header, data));
            Ok(0)
        })
    }

    fn set_protocol_version(&mut self, _version: MavlinkVersion) {}

    fn protocol_version(&self) -> MavlinkVersion {
        MavlinkVersion::V2
    }

    fn set_allow_recv_any_version(&mut self, _allow: bool) {}

    fn allow_recv_any_version(&self) -> bool {
        true
    }
}

#[cfg(feature = "test-support")]
fn default_header() -> MavHeader {
    MavHeader {
        system_id: 1,
        component_id: 1,
        sequence: 0,
    }
}

#[cfg(feature = "test-support")]
fn heartbeat_msg() -> dialect::MavMessage {
    dialect::MavMessage::HEARTBEAT(dialect::HEARTBEAT_DATA {
        custom_mode: 7,
        mavtype: dialect::MavType::MAV_TYPE_QUADROTOR,
        autopilot: dialect::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode: dialect::MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        system_status: dialect::MavState::MAV_STATE_STANDBY,
        mavlink_version: 3,
    })
}

#[cfg(feature = "test-support")]
fn fast_config() -> mavkit::VehicleConfig {
    mavkit::VehicleConfig {
        connect_timeout: Duration::from_millis(150),
        command_timeout: Duration::from_millis(50),
        command_completion_timeout: Duration::from_millis(150),
        auto_request_home: false,
        ..mavkit::VehicleConfig::default()
    }
}

#[cfg(feature = "test-support")]
fn command_ack_accepted_msg(command: u16) -> PyResult<dialect::MavMessage> {
    let command = match command {
        400 => dialect::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
        511 => dialect::MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
        512 => dialect::MavCmd::MAV_CMD_REQUEST_MESSAGE,
        other => {
            return Err(PyValueError::new_err(format!(
                "unsupported test command ack value {other}",
            )));
        }
    };

    Ok(dialect::MavMessage::COMMAND_ACK(
        dialect::COMMAND_ACK_DATA {
            command,
            result: dialect::MavResult::MAV_RESULT_ACCEPTED,
            progress: 0,
            result_param2: 0,
            target_system: 0,
            target_component: 0,
        },
    ))
}

#[cfg(feature = "test-support")]
fn mission_type_msg(mission_type: mavkit::MissionType) -> dialect::MavMissionType {
    match mission_type {
        mavkit::MissionType::Mission => dialect::MavMissionType::MAV_MISSION_TYPE_MISSION,
        mavkit::MissionType::Fence => dialect::MavMissionType::MAV_MISSION_TYPE_FENCE,
        mavkit::MissionType::Rally => dialect::MavMissionType::MAV_MISSION_TYPE_RALLY,
    }
}

#[cfg(feature = "test-support")]
fn mission_frame_msg(frame: mavkit::mission::commands::MissionFrame) -> dialect::MavFrame {
    match frame {
        mavkit::mission::commands::MissionFrame::Mission => dialect::MavFrame::MAV_FRAME_MISSION,
        mavkit::mission::commands::MissionFrame::Global => dialect::MavFrame::MAV_FRAME_GLOBAL,
        mavkit::mission::commands::MissionFrame::GlobalRelativeAlt => {
            dialect::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT
        }
        mavkit::mission::commands::MissionFrame::GlobalTerrainAlt => {
            dialect::MavFrame::MAV_FRAME_GLOBAL_TERRAIN_ALT
        }
        mavkit::mission::commands::MissionFrame::Other(_) => dialect::MavFrame::MAV_FRAME_MISSION,
    }
}

#[cfg(feature = "test-support")]
fn mission_request_int_msg(seq: u16, mission_type: mavkit::MissionType) -> dialect::MavMessage {
    dialect::MavMessage::MISSION_REQUEST_INT(dialect::MISSION_REQUEST_INT_DATA {
        seq,
        target_system: 0,
        target_component: 0,
        mission_type: mission_type_msg(mission_type),
    })
}

#[cfg(feature = "test-support")]
fn mission_count_msg(count: u16, mission_type: mavkit::MissionType) -> dialect::MavMessage {
    dialect::MavMessage::MISSION_COUNT(dialect::MISSION_COUNT_DATA {
        target_system: 0,
        target_component: 0,
        count,
        mission_type: mission_type_msg(mission_type),
        opaque_id: 0,
    })
}

#[cfg(feature = "test-support")]
fn mission_ack_accepted_msg_for_type(mission_type: mavkit::MissionType) -> dialect::MavMessage {
    dialect::MavMessage::MISSION_ACK(dialect::MISSION_ACK_DATA {
        target_system: 0,
        target_component: 0,
        mavtype: dialect::MavMissionResult::MAV_MISSION_ACCEPTED,
        mission_type: mission_type_msg(mission_type),
        opaque_id: 0,
    })
}

#[cfg(feature = "test-support")]
fn mission_item_int_msg(
    item: &mavkit::MissionItem,
    seq: u16,
    mission_type: mavkit::MissionType,
) -> PyResult<dialect::MavMessage> {
    let (command, frame, params, x, y, z) = item.command.clone().into_wire();
    let command = num_traits::FromPrimitive::from_u16(command).ok_or_else(|| {
        PyValueError::new_err(format!(
            "unsupported test mission command value {}",
            command
        ))
    })?;

    Ok(dialect::MavMessage::MISSION_ITEM_INT(
        dialect::MISSION_ITEM_INT_DATA {
            param1: params[0],
            param2: params[1],
            param3: params[2],
            param4: params[3],
            x,
            y,
            z,
            seq,
            command,
            target_system: 0,
            target_component: 0,
            frame: mission_frame_msg(frame),
            current: 0,
            autocontinue: u8::from(item.autocontinue),
            mission_type: mission_type_msg(mission_type),
        },
    ))
}

#[cfg(feature = "test-support")]
fn global_position_int_msg() -> dialect::MavMessage {
    dialect::MavMessage::GLOBAL_POSITION_INT(dialect::GLOBAL_POSITION_INT_DATA {
        time_boot_ms: 42,
        lat: 473_977_420,
        lon: 85_455_940,
        alt: 500_000,
        relative_alt: 15_000,
        vx: 0,
        vy: 0,
        vz: 0,
        hdg: 9_000,
    })
}

#[cfg(feature = "test-support")]
#[pyclass(name = "_TestVehicleHarness", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyTestVehicleHarness {
    msg_tx: mpsc::Sender<(MavHeader, dialect::MavMessage)>,
    sent: TestSentMessages,
}

#[cfg(feature = "test-support")]
#[pymethods]
impl PyTestVehicleHarness {
    fn push_command_ack_accepted<'py>(
        &self,
        py: Python<'py>,
        command: u16,
    ) -> PyResult<Bound<'py, PyAny>> {
        let msg_tx = self.msg_tx.clone();
        let message = command_ack_accepted_msg(command)?;
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            msg_tx
                .send((default_header(), message))
                .await
                .map_err(|_| PyRuntimeError::new_err("mock vehicle channel closed"))?;
            Ok(())
        })
    }

    fn push_global_position_int<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let msg_tx = self.msg_tx.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            msg_tx
                .send((default_header(), global_position_int_msg()))
                .await
                .map_err(|_| PyRuntimeError::new_err("mock vehicle channel closed"))?;
            Ok(())
        })
    }

    fn push_mission_upload_success<'py>(
        &self,
        py: Python<'py>,
        plan: &PyMissionPlan,
    ) -> PyResult<Bound<'py, PyAny>> {
        let msg_tx = self.msg_tx.clone();
        let mission_type = mavkit::MissionType::Mission;
        let wire_items = mavkit::mission_items_for_upload(&plan.inner);
        let mut responses = Vec::with_capacity(wire_items.len() + 1);
        for seq in 0..wire_items.len() {
            responses.push(mission_request_int_msg(seq as u16, mission_type));
        }
        responses.push(mission_ack_accepted_msg_for_type(mission_type));

        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            for message in responses {
                msg_tx
                    .send((default_header(), message))
                    .await
                    .map_err(|_| PyRuntimeError::new_err("mock vehicle channel closed"))?;
            }
            Ok(())
        })
    }

    fn push_mission_download_success<'py>(
        &self,
        py: Python<'py>,
        plan: &PyMissionPlan,
    ) -> PyResult<Bound<'py, PyAny>> {
        let msg_tx = self.msg_tx.clone();
        let mission_type = mavkit::MissionType::Mission;
        let wire_items = mavkit::mission_items_for_upload(&plan.inner);
        let wire_item_count: u16 = wire_items.len().try_into().map_err(|_| {
            PyValueError::new_err("plan has too many items for MAVLink mission count")
        })?;
        let mut responses = Vec::with_capacity(wire_items.len() + 1);
        responses.push(mission_count_msg(wire_item_count, mission_type));
        for (seq, item) in wire_items.iter().enumerate() {
            responses.push(mission_item_int_msg(item, seq as u16, mission_type)?);
        }

        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            for message in responses {
                msg_tx
                    .send((default_header(), message))
                    .await
                    .map_err(|_| PyRuntimeError::new_err("mock vehicle channel closed"))?;
            }
            Ok(())
        })
    }

    fn sent_message_count(&self) -> usize {
        self.sent.lock().unwrap().len()
    }
}

#[cfg(feature = "test-support")]
#[pyfunction]
pub fn _connect_test_vehicle<'py>(py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
    let (msg_tx, msg_rx) = mpsc::channel(16);
    let (conn, sent) = MockConnection::new(msg_rx);
    pyo3_async_runtimes::tokio::future_into_py(py, async move {
        let connect_task = tokio::spawn(async move {
            mavkit::Vehicle::from_connection(Box::new(conn), fast_config()).await
        });

        msg_tx
            .send((default_header(), heartbeat_msg()))
            .await
            .map_err(|_| PyRuntimeError::new_err("mock heartbeat should be delivered"))?;

        let vehicle = timeout(Duration::from_millis(250), connect_task)
            .await
            .map_err(|_| PyRuntimeError::new_err("mock vehicle connect timed out"))?
            .map_err(|_| PyRuntimeError::new_err("mock vehicle connect task failed"))?
            .map_err(to_py_err)?;

        Ok((
            PyVehicle {
                inner: vehicle.clone(),
            },
            PyTestVehicleHarness { msg_tx, sent },
        ))
    })
}

#[pyclass(name = "Vehicle", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyVehicle {
    inner: mavkit::Vehicle,
}

#[pymethods]
impl PyVehicle {
    #[staticmethod]
    fn connect<'py>(py: Python<'py>, address: &str) -> PyResult<Bound<'py, PyAny>> {
        let addr = address.to_string();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let vehicle = mavkit::Vehicle::connect(&addr).await.map_err(to_py_err)?;
            Ok(Self { inner: vehicle })
        })
    }

    #[staticmethod]
    fn connect_udp<'py>(py: Python<'py>, bind_addr: &str) -> PyResult<Bound<'py, PyAny>> {
        let addr = bind_addr.to_string();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let vehicle = mavkit::Vehicle::connect_udp(&addr)
                .await
                .map_err(to_py_err)?;
            Ok(Self { inner: vehicle })
        })
    }

    #[staticmethod]
    fn connect_tcp<'py>(py: Python<'py>, addr: &str) -> PyResult<Bound<'py, PyAny>> {
        let address = addr.to_string();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let vehicle = mavkit::Vehicle::connect_tcp(&address)
                .await
                .map_err(to_py_err)?;
            Ok(Self { inner: vehicle })
        })
    }

    #[staticmethod]
    fn connect_serial<'py>(py: Python<'py>, port: &str, baud: u32) -> PyResult<Bound<'py, PyAny>> {
        let serial_port = port.to_string();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let vehicle = mavkit::Vehicle::connect_serial(&serial_port, baud)
                .await
                .map_err(to_py_err)?;
            Ok(Self { inner: vehicle })
        })
    }

    #[staticmethod]
    fn connect_with_config<'py>(
        py: Python<'py>,
        address: &str,
        config: PyVehicleConfig,
    ) -> PyResult<Bound<'py, PyAny>> {
        let addr = address.to_string();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let vehicle = mavkit::Vehicle::connect_with_config(&addr, config.into_inner())
                .await
                .map_err(to_py_err)?;
            Ok(Self { inner: vehicle })
        })
    }

    fn telemetry(&self) -> PyTelemetryHandle {
        PyTelemetryHandle::new(self.inner.clone())
    }

    fn available_modes(&self) -> PyModesHandle {
        PyModesHandle::new(self.inner.clone())
    }

    fn info(&self) -> PyInfoHandle {
        PyInfoHandle::new(self.inner.clone())
    }

    fn support(&self) -> PySupportHandle {
        PySupportHandle::new(self.inner.clone())
    }

    fn link(&self) -> PyLinkHandle {
        PyLinkHandle::new(self.inner.clone())
    }

    fn mission(&self) -> PyMissionHandle {
        PyMissionHandle::new(self.inner.clone())
    }

    fn fence(&self) -> PyFenceHandle {
        PyFenceHandle::new(self.inner.clone())
    }

    fn rally(&self) -> PyRallyHandle {
        PyRallyHandle::new(self.inner.clone())
    }

    fn params(&self) -> PyParamsHandle {
        PyParamsHandle::new(self.inner.clone())
    }

    fn raw(&self) -> PyRawHandle {
        PyRawHandle::new(self.inner.clone())
    }

    fn ardupilot(&self) -> PyArduPilotHandle {
        PyArduPilotHandle::new(self.inner.clone())
    }

    fn identity(&self) -> PyVehicleIdentity {
        PyVehicleIdentity {
            inner: self.inner.identity(),
        }
    }

    fn arm<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            vehicle.arm().await.map_err(to_py_err)?;
            Ok(())
        })
    }

    fn arm_no_wait<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            vehicle.arm_no_wait().await.map_err(to_py_err)?;
            Ok(())
        })
    }

    fn force_arm<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            vehicle.force_arm().await.map_err(to_py_err)?;
            Ok(())
        })
    }

    fn force_arm_no_wait<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            vehicle.force_arm_no_wait().await.map_err(to_py_err)?;
            Ok(())
        })
    }

    fn disarm<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            vehicle.disarm().await.map_err(to_py_err)?;
            Ok(())
        })
    }

    fn disarm_no_wait<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            vehicle.disarm_no_wait().await.map_err(to_py_err)?;
            Ok(())
        })
    }

    fn force_disarm<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            vehicle.force_disarm().await.map_err(to_py_err)?;
            Ok(())
        })
    }

    fn force_disarm_no_wait<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            vehicle.force_disarm_no_wait().await.map_err(to_py_err)?;
            Ok(())
        })
    }

    fn set_mode<'py>(&self, py: Python<'py>, custom_mode: u32) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            vehicle.set_mode(custom_mode).await.map_err(to_py_err)?;
            Ok(())
        })
    }

    fn set_mode_no_wait<'py>(
        &self,
        py: Python<'py>,
        custom_mode: u32,
    ) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            vehicle
                .set_mode_no_wait(custom_mode)
                .await
                .map_err(to_py_err)?;
            Ok(())
        })
    }

    fn set_mode_by_name<'py>(&self, py: Python<'py>, name: &str) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        let mode_name = name.to_string();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            vehicle
                .set_mode_by_name(&mode_name)
                .await
                .map_err(to_py_err)?;
            Ok(())
        })
    }

    fn set_mode_by_name_no_wait<'py>(
        &self,
        py: Python<'py>,
        name: &str,
    ) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        let mode_name = name.to_string();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            vehicle
                .set_mode_by_name_no_wait(&mode_name)
                .await
                .map_err(to_py_err)?;
            Ok(())
        })
    }

    #[pyo3(signature = (*, latitude_deg, longitude_deg, altitude_msl_m))]
    fn set_home<'py>(
        &self,
        py: Python<'py>,
        latitude_deg: f64,
        longitude_deg: f64,
        altitude_msl_m: f64,
    ) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        let point = geo_point_msl(latitude_deg, longitude_deg, altitude_msl_m);
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            vehicle.set_home(point).await.map_err(to_py_err)?;
            Ok(())
        })
    }

    fn set_home_current<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            vehicle.set_home_current().await.map_err(to_py_err)?;
            Ok(())
        })
    }

    #[pyo3(signature = (*, latitude_deg, longitude_deg, altitude_msl_m))]
    fn set_origin<'py>(
        &self,
        py: Python<'py>,
        latitude_deg: f64,
        longitude_deg: f64,
        altitude_msl_m: f64,
    ) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        let point = geo_point_msl(latitude_deg, longitude_deg, altitude_msl_m);
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            vehicle.set_origin(point).await.map_err(to_py_err)?;
            Ok(())
        })
    }

    fn disconnect<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            vehicle.disconnect().await.map_err(to_py_err)?;
            Ok(())
        })
    }

    fn __aenter__<'py>(slf: &Bound<'py, Self>, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let obj = slf.clone().into_any().unbind();
        pyo3_async_runtimes::tokio::future_into_py(py, async move { Ok(obj) })
    }

    #[pyo3(signature = (_exc_type=None, _exc_val=None, _exc_tb=None))]
    fn __aexit__<'py>(
        &self,
        py: Python<'py>,
        _exc_type: Option<&Bound<'py, PyAny>>,
        _exc_val: Option<&Bound<'py, PyAny>>,
        _exc_tb: Option<&Bound<'py, PyAny>>,
    ) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            vehicle.disconnect().await.map_err(to_py_err)?;
            Ok(false)
        })
    }

    fn __repr__(&self) -> String {
        let identity = self.inner.identity();
        format!(
            "Vehicle(sys={}, comp={}, autopilot={:?}, type={:?})",
            identity.system_id, identity.component_id, identity.autopilot, identity.vehicle_type,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavkit::dialect;
    use mavlink::{
        AsyncMavConnection, MAVLinkMessageRaw, MAVLinkV2MessageRaw, MavHeader, MavlinkVersion,
    };
    use std::sync::{Arc, Mutex};
    use std::time::Duration;
    use tokio::sync::mpsc;
    use tokio::time::timeout;

    type SentMessages = Arc<Mutex<Vec<(MavHeader, dialect::MavMessage)>>>;

    struct MockConnection {
        recv_rx: tokio::sync::Mutex<mpsc::Receiver<(MavHeader, dialect::MavMessage)>>,
        sent: SentMessages,
    }

    impl MockConnection {
        fn new(rx: mpsc::Receiver<(MavHeader, dialect::MavMessage)>) -> (Self, SentMessages) {
            let sent = Arc::new(Mutex::new(Vec::new()));
            (
                Self {
                    recv_rx: tokio::sync::Mutex::new(rx),
                    sent: sent.clone(),
                },
                sent,
            )
        }
    }

    impl AsyncMavConnection<dialect::MavMessage> for MockConnection {
        fn recv<'life0, 'async_trait>(
            &'life0 self,
        ) -> std::pin::Pin<
            Box<
                dyn std::future::Future<
                        Output = Result<
                            (MavHeader, dialect::MavMessage),
                            mavlink::error::MessageReadError,
                        >,
                    > + Send
                    + 'async_trait,
            >,
        >
        where
            'life0: 'async_trait,
            Self: 'async_trait,
        {
            Box::pin(async move {
                let mut rx = self.recv_rx.lock().await;
                match rx.recv().await {
                    Some(message) => Ok(message),
                    None => Err(mavlink::error::MessageReadError::Io(std::io::Error::new(
                        std::io::ErrorKind::ConnectionReset,
                        "mock connection closed",
                    ))),
                }
            })
        }

        fn recv_raw<'life0, 'async_trait>(
            &'life0 self,
        ) -> std::pin::Pin<
            Box<
                dyn std::future::Future<
                        Output = Result<
                            mavlink::MAVLinkMessageRaw,
                            mavlink::error::MessageReadError,
                        >,
                    > + Send
                    + 'async_trait,
            >,
        >
        where
            'life0: 'async_trait,
            Self: 'async_trait,
        {
            Box::pin(async move {
                let (header, message) = self.recv().await?;
                let mut raw = MAVLinkV2MessageRaw::new();
                raw.serialize_message(header, &message);
                Ok(MAVLinkMessageRaw::V2(raw))
            })
        }

        fn send<'life0, 'life1, 'life2, 'async_trait>(
            &'life0 self,
            header: &'life1 MavHeader,
            data: &'life2 dialect::MavMessage,
        ) -> std::pin::Pin<
            Box<
                dyn std::future::Future<Output = Result<usize, mavlink::error::MessageWriteError>>
                    + Send
                    + 'async_trait,
            >,
        >
        where
            'life0: 'async_trait,
            'life1: 'async_trait,
            'life2: 'async_trait,
            Self: 'async_trait,
        {
            let header = *header;
            let data = data.clone();
            Box::pin(async move {
                self.sent.lock().unwrap().push((header, data));
                Ok(0)
            })
        }

        fn set_protocol_version(&mut self, _version: MavlinkVersion) {}

        fn protocol_version(&self) -> MavlinkVersion {
            MavlinkVersion::V2
        }

        fn set_allow_recv_any_version(&mut self, _allow: bool) {}

        fn allow_recv_any_version(&self) -> bool {
            true
        }
    }

    fn default_header() -> MavHeader {
        MavHeader {
            system_id: 1,
            component_id: 1,
            sequence: 0,
        }
    }

    fn heartbeat_msg() -> dialect::MavMessage {
        dialect::MavMessage::HEARTBEAT(dialect::HEARTBEAT_DATA {
            custom_mode: 7,
            mavtype: dialect::MavType::MAV_TYPE_QUADROTOR,
            autopilot: dialect::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode: dialect::MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            system_status: dialect::MavState::MAV_STATE_STANDBY,
            mavlink_version: 3,
        })
    }

    fn fast_config() -> mavkit::VehicleConfig {
        mavkit::VehicleConfig {
            connect_timeout: Duration::from_millis(150),
            command_timeout: Duration::from_millis(50),
            command_completion_timeout: Duration::from_millis(150),
            auto_request_home: false,
            ..mavkit::VehicleConfig::default()
        }
    }

    async fn connect_mock_vehicle() -> mavkit::Vehicle {
        let (msg_tx, msg_rx) = mpsc::channel(16);
        let (conn, _sent) = MockConnection::new(msg_rx);
        let connect_task = tokio::spawn(async move {
            mavkit::Vehicle::from_connection(Box::new(conn), fast_config()).await
        });

        msg_tx
            .send((default_header(), heartbeat_msg()))
            .await
            .expect("heartbeat should be delivered");

        timeout(Duration::from_millis(250), connect_task)
            .await
            .expect("connect should complete")
            .expect("connect task should join")
            .expect("mock vehicle should connect")
    }

    #[tokio::test(flavor = "current_thread")]
    async fn identity_uses_connected_vehicle_state() {
        let vehicle = connect_mock_vehicle().await;
        let py_vehicle = PyVehicle { inner: vehicle };

        let identity = py_vehicle.identity();

        assert_eq!(identity.inner.system_id, 1);
        assert_eq!(identity.inner.component_id, 1);
        assert_eq!(
            identity.inner.autopilot,
            mavkit::AutopilotType::ArduPilotMega
        );
        assert_eq!(identity.inner.vehicle_type, mavkit::VehicleType::Quadrotor);
    }

    #[tokio::test(flavor = "current_thread")]
    async fn repeated_handle_access_wraps_shared_vehicle_clone_semantics() {
        let vehicle = connect_mock_vehicle().await;
        let py_vehicle = PyVehicle { inner: vehicle };

        let telemetry_a = py_vehicle.telemetry();
        let telemetry_b = py_vehicle.telemetry();
        let modes_a = py_vehicle.available_modes();
        let modes_b = py_vehicle.available_modes();

        assert_eq!(telemetry_a.inner.identity(), telemetry_b.inner.identity());
        assert_eq!(
            modes_a.inner.available_modes().len(),
            modes_b.inner.available_modes().len()
        );
        assert_eq!(modes_a.inner.identity(), py_vehicle.inner.identity());
    }
}
