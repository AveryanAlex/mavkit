use pyo3::prelude::*;

use crate::enums::*;

// --- MissionItem ---

#[pyclass(name = "MissionItem", frozen, from_py_object)]
#[derive(Clone)]
pub struct PyMissionItem {
    pub(crate) inner: mavkit::MissionItem,
}

#[pymethods]
impl PyMissionItem {
    #[new]
    #[pyo3(signature = (*, seq, command, frame, x=0, y=0, z=0.0, param1=0.0, param2=0.0, param3=0.0, param4=0.0, current=false, autocontinue=true))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        seq: u16,
        command: u16,
        frame: PyMissionFrame,
        x: i32,
        y: i32,
        z: f32,
        param1: f32,
        param2: f32,
        param3: f32,
        param4: f32,
        current: bool,
        autocontinue: bool,
    ) -> Self {
        Self {
            inner: mavkit::MissionItem {
                seq,
                command,
                frame: frame.into(),
                current,
                autocontinue,
                param1,
                param2,
                param3,
                param4,
                x,
                y,
                z,
            },
        }
    }

    #[getter]
    fn seq(&self) -> u16 {
        self.inner.seq
    }
    #[getter]
    fn command(&self) -> u16 {
        self.inner.command
    }
    #[getter]
    fn frame(&self) -> PyMissionFrame {
        self.inner.frame.into()
    }
    #[getter]
    fn current(&self) -> bool {
        self.inner.current
    }
    #[getter]
    fn autocontinue(&self) -> bool {
        self.inner.autocontinue
    }
    #[getter]
    fn param1(&self) -> f32 {
        self.inner.param1
    }
    #[getter]
    fn param2(&self) -> f32 {
        self.inner.param2
    }
    #[getter]
    fn param3(&self) -> f32 {
        self.inner.param3
    }
    #[getter]
    fn param4(&self) -> f32 {
        self.inner.param4
    }
    #[getter]
    fn x(&self) -> i32 {
        self.inner.x
    }
    #[getter]
    fn y(&self) -> i32 {
        self.inner.y
    }
    #[getter]
    fn z(&self) -> f32 {
        self.inner.z
    }

    fn __repr__(&self) -> String {
        format!(
            "MissionItem(seq={}, cmd={}, frame={:?})",
            self.inner.seq, self.inner.command, self.inner.frame
        )
    }
}

// --- HomePosition ---

#[pyclass(name = "HomePosition", frozen, from_py_object)]
#[derive(Clone)]
pub struct PyHomePosition {
    pub(crate) inner: mavkit::HomePosition,
}

#[pymethods]
impl PyHomePosition {
    #[new]
    #[pyo3(signature = (*, latitude_deg, longitude_deg, altitude_m=0.0))]
    fn new(latitude_deg: f64, longitude_deg: f64, altitude_m: f32) -> Self {
        Self {
            inner: mavkit::HomePosition {
                latitude_deg,
                longitude_deg,
                altitude_m,
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
    fn altitude_m(&self) -> f32 {
        self.inner.altitude_m
    }

    fn __repr__(&self) -> String {
        format!(
            "HomePosition(lat={:.6}, lon={:.6}, alt={:.1})",
            self.inner.latitude_deg, self.inner.longitude_deg, self.inner.altitude_m
        )
    }
}

// --- MissionPlan ---

#[pyclass(name = "MissionPlan", frozen, from_py_object)]
#[derive(Clone)]
pub struct PyMissionPlan {
    pub(crate) inner: mavkit::MissionPlan,
}

#[pymethods]
impl PyMissionPlan {
    #[new]
    #[pyo3(signature = (*, mission_type, items, home=None))]
    fn new(
        mission_type: PyMissionType,
        items: Vec<PyMissionItem>,
        home: Option<PyHomePosition>,
    ) -> Self {
        Self {
            inner: mavkit::MissionPlan {
                mission_type: mission_type.into(),
                home: home.map(|h| h.inner),
                items: items.into_iter().map(|i| i.inner).collect(),
            },
        }
    }

    #[getter]
    fn mission_type(&self) -> PyMissionType {
        self.inner.mission_type.into()
    }

    #[getter]
    fn home(&self) -> Option<PyHomePosition> {
        self.inner
            .home
            .as_ref()
            .map(|h| PyHomePosition { inner: h.clone() })
    }

    #[getter]
    fn items(&self) -> Vec<PyMissionItem> {
        self.inner
            .items
            .iter()
            .map(|i| PyMissionItem { inner: i.clone() })
            .collect()
    }

    fn __repr__(&self) -> String {
        format!(
            "MissionPlan(type={:?}, home={}, items={})",
            self.inner.mission_type,
            if self.inner.home.is_some() {
                "set"
            } else {
                "None"
            },
            self.inner.items.len()
        )
    }

    fn __len__(&self) -> usize {
        self.inner.items.len()
    }
}

// --- MissionIssue ---

#[pyclass(name = "MissionIssue", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyMissionIssue {
    inner: mavkit::MissionIssue,
}

#[pymethods]
impl PyMissionIssue {
    #[getter]
    fn code(&self) -> &str {
        &self.inner.code
    }
    #[getter]
    fn message(&self) -> &str {
        &self.inner.message
    }
    #[getter]
    fn seq(&self) -> Option<u16> {
        self.inner.seq
    }
    #[getter]
    fn severity(&self) -> PyIssueSeverity {
        self.inner.severity.into()
    }

    fn __repr__(&self) -> String {
        format!(
            "MissionIssue({:?}: {} - '{}')",
            self.inner.severity, self.inner.code, self.inner.message
        )
    }
}

// --- TransferProgress ---

#[pyclass(name = "TransferProgress", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyTransferProgress {
    pub(crate) inner: mavkit::TransferProgress,
}

#[pymethods]
impl PyTransferProgress {
    #[getter]
    fn direction(&self) -> PyTransferDirection {
        self.inner.direction.into()
    }
    #[getter]
    fn mission_type(&self) -> PyMissionType {
        self.inner.mission_type.into()
    }
    #[getter]
    fn phase(&self) -> PyTransferPhase {
        self.inner.phase.into()
    }
    #[getter]
    fn completed_items(&self) -> u16 {
        self.inner.completed_items
    }
    #[getter]
    fn total_items(&self) -> u16 {
        self.inner.total_items
    }
    #[getter]
    fn retries_used(&self) -> u8 {
        self.inner.retries_used
    }

    fn __repr__(&self) -> String {
        format!(
            "TransferProgress({:?} {:?}: {}/{})",
            self.inner.direction,
            self.inner.phase,
            self.inner.completed_items,
            self.inner.total_items
        )
    }
}

// --- TransferError ---

#[pyclass(name = "TransferError", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyTransferError {
    inner: mavkit::TransferError,
}

#[pymethods]
impl PyTransferError {
    #[getter]
    fn code(&self) -> &str {
        &self.inner.code
    }
    #[getter]
    fn message(&self) -> &str {
        &self.inner.message
    }

    fn __repr__(&self) -> String {
        format!(
            "TransferError(code='{}', message='{}')",
            self.inner.code, self.inner.message
        )
    }
}

// --- RetryPolicy ---

#[pyclass(name = "RetryPolicy", frozen, from_py_object)]
#[derive(Clone)]
pub struct PyRetryPolicy {
    pub(crate) inner: mavkit::RetryPolicy,
}

#[pymethods]
impl PyRetryPolicy {
    #[new]
    #[pyo3(signature = (*, request_timeout_ms=1500, item_timeout_ms=250, max_retries=5))]
    fn new(request_timeout_ms: u64, item_timeout_ms: u64, max_retries: u8) -> Self {
        Self {
            inner: mavkit::RetryPolicy {
                request_timeout_ms,
                item_timeout_ms,
                max_retries,
            },
        }
    }

    #[getter]
    fn request_timeout_ms(&self) -> u64 {
        self.inner.request_timeout_ms
    }
    #[getter]
    fn item_timeout_ms(&self) -> u64 {
        self.inner.item_timeout_ms
    }
    #[getter]
    fn max_retries(&self) -> u8 {
        self.inner.max_retries
    }

    fn __repr__(&self) -> String {
        format!(
            "RetryPolicy(request_timeout_ms={}, item_timeout_ms={}, max_retries={})",
            self.inner.request_timeout_ms, self.inner.item_timeout_ms, self.inner.max_retries
        )
    }
}

// --- CompareTolerance ---

#[pyclass(name = "CompareTolerance", frozen, from_py_object)]
#[derive(Clone)]
pub struct PyCompareTolerance {
    inner: mavkit::CompareTolerance,
}

#[pymethods]
impl PyCompareTolerance {
    #[new]
    #[pyo3(signature = (*, param_epsilon=0.0001, altitude_epsilon_m=0.01))]
    fn new(param_epsilon: f32, altitude_epsilon_m: f32) -> Self {
        Self {
            inner: mavkit::CompareTolerance {
                param_epsilon,
                altitude_epsilon_m,
            },
        }
    }

    #[getter]
    fn param_epsilon(&self) -> f32 {
        self.inner.param_epsilon
    }
    #[getter]
    fn altitude_epsilon_m(&self) -> f32 {
        self.inner.altitude_epsilon_m
    }

    fn __repr__(&self) -> String {
        format!(
            "CompareTolerance(param_epsilon={}, altitude_epsilon_m={})",
            self.inner.param_epsilon, self.inner.altitude_epsilon_m
        )
    }
}

// --- Free functions ---

#[pyfunction]
pub fn validate_plan(plan: &PyMissionPlan) -> Vec<PyMissionIssue> {
    mavkit::validate_plan(&plan.inner)
        .into_iter()
        .map(|i| PyMissionIssue { inner: i })
        .collect()
}

#[pyfunction]
#[pyo3(signature = (lhs, rhs, tolerance=None))]
pub fn plans_equivalent(
    lhs: &PyMissionPlan,
    rhs: &PyMissionPlan,
    tolerance: Option<&PyCompareTolerance>,
) -> bool {
    let tol = tolerance.map(|t| t.inner).unwrap_or_default();
    mavkit::plans_equivalent(&lhs.inner, &rhs.inner, tol)
}

#[pyfunction]
pub fn normalize_for_compare(plan: &PyMissionPlan) -> PyMissionPlan {
    PyMissionPlan {
        inner: mavkit::normalize_for_compare(&plan.inner),
    }
}

#[pyfunction]
pub fn items_for_wire_upload(plan: &PyMissionPlan) -> Vec<PyMissionItem> {
    mavkit::items_for_wire_upload(&plan.inner)
        .into_iter()
        .map(|i| PyMissionItem { inner: i })
        .collect()
}

#[pyfunction]
pub fn plan_from_wire_download(
    mission_type: PyMissionType,
    items: Vec<PyMissionItem>,
) -> PyMissionPlan {
    let wire_items: Vec<mavkit::MissionItem> = items.into_iter().map(|i| i.inner).collect();
    PyMissionPlan {
        inner: mavkit::plan_from_wire_download(mission_type.into(), wire_items),
    }
}
