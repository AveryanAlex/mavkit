use pyo3::prelude::*;
use pyo3::types::PyAny;

use crate::enums::*;

use super::commands::{command_frame_from_py, py_frame_from_command, typed_command_from_py};

pub(crate) fn wire_parts(
    item: &mavkit::MissionItem,
) -> (u16, PyMissionFrame, [f32; 4], i32, i32, f32) {
    let (command, frame, params, x, y, z) = item.command.clone().into_wire();
    (command, py_frame_from_command(frame), params, x, y, z)
}

// --- MissionItem ---

#[pyclass(name = "MissionItem", frozen, from_py_object)]
#[derive(Clone)]
pub struct PyMissionItem {
    pub(crate) inner: mavkit::MissionItem,
}

#[pymethods]
impl PyMissionItem {
    #[new]
    #[pyo3(signature = (*, command, frame=None, x=0, y=0, z=0.0, param1=0.0, param2=0.0, param3=0.0, param4=0.0, autocontinue=true))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        command: &Bound<'_, PyAny>,
        frame: Option<PyMissionFrame>,
        x: i32,
        y: i32,
        z: f32,
        param1: f32,
        param2: f32,
        param3: f32,
        param4: f32,
        autocontinue: bool,
    ) -> PyResult<Self> {
        let uses_legacy_wire_fields = frame.is_some()
            || x != 0
            || y != 0
            || z != 0.0
            || param1 != 0.0
            || param2 != 0.0
            || param3 != 0.0
            || param4 != 0.0;

        let mission_command = if let Some(typed_command) = typed_command_from_py(command)? {
            if uses_legacy_wire_fields {
                return Err(pyo3::exceptions::PyTypeError::new_err(
                    "frame/x/y/z/param1..4 are only valid when command is an int",
                ));
            }
            typed_command
        } else {
            let command_id = command.extract::<u16>().map_err(|_| {
                pyo3::exceptions::PyTypeError::new_err(
                    "command must be a supported typed mission command, RawMissionCommand, or int",
                )
            })?;

            let frame = frame.ok_or_else(|| {
                pyo3::exceptions::PyTypeError::new_err("frame is required when command is an int")
            })?;

            mavkit::MissionCommand::Other(mavkit::RawMissionCommand {
                command: command_id,
                frame: command_frame_from_py(frame),
                param1,
                param2,
                param3,
                param4,
                x,
                y,
                z,
            })
        };

        Ok(Self {
            inner: mavkit::MissionItem {
                command: mission_command,
                autocontinue,
            },
        })
    }

    #[getter]
    fn command(&self) -> u16 {
        wire_parts(&self.inner).0
    }
    #[getter]
    fn frame(&self) -> PyMissionFrame {
        wire_parts(&self.inner).1
    }
    #[getter]
    fn autocontinue(&self) -> bool {
        self.inner.autocontinue
    }
    #[getter]
    fn param1(&self) -> f32 {
        wire_parts(&self.inner).2[0]
    }
    #[getter]
    fn param2(&self) -> f32 {
        wire_parts(&self.inner).2[1]
    }
    #[getter]
    fn param3(&self) -> f32 {
        wire_parts(&self.inner).2[2]
    }
    #[getter]
    fn param4(&self) -> f32 {
        wire_parts(&self.inner).2[3]
    }
    #[getter]
    fn x(&self) -> i32 {
        wire_parts(&self.inner).3
    }
    #[getter]
    fn y(&self) -> i32 {
        wire_parts(&self.inner).4
    }
    #[getter]
    fn z(&self) -> f32 {
        wire_parts(&self.inner).5
    }

    fn __repr__(&self) -> String {
        format!(
            "MissionItem(cmd={}, frame={:?})",
            self.command(),
            self.frame()
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
    fn new(latitude_deg: f64, longitude_deg: f64, altitude_m: f64) -> Self {
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
    fn altitude_m(&self) -> f64 {
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
    #[pyo3(signature = (*, items))]
    fn new(items: Vec<PyMissionItem>) -> Self {
        Self {
            inner: mavkit::MissionPlan {
                items: items.into_iter().map(|i| i.inner).collect(),
            },
        }
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
        format!("MissionPlan(items={})", self.inner.items.len())
    }

    fn __len__(&self) -> usize {
        self.inner.items.len()
    }
}

// --- MissionIssue ---

#[pyclass(name = "MissionIssue", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyMissionIssue {
    pub(crate) inner: mavkit::MissionIssue,
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
    pub(crate) inner: mavkit::CompareTolerance,
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
