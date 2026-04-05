use pyo3::prelude::*;

use crate::macros::py_subscription;
use crate::mission::PyMissionPlan;
use crate::params::PySyncState;
use crate::vehicle::stored_plans::{PyFencePlan, PyRallyPlan};

pub(crate) fn stored_plan_operation_name(kind: mavkit::StoredPlanOperationKind) -> &'static str {
    match kind {
        mavkit::StoredPlanOperationKind::Upload => "upload",
        mavkit::StoredPlanOperationKind::Download => "download",
        mavkit::StoredPlanOperationKind::Clear => "clear",
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
