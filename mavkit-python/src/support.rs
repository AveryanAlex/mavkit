use pyo3::prelude::*;

use crate::macros::define_observation_wrapper;
use crate::vehicle::vehicle_label;

#[pyclass(name = "SupportState", eq, frozen, from_py_object)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PySupportState {
    Unknown,
    Supported,
    Unsupported,
}

impl From<mavkit::SupportState> for PySupportState {
    fn from(value: mavkit::SupportState) -> Self {
        match value {
            mavkit::SupportState::Unknown => Self::Unknown,
            mavkit::SupportState::Supported => Self::Supported,
            mavkit::SupportState::Unsupported => Self::Unsupported,
        }
    }
}

define_observation_wrapper!(
    PySupportStateHandle,
    PySupportStateSubscription,
    mavkit::SupportState,
    PySupportState,
    "SupportStateHandle",
    "SupportStateSubscription",
    "support-state subscription closed"
);

#[pyclass(name = "SupportHandle", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PySupportHandle {
    pub(crate) inner: mavkit::Vehicle,
}

impl PySupportHandle {
    pub(crate) fn new(inner: mavkit::Vehicle) -> Self {
        Self { inner }
    }
}

#[pymethods]
impl PySupportHandle {
    fn command_int(&self) -> PySupportStateHandle {
        PySupportStateHandle::new(self.inner.support().command_int())
    }

    fn ftp(&self) -> PySupportStateHandle {
        PySupportStateHandle::new(self.inner.support().ftp())
    }

    fn terrain(&self) -> PySupportStateHandle {
        PySupportStateHandle::new(self.inner.support().terrain())
    }

    fn mission_fence(&self) -> PySupportStateHandle {
        PySupportStateHandle::new(self.inner.support().mission_fence())
    }

    fn mission_rally(&self) -> PySupportStateHandle {
        PySupportStateHandle::new(self.inner.support().mission_rally())
    }

    fn ardupilot(&self) -> PySupportStateHandle {
        PySupportStateHandle::new(self.inner.support().ardupilot())
    }

    fn __repr__(&self) -> String {
        format!("SupportHandle({})", vehicle_label(&self.inner))
    }
}
