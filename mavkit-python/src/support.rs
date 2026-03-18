use std::sync::Arc;

use pyo3::exceptions::PyStopAsyncIteration;
use pyo3::prelude::*;

use crate::error::{duration_from_secs, to_py_err};
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

#[pyclass(name = "SupportStateSubscription", frozen, skip_from_py_object)]
pub struct PySupportStateSubscription {
    inner: Arc<tokio::sync::Mutex<mavkit::ObservationSubscription<mavkit::SupportState>>>,
}

#[pymethods]
impl PySupportStateSubscription {
    fn recv<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let inner = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let mut guard = inner.lock().await;
            match guard.recv().await {
                Some(value) => Ok(PySupportState::from(value)),
                None => Err(PyStopAsyncIteration::new_err(
                    "support-state subscription closed",
                )),
            }
        })
    }

    fn __aiter__(slf: PyRef<'_, Self>) -> PyRef<'_, Self> {
        slf
    }

    fn __anext__<'py>(slf: PyRef<'py, Self>, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let inner = slf.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let mut guard = inner.lock().await;
            match guard.recv().await {
                Some(value) => Ok(PySupportState::from(value)),
                None => Err(PyStopAsyncIteration::new_err(
                    "support-state subscription closed",
                )),
            }
        })
    }
}

#[pyclass(name = "SupportStateHandle", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PySupportStateHandle {
    inner: mavkit::ObservationHandle<mavkit::SupportState>,
}

impl PySupportStateHandle {
    pub(crate) fn new(inner: mavkit::ObservationHandle<mavkit::SupportState>) -> Self {
        Self { inner }
    }
}

#[pymethods]
impl PySupportStateHandle {
    fn latest(&self) -> Option<PySupportState> {
        self.inner.latest().map(Into::into)
    }

    fn wait<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let inner = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let value = inner.wait().await.map_err(to_py_err)?;
            Ok(PySupportState::from(value))
        })
    }

    fn wait_timeout<'py>(&self, py: Python<'py>, timeout_secs: f64) -> PyResult<Bound<'py, PyAny>> {
        let inner = self.inner.clone();
        let timeout = duration_from_secs(timeout_secs)?;
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let value = inner.wait_timeout(timeout).await.map_err(to_py_err)?;
            Ok(PySupportState::from(value))
        })
    }

    fn subscribe(&self) -> PySupportStateSubscription {
        PySupportStateSubscription {
            inner: Arc::new(tokio::sync::Mutex::new(self.inner.subscribe())),
        }
    }
}

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
