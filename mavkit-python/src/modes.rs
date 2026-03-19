use std::sync::Arc;

use pyo3::exceptions::PyStopAsyncIteration;
use pyo3::prelude::*;

use crate::error::{duration_from_secs, to_py_err};
use crate::support::PySupportStateHandle;
use crate::vehicle::vehicle_label;

#[pyclass(name = "ModeCatalogSource", eq, frozen, from_py_object)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PyModeCatalogSource {
    AvailableModes,
    StaticArduPilotTable,
}

impl From<mavkit::ModeCatalogSource> for PyModeCatalogSource {
    fn from(value: mavkit::ModeCatalogSource) -> Self {
        match value {
            mavkit::ModeCatalogSource::AvailableModes => Self::AvailableModes,
            mavkit::ModeCatalogSource::StaticArduPilotTable => Self::StaticArduPilotTable,
        }
    }
}

#[pyclass(name = "ModeDescriptor", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyModeDescriptor {
    custom_mode: u32,
    name: String,
    user_selectable: bool,
    source: PyModeCatalogSource,
}

impl From<mavkit::ModeDescriptor> for PyModeDescriptor {
    fn from(value: mavkit::ModeDescriptor) -> Self {
        Self {
            custom_mode: value.custom_mode,
            name: value.name,
            user_selectable: value.user_selectable,
            source: value.source.into(),
        }
    }
}

#[pymethods]
impl PyModeDescriptor {
    #[new]
    #[pyo3(signature = (*, custom_mode, name, user_selectable, source))]
    fn new(
        custom_mode: u32,
        name: String,
        user_selectable: bool,
        source: PyModeCatalogSource,
    ) -> Self {
        Self {
            custom_mode,
            name,
            user_selectable,
            source,
        }
    }

    #[getter]
    fn custom_mode(&self) -> u32 {
        self.custom_mode
    }

    #[getter]
    fn name(&self) -> &str {
        &self.name
    }

    #[getter]
    fn user_selectable(&self) -> bool {
        self.user_selectable
    }

    #[getter]
    fn source(&self) -> PyModeCatalogSource {
        self.source
    }

    fn __repr__(&self) -> String {
        format!(
            "ModeDescriptor(name='{}', custom_mode={}, user_selectable={})",
            self.name, self.custom_mode, self.user_selectable
        )
    }
}

#[pyclass(name = "CurrentModeSource", eq, frozen, from_py_object)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PyCurrentModeSource {
    Heartbeat,
    CurrentModeMessage,
}

impl From<mavkit::CurrentModeSource> for PyCurrentModeSource {
    fn from(value: mavkit::CurrentModeSource) -> Self {
        match value {
            mavkit::CurrentModeSource::Heartbeat => Self::Heartbeat,
            mavkit::CurrentModeSource::CurrentModeMessage => Self::CurrentModeMessage,
        }
    }
}

#[pyclass(name = "CurrentMode", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyCurrentMode {
    custom_mode: u32,
    name: String,
    intended_custom_mode: Option<u32>,
    source: PyCurrentModeSource,
}

impl From<mavkit::CurrentMode> for PyCurrentMode {
    fn from(value: mavkit::CurrentMode) -> Self {
        Self {
            custom_mode: value.custom_mode,
            name: value.name,
            intended_custom_mode: value.intended_custom_mode,
            source: value.source.into(),
        }
    }
}

#[pymethods]
impl PyCurrentMode {
    #[new]
    #[pyo3(signature = (*, custom_mode, name, intended_custom_mode=None, source))]
    fn new(
        custom_mode: u32,
        name: String,
        intended_custom_mode: Option<u32>,
        source: PyCurrentModeSource,
    ) -> Self {
        Self {
            custom_mode,
            name,
            intended_custom_mode,
            source,
        }
    }

    #[getter]
    fn custom_mode(&self) -> u32 {
        self.custom_mode
    }

    #[getter]
    fn name(&self) -> &str {
        &self.name
    }

    #[getter]
    fn intended_custom_mode(&self) -> Option<u32> {
        self.intended_custom_mode
    }

    #[getter]
    fn source(&self) -> PyCurrentModeSource {
        self.source
    }

    fn __repr__(&self) -> String {
        format!(
            "CurrentMode(name='{}', custom_mode={}, intended_custom_mode={:?})",
            self.name, self.custom_mode, self.intended_custom_mode
        )
    }
}

#[pyclass(name = "ModeCatalogSubscription", frozen, skip_from_py_object)]
pub struct PyModeCatalogSubscription {
    inner: Arc<tokio::sync::Mutex<mavkit::ObservationSubscription<Vec<mavkit::ModeDescriptor>>>>,
}

#[pymethods]
impl PyModeCatalogSubscription {
    fn recv<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let inner = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let mut guard = inner.lock().await;
            match guard.recv().await {
                Some(value) => Ok(value
                    .into_iter()
                    .map(PyModeDescriptor::from)
                    .collect::<Vec<_>>()),
                None => Err(PyStopAsyncIteration::new_err(
                    "mode-catalog subscription closed",
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
                Some(value) => Ok(value
                    .into_iter()
                    .map(PyModeDescriptor::from)
                    .collect::<Vec<_>>()),
                None => Err(PyStopAsyncIteration::new_err(
                    "mode-catalog subscription closed",
                )),
            }
        })
    }
}

#[pyclass(name = "ModeCatalogHandle", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyModeCatalogHandle {
    inner: mavkit::ObservationHandle<Vec<mavkit::ModeDescriptor>>,
}

impl PyModeCatalogHandle {
    fn new(inner: mavkit::ObservationHandle<Vec<mavkit::ModeDescriptor>>) -> Self {
        Self { inner }
    }
}

#[pymethods]
impl PyModeCatalogHandle {
    fn latest(&self) -> Option<Vec<PyModeDescriptor>> {
        self.inner.latest().map(|modes| {
            modes
                .into_iter()
                .map(PyModeDescriptor::from)
                .collect::<Vec<_>>()
        })
    }

    fn wait<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let inner = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let modes = inner.wait().await.map_err(to_py_err)?;
            Ok(modes
                .into_iter()
                .map(PyModeDescriptor::from)
                .collect::<Vec<_>>())
        })
    }

    fn wait_timeout<'py>(&self, py: Python<'py>, timeout_secs: f64) -> PyResult<Bound<'py, PyAny>> {
        let inner = self.inner.clone();
        let timeout = duration_from_secs(timeout_secs)?;
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let modes = inner.wait_timeout(timeout).await.map_err(to_py_err)?;
            Ok(modes
                .into_iter()
                .map(PyModeDescriptor::from)
                .collect::<Vec<_>>())
        })
    }

    fn subscribe(&self) -> PyModeCatalogSubscription {
        PyModeCatalogSubscription {
            inner: Arc::new(tokio::sync::Mutex::new(self.inner.subscribe())),
        }
    }
}

#[pyclass(name = "CurrentModeSubscription", frozen, skip_from_py_object)]
pub struct PyCurrentModeSubscription {
    inner: Arc<tokio::sync::Mutex<mavkit::ObservationSubscription<mavkit::CurrentMode>>>,
}

#[pymethods]
impl PyCurrentModeSubscription {
    fn recv<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let inner = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let mut guard = inner.lock().await;
            match guard.recv().await {
                Some(value) => Ok(PyCurrentMode::from(value)),
                None => Err(PyStopAsyncIteration::new_err(
                    "current-mode subscription closed",
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
                Some(value) => Ok(PyCurrentMode::from(value)),
                None => Err(PyStopAsyncIteration::new_err(
                    "current-mode subscription closed",
                )),
            }
        })
    }
}

#[pyclass(name = "CurrentModeHandle", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyCurrentModeHandle {
    inner: mavkit::ObservationHandle<mavkit::CurrentMode>,
}

impl PyCurrentModeHandle {
    fn new(inner: mavkit::ObservationHandle<mavkit::CurrentMode>) -> Self {
        Self { inner }
    }
}

#[pymethods]
impl PyCurrentModeHandle {
    fn latest(&self) -> Option<PyCurrentMode> {
        self.inner.latest().map(Into::into)
    }

    fn wait<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let inner = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let value = inner.wait().await.map_err(to_py_err)?;
            Ok(PyCurrentMode::from(value))
        })
    }

    fn wait_timeout<'py>(&self, py: Python<'py>, timeout_secs: f64) -> PyResult<Bound<'py, PyAny>> {
        let inner = self.inner.clone();
        let timeout = duration_from_secs(timeout_secs)?;
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let value = inner.wait_timeout(timeout).await.map_err(to_py_err)?;
            Ok(PyCurrentMode::from(value))
        })
    }

    fn subscribe(&self) -> PyCurrentModeSubscription {
        PyCurrentModeSubscription {
            inner: Arc::new(tokio::sync::Mutex::new(self.inner.subscribe())),
        }
    }
}

#[pyclass(name = "ModesHandle", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyModesHandle {
    pub(crate) inner: mavkit::Vehicle,
}

impl PyModesHandle {
    pub(crate) fn new(inner: mavkit::Vehicle) -> Self {
        Self { inner }
    }
}

#[pymethods]
impl PyModesHandle {
    fn support(&self) -> PySupportStateHandle {
        PySupportStateHandle::new(self.inner.available_modes().support())
    }

    fn catalog(&self) -> PyModeCatalogHandle {
        PyModeCatalogHandle::new(self.inner.available_modes().catalog())
    }

    fn current(&self) -> PyCurrentModeHandle {
        PyCurrentModeHandle::new(self.inner.available_modes().current())
    }

    fn __len__(&self) -> usize {
        self.inner.available_modes().len()
    }

    fn __bool__(&self) -> bool {
        !self.inner.available_modes().is_empty()
    }

    fn __iter__(&self) -> Vec<PyModeDescriptor> {
        self.inner
            .available_modes()
            .catalog()
            .latest()
            .unwrap_or_default()
            .into_iter()
            .map(PyModeDescriptor::from)
            .collect()
    }

    fn __repr__(&self) -> String {
        format!("ModesHandle({})", vehicle_label(&self.inner))
    }
}
