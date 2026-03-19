use pyo3::exceptions::{PyKeyError, PyStopAsyncIteration};
use pyo3::prelude::*;
use std::collections::HashMap;
use std::sync::Arc;

use crate::enums::*;
use crate::error::{duration_from_secs, to_py_err};
use crate::vehicle::vehicle_label;

#[pyclass(name = "SyncState", eq, frozen, from_py_object)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PySyncState {
    Unknown,
    Current,
    PossiblyStale,
}

impl From<mavkit::SyncState> for PySyncState {
    fn from(value: mavkit::SyncState) -> Self {
        match value {
            mavkit::SyncState::Unknown => Self::Unknown,
            mavkit::SyncState::Current => Self::Current,
            mavkit::SyncState::PossiblyStale => Self::PossiblyStale,
        }
    }
}

#[pyclass(name = "ParamOperationKind", eq, frozen, from_py_object)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PyParamOperationKind {
    DownloadAll,
    WriteBatch,
}

impl From<mavkit::ParamOperationKind> for PyParamOperationKind {
    fn from(value: mavkit::ParamOperationKind) -> Self {
        match value {
            mavkit::ParamOperationKind::DownloadAll => Self::DownloadAll,
            mavkit::ParamOperationKind::WriteBatch => Self::WriteBatch,
        }
    }
}

// --- Param ---

#[pyclass(name = "Param", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyParam {
    pub(crate) inner: mavkit::Param,
}

#[pymethods]
impl PyParam {
    #[getter]
    fn name(&self) -> &str {
        &self.inner.name
    }
    #[getter]
    fn value(&self) -> f32 {
        self.inner.value
    }
    #[getter]
    fn param_type(&self) -> PyParamType {
        self.inner.param_type.into()
    }
    #[getter]
    fn index(&self) -> u16 {
        self.inner.index
    }

    fn __repr__(&self) -> String {
        format!(
            "Param(name='{}', value={}, type={:?})",
            self.inner.name, self.inner.value, self.inner.param_type
        )
    }
}

// --- ParamStore ---

#[pyclass(name = "ParamStore", frozen, from_py_object)]
#[derive(Clone)]
pub struct PyParamStore {
    pub(crate) inner: mavkit::ParamStore,
}

#[pymethods]
impl PyParamStore {
    #[getter]
    fn expected_count(&self) -> u16 {
        self.inner.expected_count
    }

    fn get(&self, name: &str) -> Option<PyParam> {
        self.inner
            .params
            .get(name)
            .map(|p| PyParam { inner: p.clone() })
    }

    fn keys(&self) -> Vec<String> {
        let mut keys: Vec<String> = self.inner.params.keys().cloned().collect();
        keys.sort();
        keys
    }

    fn values(&self) -> Vec<PyParam> {
        let mut items: Vec<_> = self.inner.params.values().collect();
        items.sort_by_key(|p| &p.name);
        items
            .into_iter()
            .map(|p| PyParam { inner: p.clone() })
            .collect()
    }

    fn is_empty(&self) -> bool {
        self.inner.is_empty()
    }

    fn __len__(&self) -> usize {
        self.inner.params.len()
    }

    fn __bool__(&self) -> bool {
        !self.inner.is_empty()
    }

    fn __iter__(&self) -> Vec<PyParam> {
        let mut items: Vec<_> = self.inner.params.values().collect();
        items.sort_by_key(|p| &p.name);
        items
            .into_iter()
            .map(|p| PyParam { inner: p.clone() })
            .collect()
    }

    fn __contains__(&self, name: &str) -> bool {
        self.inner.params.contains_key(name)
    }

    fn __getitem__(&self, name: &str) -> PyResult<PyParam> {
        self.inner
            .params
            .get(name)
            .map(|p| PyParam { inner: p.clone() })
            .ok_or_else(|| PyKeyError::new_err(name.to_string()))
    }

    fn __repr__(&self) -> String {
        format!(
            "ParamStore({}/{} params)",
            self.inner.params.len(),
            self.inner.expected_count
        )
    }
}

// --- ParamProgress ---

#[pyclass(name = "ParamProgress", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyParamProgress {
    pub(crate) inner: mavkit::ParamOperationProgress,
}

#[pymethods]
impl PyParamProgress {
    #[getter]
    fn phase(&self) -> PyParamTransferPhase {
        match self.inner {
            mavkit::ParamOperationProgress::Downloading { .. } => PyParamTransferPhase::Downloading,
            mavkit::ParamOperationProgress::Writing { .. } => PyParamTransferPhase::Writing,
            mavkit::ParamOperationProgress::Completed => PyParamTransferPhase::Completed,
            mavkit::ParamOperationProgress::Failed => PyParamTransferPhase::Failed,
            mavkit::ParamOperationProgress::Cancelled => PyParamTransferPhase::Cancelled,
        }
    }
    #[getter]
    fn received(&self) -> u16 {
        match &self.inner {
            mavkit::ParamOperationProgress::Downloading { received, .. } => *received,
            _ => 0,
        }
    }
    #[getter]
    fn expected(&self) -> u16 {
        match &self.inner {
            mavkit::ParamOperationProgress::Downloading { expected, .. } => expected.unwrap_or(0),
            _ => 0,
        }
    }

    #[getter]
    fn expected_count(&self) -> Option<u16> {
        match &self.inner {
            mavkit::ParamOperationProgress::Downloading { expected, .. } => *expected,
            _ => None,
        }
    }

    #[getter]
    fn index(&self) -> u16 {
        match &self.inner {
            mavkit::ParamOperationProgress::Writing { index, .. } => *index,
            _ => 0,
        }
    }

    #[getter]
    fn total(&self) -> u16 {
        match &self.inner {
            mavkit::ParamOperationProgress::Writing { total, .. } => *total,
            _ => 0,
        }
    }

    #[getter]
    fn name(&self) -> Option<String> {
        match &self.inner {
            mavkit::ParamOperationProgress::Writing { name, .. } => Some(name.clone()),
            _ => None,
        }
    }

    fn __repr__(&self) -> String {
        format!("ParamProgress(phase={:?})", self.phase())
    }
}

// --- ParamWriteResult ---

#[pyclass(name = "ParamWriteResult", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyParamWriteResult {
    pub(crate) inner: mavkit::ParamWriteResult,
}

#[pymethods]
impl PyParamWriteResult {
    #[getter]
    fn name(&self) -> &str {
        &self.inner.name
    }
    #[getter]
    fn requested_value(&self) -> f32 {
        self.inner.requested_value
    }
    #[getter]
    fn confirmed_value(&self) -> f32 {
        self.inner.confirmed_value
    }
    #[getter]
    fn success(&self) -> bool {
        self.inner.success
    }

    fn __repr__(&self) -> String {
        format!(
            "ParamWriteResult(name='{}', requested={}, confirmed={}, success={})",
            self.inner.name,
            self.inner.requested_value,
            self.inner.confirmed_value,
            self.inner.success
        )
    }
}

#[pyclass(name = "ParamState", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyParamState {
    inner: mavkit::ParamState,
}

#[pymethods]
impl PyParamState {
    #[getter]
    fn store(&self) -> Option<PyParamStore> {
        self.inner.store.clone().map(|inner| PyParamStore { inner })
    }

    #[getter]
    fn sync(&self) -> PySyncState {
        self.inner.sync.into()
    }

    #[getter]
    fn active_op(&self) -> Option<PyParamOperationKind> {
        self.inner.active_op.map(Into::into)
    }

    fn __repr__(&self) -> String {
        format!(
            "ParamState(sync={:?}, active_op={:?}, has_store={})",
            self.inner.sync,
            self.inner.active_op,
            self.inner.store.is_some()
        )
    }
}

#[pyclass(name = "ParamStateSubscription", frozen, skip_from_py_object)]
pub struct PyParamStateSubscription {
    inner: Arc<tokio::sync::Mutex<mavkit::ObservationSubscription<mavkit::ParamState>>>,
}

#[pymethods]
impl PyParamStateSubscription {
    fn recv<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let inner = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let mut guard = inner.lock().await;
            match guard.recv().await {
                Some(value) => Ok(PyParamState { inner: value }),
                None => Err(PyStopAsyncIteration::new_err(
                    "param-state subscription closed",
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
                Some(value) => Ok(PyParamState { inner: value }),
                None => Err(PyStopAsyncIteration::new_err(
                    "param-state subscription closed",
                )),
            }
        })
    }
}

#[pyclass(name = "ParamProgressSubscription", frozen, skip_from_py_object)]
pub struct PyParamProgressSubscription {
    inner: Arc<tokio::sync::Mutex<mavkit::ObservationSubscription<mavkit::ParamOperationProgress>>>,
}

#[pymethods]
impl PyParamProgressSubscription {
    fn recv<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let inner = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let mut guard = inner.lock().await;
            match guard.recv().await {
                Some(value) => Ok(PyParamProgress { inner: value }),
                None => Err(PyStopAsyncIteration::new_err(
                    "param-progress subscription closed",
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
                Some(value) => Ok(PyParamProgress { inner: value }),
                None => Err(PyStopAsyncIteration::new_err(
                    "param-progress subscription closed",
                )),
            }
        })
    }
}

#[pyclass(name = "ParamDownloadOp", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyParamDownloadOp {
    inner: Arc<mavkit::ParamDownloadOp>,
}

#[pymethods]
impl PyParamDownloadOp {
    fn latest(&self) -> Option<PyParamProgress> {
        self.inner.latest().map(|inner| PyParamProgress { inner })
    }

    fn subscribe(&self) -> PyParamProgressSubscription {
        PyParamProgressSubscription {
            inner: Arc::new(tokio::sync::Mutex::new(self.inner.subscribe())),
        }
    }

    fn cancel(&self) {
        self.inner.cancel();
    }

    fn wait<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let inner = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let store = inner.wait().await.map_err(to_py_err)?;
            Ok(PyParamStore { inner: store })
        })
    }
}

#[pyclass(name = "ParamWriteBatchOp", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyParamWriteBatchOp {
    inner: Arc<mavkit::ParamWriteBatchOp>,
}

#[pymethods]
impl PyParamWriteBatchOp {
    fn latest(&self) -> Option<PyParamProgress> {
        self.inner.latest().map(|inner| PyParamProgress { inner })
    }

    fn subscribe(&self) -> PyParamProgressSubscription {
        PyParamProgressSubscription {
            inner: Arc::new(tokio::sync::Mutex::new(self.inner.subscribe())),
        }
    }

    fn cancel(&self) {
        self.inner.cancel();
    }

    fn wait<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let inner = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let results = inner.wait().await.map_err(to_py_err)?;
            Ok(results
                .into_iter()
                .map(|inner| PyParamWriteResult { inner })
                .collect::<Vec<_>>())
        })
    }
}

#[pyclass(name = "ParamsHandle", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyParamsHandle {
    pub(crate) inner: mavkit::Vehicle,
}

impl PyParamsHandle {
    pub(crate) fn new(inner: mavkit::Vehicle) -> Self {
        Self { inner }
    }
}

#[pymethods]
impl PyParamsHandle {
    /// Look up a single cached parameter by name from the last download.
    fn get(&self, name: &str) -> Option<PyParam> {
        self.inner
            .params()
            .get(name)
            .map(|inner| PyParam { inner })
    }

    fn latest(&self) -> Option<PyParamState> {
        self.inner
            .params()
            .latest()
            .map(|inner| PyParamState { inner })
    }

    fn wait<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let inner = vehicle.params().wait().await;
            Ok(PyParamState { inner })
        })
    }

    fn wait_timeout<'py>(&self, py: Python<'py>, timeout_secs: f64) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        let timeout = duration_from_secs(timeout_secs)?;
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let inner = vehicle
                .params()
                .wait_timeout(timeout)
                .await
                .map_err(to_py_err)?;
            Ok(PyParamState { inner })
        })
    }

    fn subscribe(&self) -> PyParamStateSubscription {
        PyParamStateSubscription {
            inner: Arc::new(tokio::sync::Mutex::new(self.inner.params().subscribe())),
        }
    }

    fn download_all(&self) -> PyResult<PyParamDownloadOp> {
        let op = self.inner.params().download_all().map_err(to_py_err)?;
        Ok(PyParamDownloadOp {
            inner: Arc::new(op),
        })
    }

    fn write<'py>(&self, py: Python<'py>, name: &str, value: f32) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        let name = name.to_string();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let inner = vehicle
                .params()
                .write(&name, value)
                .await
                .map_err(to_py_err)?;
            Ok(PyParamWriteResult { inner })
        })
    }

    fn write_batch(&self, batch: Vec<(String, f32)>) -> PyResult<PyParamWriteBatchOp> {
        let op = self.inner.params().write_batch(batch).map_err(to_py_err)?;
        Ok(PyParamWriteBatchOp {
            inner: Arc::new(op),
        })
    }

    fn __repr__(&self) -> String {
        format!("ParamsHandle({})", vehicle_label(&self.inner))
    }
}

// --- Free functions ---

#[pyfunction]
pub fn format_param_file(store: &PyParamStore) -> String {
    mavkit::format_param_file(&store.inner)
}

#[pyfunction]
pub fn parse_param_file(contents: &str) -> PyResult<HashMap<String, f32>> {
    mavkit::parse_param_file(contents)
        .map(|items| items.into_iter().collect())
        .map_err(|err| pyo3::exceptions::PyValueError::new_err(err.to_string()))
}
