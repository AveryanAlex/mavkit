use std::path::PathBuf;

use mavlink::Message;
use pyo3::prelude::*;

use crate::error::MavkitError;

fn tlog_to_py_err(e: mavkit::tlog::TlogError) -> PyErr {
    MavkitError::new_err(e.to_string())
}

#[pyclass(name = "TlogEntry", frozen)]
pub struct PyTlogEntry {
    timestamp_usec: u64,
    message_id: u32,
    message_name: &'static str,
    message_json: String,
}

#[pymethods]
impl PyTlogEntry {
    #[getter]
    fn timestamp_usec(&self) -> u64 {
        self.timestamp_usec
    }

    #[getter]
    fn message_id(&self) -> u32 {
        self.message_id
    }

    #[getter]
    fn message_name(&self) -> &str {
        self.message_name
    }

    fn message_json(&self) -> &str {
        &self.message_json
    }

    fn __repr__(&self) -> String {
        format!(
            "TlogEntry(timestamp_usec={}, message={})",
            self.timestamp_usec, self.message_name,
        )
    }
}

impl PyTlogEntry {
    fn from_rust(entry: mavkit::tlog::TlogEntry) -> PyResult<Self> {
        let message_id = entry.message.message_id();
        let message_name = entry.message.message_name();
        let message_json = serde_json::to_string(&entry.message)
            .map_err(|e| MavkitError::new_err(format!("JSON serialization error: {e}")))?;
        Ok(Self {
            timestamp_usec: entry.timestamp_usec,
            message_id,
            message_name,
            message_json,
        })
    }
}

#[pyclass(name = "TlogFile", frozen)]
pub struct PyTlogFile {
    path: PathBuf,
}

#[pymethods]
impl PyTlogFile {
    #[staticmethod]
    fn open<'py>(py: Python<'py>, path: &str) -> PyResult<Bound<'py, PyAny>> {
        let p = path.to_string();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let _ = mavkit::tlog::TlogFile::open(&p)
                .await
                .map_err(tlog_to_py_err)?;
            Ok(PyTlogFile {
                path: PathBuf::from(p),
            })
        })
    }

    fn entries<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let path = self.path.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let f = mavkit::tlog::TlogFile::open(&path)
                .await
                .map_err(tlog_to_py_err)?;
            let reader = f.entries().await.map_err(tlog_to_py_err)?;
            let rust_entries = reader.collect().await.map_err(tlog_to_py_err)?;
            let py_entries: Vec<PyTlogEntry> = rust_entries
                .into_iter()
                .map(PyTlogEntry::from_rust)
                .collect::<PyResult<_>>()?;
            Ok(py_entries)
        })
    }

    fn seek_to_timestamp<'py>(
        &self,
        py: Python<'py>,
        target_usec: u64,
    ) -> PyResult<Bound<'py, PyAny>> {
        let path = self.path.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let f = mavkit::tlog::TlogFile::open(&path)
                .await
                .map_err(tlog_to_py_err)?;
            let reader = f
                .seek_to_timestamp(target_usec)
                .await
                .map_err(tlog_to_py_err)?;
            let rust_entries = reader.collect().await.map_err(tlog_to_py_err)?;
            let py_entries: Vec<PyTlogEntry> = rust_entries
                .into_iter()
                .map(PyTlogEntry::from_rust)
                .collect::<PyResult<_>>()?;
            Ok(py_entries)
        })
    }

    fn time_range<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let path = self.path.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let f = mavkit::tlog::TlogFile::open(&path)
                .await
                .map_err(tlog_to_py_err)?;
            let range = f.time_range().await.map_err(tlog_to_py_err)?;
            Ok(range)
        })
    }
}
