use std::path::PathBuf;

use mavkit::dialect::MavMessage;
use mavlink::Message;
use mavlink::{MavHeader, MavlinkVersion};
use pyo3::prelude::*;

use crate::error::MavkitError;
use crate::raw_message::PyRawMessage;

type FileTlogWriter = mavkit::tlog::TlogWriter<std::io::BufWriter<std::fs::File>>;

fn tlog_to_py_err(e: mavkit::tlog::TlogError) -> PyErr {
    MavkitError::new_err(e.to_string())
}

fn parse_raw_message(message: &PyRawMessage) -> PyResult<(MavHeader, MavMessage)> {
    let payload = message.payload();
    let msg = MavMessage::parse(MavlinkVersion::V2, message.message_id(), &payload)
        .map_err(|e| MavkitError::new_err(format!("failed to parse payload: {e}")))?;
    let header = MavHeader {
        sequence: message.sequence(),
        system_id: message.system_id(),
        component_id: message.component_id(),
    };
    Ok((header, msg))
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

#[pyclass(name = "TlogWriter")]
pub struct PyTlogWriter {
    inner: std::sync::Mutex<Option<FileTlogWriter>>,
}

impl PyTlogWriter {
    fn with_writer<R>(&self, f: impl FnOnce(&mut FileTlogWriter) -> PyResult<R>) -> PyResult<R> {
        let mut guard = self
            .inner
            .lock()
            .map_err(|_| MavkitError::new_err("tlog writer lock poisoned"))?;
        let writer = guard
            .as_mut()
            .ok_or_else(|| MavkitError::new_err("tlog writer is closed"))?;
        f(writer)
    }
}

#[pymethods]
impl PyTlogWriter {
    #[new]
    fn new(path: &str) -> PyResult<Self> {
        let file = std::fs::File::create(path)
            .map_err(|e| MavkitError::new_err(format!("failed to create file {path}: {e}")))?;
        let writer =
            mavkit::tlog::TlogWriter::new(std::io::BufWriter::new(file), MavlinkVersion::V2);
        Ok(Self {
            inner: std::sync::Mutex::new(Some(writer)),
        })
    }

    fn write(&self, message: &PyRawMessage) -> PyResult<usize> {
        let (header, msg) = parse_raw_message(message)?;
        self.with_writer(|writer| writer.write_now(&header, &msg).map_err(tlog_to_py_err))
    }

    fn write_entry(&self, timestamp_usec: u64, message: &PyRawMessage) -> PyResult<usize> {
        let (header, msg) = parse_raw_message(message)?;
        self.with_writer(|writer| {
            writer
                .write_entry(timestamp_usec, &header, &msg)
                .map_err(tlog_to_py_err)
        })
    }

    fn flush(&self) -> PyResult<()> {
        self.with_writer(|writer| writer.flush().map_err(tlog_to_py_err))
    }

    fn close(&self) -> PyResult<()> {
        let mut guard = self
            .inner
            .lock()
            .map_err(|_| MavkitError::new_err("tlog writer lock poisoned"))?;
        if let Some(writer) = guard.as_mut() {
            writer.flush().map_err(tlog_to_py_err)?;
        }
        guard.take();
        Ok(())
    }

    fn __repr__(&self) -> String {
        let status = match self.inner.lock() {
            Ok(guard) => {
                if guard.is_some() {
                    "open"
                } else {
                    "closed"
                }
            }
            Err(_) => "poisoned",
        };
        format!("TlogWriter(status={status})")
    }

    fn __enter__(slf: PyRef<'_, Self>) -> PyRef<'_, Self> {
        slf
    }

    #[pyo3(signature = (_exc_type=None, _exc_val=None, _exc_tb=None))]
    fn __exit__<'py>(
        &self,
        _exc_type: Option<&Bound<'py, PyAny>>,
        _exc_val: Option<&Bound<'py, PyAny>>,
        _exc_tb: Option<&Bound<'py, PyAny>>,
    ) -> PyResult<()> {
        self.close()
    }
}
