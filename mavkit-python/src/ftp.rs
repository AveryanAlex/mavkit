use std::sync::Arc;

use pyo3::prelude::*;
use pyo3::types::PyBytes;

use crate::error::{duration_from_secs, to_py_err};
use crate::macros::py_subscription;
use crate::vehicle::vehicle_label;

#[pyclass(name = "FtpTarget", frozen, from_py_object)]
#[derive(Clone)]
pub struct PyFtpTarget {
    inner: mavkit::FtpTarget,
}

#[pymethods]
impl PyFtpTarget {
    #[new]
    #[pyo3(signature = (system_id, component_id, network_id=0))]
    fn new(system_id: u8, component_id: u8, network_id: u8) -> Self {
        let mut inner = mavkit::FtpTarget::new(system_id, component_id);
        inner.network_id = network_id;
        Self { inner }
    }

    #[getter]
    fn network_id(&self) -> u8 {
        self.inner.network_id
    }

    #[getter]
    fn system_id(&self) -> u8 {
        self.inner.system_id
    }

    #[getter]
    fn component_id(&self) -> u8 {
        self.inner.component_id
    }

    fn __repr__(&self) -> String {
        format!(
            "FtpTarget(network_id={}, system_id={}, component_id={})",
            self.inner.network_id, self.inner.system_id, self.inner.component_id
        )
    }
}

#[pyclass(name = "FtpEntryKind", eq, frozen, from_py_object)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PyFtpEntryKind {
    File,
    Directory,
    Skip,
    Unknown,
}

impl From<mavkit::FtpEntryKind> for PyFtpEntryKind {
    fn from(value: mavkit::FtpEntryKind) -> Self {
        match value {
            mavkit::FtpEntryKind::File => Self::File,
            mavkit::FtpEntryKind::Directory => Self::Directory,
            mavkit::FtpEntryKind::Skip => Self::Skip,
            mavkit::FtpEntryKind::Unknown => Self::Unknown,
        }
    }
}

#[pyclass(name = "FtpEntry", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyFtpEntry {
    inner: mavkit::FtpEntry,
}

impl From<mavkit::FtpEntry> for PyFtpEntry {
    fn from(inner: mavkit::FtpEntry) -> Self {
        Self { inner }
    }
}

#[pymethods]
impl PyFtpEntry {
    #[getter]
    fn name(&self) -> &str {
        &self.inner.name
    }

    #[getter]
    fn kind(&self) -> PyFtpEntryKind {
        self.inner.kind.into()
    }

    #[getter]
    fn size(&self) -> Option<u64> {
        self.inner.size
    }

    fn __repr__(&self) -> String {
        format!(
            "FtpEntry(name={:?}, kind={:?}, size={:?})",
            self.inner.name, self.inner.kind, self.inner.size
        )
    }
}

fn ftp_progress_phase_name(progress: &mavkit::FtpOperationProgress) -> &'static str {
    match progress {
        mavkit::FtpOperationProgress::Opening => "opening",
        mavkit::FtpOperationProgress::Downloading { .. } => "downloading",
        mavkit::FtpOperationProgress::Uploading { .. } => "uploading",
        mavkit::FtpOperationProgress::Finalizing => "finalizing",
        mavkit::FtpOperationProgress::Completed => "completed",
        mavkit::FtpOperationProgress::Failed => "failed",
        mavkit::FtpOperationProgress::Cancelled => "cancelled",
    }
}

#[pyclass(name = "FtpOperationProgress", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyFtpOperationProgress {
    inner: mavkit::FtpOperationProgress,
}

impl From<mavkit::FtpOperationProgress> for PyFtpOperationProgress {
    fn from(inner: mavkit::FtpOperationProgress) -> Self {
        Self { inner }
    }
}

#[pymethods]
impl PyFtpOperationProgress {
    #[getter]
    fn phase(&self) -> &str {
        ftp_progress_phase_name(&self.inner)
    }

    #[getter]
    fn bytes_transferred(&self) -> u64 {
        match self.inner {
            mavkit::FtpOperationProgress::Downloading {
                bytes_transferred, ..
            }
            | mavkit::FtpOperationProgress::Uploading {
                bytes_transferred, ..
            } => bytes_transferred,
            _ => 0,
        }
    }

    #[getter]
    fn total_bytes(&self) -> Option<u64> {
        match self.inner {
            mavkit::FtpOperationProgress::Downloading { total_bytes, .. } => total_bytes,
            mavkit::FtpOperationProgress::Uploading { total_bytes, .. } => Some(total_bytes),
            _ => None,
        }
    }

    fn __repr__(&self) -> String {
        format!("FtpOperationProgress(phase={:?})", self.phase())
    }
}

py_subscription!(
    PyFtpProgressSubscription,
    mavkit::FtpOperationProgress,
    PyFtpOperationProgress,
    "FtpProgressSubscription",
    "FTP-progress subscription closed"
);

macro_rules! define_ftp_op {
    ($rust_name:ident, $py_name:literal, $inner:ty, $map:expr) => {
        #[pyclass(name = $py_name, frozen, skip_from_py_object)]
        #[derive(Clone)]
        pub struct $rust_name {
            inner: Arc<$inner>,
        }

        #[pymethods]
        impl $rust_name {
            fn latest(&self) -> Option<PyFtpOperationProgress> {
                self.inner.latest().map(Into::into)
            }

            fn subscribe(&self) -> PyFtpProgressSubscription {
                PyFtpProgressSubscription {
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

define_ftp_op!(
    PyFtpDownloadOp,
    "FtpDownloadOp",
    mavkit::FtpDownloadOp,
    |value: Vec<u8>| Ok(Python::attach(|py| PyBytes::new(py, &value).unbind()))
);

define_ftp_op!(
    PyFtpUploadOp,
    "FtpUploadOp",
    mavkit::FtpUploadOp,
    |_value: ()| Ok(())
);

#[pyclass(name = "FtpHandle", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyFtpHandle {
    inner: mavkit::Vehicle,
    target: Option<mavkit::FtpTarget>,
}

impl PyFtpHandle {
    pub(crate) fn new(inner: mavkit::Vehicle) -> Self {
        Self {
            inner,
            target: None,
        }
    }

    fn rust_handle(&self) -> mavkit::FtpHandle<'_> {
        ftp_handle(&self.inner, self.target)
    }
}

fn ftp_handle(
    vehicle: &mavkit::Vehicle,
    target: Option<mavkit::FtpTarget>,
) -> mavkit::FtpHandle<'_> {
    let handle = vehicle.ftp();
    match target {
        Some(target) => handle.with_target(target),
        None => handle,
    }
}

#[pymethods]
impl PyFtpHandle {
    fn with_target(&self, target: &PyFtpTarget) -> Self {
        Self {
            inner: self.inner.clone(),
            target: Some(target.inner),
        }
    }

    fn list_directory<'py>(&self, py: Python<'py>, path: &str) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        let target = self.target;
        let path = path.to_string();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let entries = ftp_handle(&vehicle, target)
                .list_directory(&path)
                .await
                .map_err(to_py_err)?;
            Ok(entries
                .into_iter()
                .map(Into::into)
                .collect::<Vec<PyFtpEntry>>())
        })
    }

    fn download(&self, remote_path: &str) -> PyResult<PyFtpDownloadOp> {
        let op = self
            .rust_handle()
            .download(remote_path)
            .map_err(to_py_err)?;
        Ok(PyFtpDownloadOp {
            inner: Arc::new(op),
        })
    }

    fn upload(&self, remote_path: &str, data: Vec<u8>) -> PyResult<PyFtpUploadOp> {
        let op = self
            .rust_handle()
            .upload(remote_path, data)
            .map_err(to_py_err)?;
        Ok(PyFtpUploadOp {
            inner: Arc::new(op),
        })
    }

    fn remove_file<'py>(&self, py: Python<'py>, path: &str) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        let target = self.target;
        let path = path.to_string();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            ftp_handle(&vehicle, target)
                .remove_file(&path)
                .await
                .map_err(to_py_err)
        })
    }

    fn create_directory<'py>(&self, py: Python<'py>, path: &str) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        let target = self.target;
        let path = path.to_string();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            ftp_handle(&vehicle, target)
                .create_directory(&path)
                .await
                .map_err(to_py_err)
        })
    }

    fn remove_directory<'py>(&self, py: Python<'py>, path: &str) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        let target = self.target;
        let path = path.to_string();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            ftp_handle(&vehicle, target)
                .remove_directory(&path)
                .await
                .map_err(to_py_err)
        })
    }

    fn rename<'py>(
        &self,
        py: Python<'py>,
        from_path: &str,
        to_path: &str,
    ) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        let target = self.target;
        let from_path = from_path.to_string();
        let to_path = to_path.to_string();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            ftp_handle(&vehicle, target)
                .rename(&from_path, &to_path)
                .await
                .map_err(to_py_err)
        })
    }

    fn truncate<'py>(
        &self,
        py: Python<'py>,
        path: &str,
        length: u32,
    ) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        let target = self.target;
        let path = path.to_string();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            ftp_handle(&vehicle, target)
                .truncate(&path, length)
                .await
                .map_err(to_py_err)
        })
    }

    fn crc32<'py>(&self, py: Python<'py>, path: &str) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        let target = self.target;
        let path = path.to_string();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            ftp_handle(&vehicle, target)
                .crc32(&path)
                .await
                .map_err(to_py_err)
        })
    }

    fn __repr__(&self) -> String {
        format!("FtpHandle({})", vehicle_label(&self.inner))
    }
}

pub fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyFtpTarget>()?;
    m.add_class::<PyFtpEntryKind>()?;
    m.add_class::<PyFtpEntry>()?;
    m.add_class::<PyFtpOperationProgress>()?;
    m.add_class::<PyFtpProgressSubscription>()?;
    m.add_class::<PyFtpDownloadOp>()?;
    m.add_class::<PyFtpUploadOp>()?;
    m.add_class::<PyFtpHandle>()?;
    Ok(())
}
