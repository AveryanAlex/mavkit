use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use pyo3::prelude::*;
use tokio::sync::Mutex;

use crate::error::to_py_err;
use crate::macros::py_async_unit;

use super::{
    PyArduCopterGuidedHandle, PyArduPlaneGuidedHandle, PyArduRoverGuidedHandle,
    PyArduSubGuidedHandle, session_closed_error,
};

pub(super) struct PyGuidedSessionShared {
    session: Mutex<Option<mavkit::ArduGuidedSession>>,
    kind: mavkit::ArduGuidedKind,
    plane_kind: Option<mavkit::ArduPlaneKind>,
    closed: AtomicBool,
    label: String,
}

impl PyGuidedSessionShared {
    fn new(
        session: mavkit::ArduGuidedSession,
        kind: mavkit::ArduGuidedKind,
        plane_kind: Option<mavkit::ArduPlaneKind>,
        label: String,
    ) -> Self {
        Self {
            session: Mutex::new(Some(session)),
            kind,
            plane_kind,
            closed: AtomicBool::new(false),
            label,
        }
    }

    pub(super) fn ensure_python_open(&self) -> Result<(), mavkit::VehicleError> {
        if self.closed.load(Ordering::Acquire) {
            return Err(session_closed_error());
        }
        Ok(())
    }

    pub(super) async fn session_clone(
        &self,
    ) -> Result<mavkit::ArduGuidedSession, mavkit::VehicleError> {
        self.ensure_python_open()?;
        let guard = self.session.lock().await;
        guard.as_ref().cloned().ok_or_else(session_closed_error)
    }

    pub(super) async fn close(&self) -> Result<(), mavkit::VehicleError> {
        if self.closed.swap(true, Ordering::AcqRel) {
            return Ok(());
        }

        let session = self.session.lock().await.take();
        if let Some(session) = session {
            session.close().await?;
        }
        Ok(())
    }

    fn kind(&self) -> mavkit::ArduGuidedKind {
        self.kind
    }

    pub(super) fn plane_kind(&self) -> Option<mavkit::ArduPlaneKind> {
        self.plane_kind
    }

    pub(super) fn label(&self) -> &str {
        &self.label
    }

    fn is_closed(&self) -> bool {
        self.closed.load(Ordering::Acquire)
    }
}

#[pyclass(name = "ArduGuidedSession", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyArduGuidedSession {
    shared: Arc<PyGuidedSessionShared>,
}

impl PyArduGuidedSession {
    pub(crate) fn new(session: mavkit::ArduGuidedSession, label: String) -> Self {
        let kind = session.kind();
        let plane_kind = match session.specific() {
            mavkit::GuidedSpecific::Plane(plane) => Some(plane.kind()),
            _ => None,
        };

        Self {
            shared: Arc::new(PyGuidedSessionShared::new(session, kind, plane_kind, label)),
        }
    }

    #[cfg_attr(not(test), allow(dead_code))]
    pub(crate) async fn close_impl(&self) -> Result<(), mavkit::VehicleError> {
        self.shared.close().await
    }

    fn status(&self) -> &'static str {
        if self.shared.is_closed() {
            "closed"
        } else {
            "open"
        }
    }

    pub(crate) fn copter_impl(
        &self,
    ) -> Result<Option<PyArduCopterGuidedHandle>, mavkit::VehicleError> {
        self.shared.ensure_python_open()?;
        Ok((self.shared.kind() == mavkit::ArduGuidedKind::Copter)
            .then_some(PyArduCopterGuidedHandle::new(self.shared.clone())))
    }

    pub(crate) fn plane_impl(
        &self,
    ) -> Result<Option<PyArduPlaneGuidedHandle>, mavkit::VehicleError> {
        self.shared.ensure_python_open()?;
        Ok((self.shared.kind() == mavkit::ArduGuidedKind::Plane)
            .then_some(PyArduPlaneGuidedHandle::new(self.shared.clone())))
    }

    pub(crate) fn rover_impl(
        &self,
    ) -> Result<Option<PyArduRoverGuidedHandle>, mavkit::VehicleError> {
        self.shared.ensure_python_open()?;
        Ok((self.shared.kind() == mavkit::ArduGuidedKind::Rover)
            .then_some(PyArduRoverGuidedHandle::new(self.shared.clone())))
    }

    pub(crate) fn sub_impl(&self) -> Result<Option<PyArduSubGuidedHandle>, mavkit::VehicleError> {
        self.shared.ensure_python_open()?;
        Ok((self.shared.kind() == mavkit::ArduGuidedKind::Sub)
            .then_some(PyArduSubGuidedHandle::new(self.shared.clone())))
    }
}

#[pymethods]
impl PyArduGuidedSession {
    fn close<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(py, shared = self.shared.clone(); shared.close())
    }

    fn copter(&self) -> PyResult<Option<PyArduCopterGuidedHandle>> {
        self.copter_impl().map_err(to_py_err)
    }

    fn plane(&self) -> PyResult<Option<PyArduPlaneGuidedHandle>> {
        self.plane_impl().map_err(to_py_err)
    }

    fn rover(&self) -> PyResult<Option<PyArduRoverGuidedHandle>> {
        self.rover_impl().map_err(to_py_err)
    }

    fn sub(&self) -> PyResult<Option<PyArduSubGuidedHandle>> {
        self.sub_impl().map_err(to_py_err)
    }

    fn __aenter__<'py>(slf: &Bound<'py, Self>, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let obj = slf.clone().into_any().unbind();
        pyo3_async_runtimes::tokio::future_into_py(py, async move { Ok(obj) })
    }

    #[pyo3(signature = (_exc_type=None, _exc_val=None, _exc_tb=None))]
    fn __aexit__<'py>(
        &self,
        py: Python<'py>,
        _exc_type: Option<&Bound<'py, PyAny>>,
        _exc_val: Option<&Bound<'py, PyAny>>,
        _exc_tb: Option<&Bound<'py, PyAny>>,
    ) -> PyResult<Bound<'py, PyAny>> {
        let shared = self.shared.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            shared.close().await.map_err(to_py_err)?;
            Ok(false)
        })
    }

    fn __repr__(&self) -> String {
        format!(
            "ArduGuidedSession({}, status={})",
            self.shared.label(),
            self.status()
        )
    }
}
