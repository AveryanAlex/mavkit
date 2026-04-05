use pyo3::prelude::*;

use crate::error::{duration_from_secs, to_py_err};
use crate::raw_message::{PyRawMessage, PyRawMessageStream};
use crate::vehicle::vehicle_label;

#[pyclass(name = "CommandAck", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyCommandAck {
    inner: mavkit::CommandAck,
}

#[pymethods]
impl PyCommandAck {
    #[getter]
    fn command(&self) -> u16 {
        self.inner.command
    }

    #[getter]
    fn result(&self) -> u8 {
        self.inner.result
    }

    #[getter]
    fn progress(&self) -> Option<u8> {
        self.inner.progress
    }

    #[getter]
    fn result_param2(&self) -> Option<i32> {
        self.inner.result_param2
    }
}

#[pyclass(name = "RawHandle", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyRawHandle {
    inner: mavkit::Vehicle,
}

impl PyRawHandle {
    pub(crate) fn new(inner: mavkit::Vehicle) -> Self {
        Self { inner }
    }
}

#[pymethods]
impl PyRawHandle {
    fn command_long<'py>(
        &self,
        py: Python<'py>,
        command: u16,
        params: [f32; 7],
    ) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let inner = vehicle
                .raw()
                .command_long(command, params)
                .await
                .map_err(to_py_err)?;
            Ok(PyCommandAck { inner })
        })
    }

    #[allow(clippy::too_many_arguments)]
    fn command_int<'py>(
        &self,
        py: Python<'py>,
        command: u16,
        frame: u8,
        current: u8,
        autocontinue: u8,
        params: [f32; 4],
        x: i32,
        y: i32,
        z: f32,
    ) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let inner = vehicle
                .raw()
                .command_int(command, frame, current, autocontinue, params, x, y, z)
                .await
                .map_err(to_py_err)?;
            Ok(PyCommandAck { inner })
        })
    }

    fn request_message<'py>(
        &self,
        py: Python<'py>,
        message_id: u32,
        timeout_secs: f64,
    ) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        let timeout = duration_from_secs(timeout_secs)?;
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let message = vehicle
                .raw()
                .request_message(message_id, timeout)
                .await
                .map_err(to_py_err)?;
            PyRawMessage::from_raw_message(message)
        })
    }

    fn set_message_interval<'py>(
        &self,
        py: Python<'py>,
        message_id: u32,
        interval_us: i32,
    ) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            vehicle
                .raw()
                .set_message_interval(message_id, interval_us)
                .await
                .map_err(to_py_err)?;
            Ok(())
        })
    }

    fn send<'py>(&self, py: Python<'py>, message: &PyRawMessage) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        let message = message.to_rust();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            vehicle.raw().send(message).await.map_err(to_py_err)?;
            Ok(())
        })
    }

    fn subscribe(&self) -> PyRawMessageStream {
        PyRawMessageStream::from_subscription(Box::pin(self.inner.raw().subscribe()))
    }

    fn __repr__(&self) -> String {
        format!("RawHandle({})", vehicle_label(&self.inner))
    }
}
