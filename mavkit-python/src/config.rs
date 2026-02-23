use std::time::Duration;

use pyo3::prelude::*;

use crate::mission::PyRetryPolicy;

#[pyclass(name = "VehicleConfig", frozen, from_py_object)]
#[derive(Clone)]
pub struct PyVehicleConfig {
    gcs_system_id: u8,
    gcs_component_id: u8,
    auto_request_home: bool,
    command_buffer_size: usize,
    connect_timeout: Duration,
    retry: mavkit::RetryPolicy,
}

impl PyVehicleConfig {
    pub(crate) fn into_inner(self) -> mavkit::VehicleConfig {
        mavkit::VehicleConfig {
            gcs_system_id: self.gcs_system_id,
            gcs_component_id: self.gcs_component_id,
            retry_policy: self.retry,
            auto_request_home: self.auto_request_home,
            command_buffer_size: self.command_buffer_size,
            connect_timeout: self.connect_timeout,
        }
    }
}

#[pymethods]
impl PyVehicleConfig {
    #[new]
    #[pyo3(signature = (
        *,
        gcs_system_id = 255,
        gcs_component_id = 190,
        auto_request_home = true,
        command_buffer_size = 32,
        connect_timeout_secs = 30.0,
        retry_policy = None,
    ))]
    fn new(
        gcs_system_id: u8,
        gcs_component_id: u8,
        auto_request_home: bool,
        command_buffer_size: usize,
        connect_timeout_secs: f64,
        retry_policy: Option<PyRetryPolicy>,
    ) -> Self {
        Self {
            gcs_system_id,
            gcs_component_id,
            auto_request_home,
            command_buffer_size,
            connect_timeout: Duration::from_secs_f64(connect_timeout_secs),
            retry: retry_policy.map(|r| r.inner).unwrap_or_default(),
        }
    }

    #[getter]
    fn gcs_system_id(&self) -> u8 {
        self.gcs_system_id
    }
    #[getter]
    fn gcs_component_id(&self) -> u8 {
        self.gcs_component_id
    }
    #[getter]
    fn auto_request_home(&self) -> bool {
        self.auto_request_home
    }
    #[getter]
    fn command_buffer_size(&self) -> usize {
        self.command_buffer_size
    }
    #[getter]
    fn connect_timeout_secs(&self) -> f64 {
        self.connect_timeout.as_secs_f64()
    }
    #[getter]
    fn retry_policy(&self) -> PyRetryPolicy {
        PyRetryPolicy { inner: self.retry }
    }

    fn __repr__(&self) -> String {
        format!(
            "VehicleConfig(sys_id={}, comp_id={}, timeout={:.1}s)",
            self.gcs_system_id,
            self.gcs_component_id,
            self.connect_timeout.as_secs_f64()
        )
    }
}
