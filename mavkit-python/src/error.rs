use std::time::Duration;

use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

pyo3::create_exception!(mavkit, MavkitError, pyo3::exceptions::PyException);

pub fn to_py_err(e: mavkit::VehicleError) -> PyErr {
    MavkitError::new_err(e.to_string())
}

pub fn duration_from_secs(timeout_secs: f64) -> PyResult<Duration> {
    if !timeout_secs.is_finite() || timeout_secs < 0.0 {
        return Err(PyValueError::new_err(
            "timeout_secs must be a finite non-negative number",
        ));
    }
    Ok(Duration::from_secs_f64(timeout_secs))
}
