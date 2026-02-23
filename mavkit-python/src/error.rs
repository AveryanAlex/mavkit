use pyo3::prelude::*;

pyo3::create_exception!(mavkit, MavkitError, pyo3::exceptions::PyException);

pub fn to_py_err(e: mavkit::VehicleError) -> PyErr {
    MavkitError::new_err(e.to_string())
}
