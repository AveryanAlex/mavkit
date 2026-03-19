use std::time::Duration;

use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

pyo3::create_exception!(mavkit, MavkitError, pyo3::exceptions::PyException);

// Subclass exceptions so callers can catch specific error kinds.
pyo3::create_exception!(mavkit, ConnectionError, MavkitError);
pyo3::create_exception!(mavkit, DisconnectedError, MavkitError);
pyo3::create_exception!(mavkit, CommandRejectedError, MavkitError);
pyo3::create_exception!(mavkit, TimeoutError, MavkitError);
pyo3::create_exception!(mavkit, UnsupportedError, MavkitError);
pyo3::create_exception!(mavkit, InvalidParameterError, MavkitError);
pyo3::create_exception!(mavkit, ModeNotAvailableError, MavkitError);
pyo3::create_exception!(mavkit, TransferFailedError, MavkitError);
pyo3::create_exception!(mavkit, OperationConflictError, MavkitError);
pyo3::create_exception!(mavkit, CancelledError, MavkitError);
pyo3::create_exception!(mavkit, ValidationError, MavkitError);

pub fn to_py_err(e: mavkit::VehicleError) -> PyErr {
    let msg = e.to_string();
    match e {
        mavkit::VehicleError::ConnectionFailed(_) => ConnectionError::new_err(msg),
        mavkit::VehicleError::Disconnected => DisconnectedError::new_err(msg),
        mavkit::VehicleError::CommandRejected { .. }
        | mavkit::VehicleError::OutcomeUnknown { .. } => CommandRejectedError::new_err(msg),
        mavkit::VehicleError::Timeout(_) => TimeoutError::new_err(msg),
        mavkit::VehicleError::Unsupported(_) => UnsupportedError::new_err(msg),
        mavkit::VehicleError::InvalidParameter(_) => InvalidParameterError::new_err(msg),
        mavkit::VehicleError::ModeNotAvailable(_) => ModeNotAvailableError::new_err(msg),
        mavkit::VehicleError::TransferFailed { .. } => TransferFailedError::new_err(msg),
        mavkit::VehicleError::OperationConflict { .. } => OperationConflictError::new_err(msg),
        mavkit::VehicleError::Cancelled => CancelledError::new_err(msg),
        mavkit::VehicleError::MissionValidation(_) => ValidationError::new_err(msg),
        mavkit::VehicleError::IdentityUnknown => ConnectionError::new_err(msg),
        mavkit::VehicleError::Io(_) => ConnectionError::new_err(msg),
    }
}

pub fn duration_from_secs(timeout_secs: f64) -> PyResult<Duration> {
    if !timeout_secs.is_finite() || timeout_secs < 0.0 {
        return Err(PyValueError::new_err(
            "timeout_secs must be a finite non-negative number",
        ));
    }
    Ok(Duration::from_secs_f64(timeout_secs))
}
