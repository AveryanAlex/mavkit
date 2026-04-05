use std::sync::OnceLock;
use std::time::Instant;

use pyo3::prelude::*;

pub(super) static PY_INSTANT_EPOCH: OnceLock<Instant> = OnceLock::new();

pub(super) fn monotonic_seconds(instant: Instant) -> f64 {
    let epoch = *PY_INSTANT_EPOCH.get_or_init(Instant::now);
    instant
        .checked_duration_since(epoch)
        .unwrap_or_default()
        .as_secs_f64()
}

pub(super) fn support_state_name(
    state: Option<mavkit::observation::SupportState>,
) -> Option<&'static str> {
    match state {
        Some(mavkit::observation::SupportState::Unknown) => Some("unknown"),
        Some(mavkit::observation::SupportState::Supported) => Some("supported"),
        Some(mavkit::observation::SupportState::Unsupported) => Some("unsupported"),
        None => None,
    }
}

pub(super) fn vehicle_timestamp_to_py(
    py: Python<'_>,
    timestamp: Option<mavkit::VehicleTimestamp>,
) -> PyResult<Option<Py<PyAny>>> {
    match timestamp {
        Some(mavkit::VehicleTimestamp::BootTime(duration)) => Ok(Some(
            ("boot_time", duration.as_secs_f64())
                .into_pyobject(py)?
                .to_owned()
                .into_any()
                .unbind(),
        )),
        Some(mavkit::VehicleTimestamp::UnixEpochMicros(micros)) => Ok(Some(
            ("unix_epoch_micros", micros)
                .into_pyobject(py)?
                .to_owned()
                .into_any()
                .unbind(),
        )),
        None => Ok(None),
    }
}
