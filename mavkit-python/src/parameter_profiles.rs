use pyo3::prelude::*;

/// ArduPilot parameter encodings and GPS sources as JSON-compatible dictionaries.
#[pyfunction]
pub fn ardupilot_parameter_profiles(py: Python<'_>) -> PyResult<Py<PyAny>> {
    use mavkit::ardupilot::parameters as profiles;
    let json = serde_json::json!({
        "rtl": profiles::RTL_PARAMETER_ENCODINGS,
        "gps_sources": profiles::GPS_SOURCES,
        "primary_gps_type_names": profiles::PRIMARY_GPS_TYPE_NAMES,
        "primary_gnss_mode_names": profiles::PRIMARY_GNSS_MODE_NAMES,
    });
    let json_module = py.import("json")?;
    Ok(json_module
        .call_method1("loads", (json.to_string(),))?
        .unbind())
}
