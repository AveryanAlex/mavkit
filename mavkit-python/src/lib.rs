mod config;
mod enums;
mod error;
mod mission;
mod params;
mod state;
mod tlog;
mod vehicle;

use pyo3::prelude::*;

#[pymodule]
fn mavkit(py: Python<'_>, m: &Bound<'_, PyModule>) -> PyResult<()> {
    // Exception
    m.add("MavkitError", py.get_type::<error::MavkitError>())?;

    // Enums
    m.add_class::<enums::PySystemStatus>()?;
    m.add_class::<enums::PyVehicleType>()?;
    m.add_class::<enums::PyAutopilotType>()?;
    m.add_class::<enums::PyGpsFixType>()?;
    m.add_class::<enums::PyMissionType>()?;
    m.add_class::<enums::PyMissionFrame>()?;
    m.add_class::<enums::PyIssueSeverity>()?;
    m.add_class::<enums::PyTransferDirection>()?;
    m.add_class::<enums::PyTransferPhase>()?;
    m.add_class::<enums::PyParamTransferPhase>()?;
    m.add_class::<enums::PyParamType>()?;

    // State types
    m.add_class::<state::PyVehicleState>()?;
    m.add_class::<state::PyTelemetry>()?;
    m.add_class::<state::PyMissionState>()?;
    m.add_class::<state::PyLinkState>()?;
    m.add_class::<state::PyVehicleIdentity>()?;
    m.add_class::<state::PyFlightMode>()?;
    m.add_class::<state::PyStatusMessage>()?;

    // Mission types
    m.add_class::<mission::PyMissionItem>()?;
    m.add_class::<mission::PyHomePosition>()?;
    m.add_class::<mission::PyMissionPlan>()?;
    m.add_class::<mission::PyMissionIssue>()?;
    m.add_class::<mission::PyTransferProgress>()?;
    m.add_class::<mission::PyTransferError>()?;
    m.add_class::<mission::PyRetryPolicy>()?;
    m.add_class::<mission::PyCompareTolerance>()?;

    // Param types
    m.add_class::<params::PyParam>()?;
    m.add_class::<params::PyParamStore>()?;
    m.add_class::<params::PyParamProgress>()?;
    m.add_class::<params::PyParamWriteResult>()?;

    // Config
    m.add_class::<config::PyVehicleConfig>()?;

    // Tlog types
    m.add_class::<tlog::PyTlogEntry>()?;
    m.add_class::<tlog::PyTlogFile>()?;

    // Vehicle
    m.add_class::<vehicle::PyVehicle>()?;

    // Free functions (mission)
    m.add_function(wrap_pyfunction!(mission::validate_plan, m)?)?;
    m.add_function(wrap_pyfunction!(mission::plans_equivalent, m)?)?;
    m.add_function(wrap_pyfunction!(mission::normalize_for_compare, m)?)?;
    m.add_function(wrap_pyfunction!(mission::items_for_wire_upload, m)?)?;
    m.add_function(wrap_pyfunction!(mission::plan_from_wire_download, m)?)?;

    // Free functions (params)
    m.add_function(wrap_pyfunction!(params::format_param_file, m)?)?;
    m.add_function(wrap_pyfunction!(params::parse_param_file, m)?)?;

    Ok(())
}
