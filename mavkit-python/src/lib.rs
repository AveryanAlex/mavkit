mod ardupilot;
mod config;
mod enums;
mod error;
mod geo;
mod guided;
mod info;
mod link;
mod macros;
mod mission;
mod modes;
mod params;
mod raw_message;
mod support;
mod telemetry;
mod tlog;
mod vehicle;

use pyo3::prelude::*;

#[pymodule]
fn mavkit(py: Python<'_>, m: &Bound<'_, PyModule>) -> PyResult<()> {
    // Exceptions — MavkitError is the base; subclasses allow granular catching
    m.add("MavkitError", py.get_type::<error::MavkitError>())?;
    m.add("ConnectionError", py.get_type::<error::ConnectionError>())?;
    m.add(
        "DisconnectedError",
        py.get_type::<error::DisconnectedError>(),
    )?;
    m.add(
        "CommandRejectedError",
        py.get_type::<error::CommandRejectedError>(),
    )?;
    m.add("TimeoutError", py.get_type::<error::TimeoutError>())?;
    m.add("UnsupportedError", py.get_type::<error::UnsupportedError>())?;
    m.add(
        "InvalidParameterError",
        py.get_type::<error::InvalidParameterError>(),
    )?;
    m.add(
        "ModeNotAvailableError",
        py.get_type::<error::ModeNotAvailableError>(),
    )?;
    m.add(
        "TransferFailedError",
        py.get_type::<error::TransferFailedError>(),
    )?;
    m.add(
        "OperationConflictError",
        py.get_type::<error::OperationConflictError>(),
    )?;
    m.add("CancelledError", py.get_type::<error::CancelledError>())?;
    m.add("ValidationError", py.get_type::<error::ValidationError>())?;

    // Enums
    m.add_class::<enums::PySystemStatus>()?;
    m.add_class::<enums::PyVehicleType>()?;
    m.add_class::<enums::PyAutopilotType>()?;
    m.add_class::<enums::PyGpsFixType>()?;
    m.add_class::<enums::PyMavSeverity>()?;
    m.add_class::<enums::PyMissionType>()?;
    m.add_class::<enums::PyMissionFrame>()?;
    m.add_class::<enums::PyIssueSeverity>()?;
    m.add_class::<enums::PyTransferDirection>()?;
    m.add_class::<enums::PyTransferPhase>()?;
    m.add_class::<enums::PyParamTransferPhase>()?;
    m.add_class::<enums::PyParamType>()?;

    telemetry::register(m)?;
    m.add_class::<geo::PyGeoPoint3dMsl>()?;

    m.add_class::<support::PySupportState>()?;
    m.add_class::<support::PySupportStateHandle>()?;
    m.add_class::<support::PySupportStateSubscription>()?;
    m.add_class::<link::PyLinkState>()?;
    m.add_class::<link::PyLinkStateHandle>()?;
    m.add_class::<link::PyLinkStateSubscription>()?;
    m.add_class::<link::PyLinkHandle>()?;
    m.add_class::<modes::PyModeCatalogSource>()?;
    m.add_class::<modes::PyModeDescriptor>()?;
    m.add_class::<modes::PyCurrentModeSource>()?;
    m.add_class::<modes::PyCurrentMode>()?;
    m.add_class::<modes::PyModeCatalogHandle>()?;
    m.add_class::<modes::PyModeCatalogSubscription>()?;
    m.add_class::<modes::PyCurrentModeHandle>()?;
    m.add_class::<modes::PyCurrentModeSubscription>()?;
    m.add_class::<info::PyFirmwareInfo>()?;
    m.add_class::<info::PyHardwareInfo>()?;
    m.add_class::<info::PyUniqueIds>()?;
    m.add_class::<info::PyPersistentIdentity>()?;
    m.add_class::<info::PyFirmwareInfoHandle>()?;
    m.add_class::<info::PyFirmwareInfoSubscription>()?;
    m.add_class::<info::PyHardwareInfoHandle>()?;
    m.add_class::<info::PyHardwareInfoSubscription>()?;
    m.add_class::<info::PyUniqueIdsHandle>()?;
    m.add_class::<info::PyUniqueIdsSubscription>()?;
    m.add_class::<info::PyPersistentIdentityHandle>()?;
    m.add_class::<info::PyPersistentIdentitySubscription>()?;

    // Mission types
    mission::register(m)?;
    m.add_class::<geo::PyGeoPoint2d>()?;
    m.add_class::<geo::PyGeoPoint3dRelHome>()?;
    m.add_class::<geo::PyGeoPoint3dTerrain>()?;

    // Param types
    m.add_class::<params::PyParam>()?;
    m.add_class::<params::PyParamStore>()?;
    m.add_class::<params::PyParamProgress>()?;
    m.add_class::<params::PyParamWriteResult>()?;
    m.add_class::<params::PySyncState>()?;
    m.add_class::<params::PyParamOperationKind>()?;
    m.add_class::<params::PyParamState>()?;
    m.add_class::<params::PyParamStateSubscription>()?;
    m.add_class::<params::PyParamProgressSubscription>()?;
    m.add_class::<params::PyParamDownloadOp>()?;
    m.add_class::<params::PyParamWriteBatchOp>()?;

    // Config
    m.add_class::<config::PyVehicleConfig>()?;

    // Tlog types
    m.add_class::<tlog::PyTlogEntry>()?;
    m.add_class::<tlog::PyTlogFile>()?;
    m.add_class::<tlog::PyTlogWriter>()?;

    // Raw message types
    m.add_class::<raw_message::PyRawMessage>()?;
    m.add_class::<raw_message::PyRawMessageStream>()?;
    vehicle::register(m)?;

    // Vehicle
    m.add_class::<modes::PyModesHandle>()?;
    m.add_class::<info::PyInfoHandle>()?;
    m.add_class::<support::PySupportHandle>()?;
    m.add_class::<params::PyParamsHandle>()?;
    m.add_class::<ardupilot::PyArduCopterHandle>()?;
    m.add_class::<ardupilot::PyArduPlaneHandle>()?;
    m.add_class::<ardupilot::PyArduPlaneVtolHandle>()?;
    m.add_class::<ardupilot::PyArduRoverHandle>()?;
    m.add_class::<ardupilot::PyArduSubHandle>()?;
    m.add_class::<ardupilot::PyArduPilotHandle>()?;
    m.add_class::<guided::PyArduGuidedSession>()?;
    m.add_class::<guided::PyArduCopterGuidedHandle>()?;
    m.add_class::<guided::PyArduPlaneGuidedHandle>()?;
    m.add_class::<guided::PyArduPlaneVtolGuidedHandle>()?;
    m.add_class::<guided::PyArduRoverGuidedHandle>()?;
    m.add_class::<guided::PyArduSubGuidedHandle>()?;

    // Free functions (params)
    m.add_function(wrap_pyfunction!(params::format_param_file, m)?)?;
    m.add_function(wrap_pyfunction!(params::parse_param_file, m)?)?;
    Ok(())
}
