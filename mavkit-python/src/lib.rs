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

    m.add_class::<vehicle::PyMissionState>()?;
    m.add_class::<vehicle::PyVehicleIdentity>()?;

    m.add_class::<telemetry::PySensorHealthState>()?;
    m.add_class::<telemetry::PyGlobalPosition>()?;
    m.add_class::<telemetry::PyEulerAttitude>()?;
    m.add_class::<telemetry::PyGpsQuality>()?;
    m.add_class::<telemetry::PyCellVoltages>()?;
    m.add_class::<telemetry::PyWaypointProgress>()?;
    m.add_class::<telemetry::PyGuidanceState>()?;
    m.add_class::<telemetry::PyTerrainClearance>()?;
    m.add_class::<geo::PyGeoPoint3dMsl>()?;
    m.add_class::<telemetry::PySensorHealthSummary>()?;
    m.add_class::<telemetry::PyStatusTextEvent>()?;
    m.add_class::<telemetry::PyMetricSample>()?;
    m.add_class::<telemetry::PyMessageSample>()?;
    m.add_class::<telemetry::PyMetricSubscription>()?;
    m.add_class::<telemetry::PyMessageSubscription>()?;
    m.add_class::<telemetry::PyMetricHandle>()?;
    m.add_class::<telemetry::PyPeriodicMessageHandle>()?;
    m.add_class::<telemetry::PyEventMessageHandle>()?;
    m.add_class::<telemetry::PyMessageHandle>()?;
    m.add_class::<telemetry::PyTelemetryPositionNamespace>()?;
    m.add_class::<telemetry::PyTelemetryAttitudeNamespace>()?;
    m.add_class::<telemetry::PyTelemetryBatteryNamespace>()?;
    m.add_class::<telemetry::PyTelemetryGpsNamespace>()?;
    m.add_class::<telemetry::PyTelemetryNavigationNamespace>()?;
    m.add_class::<telemetry::PyTelemetryTerrainNamespace>()?;
    m.add_class::<telemetry::PyTelemetryRcNamespace>()?;
    m.add_class::<telemetry::PyTelemetryActuatorsNamespace>()?;
    m.add_class::<telemetry::PyTelemetryMessagesHandle>()?;

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
    m.add_class::<geo::PyGeoPoint3d>()?;
    m.add_class::<mission::PyRawMissionCommand>()?;
    m.add_class::<mission::PyNavWaypoint>()?;
    m.add_class::<mission::PyNavTakeoff>()?;
    m.add_class::<mission::PyNavLand>()?;
    m.add_class::<mission::PyNavLoiterTime>()?;
    m.add_class::<mission::PyNavGuidedEnable>()?;
    m.add_class::<mission::PyNavReturnToLaunch>()?;
    m.add_class::<mission::PyNavSplineWaypoint>()?;
    m.add_class::<mission::PyNavArcWaypoint>()?;
    m.add_class::<mission::PyNavLoiterUnlimited>()?;
    m.add_class::<mission::PyNavLoiterTurns>()?;
    m.add_class::<mission::PyNavLoiterToAlt>()?;
    m.add_class::<mission::PyNavContinueAndChangeAlt>()?;
    m.add_class::<mission::PyNavDelay>()?;
    m.add_class::<mission::PyNavAltitudeWait>()?;
    m.add_class::<mission::PyNavVtolTakeoff>()?;
    m.add_class::<mission::PyNavVtolLand>()?;
    m.add_class::<mission::PyNavPayloadPlace>()?;
    m.add_class::<mission::PyNavSetYawSpeed>()?;
    m.add_class::<mission::PyNavScriptTime>()?;
    m.add_class::<mission::PyNavAttitudeTime>()?;
    m.add_class::<mission::PyDoChangeSpeed>()?;
    m.add_class::<mission::PyDoSetHome>()?;
    m.add_class::<mission::PyDoSetRelay>()?;
    m.add_class::<mission::PyDoSetRoiNone>()?;
    m.add_class::<mission::PyDoJump>()?;
    m.add_class::<mission::PyDoJumpTag>()?;
    m.add_class::<mission::PyDoTag>()?;
    m.add_class::<mission::PyDoPauseContinue>()?;
    m.add_class::<mission::PyDoSetReverse>()?;
    m.add_class::<mission::PyDoLandStart>()?;
    m.add_class::<mission::PyDoReturnPathStart>()?;
    m.add_class::<mission::PyDoGoAround>()?;
    m.add_class::<mission::PyDoSetRoiLocation>()?;
    m.add_class::<mission::PyDoSetRoi>()?;
    m.add_class::<mission::PyDoMountControl>()?;
    m.add_class::<mission::PyDoGimbalManagerPitchYaw>()?;
    m.add_class::<mission::PyDoCamTriggerDistance>()?;
    m.add_class::<mission::PyDoDigicamConfigure>()?;
    m.add_class::<mission::PyDoDigicamControl>()?;
    m.add_class::<mission::PyDoFenceEnable>()?;
    m.add_class::<mission::PyDoParachute>()?;
    m.add_class::<mission::PyDoGripper>()?;
    m.add_class::<mission::PyDoSprayer>()?;
    m.add_class::<mission::PyDoWinch>()?;
    m.add_class::<mission::PyDoEngineControl>()?;
    m.add_class::<mission::PyDoInvertedFlight>()?;
    m.add_class::<mission::PyDoAutotuneEnable>()?;
    m.add_class::<mission::PyDoSetServo>()?;
    m.add_class::<mission::PyDoRepeatServo>()?;
    m.add_class::<mission::PyDoRepeatRelay>()?;
    m.add_class::<mission::PyDoSetResumeRepeatDist>()?;
    m.add_class::<mission::PyDoAuxFunction>()?;
    m.add_class::<mission::PyDoSendScriptMessage>()?;
    m.add_class::<mission::PyDoImageStartCapture>()?;
    m.add_class::<mission::PyDoImageStopCapture>()?;
    m.add_class::<mission::PyDoVideoStartCapture>()?;
    m.add_class::<mission::PyDoVideoStopCapture>()?;
    m.add_class::<mission::PyDoSetCameraZoom>()?;
    m.add_class::<mission::PyDoSetCameraFocus>()?;
    m.add_class::<mission::PyDoSetCameraSource>()?;
    m.add_class::<mission::PyDoGuidedLimits>()?;
    m.add_class::<mission::PyDoVtolTransition>()?;
    m.add_class::<mission::PyCondDelay>()?;
    m.add_class::<mission::PyCondDistance>()?;
    m.add_class::<mission::PyCondYaw>()?;
    m.add_class::<mission::PyMissionItem>()?;
    m.add_class::<mission::PyHomePosition>()?;
    m.add_class::<mission::PyMissionPlan>()?;
    m.add_class::<mission::PyMissionIssue>()?;
    m.add_class::<mission::PyTransferProgress>()?;
    m.add_class::<mission::PyTransferError>()?;
    m.add_class::<mission::PyRetryPolicy>()?;
    m.add_class::<mission::PyCompareTolerance>()?;
    m.add_class::<geo::PyGeoPoint2d>()?;
    m.add_class::<geo::PyGeoPoint3dRelHome>()?;
    m.add_class::<geo::PyGeoPoint3dTerrain>()?;
    m.add_class::<vehicle::PyFenceInclusionPolygon>()?;
    m.add_class::<vehicle::PyFenceExclusionPolygon>()?;
    m.add_class::<vehicle::PyFenceInclusionCircle>()?;
    m.add_class::<vehicle::PyFenceExclusionCircle>()?;
    m.add_class::<vehicle::PyFencePlan>()?;
    m.add_class::<vehicle::PyRallyPlan>()?;

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
    m.add_class::<vehicle::PyCommandAck>()?;
    m.add_class::<vehicle::PyFenceState>()?;
    m.add_class::<vehicle::PyRallyState>()?;
    m.add_class::<vehicle::PyMissionStateSubscription>()?;
    m.add_class::<vehicle::PyFenceStateSubscription>()?;
    m.add_class::<vehicle::PyRallyStateSubscription>()?;
    m.add_class::<vehicle::PyMissionOperationProgress>()?;
    m.add_class::<vehicle::PyMissionProgressSubscription>()?;
    m.add_class::<vehicle::PyMissionUploadOp>()?;
    m.add_class::<vehicle::PyMissionDownloadOp>()?;
    m.add_class::<vehicle::PyMissionClearOp>()?;
    m.add_class::<vehicle::PyMissionVerifyOp>()?;
    m.add_class::<vehicle::PyFenceUploadOp>()?;
    m.add_class::<vehicle::PyFenceDownloadOp>()?;
    m.add_class::<vehicle::PyFenceClearOp>()?;
    m.add_class::<vehicle::PyRallyUploadOp>()?;
    m.add_class::<vehicle::PyRallyDownloadOp>()?;
    m.add_class::<vehicle::PyRallyClearOp>()?;
    #[cfg(feature = "test-support")]
    m.add_class::<vehicle::PyTestVehicleHarness>()?;

    // Vehicle
    m.add_class::<telemetry::PyTelemetryHandle>()?;
    m.add_class::<modes::PyModesHandle>()?;
    m.add_class::<info::PyInfoHandle>()?;
    m.add_class::<support::PySupportHandle>()?;
    m.add_class::<vehicle::PyMissionHandle>()?;
    m.add_class::<vehicle::PyFenceHandle>()?;
    m.add_class::<vehicle::PyRallyHandle>()?;
    m.add_class::<params::PyParamsHandle>()?;
    m.add_class::<vehicle::PyRawHandle>()?;
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
    m.add_class::<vehicle::PyVehicle>()?;

    // Free functions (mission)
    m.add_function(wrap_pyfunction!(mission::validate_plan, m)?)?;
    m.add_function(wrap_pyfunction!(mission::plans_equivalent, m)?)?;
    m.add_function(wrap_pyfunction!(mission::normalize_for_compare, m)?)?;
    m.add_function(wrap_pyfunction!(mission::mission_items_for_upload, m)?)?;
    m.add_function(wrap_pyfunction!(mission::mission_plan_from_download, m)?)?;

    // Free functions (params)
    m.add_function(wrap_pyfunction!(params::format_param_file, m)?)?;
    m.add_function(wrap_pyfunction!(params::parse_param_file, m)?)?;
    #[cfg(feature = "test-support")]
    m.add_function(wrap_pyfunction!(vehicle::_connect_test_vehicle, m)?)?;

    Ok(())
}
