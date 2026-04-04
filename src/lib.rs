#![doc = include_str!("../README.md")]

pub mod ardupilot;
pub(crate) mod command;
pub mod config;
pub mod error;
pub(crate) mod event_loop;
pub mod fence;
pub mod geo;
pub mod info;
pub mod link;
pub mod mission;
pub mod modes;
pub mod observation;
pub(crate) mod operation;
pub mod params;
pub mod rally;
pub mod raw;
mod state;
mod stored_plan;
#[cfg(feature = "stream")]
pub mod stream;
pub mod support;
pub mod telemetry;
#[cfg(test)]
pub(crate) mod test_support;
#[cfg(feature = "tlog")]
pub mod tlog;
pub mod types;
pub mod vehicle;

pub use mavlink::all as dialect;

pub use ardupilot::{
    ArduCopterGuidedHandle, ArduCopterHandle, ArduGuidedKind, ArduGuidedSession, ArduPilotHandle,
    ArduPlaneGuidedHandle, ArduPlaneHandle, ArduPlaneKind, ArduPlaneVtolGuidedHandle,
    ArduPlaneVtolHandle, ArduRoverGuidedHandle, ArduRoverHandle, ArduSubGuidedHandle,
    ArduSubHandle, GuidedSpecific, RelativeClimbTarget, SubGotoDepthTarget,
};
pub use config::{InitDomainPolicy, InitPolicyConfig, VehicleConfig};
pub use error::{CommandResult, MissionValidationReason, VehicleError};
pub use fence::{
    FenceClearOp, FenceDownloadOp, FenceExclusionCircle, FenceExclusionPolygon,
    FenceInclusionCircle, FenceInclusionPolygon, FencePlan, FenceRegion, FenceState, FenceUploadOp,
};
pub use geo::{GeoPoint2d, GeoPoint3d, GeoPoint3dMsl, GeoPoint3dRelHome, GeoPoint3dTerrain};
pub use info::{FirmwareInfo, HardwareInfo, InfoHandle, PersistentIdentity, UniqueIds};
pub use link::LinkHandle;
pub use modes::{
    CurrentMode, CurrentModeSource, FlightMode, ModeCatalogSource, ModeDescriptor, ModesHandle,
};
pub use rally::{RallyClearOp, RallyDownloadOp, RallyPlan, RallyState, RallyUploadOp};
pub use raw::{CommandAck, RawHandle, RawMessage};
pub use state::{LinkState, MavSeverity, SystemStatus};
pub use support::SupportHandle;
pub use vehicle::{
    AutopilotType, FenceHandle, MissionHandle, ParamsHandle, RallyHandle, RcOverride,
    RcOverrideChannelValue, Vehicle, VehicleIdentity, VehicleType,
};

pub use mission::commands::{
    AltChangeAction, CondDelay, CondDistance, CondYaw, ConditionCommand, DoAutotuneEnable,
    DoAuxFunction, DoCamTriggerDistance, DoChangeSpeed, DoCommand, DoDigicamConfigure,
    DoDigicamControl, DoEngineControl, DoFenceEnable, DoGimbalManagerPitchYaw, DoGoAround,
    DoGripper, DoGuidedLimits, DoImageStartCapture, DoImageStopCapture, DoInvertedFlight, DoJump,
    DoJumpTag, DoLandStart, DoMountControl, DoParachute, DoPauseContinue, DoRepeatRelay,
    DoRepeatServo, DoReturnPathStart, DoSendScriptMessage, DoSetCameraFocus, DoSetCameraSource,
    DoSetCameraZoom, DoSetHome, DoSetRelay, DoSetResumeRepeatDist, DoSetReverse, DoSetRoi,
    DoSetRoiLocation, DoSetServo, DoSprayer, DoTag, DoVideoStartCapture, DoVideoStopCapture,
    DoVtolTransition, DoWinch, FenceAction, GripperAction, LoiterDirection, NavAltitudeWait,
    NavArcWaypoint, NavAttitudeTime, NavCommand, NavContinueAndChangeAlt, NavDelay,
    NavGuidedEnable, NavLand, NavLoiterTime, NavLoiterToAlt, NavLoiterTurns, NavLoiterUnlimited,
    NavPayloadPlace, NavScriptTime, NavSetYawSpeed, NavSplineWaypoint, NavTakeoff, NavVtolLand,
    NavVtolTakeoff, NavWaypoint, ParachuteAction, SpeedType, WinchAction, YawDirection,
};
pub use mission::{
    CompareTolerance, HomePosition, IssueSeverity, MissionClearOp, MissionCommand,
    MissionDownloadOp, MissionFrame, MissionIssue, MissionItem, MissionPlan,
    MissionTransferMachine, MissionType, MissionUploadOp, MissionVerifyOp, RawMissionCommand,
    RetryPolicy, TransferDirection, TransferError, TransferEvent, TransferPhase, TransferProgress,
    mission_items_for_upload, mission_plan_from_download, normalize_for_compare, plans_equivalent,
    validate_plan,
};

pub use observation::{
    MessageHandle, MessageSample, MetricHandle, MetricSample, ObservationHandle,
    ObservationSubscription, ObservationWriter,
};

pub use telemetry::{
    ActuatorsNamespace, AttitudeNamespace, BatteryNamespace, CellVoltages, EulerAttitude,
    EventMessageHandle, GlobalPosition, GpsFixType, GpsNamespace, GpsQuality, GuidanceState,
    MessagesHandle, NavigationNamespace, PeriodicMessageHandle, PositionNamespace, RcNamespace,
    StatusTextEvent, TelemetryHandle, TelemetryMessageKind, TerrainClearance, TerrainNamespace,
    VehicleTimestamp, WaypointProgress, infer_time_boot_ms, infer_time_usec,
    infer_timestamp_from_time_boot_ms, infer_timestamp_from_time_usec,
};

pub use params::{
    Param, ParamDownloadOp, ParamState, ParamStore, ParamType, ParamWriteBatchOp, ParamWriteResult,
    format_param_file, parse_param_file,
};

pub use types::{
    MissionOperationKind, MissionOperationProgress, OperationConflict, ParamOperationKind,
    ParamOperationProgress, SensorHealthState, SensorHealthSummary, StoredPlanOperationKind,
    SupportState, SyncState,
};
