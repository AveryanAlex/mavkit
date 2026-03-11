#![doc = include_str!("../README.md")]

#[cfg(feature = "stream")]
pub mod ble_transport;
pub mod command;
pub mod config;
pub mod error;
pub mod event_loop;
pub mod mission;
pub mod modes;
pub mod params;
pub mod state;
#[cfg(feature = "stream")]
pub mod stream_connection;
#[cfg(feature = "tlog")]
pub mod tlog;
pub mod vehicle;

pub use config::VehicleConfig;
pub use error::VehicleError;
pub use vehicle::Vehicle;

pub use state::{
    Attitude, AutopilotType, Battery, FlightMode, Gps, GpsFixType, LinkState, MagCalProgress,
    MagCalReport, MagCalStatus, MavSeverity, MissionState, Navigation, Position, RcChannels,
    SensorHealth, SensorId, SensorStatus, StatusMessage, SystemStatus, Telemetry, Terrain,
    VehicleIdentity, VehicleState, VehicleType,
};

pub use mission::{
    CompareTolerance, HomePosition, IssueSeverity, MissionFrame, MissionHandle, MissionIssue,
    MissionItem, MissionPlan, MissionTransferMachine, MissionType, RetryPolicy, TransferDirection,
    TransferError, TransferEvent, TransferPhase, TransferProgress, items_for_wire_upload,
    normalize_for_compare, plan_from_wire_download, plans_equivalent, validate_plan,
};

pub use params::{
    Param, ParamProgress, ParamStore, ParamTransferPhase, ParamType, ParamWriteResult,
    ParamsHandle, format_param_file, parse_param_file,
};
