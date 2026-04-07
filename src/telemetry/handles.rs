//! Thin root for telemetry metric backing stores and grouped accessors.
//!
//! The backing-store construction helpers stay crate-private, while the grouped accessors reached
//! from [`crate::Vehicle::telemetry`] are re-exported here through stable public handle and
//! namespace types. This keeps the telemetry paths intact without exposing storage details.

mod backing;
mod namespace;

pub(crate) use backing::{
    TelemetryMetricHandles, TelemetryMetricWriters, create_telemetry_backing_stores,
};

pub use namespace::{
    ActuatorsNamespace, AttitudeNamespace, BatteryNamespace, GpsNamespace, NavigationNamespace,
    PositionNamespace, RcNamespace, TelemetryHandle, TerrainNamespace,
};
