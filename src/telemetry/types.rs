use std::time::Duration;

use serde::{Deserialize, Serialize};

pub const TIME_USEC_EPOCH_CUTOFF: u64 = 1_000_000_000_000;

#[cfg(feature = "typescript")]
#[allow(dead_code)]
#[derive(specta::Type)]
struct VehicleTimestampBootTime {
    secs: f64,
    nanos: u32,
}

/// Message-family identifier used for metric provenance tagging.
#[cfg_attr(feature = "typescript", derive(specta::Type))]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TelemetryMessageKind {
    Heartbeat,
    VfrHud,
    GlobalPositionInt,
    LocalPositionNed,
    GpsRawInt,
    Attitude,
    SysStatus,
    BatteryStatus,
    NavControllerOutput,
    TerrainReport,
    RcChannels,
    ServoOutputRaw,
    HomePosition,
    GpsGlobalOrigin,
    StatusText,
}

impl TelemetryMessageKind {
    pub const ALL: [Self; 15] = [
        Self::Heartbeat,
        Self::VfrHud,
        Self::GlobalPositionInt,
        Self::LocalPositionNed,
        Self::GpsRawInt,
        Self::Attitude,
        Self::SysStatus,
        Self::BatteryStatus,
        Self::NavControllerOutput,
        Self::TerrainReport,
        Self::RcChannels,
        Self::ServoOutputRaw,
        Self::HomePosition,
        Self::GpsGlobalOrigin,
        Self::StatusText,
    ];
}

/// Vehicle-provided timestamp, either boot-relative or Unix epoch.
#[cfg_attr(feature = "typescript", derive(specta::Type))]
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum VehicleTimestamp {
    #[cfg_attr(feature = "typescript", specta(type = VehicleTimestampBootTime))]
    BootTime(Duration),
    #[cfg_attr(feature = "typescript", specta(type = f64))]
    UnixEpochMicros(u64),
}

pub fn infer_time_boot_ms(time_boot_ms: u32) -> VehicleTimestamp {
    VehicleTimestamp::BootTime(Duration::from_millis(u64::from(time_boot_ms)))
}

pub fn infer_time_usec(time_usec: u64) -> Option<VehicleTimestamp> {
    match time_usec {
        0 | u64::MAX => None,
        value if value >= TIME_USEC_EPOCH_CUTOFF => Some(VehicleTimestamp::UnixEpochMicros(value)),
        value => Some(VehicleTimestamp::BootTime(Duration::from_micros(value))),
    }
}

pub fn infer_timestamp_from_time_boot_ms(
    kind: TelemetryMessageKind,
    time_boot_ms: u32,
) -> Option<VehicleTimestamp> {
    match kind {
        TelemetryMessageKind::GlobalPositionInt
        | TelemetryMessageKind::LocalPositionNed
        | TelemetryMessageKind::Attitude => Some(infer_time_boot_ms(time_boot_ms)),
        _ => None,
    }
}

pub fn infer_timestamp_from_time_usec(
    kind: TelemetryMessageKind,
    time_usec: u64,
) -> Option<VehicleTimestamp> {
    match kind {
        TelemetryMessageKind::GpsRawInt
        | TelemetryMessageKind::HomePosition
        | TelemetryMessageKind::GpsGlobalOrigin => infer_time_usec(time_usec),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn telemetry_message_kind_has_expected_inventory() {
        assert_eq!(TelemetryMessageKind::ALL.len(), 15);
        assert!(TelemetryMessageKind::ALL.contains(&TelemetryMessageKind::Heartbeat));
        assert!(TelemetryMessageKind::ALL.contains(&TelemetryMessageKind::StatusText));
        assert!(TelemetryMessageKind::ALL.contains(&TelemetryMessageKind::GpsGlobalOrigin));
    }

    #[test]
    fn timestamp_inference_boot_time_ms() {
        assert_eq!(
            infer_time_boot_ms(1000),
            VehicleTimestamp::BootTime(Duration::from_secs(1))
        );
    }

    #[test]
    fn timestamp_inference_time_usec_magnitude_and_placeholders() {
        assert_eq!(infer_time_usec(0), None);
        assert_eq!(infer_time_usec(u64::MAX), None);
        assert_eq!(
            infer_time_usec(12_345),
            Some(VehicleTimestamp::BootTime(Duration::from_micros(12_345)))
        );
        assert_eq!(
            infer_time_usec(1_700_000_000_000_000),
            Some(VehicleTimestamp::UnixEpochMicros(1_700_000_000_000_000))
        );
    }

    #[test]
    fn timestamp_inference_only_applies_to_expected_message_families() {
        assert_eq!(
            infer_timestamp_from_time_boot_ms(TelemetryMessageKind::GlobalPositionInt, 250),
            Some(VehicleTimestamp::BootTime(Duration::from_millis(250)))
        );
        assert_eq!(
            infer_timestamp_from_time_boot_ms(TelemetryMessageKind::Heartbeat, 250),
            None
        );

        assert_eq!(
            infer_timestamp_from_time_usec(TelemetryMessageKind::GpsRawInt, 123),
            Some(VehicleTimestamp::BootTime(Duration::from_micros(123)))
        );
        assert_eq!(
            infer_timestamp_from_time_usec(TelemetryMessageKind::StatusText, 1_700_000_000_000_000),
            None
        );
    }
}
