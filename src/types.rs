use serde::{Deserialize, Serialize};

/// Capability support status for a feature family.
#[cfg_attr(feature = "typescript", derive(specta::Type))]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SupportState {
    #[default]
    Unknown,
    Supported,
    Unsupported,
}

/// Freshness marker for cached domain state.
#[cfg_attr(feature = "typescript", derive(specta::Type))]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SyncState {
    #[default]
    Unknown,
    Current,
    PossiblyStale,
}

/// Operation kind used by stored-plan domains such as fence and rally.
#[cfg_attr(feature = "typescript", derive(specta::Type))]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum StoredPlanOperationKind {
    Upload,
    Download,
    Clear,
}

/// Operation kind used by mission-domain state and conflicts.
#[cfg_attr(feature = "typescript", derive(specta::Type))]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MissionOperationKind {
    Upload,
    Download,
    Clear,
    Verify,
}

/// Operation kind used by parameter-domain state and conflicts.
#[cfg_attr(feature = "typescript", derive(specta::Type))]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ParamOperationKind {
    DownloadAll,
    WriteBatch,
}

/// Details about an operation rejected because another operation is active.
#[cfg_attr(feature = "typescript", derive(specta::Type))]
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct OperationConflict {
    pub conflicting_domain: String,
    pub conflicting_op: String,
}

/// Health state of one sensor family derived from MAVLink bitmasks.
#[cfg_attr(feature = "typescript", derive(specta::Type))]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SensorHealthState {
    #[default]
    NotPresent,
    Disabled,
    Unhealthy,
    Healthy,
}

/// Fixed-shape sensor health snapshot for core pre-arm sensor families.
#[cfg_attr(feature = "typescript", derive(specta::Type))]
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct SensorHealthSummary {
    pub gyro: SensorHealthState,
    pub accel: SensorHealthState,
    pub mag: SensorHealthState,
    pub baro: SensorHealthState,
    pub gps: SensorHealthState,
    pub airspeed: SensorHealthState,
    pub rc_receiver: SensorHealthState,
    pub battery: SensorHealthState,
    pub terrain: SensorHealthState,
    pub geofence: SensorHealthState,
}

/// Lifecycle phases for one mission operation handle.
#[cfg_attr(feature = "typescript", derive(specta::Type))]
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MissionOperationProgress {
    RequestCount,
    SendingItem { current: u16, total: u16 },
    ReceivingItem { current: u16, total: u16 },
    AwaitingAck,
    Verifying,
    Completed,
    Failed,
    Cancelled,
}

/// Lifecycle phases for one parameter operation handle.
#[cfg_attr(feature = "typescript", derive(specta::Type))]
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ParamOperationProgress {
    Downloading {
        received: u16,
        expected: Option<u16>,
    },
    Writing {
        index: u16,
        total: u16,
        name: String,
    },
    Completed,
    Failed,
    Cancelled,
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde::{Deserialize, Serialize};

    fn assert_serde<T: Serialize + for<'de> Deserialize<'de>>() {}

    #[test]
    fn shared_state_enums_exist_with_expected_variants() {
        let support = SupportState::Supported;
        let sync = SyncState::PossiblyStale;
        let stored = StoredPlanOperationKind::Download;
        let mission = MissionOperationKind::Verify;
        let param = ParamOperationKind::WriteBatch;

        assert_eq!(support, SupportState::Supported);
        assert_eq!(sync, SyncState::PossiblyStale);
        assert_eq!(stored, StoredPlanOperationKind::Download);
        assert_eq!(mission, MissionOperationKind::Verify);
        assert_eq!(param, ParamOperationKind::WriteBatch);

        assert_serde::<SupportState>();
        assert_serde::<SyncState>();
        assert_serde::<StoredPlanOperationKind>();
        assert_serde::<MissionOperationKind>();
        assert_serde::<ParamOperationKind>();
    }

    #[test]
    fn operation_conflict_exists() {
        let conflict = OperationConflict {
            conflicting_domain: "mission".to_string(),
            conflicting_op: "upload".to_string(),
        };

        assert_eq!(conflict.conflicting_domain, "mission");
        assert_eq!(conflict.conflicting_op, "upload");
        assert_serde::<OperationConflict>();
    }

    #[test]
    fn sensor_health_summary_has_all_fields_and_explicit_not_present() {
        let summary = SensorHealthSummary {
            gyro: SensorHealthState::Healthy,
            accel: SensorHealthState::Unhealthy,
            mag: SensorHealthState::Disabled,
            baro: SensorHealthState::NotPresent,
            gps: SensorHealthState::Healthy,
            airspeed: SensorHealthState::Disabled,
            rc_receiver: SensorHealthState::Healthy,
            battery: SensorHealthState::Healthy,
            terrain: SensorHealthState::NotPresent,
            geofence: SensorHealthState::Unhealthy,
        };

        assert_eq!(summary.gyro, SensorHealthState::Healthy);
        assert_eq!(summary.accel, SensorHealthState::Unhealthy);
        assert_eq!(summary.mag, SensorHealthState::Disabled);
        assert_eq!(summary.baro, SensorHealthState::NotPresent);
        assert_eq!(summary.gps, SensorHealthState::Healthy);
        assert_eq!(summary.airspeed, SensorHealthState::Disabled);
        assert_eq!(summary.rc_receiver, SensorHealthState::Healthy);
        assert_eq!(summary.battery, SensorHealthState::Healthy);
        assert_eq!(summary.terrain, SensorHealthState::NotPresent);
        assert_eq!(summary.geofence, SensorHealthState::Unhealthy);

        assert_serde::<SensorHealthState>();
        assert_serde::<SensorHealthSummary>();
    }

    #[test]
    fn mission_progress_covers_expected_phases() {
        let phases = [
            MissionOperationProgress::RequestCount,
            MissionOperationProgress::SendingItem {
                current: 1,
                total: 10,
            },
            MissionOperationProgress::ReceivingItem {
                current: 2,
                total: 10,
            },
            MissionOperationProgress::AwaitingAck,
            MissionOperationProgress::Verifying,
            MissionOperationProgress::Completed,
            MissionOperationProgress::Failed,
            MissionOperationProgress::Cancelled,
        ];

        assert_eq!(phases.len(), 8);
        assert_serde::<MissionOperationProgress>();
    }

    #[test]
    fn param_progress_covers_expected_phases() {
        let phases = [
            ParamOperationProgress::Downloading {
                received: 5,
                expected: Some(9),
            },
            ParamOperationProgress::Writing {
                index: 3,
                total: 7,
                name: "GPS_TYPE".to_string(),
            },
            ParamOperationProgress::Completed,
            ParamOperationProgress::Failed,
            ParamOperationProgress::Cancelled,
        ];

        assert_eq!(phases.len(), 5);
        assert_serde::<ParamOperationProgress>();
    }
}
