use serde::{Deserialize, Serialize};

use super::commands::{MissionCommand, MissionFrame as CommandFrame, RawMissionCommand};
use crate::types::{MissionOperationKind, SyncState};

/// MAVLink mission storage type.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum MissionType {
    Mission,
    Fence,
    Rally,
}

/// Coordinate frame for a mission item.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum MissionFrame {
    Mission,
    GlobalInt,
    GlobalRelativeAltInt,
    GlobalTerrainAltInt,
    LocalNed,
    Other,
}

impl MissionFrame {
    pub fn is_global_position(self) -> bool {
        matches!(
            self,
            MissionFrame::GlobalInt
                | MissionFrame::GlobalRelativeAltInt
                | MissionFrame::GlobalTerrainAltInt
        )
    }
}

/// A single mission item. Coordinates `x`/`y` are in degE7 for global frames.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct MissionItem {
    pub seq: u16,
    pub command: MissionCommand,
    pub current: bool,
    pub autocontinue: bool,
}

/// Home position in WGS84 coordinates.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct HomePosition {
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    pub altitude_m: f32,
}

impl HomePosition {
    /// Convert to a MAV_CMD_NAV_WAYPOINT mission item at the given sequence number.
    pub fn to_mission_item(&self, seq: u16) -> MissionItem {
        MissionItem {
            seq,
            command: MissionCommand::Other(RawMissionCommand {
                command: 16,
                frame: CommandFrame::Global,
                param1: 0.0,
                param2: 0.0,
                param3: 0.0,
                param4: 0.0,
                x: (self.latitude_deg * 1e7) as i32,
                y: (self.longitude_deg * 1e7) as i32,
                z: self.altitude_m,
            }),
            current: false,
            autocontinue: true,
        }
    }

    /// Extract a home position from a mission item, if it is a global waypoint.
    pub fn from_mission_item(item: &MissionItem) -> Option<Self> {
        let (command, frame, _params, x, y, z) = item.command.clone().into_wire();
        if command == 16 && frame == CommandFrame::Global {
            Some(HomePosition {
                latitude_deg: x as f64 / 1e7,
                longitude_deg: y as f64 / 1e7,
                altitude_m: z,
            })
        } else {
            None
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
/// Ordered mission plan payload for upload/download/verify operations.
pub struct MissionPlan {
    pub mission_type: MissionType,
    pub items: Vec<MissionItem>,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq)]
/// Cached mission-domain state plus sync and active-operation markers.
pub struct MissionState {
    pub plan: Option<MissionPlan>,
    pub current_index: Option<u16>,
    pub sync: SyncState,
    pub active_op: Option<MissionOperationKind>,
}

/// Severity level of a mission validation issue.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum IssueSeverity {
    Error,
    Warning,
}

/// A validation issue found in a mission plan.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct MissionIssue {
    pub code: String,
    pub message: String,
    pub seq: Option<u16>,
    pub severity: IssueSeverity,
}
