use serde::{Deserialize, Serialize};

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
    pub command: u16,
    pub frame: MissionFrame,
    pub current: bool,
    pub autocontinue: bool,
    pub param1: f32,
    pub param2: f32,
    pub param3: f32,
    pub param4: f32,
    pub x: i32,
    pub y: i32,
    pub z: f32,
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
            command: 16,
            frame: MissionFrame::GlobalInt,
            current: false,
            autocontinue: true,
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            x: (self.latitude_deg * 1e7) as i32,
            y: (self.longitude_deg * 1e7) as i32,
            z: self.altitude_m,
        }
    }

    /// Extract a home position from a mission item, if it is a global waypoint.
    pub fn from_mission_item(item: &MissionItem) -> Option<Self> {
        if item.command == 16 && item.frame == MissionFrame::GlobalInt {
            Some(HomePosition {
                latitude_deg: item.x as f64 / 1e7,
                longitude_deg: item.y as f64 / 1e7,
                altitude_m: item.z,
            })
        } else {
            None
        }
    }
}

/// A complete mission plan with optional home position and ordered items.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct MissionPlan {
    pub mission_type: MissionType,
    pub home: Option<HomePosition>,
    pub items: Vec<MissionItem>,
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
