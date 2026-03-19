use serde::{Deserialize, Serialize};

use super::commands::{MissionCommand, MissionFrame as CommandFrame, RawMissionCommand};
use crate::geo::quantize_degrees_e7;
use crate::stored_plan::StoredPlanState;
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
    pub command: MissionCommand,
    pub current: bool,
    pub autocontinue: bool,
}

/// Home position in WGS84 coordinates.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct HomePosition {
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    pub altitude_m: f64,
}

impl HomePosition {
    /// Convert to a MAV_CMD_NAV_WAYPOINT mission item at the given sequence number.
    pub fn to_mission_item(&self) -> MissionItem {
        MissionItem {
            command: MissionCommand::Other(RawMissionCommand {
                command: 16,
                frame: CommandFrame::Global,
                param1: 0.0,
                param2: 0.0,
                param3: 0.0,
                param4: 0.0,
                x: quantize_degrees_e7(self.latitude_deg),
                y: quantize_degrees_e7(self.longitude_deg),
                // Narrow to f32 at the wire boundary (MAVLink z field is f32).
                z: self.altitude_m as f32,
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
                latitude_deg: f64::from(x) / 1e7,
                longitude_deg: f64::from(y) / 1e7,
                altitude_m: f64::from(z),
            })
        } else {
            None
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
/// Ordered mission plan payload for upload/download/verify operations.
pub struct MissionPlan {
    pub items: Vec<MissionItem>,
}

impl MissionPlan {
    /// Create a plan from a list of mission items.
    pub fn new(items: Vec<MissionItem>) -> Self {
        Self { items }
    }

    /// Create an empty plan with no items.
    pub fn empty() -> Self {
        Self { items: Vec::new() }
    }

    /// Number of items in the plan.
    pub fn len(&self) -> usize {
        self.items.len()
    }

    /// Whether the plan has no items.
    pub fn is_empty(&self) -> bool {
        self.items.is_empty()
    }
}

impl FromIterator<MissionItem> for MissionPlan {
    fn from_iter<I: IntoIterator<Item = MissionItem>>(iter: I) -> Self {
        Self {
            items: iter.into_iter().collect(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
/// Internal wire-level mission plan that carries the MAVLink mission type.
/// Used by the event loop and fence/rally domain converters.
pub(crate) struct WireMissionPlan {
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

impl StoredPlanState for MissionState {
    type Plan = MissionPlan;
    type OperationKind = MissionOperationKind;

    fn set_active_op(&mut self, kind: Option<Self::OperationKind>) {
        self.active_op = kind;
    }

    fn set_plan(&mut self, plan: Self::Plan) {
        self.plan = Some(plan);
    }

    fn set_sync(&mut self, sync: SyncState) {
        self.sync = sync;
    }

    fn cleared_plan() -> Self::Plan {
        MissionPlan::empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mission::test_support::sample_item;

    fn dummy_item() -> MissionItem {
        sample_item()
    }

    #[test]
    fn new_creates_plan_with_items() {
        let plan = MissionPlan::new(vec![dummy_item(), dummy_item()]);
        assert_eq!(plan.len(), 2);
    }

    #[test]
    fn empty_creates_empty_plan() {
        let plan = MissionPlan::empty();
        assert!(plan.is_empty());
        assert_eq!(plan.len(), 0);
    }

    #[test]
    fn from_iterator_collects_items() {
        let items = vec![dummy_item(), dummy_item(), dummy_item()];
        let plan: MissionPlan = items.into_iter().collect();
        assert_eq!(plan.len(), 3);
    }

    #[test]
    fn len_and_is_empty_reflect_contents() {
        let plan = MissionPlan::new(vec![dummy_item()]);
        assert_eq!(plan.len(), 1);
        assert!(!plan.is_empty());
    }
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
