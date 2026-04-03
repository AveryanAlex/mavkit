#[allow(unused_imports)]
use super::MavCmd::*;
use super::MissionFrame;
use super::wire_support::{
    bool_from_param, bool_to_param, empty_unit_from_wire, mission_command_to_wire,
    position_command_to_wire, position_from_wire, u8_from_param, unit_command_to_wire,
};
use crate::geo::GeoPoint3d;
use mavkit_macros::mavkit_command;
use serde::{Deserialize, Serialize};

/// Typed mission command API item used by plan serialization and validation.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum LoiterDirection {
    Clockwise,
    CounterClockwise,
}

/// Typed mission command API item used by plan serialization and validation.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum AltChangeAction {
    Neutral,
    Climb,
    Descend,
}

pub(super) fn signed_value(value: f32, direction: LoiterDirection) -> f32 {
    match direction {
        LoiterDirection::Clockwise => value.abs(),
        LoiterDirection::CounterClockwise => -value.abs(),
    }
}

pub(super) fn direction_from_signed(value: f32) -> LoiterDirection {
    if value.is_sign_negative() {
        LoiterDirection::CounterClockwise
    } else {
        LoiterDirection::Clockwise
    }
}

pub(super) fn alt_change_action_to_param(action: AltChangeAction) -> f32 {
    match action {
        AltChangeAction::Neutral => 0.0,
        AltChangeAction::Climb => 1.0,
        AltChangeAction::Descend => 2.0,
    }
}

pub(super) fn alt_change_action_from_param(value: f32) -> AltChangeAction {
    match value.round() as i32 {
        1 => AltChangeAction::Climb,
        2 => AltChangeAction::Descend,
        _ => AltChangeAction::Neutral,
    }
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = MAV_CMD_NAV_WAYPOINT, category = Nav)]
pub struct NavWaypoint {
    #[position]
    pub position: GeoPoint3d,
    #[param(1)]
    pub hold_time_s: f32,
    #[param(2)]
    pub acceptance_radius_m: f32,
    #[param(3)]
    pub pass_radius_m: f32,
    #[param(4)]
    pub yaw_deg: f32,
}

impl NavWaypoint {
    pub fn from_point(position: impl Into<GeoPoint3d>) -> Self {
        Self {
            position: position.into(),
            hold_time_s: 0.0,
            acceptance_radius_m: 0.0,
            pass_radius_m: 0.0,
            yaw_deg: 0.0,
        }
    }
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = MAV_CMD_NAV_SPLINE_WAYPOINT, category = Nav)]
pub struct NavSplineWaypoint {
    #[position]
    pub position: GeoPoint3d,
    #[param(1)]
    pub hold_time_s: f32,
}

impl NavSplineWaypoint {
    pub fn from_point(position: impl Into<GeoPoint3d>) -> Self {
        Self {
            position: position.into(),
            hold_time_s: 0.0,
        }
    }
}

/// Typed mission command API item used by plan serialization and validation.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct NavArcWaypoint {
    pub position: GeoPoint3d,
    pub arc_angle_deg: f32,
    pub direction: LoiterDirection,
}

impl NavArcWaypoint {
    pub fn from_point(position: impl Into<GeoPoint3d>) -> Self {
        Self {
            position: position.into(),
            arc_angle_deg: 0.0,
            direction: LoiterDirection::Clockwise,
        }
    }
}

pub(super) fn nav_arc_waypoint_to_wire(
    command: NavArcWaypoint,
) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    position_command_to_wire(
        command.position,
        [
            signed_value(command.arc_angle_deg, command.direction),
            0.0,
            0.0,
            0.0,
        ],
    )
}

pub(super) fn nav_arc_waypoint_from_wire(
    frame: MissionFrame,
    params: [f32; 4],
    x: i32,
    y: i32,
    z: f32,
) -> NavArcWaypoint {
    NavArcWaypoint {
        position: position_from_wire(frame, x, y, z),
        arc_angle_deg: params[0].abs(),
        direction: direction_from_signed(params[0]),
    }
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = MAV_CMD_NAV_TAKEOFF, category = Nav)]
pub struct NavTakeoff {
    #[position]
    pub position: GeoPoint3d,
    #[param(1)]
    pub pitch_deg: f32,
}

impl NavTakeoff {
    pub fn from_point(position: impl Into<GeoPoint3d>) -> Self {
        Self {
            position: position.into(),
            pitch_deg: 0.0,
        }
    }
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = MAV_CMD_NAV_LAND, category = Nav)]
pub struct NavLand {
    #[position]
    pub position: GeoPoint3d,
    #[param(1)]
    pub abort_alt_m: f32,
}

impl NavLand {
    pub fn from_point(position: impl Into<GeoPoint3d>) -> Self {
        Self {
            position: position.into(),
            abort_alt_m: 0.0,
        }
    }
}

pub(super) fn return_to_launch_to_wire() -> (MissionFrame, [f32; 4], i32, i32, f32) {
    unit_command_to_wire()
}

pub(super) fn return_to_launch_from_wire(
    frame: MissionFrame,
    params: [f32; 4],
    x: i32,
    y: i32,
    z: f32,
) {
    empty_unit_from_wire(frame, params, x, y, z);
}

/// Typed mission command API item used by plan serialization and validation.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct NavLoiterUnlimited {
    pub position: GeoPoint3d,
    pub radius_m: f32,
    pub direction: LoiterDirection,
}

impl NavLoiterUnlimited {
    pub fn from_point(position: impl Into<GeoPoint3d>) -> Self {
        Self {
            position: position.into(),
            radius_m: 0.0,
            direction: LoiterDirection::Clockwise,
        }
    }
}

pub(super) fn nav_loiter_unlimited_to_wire(
    command: NavLoiterUnlimited,
) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    position_command_to_wire(
        command.position,
        [
            0.0,
            0.0,
            signed_value(command.radius_m, command.direction),
            0.0,
        ],
    )
}

pub(super) fn nav_loiter_unlimited_from_wire(
    frame: MissionFrame,
    params: [f32; 4],
    x: i32,
    y: i32,
    z: f32,
) -> NavLoiterUnlimited {
    NavLoiterUnlimited {
        position: position_from_wire(frame, x, y, z),
        radius_m: params[2].abs(),
        direction: direction_from_signed(params[2]),
    }
}

/// Typed mission command API item used by plan serialization and validation.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct NavLoiterTurns {
    pub position: GeoPoint3d,
    pub turns: f32,
    pub radius_m: f32,
    pub direction: LoiterDirection,
    pub exit_xtrack: bool,
}

impl NavLoiterTurns {
    pub fn from_point(position: impl Into<GeoPoint3d>) -> Self {
        Self {
            position: position.into(),
            turns: 0.0,
            radius_m: 0.0,
            direction: LoiterDirection::Clockwise,
            exit_xtrack: false,
        }
    }
}

pub(super) fn nav_loiter_turns_to_wire(
    command: NavLoiterTurns,
) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    position_command_to_wire(
        command.position,
        [
            command.turns,
            0.0,
            signed_value(command.radius_m, command.direction),
            bool_to_param(command.exit_xtrack),
        ],
    )
}

pub(super) fn nav_loiter_turns_from_wire(
    frame: MissionFrame,
    params: [f32; 4],
    x: i32,
    y: i32,
    z: f32,
) -> NavLoiterTurns {
    NavLoiterTurns {
        position: position_from_wire(frame, x, y, z),
        turns: params[0],
        radius_m: params[2].abs(),
        direction: direction_from_signed(params[2]),
        exit_xtrack: bool_from_param(params[3]),
    }
}

/// Typed mission command API item used by plan serialization and validation.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct NavLoiterTime {
    pub position: GeoPoint3d,
    pub time_s: f32,
    pub direction: LoiterDirection,
    pub exit_xtrack: bool,
}

impl NavLoiterTime {
    pub fn from_point(position: impl Into<GeoPoint3d>) -> Self {
        Self {
            position: position.into(),
            time_s: 0.0,
            direction: LoiterDirection::Clockwise,
            exit_xtrack: false,
        }
    }
}

pub(super) fn nav_loiter_time_to_wire(
    command: NavLoiterTime,
) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    position_command_to_wire(
        command.position,
        [
            command.time_s,
            0.0,
            signed_value(0.0, command.direction),
            bool_to_param(command.exit_xtrack),
        ],
    )
}

pub(super) fn nav_loiter_time_from_wire(
    frame: MissionFrame,
    params: [f32; 4],
    x: i32,
    y: i32,
    z: f32,
) -> NavLoiterTime {
    NavLoiterTime {
        position: position_from_wire(frame, x, y, z),
        time_s: params[0],
        direction: direction_from_signed(params[2]),
        exit_xtrack: bool_from_param(params[3]),
    }
}

/// Typed mission command API item used by plan serialization and validation.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct NavLoiterToAlt {
    pub position: GeoPoint3d,
    pub radius_m: f32,
    pub direction: LoiterDirection,
    pub exit_xtrack: bool,
}

impl NavLoiterToAlt {
    pub fn from_point(position: impl Into<GeoPoint3d>) -> Self {
        Self {
            position: position.into(),
            radius_m: 0.0,
            direction: LoiterDirection::Clockwise,
            exit_xtrack: false,
        }
    }
}

pub(super) fn nav_loiter_to_alt_to_wire(
    command: NavLoiterToAlt,
) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    position_command_to_wire(
        command.position,
        [
            0.0,
            signed_value(command.radius_m, command.direction),
            0.0,
            bool_to_param(command.exit_xtrack),
        ],
    )
}

pub(super) fn nav_loiter_to_alt_from_wire(
    frame: MissionFrame,
    params: [f32; 4],
    x: i32,
    y: i32,
    z: f32,
) -> NavLoiterToAlt {
    NavLoiterToAlt {
        position: position_from_wire(frame, x, y, z),
        radius_m: params[1].abs(),
        direction: direction_from_signed(params[1]),
        exit_xtrack: bool_from_param(params[3]),
    }
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT, category = Nav)]
pub struct NavContinueAndChangeAlt {
    #[position]
    pub position: GeoPoint3d,
    #[param(1, via = alt_change_action_to_param, from = alt_change_action_from_param)]
    pub action: AltChangeAction,
}

impl NavContinueAndChangeAlt {
    pub fn from_point(position: impl Into<GeoPoint3d>) -> Self {
        Self {
            position: position.into(),
            action: AltChangeAction::Neutral,
        }
    }
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = MAV_CMD_NAV_DELAY, category = Nav)]
pub struct NavDelay {
    #[param(1)]
    pub seconds: f32,
    #[param(2)]
    pub hour_utc: f32,
    #[param(3)]
    pub min_utc: f32,
    #[param(4)]
    pub sec_utc: f32,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = MAV_CMD_NAV_GUIDED_ENABLE, category = Nav)]
pub struct NavGuidedEnable {
    #[param(1)]
    pub enabled: bool,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = MAV_CMD_NAV_ALTITUDE_WAIT, category = Nav)]
pub struct NavAltitudeWait {
    #[param(1)]
    pub altitude_m: f32,
    #[param(2)]
    pub descent_rate_mps: f32,
    #[param(3)]
    pub wiggle_time_s: f32,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = MAV_CMD_NAV_VTOL_TAKEOFF, category = Nav)]
pub struct NavVtolTakeoff {
    #[position]
    pub position: GeoPoint3d,
}

impl NavVtolTakeoff {
    pub fn from_point(position: impl Into<GeoPoint3d>) -> Self {
        Self {
            position: position.into(),
        }
    }
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = MAV_CMD_NAV_VTOL_LAND, category = Nav)]
pub struct NavVtolLand {
    #[position]
    pub position: GeoPoint3d,
    #[param(1)]
    pub options: u8,
}

impl NavVtolLand {
    pub fn from_point(position: impl Into<GeoPoint3d>) -> Self {
        Self {
            position: position.into(),
            options: 0,
        }
    }
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = MAV_CMD_NAV_PAYLOAD_PLACE, category = Nav)]
pub struct NavPayloadPlace {
    #[position]
    pub position: GeoPoint3d,
    #[param(1)]
    pub max_descent_m: f32,
}

impl NavPayloadPlace {
    pub fn from_point(position: impl Into<GeoPoint3d>) -> Self {
        Self {
            position: position.into(),
            max_descent_m: 0.0,
        }
    }
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = MAV_CMD_NAV_SET_YAW_SPEED, category = Nav)]
pub struct NavSetYawSpeed {
    #[param(1)]
    pub angle_deg: f32,
    #[param(2)]
    pub speed_mps: f32,
    #[param(3)]
    pub relative: bool,
}

/// Typed mission command API item used by plan serialization and validation.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct NavScriptTime {
    pub command: u16,
    pub timeout_s: f32,
    pub arg1: f32,
    pub arg2: f32,
    pub arg3: i16,
    pub arg4: i16,
}

pub(super) fn nav_script_time_to_wire(
    command: NavScriptTime,
) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire(
        [
            f32::from(command.command),
            command.timeout_s,
            command.arg1,
            command.arg2,
        ],
        i32::from(command.arg3),
        i32::from(command.arg4),
        0.0,
    )
}

pub(super) fn nav_script_time_from_wire(
    _frame: MissionFrame,
    params: [f32; 4],
    x: i32,
    y: i32,
    _z: f32,
) -> NavScriptTime {
    NavScriptTime {
        // f32→u16 saturates in Rust (NaN→0, overflow→clamp).
        command: params[0] as u16,
        timeout_s: params[1],
        arg1: params[2],
        arg2: params[3],
        arg3: i16::try_from(x).unwrap_or(if x > 0 { i16::MAX } else { i16::MIN }),
        arg4: i16::try_from(y).unwrap_or(if y > 0 { i16::MAX } else { i16::MIN }),
    }
}

/// Typed mission command API item used by plan serialization and validation.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct NavAttitudeTime {
    pub time_s: f32,
    pub roll_deg: f32,
    pub pitch_deg: f32,
    pub yaw_deg: f32,
    pub climb_rate_mps: f32,
}

pub(super) fn nav_attitude_time_to_wire(
    command: NavAttitudeTime,
) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    // x field is i32 on wire; round to nearest integer for the climb rate.
    mission_command_to_wire(
        [
            command.time_s,
            command.roll_deg,
            command.pitch_deg,
            command.yaw_deg,
        ],
        command.climb_rate_mps.round() as i32,
        0,
        0.0,
    )
}

pub(super) fn nav_attitude_time_from_wire(
    _frame: MissionFrame,
    params: [f32; 4],
    x: i32,
    _y: i32,
    _z: f32,
) -> NavAttitudeTime {
    NavAttitudeTime {
        time_s: params[0],
        roll_deg: params[1],
        pitch_deg: params[2],
        yaw_deg: params[3],
        // Wire x is i32; widen to f32 (lossless for practical climb rates).
        climb_rate_mps: x as f32,
    }
}
