use super::MissionFrame;
use super::wire_support::{
    bool_from_param, bool_to_param, empty_unit_from_wire, i16_to_wire_i32, mission_command_to_wire,
    position_command_to_wire, position_from_wire, round_f32_to_i32, saturating_i32_to_i16,
    u8_from_param, u16_from_param_saturating_cast, unit_command_to_wire, wire_i32_to_f32,
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

/// Implements a defaulted `from_point` constructor for simple position-backed commands.
macro_rules! impl_position_from_point {
    ($type:ident { $($field:ident : $value:expr),* $(,)? }) => {
        impl $type {
            pub fn from_point(position: impl Into<GeoPoint3d>) -> Self {
                Self {
                    position: position.into(),
                    $(
                        $field: $value,
                    )*
                }
            }
        }
    };
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 16, category = Nav)]
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

impl_position_from_point!(NavWaypoint {
    hold_time_s: 0.0,
    acceptance_radius_m: 0.0,
    pass_radius_m: 0.0,
    yaw_deg: 0.0,
});

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 82, category = Nav)]
pub struct NavSplineWaypoint {
    #[position]
    pub position: GeoPoint3d,
    #[param(1)]
    pub hold_time_s: f32,
}

impl_position_from_point!(NavSplineWaypoint { hold_time_s: 0.0 });

/// Typed mission command API item used by plan serialization and validation.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct NavArcWaypoint {
    pub position: GeoPoint3d,
    pub arc_angle_deg: f32,
    pub direction: LoiterDirection,
}

impl NavArcWaypoint {
    pub(crate) const COMMAND_ID: u16 = 36;

    pub fn from_point(position: impl Into<GeoPoint3d>) -> Self {
        Self {
            position: position.into(),
            arc_angle_deg: 0.0,
            direction: LoiterDirection::Clockwise,
        }
    }

    pub(crate) fn into_wire(self) -> (MissionFrame, [f32; 4], i32, i32, f32) {
        position_command_to_wire(
            self.position,
            [
                signed_value(self.arc_angle_deg, self.direction),
                0.0,
                0.0,
                0.0,
            ],
        )
    }

    pub(crate) fn from_wire(frame: MissionFrame, params: [f32; 4], x: i32, y: i32, z: f32) -> Self {
        Self {
            position: position_from_wire(frame, x, y, z),
            arc_angle_deg: params[0].abs(),
            direction: direction_from_signed(params[0]),
        }
    }
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 22, category = Nav)]
pub struct NavTakeoff {
    #[position]
    pub position: GeoPoint3d,
    #[param(1)]
    pub pitch_deg: f32,
}

impl_position_from_point!(NavTakeoff { pitch_deg: 0.0 });

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 21, category = Nav)]
pub struct NavLand {
    #[position]
    pub position: GeoPoint3d,
    #[param(1)]
    pub abort_alt_m: f32,
}

impl_position_from_point!(NavLand { abort_alt_m: 0.0 });

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
    pub(crate) const COMMAND_ID: u16 = 17;

    pub fn from_point(position: impl Into<GeoPoint3d>) -> Self {
        Self {
            position: position.into(),
            radius_m: 0.0,
            direction: LoiterDirection::Clockwise,
        }
    }

    pub(crate) fn into_wire(self) -> (MissionFrame, [f32; 4], i32, i32, f32) {
        position_command_to_wire(
            self.position,
            [0.0, 0.0, signed_value(self.radius_m, self.direction), 0.0],
        )
    }

    pub(crate) fn from_wire(frame: MissionFrame, params: [f32; 4], x: i32, y: i32, z: f32) -> Self {
        Self {
            position: position_from_wire(frame, x, y, z),
            radius_m: params[2].abs(),
            direction: direction_from_signed(params[2]),
        }
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
    pub(crate) const COMMAND_ID: u16 = 18;

    pub fn from_point(position: impl Into<GeoPoint3d>) -> Self {
        Self {
            position: position.into(),
            turns: 0.0,
            radius_m: 0.0,
            direction: LoiterDirection::Clockwise,
            exit_xtrack: false,
        }
    }

    pub(crate) fn into_wire(self) -> (MissionFrame, [f32; 4], i32, i32, f32) {
        position_command_to_wire(
            self.position,
            [
                self.turns,
                0.0,
                signed_value(self.radius_m, self.direction),
                bool_to_param(self.exit_xtrack),
            ],
        )
    }

    pub(crate) fn from_wire(frame: MissionFrame, params: [f32; 4], x: i32, y: i32, z: f32) -> Self {
        Self {
            position: position_from_wire(frame, x, y, z),
            turns: params[0],
            radius_m: params[2].abs(),
            direction: direction_from_signed(params[2]),
            exit_xtrack: bool_from_param(params[3]),
        }
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
    pub(crate) const COMMAND_ID: u16 = 19;

    pub fn from_point(position: impl Into<GeoPoint3d>) -> Self {
        Self {
            position: position.into(),
            time_s: 0.0,
            direction: LoiterDirection::Clockwise,
            exit_xtrack: false,
        }
    }

    pub(crate) fn into_wire(self) -> (MissionFrame, [f32; 4], i32, i32, f32) {
        position_command_to_wire(
            self.position,
            [
                self.time_s,
                0.0,
                signed_value(0.0, self.direction),
                bool_to_param(self.exit_xtrack),
            ],
        )
    }

    pub(crate) fn from_wire(frame: MissionFrame, params: [f32; 4], x: i32, y: i32, z: f32) -> Self {
        Self {
            position: position_from_wire(frame, x, y, z),
            time_s: params[0],
            direction: direction_from_signed(params[2]),
            exit_xtrack: bool_from_param(params[3]),
        }
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
    pub(crate) const COMMAND_ID: u16 = 31;

    pub fn from_point(position: impl Into<GeoPoint3d>) -> Self {
        Self {
            position: position.into(),
            radius_m: 0.0,
            direction: LoiterDirection::Clockwise,
            exit_xtrack: false,
        }
    }

    pub(crate) fn into_wire(self) -> (MissionFrame, [f32; 4], i32, i32, f32) {
        position_command_to_wire(
            self.position,
            [
                0.0,
                signed_value(self.radius_m, self.direction),
                0.0,
                bool_to_param(self.exit_xtrack),
            ],
        )
    }

    pub(crate) fn from_wire(frame: MissionFrame, params: [f32; 4], x: i32, y: i32, z: f32) -> Self {
        Self {
            position: position_from_wire(frame, x, y, z),
            radius_m: params[1].abs(),
            direction: direction_from_signed(params[1]),
            exit_xtrack: bool_from_param(params[3]),
        }
    }
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 30, category = Nav)]
pub struct NavContinueAndChangeAlt {
    #[position]
    pub position: GeoPoint3d,
    #[param(1, via = alt_change_action_to_param, from = alt_change_action_from_param)]
    pub action: AltChangeAction,
}

impl_position_from_point!(NavContinueAndChangeAlt {
    action: AltChangeAction::Neutral,
});

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 93, category = Nav)]
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
#[mavkit_command(id = 92, category = Nav)]
pub struct NavGuidedEnable {
    #[param(1)]
    pub enabled: bool,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 83, category = Nav)]
pub struct NavAltitudeWait {
    #[param(1)]
    pub altitude_m: f32,
    #[param(2)]
    pub descent_rate_mps: f32,
    #[param(3)]
    pub wiggle_time_s: f32,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 84, category = Nav)]
pub struct NavVtolTakeoff {
    #[position]
    pub position: GeoPoint3d,
}

impl_position_from_point!(NavVtolTakeoff {});

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 85, category = Nav)]
pub struct NavVtolLand {
    #[position]
    pub position: GeoPoint3d,
    #[param(1)]
    pub options: u8,
}

impl_position_from_point!(NavVtolLand { options: 0 });

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 94, category = Nav)]
pub struct NavPayloadPlace {
    #[position]
    pub position: GeoPoint3d,
    #[param(1)]
    pub max_descent_m: f32,
}

impl_position_from_point!(NavPayloadPlace { max_descent_m: 0.0 });

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 213, category = Nav)]
pub struct NavSetYawSpeed {
    #[param(1)]
    pub angle_deg: f32,
    #[param(2)]
    pub speed_mps: f32,
    #[param(3)]
    pub relative: bool,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 42_702, category = Nav)]
pub struct NavScriptTime {
    // f32→u16 saturates in Rust (NaN→0, overflow→clamp).
    #[param(1, via = f32::from, from = u16_from_param_saturating_cast)]
    pub command: u16,
    #[param(2)]
    pub timeout_s: f32,
    #[param(3)]
    pub arg1: f32,
    #[param(4)]
    pub arg2: f32,
    #[wire_x(via = i16_to_wire_i32, from = saturating_i32_to_i16)]
    pub arg3: i16,
    #[wire_y(via = i16_to_wire_i32, from = saturating_i32_to_i16)]
    pub arg4: i16,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 42_703, category = Nav)]
pub struct NavAttitudeTime {
    #[param(1)]
    pub time_s: f32,
    #[param(2)]
    pub roll_deg: f32,
    #[param(3)]
    pub pitch_deg: f32,
    #[param(4)]
    pub yaw_deg: f32,
    // x field is i32 on wire; round to nearest integer for the climb rate.
    // Wire x is i32; widen to f32 (lossless for practical climb rates).
    #[wire_x(via = round_f32_to_i32, from = wire_i32_to_f32)]
    pub climb_rate_mps: f32,
}
