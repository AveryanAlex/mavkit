#[allow(unused_imports)]
use super::MavCmd::*;
use super::MissionFrame;
use super::wire_support::{bool_from_param, bool_to_param, mission_command_to_wire};
use mavkit_macros::mavkit_command;
use serde::{Deserialize, Serialize};

/// Typed mission command API item used by plan serialization and validation.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum YawDirection {
    Clockwise,
    CounterClockwise,
}

pub(super) fn yaw_direction_to_param(direction: YawDirection) -> f32 {
    match direction {
        YawDirection::Clockwise => 1.0,
        YawDirection::CounterClockwise => -1.0,
    }
}

pub(super) fn yaw_direction_from_param(value: f32) -> YawDirection {
    if value.is_sign_negative() {
        YawDirection::CounterClockwise
    } else {
        YawDirection::Clockwise
    }
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = MAV_CMD_CONDITION_DELAY, category = Condition)]
#[derive(Copy)]
pub struct CondDelay {
    #[param(1)]
    pub delay_s: f32,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = MAV_CMD_CONDITION_DISTANCE, category = Condition)]
#[derive(Copy)]
pub struct CondDistance {
    #[param(1)]
    pub distance_m: f32,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = MAV_CMD_CONDITION_YAW, category = Condition)]
#[derive(Copy)]
pub struct CondYaw {
    #[param(1)]
    pub angle_deg: f32,
    #[param(2)]
    pub turn_rate_dps: f32,
    #[param(3, via = yaw_direction_to_param, from = yaw_direction_from_param)]
    pub direction: YawDirection,
    #[param(4)]
    pub relative: bool,
}
