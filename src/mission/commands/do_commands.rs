use super::MissionFrame;
use super::wire_support::{
    bool_from_param, bool_to_param, empty_unit_from_wire, i8_from_param, mission_command_to_wire,
    position_command_to_wire, position_from_wire, u8_from_param, u16_from_param, u32_from_param,
    unit_command_to_wire,
};
use crate::geo::GeoPoint3d;
use mavkit_macros::mavkit_command;
use serde::{Deserialize, Serialize};

/// Typed mission command API item used by plan serialization and validation.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum SpeedType {
    Airspeed,
    Groundspeed,
}

/// Typed mission command API item used by plan serialization and validation.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum FenceAction {
    Disable,
    Enable,
    DisableFloor,
}

/// Typed mission command API item used by plan serialization and validation.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ParachuteAction {
    Disable,
    Enable,
    Release,
}

/// Typed mission command API item used by plan serialization and validation.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum GripperAction {
    Release,
    Grab,
}

/// Typed mission command API item used by plan serialization and validation.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum WinchAction {
    Relax,
    LengthControl,
    RateControl,
}

pub(super) fn speed_type_to_param(speed_type: SpeedType) -> f32 {
    match speed_type {
        SpeedType::Airspeed => 0.0,
        SpeedType::Groundspeed => 1.0,
    }
}

pub(super) fn speed_type_from_param(value: f32) -> SpeedType {
    match value.round() as i32 {
        0 => SpeedType::Airspeed,
        _ => SpeedType::Groundspeed,
    }
}

pub(super) fn fence_action_to_param(action: FenceAction) -> f32 {
    match action {
        FenceAction::Disable => 0.0,
        FenceAction::Enable => 1.0,
        FenceAction::DisableFloor => 2.0,
    }
}

pub(super) fn fence_action_from_param(value: f32) -> FenceAction {
    match value.round() as i32 {
        1 => FenceAction::Enable,
        2 => FenceAction::DisableFloor,
        _ => FenceAction::Disable,
    }
}

pub(super) fn parachute_action_to_param(action: ParachuteAction) -> f32 {
    match action {
        ParachuteAction::Disable => 0.0,
        ParachuteAction::Enable => 1.0,
        ParachuteAction::Release => 2.0,
    }
}

pub(super) fn parachute_action_from_param(value: f32) -> ParachuteAction {
    match value.round() as i32 {
        1 => ParachuteAction::Enable,
        2 => ParachuteAction::Release,
        _ => ParachuteAction::Disable,
    }
}

pub(super) fn gripper_action_to_param(action: GripperAction) -> f32 {
    match action {
        GripperAction::Release => 0.0,
        GripperAction::Grab => 1.0,
    }
}

pub(super) fn gripper_action_from_param(value: f32) -> GripperAction {
    match value.round() as i32 {
        1 => GripperAction::Grab,
        _ => GripperAction::Release,
    }
}

pub(super) fn winch_action_to_param(action: WinchAction) -> f32 {
    match action {
        WinchAction::Relax => 0.0,
        WinchAction::LengthControl => 1.0,
        WinchAction::RateControl => 2.0,
    }
}

pub(super) fn winch_action_from_param(value: f32) -> WinchAction {
    match value.round() as i32 {
        1 => WinchAction::LengthControl,
        2 => WinchAction::RateControl,
        _ => WinchAction::Relax,
    }
}

pub(super) fn engine_allow_disarmed_to_param(value: bool) -> f32 {
    if value { 1.0 } else { 0.0 }
}

pub(super) fn engine_allow_disarmed_from_param(value: f32) -> bool {
    (value.round() as u32 & 1) != 0
}

/// Wire encodes `pause` as the inverse: 1.0 means "continue" (not paused).
pub(super) fn inverted_bool_to_param(value: bool) -> f32 {
    bool_to_param(!value)
}

/// Wire decodes `pause` as the inverse: >0.5 means "continue" (not paused).
pub(super) fn inverted_bool_from_param(value: f32) -> bool {
    !bool_from_param(value)
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
#[mavkit_command(id = 177, category = Do)]
#[derive(Copy)]
pub struct DoJump {
    #[param(1)]
    pub target_index: u16,
    #[param(2)]
    pub repeat_count: u16,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 601, category = Do)]
#[derive(Copy)]
pub struct DoJumpTag {
    #[param(1)]
    pub tag: u16,
    #[param(2)]
    pub repeat_count: u16,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 600, category = Do)]
#[derive(Copy)]
pub struct DoTag {
    #[param(1)]
    pub tag: u16,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 193, category = Do)]
#[derive(Copy)]
pub struct DoPauseContinue {
    #[param(1, via = inverted_bool_to_param, from = inverted_bool_from_param)]
    pub pause: bool,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 178, category = Do)]
#[derive(Copy)]
pub struct DoChangeSpeed {
    #[param(1, via = speed_type_to_param, from = speed_type_from_param)]
    pub speed_type: SpeedType,
    #[param(2)]
    pub speed_mps: f32,
    #[param(3)]
    pub throttle_pct: f32,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 194, category = Do)]
#[derive(Copy)]
pub struct DoSetReverse {
    #[param(1)]
    pub reverse: bool,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 179, category = Do)]
pub struct DoSetHome {
    #[position]
    pub position: GeoPoint3d,
    #[param(1)]
    pub use_current: bool,
}

impl_position_from_point!(DoSetHome { use_current: false });

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 189, category = Do)]
pub struct DoLandStart {
    #[position]
    pub position: GeoPoint3d,
}

impl_position_from_point!(DoLandStart {});

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 188, category = Do)]
pub struct DoReturnPathStart {
    #[position]
    pub position: GeoPoint3d,
}

impl_position_from_point!(DoReturnPathStart {});

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 191, category = Do)]
pub struct DoGoAround {
    #[position]
    pub position: GeoPoint3d,
}

impl_position_from_point!(DoGoAround {});

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 195, category = Do)]
pub struct DoSetRoiLocation {
    #[position]
    pub position: GeoPoint3d,
}

impl_position_from_point!(DoSetRoiLocation {});

pub(super) fn do_set_roi_none_to_wire() -> (MissionFrame, [f32; 4], i32, i32, f32) {
    unit_command_to_wire()
}

pub(super) fn do_set_roi_none_from_wire(
    frame: MissionFrame,
    params: [f32; 4],
    x: i32,
    y: i32,
    z: f32,
) {
    empty_unit_from_wire(frame, params, x, y, z);
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 201, category = Do)]
pub struct DoSetRoi {
    #[param(1)]
    pub mode: u8,
    #[position]
    pub position: GeoPoint3d,
}

impl_position_from_point!(DoSetRoi { mode: 0 });

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 205, category = Do)]
#[derive(Copy)]
pub struct DoMountControl {
    #[param(1)]
    pub pitch_deg: f32,
    #[param(2)]
    pub roll_deg: f32,
    #[param(3)]
    pub yaw_deg: f32,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 1000, category = Do)]
#[derive(Copy)]
pub struct DoGimbalManagerPitchYaw {
    #[param(1)]
    pub pitch_deg: f32,
    #[param(2)]
    pub yaw_deg: f32,
    #[param(3)]
    pub pitch_rate_dps: f32,
    #[param(4)]
    pub yaw_rate_dps: f32,
    #[wire_x]
    pub flags: u32,
    #[wire_z]
    pub gimbal_id: u8,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 206, category = Do)]
#[derive(Copy)]
pub struct DoCamTriggerDistance {
    #[param(1)]
    pub meters: f32,
    #[param(3)]
    pub trigger_now: bool,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 2000, category = Do)]
#[derive(Copy)]
pub struct DoImageStartCapture {
    #[param(1)]
    pub instance: u8,
    #[param(2)]
    pub interval_s: f32,
    #[param(3)]
    pub total_images: u32,
    #[param(4)]
    pub start_number: u32,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 2001, category = Do)]
#[derive(Copy)]
pub struct DoImageStopCapture {
    #[param(1)]
    pub instance: u8,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 2500, category = Do)]
#[derive(Copy)]
pub struct DoVideoStartCapture {
    #[param(1)]
    pub stream_id: u8,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 2501, category = Do)]
#[derive(Copy)]
pub struct DoVideoStopCapture {
    #[param(1)]
    pub stream_id: u8,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 531, category = Do)]
#[derive(Copy)]
pub struct DoSetCameraZoom {
    #[param(1)]
    pub zoom_type: u8,
    #[param(2)]
    pub zoom_value: f32,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 532, category = Do)]
#[derive(Copy)]
pub struct DoSetCameraFocus {
    #[param(1)]
    pub focus_type: u8,
    #[param(2)]
    pub focus_value: f32,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 534, category = Do)]
#[derive(Copy)]
pub struct DoSetCameraSource {
    #[param(1)]
    pub instance: u8,
    #[param(2)]
    pub primary: u8,
    #[param(3)]
    pub secondary: u8,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 202, category = Do)]
#[derive(Copy)]
pub struct DoDigicamConfigure {
    #[param(1)]
    pub shooting_mode: u8,
    #[param(2)]
    pub shutter_speed: u16,
    #[param(3)]
    pub aperture: f32,
    #[param(4)]
    pub iso: u16,
    #[wire_x]
    pub exposure_type: u8,
    #[wire_y]
    pub cmd_id: u8,
    #[wire_z]
    pub cutoff_time: f32,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 203, category = Do)]
#[derive(Copy)]
pub struct DoDigicamControl {
    #[param(1)]
    pub session: u8,
    #[param(2)]
    pub zoom_pos: u8,
    #[param(3)]
    pub zoom_step: i8,
    #[param(4)]
    pub focus_lock: u8,
    #[wire_x]
    pub shooting_cmd: u8,
    #[wire_y]
    pub cmd_id: u8,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 183, category = Do)]
#[derive(Copy)]
pub struct DoSetServo {
    #[param(1)]
    pub channel: u16,
    #[param(2)]
    pub pwm: u16,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 181, category = Do)]
#[derive(Copy)]
pub struct DoSetRelay {
    #[param(1)]
    pub number: u8,
    #[param(2)]
    pub state: bool,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 184, category = Do)]
#[derive(Copy)]
pub struct DoRepeatServo {
    #[param(1)]
    pub channel: u16,
    #[param(2)]
    pub pwm: u16,
    #[param(3)]
    pub count: u16,
    #[param(4)]
    pub cycle_time_s: f32,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 182, category = Do)]
#[derive(Copy)]
pub struct DoRepeatRelay {
    #[param(1)]
    pub number: u8,
    #[param(2)]
    pub count: u16,
    #[param(3)]
    pub cycle_time_s: f32,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 207, category = Do)]
#[derive(Copy)]
pub struct DoFenceEnable {
    #[param(1, via = fence_action_to_param, from = fence_action_from_param)]
    pub action: FenceAction,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 208, category = Do)]
#[derive(Copy)]
pub struct DoParachute {
    #[param(1, via = parachute_action_to_param, from = parachute_action_from_param)]
    pub action: ParachuteAction,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 211, category = Do)]
#[derive(Copy)]
pub struct DoGripper {
    #[param(1)]
    pub number: u8,
    #[param(2, via = gripper_action_to_param, from = gripper_action_from_param)]
    pub action: GripperAction,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 216, category = Do)]
#[derive(Copy)]
pub struct DoSprayer {
    #[param(1)]
    pub enabled: bool,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 42_600, category = Do)]
#[derive(Copy)]
pub struct DoWinch {
    #[param(1)]
    pub number: u8,
    #[param(2, via = winch_action_to_param, from = winch_action_from_param)]
    pub action: WinchAction,
    #[param(3)]
    pub release_length_m: f32,
    #[param(4)]
    pub release_rate_mps: f32,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 223, category = Do)]
#[derive(Copy)]
pub struct DoEngineControl {
    #[param(1)]
    pub start: bool,
    #[param(2)]
    pub cold_start: bool,
    #[param(3)]
    pub height_delay_m: f32,
    #[param(4, via = engine_allow_disarmed_to_param, from = engine_allow_disarmed_from_param)]
    pub allow_disarmed: bool,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 210, category = Do)]
#[derive(Copy)]
pub struct DoInvertedFlight {
    #[param(1)]
    pub inverted: bool,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 212, category = Do)]
#[derive(Copy)]
pub struct DoAutotuneEnable {
    #[param(1)]
    pub enabled: bool,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 3000, category = Do)]
#[derive(Copy)]
pub struct DoVtolTransition {
    #[param(1)]
    pub target_state: u8,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 222, category = Do)]
#[derive(Copy)]
pub struct DoGuidedLimits {
    #[param(1)]
    pub max_time_s: f32,
    #[param(2)]
    pub min_alt_m: f32,
    #[param(3)]
    pub max_alt_m: f32,
    #[param(4)]
    pub max_horiz_m: f32,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 215, category = Do)]
#[derive(Copy)]
pub struct DoSetResumeRepeatDist {
    #[param(1)]
    pub distance_m: f32,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 218, category = Do)]
#[derive(Copy)]
pub struct DoAuxFunction {
    #[param(1)]
    pub function: u16,
    #[param(2)]
    pub switch_pos: u8,
}

/// Typed mission command API item used by plan serialization and validation.
#[mavkit_command(id = 217, category = Do)]
#[derive(Copy)]
pub struct DoSendScriptMessage {
    #[param(1)]
    pub id: u16,
    #[param(2)]
    pub p1: f32,
    #[param(3)]
    pub p2: f32,
    #[param(4)]
    pub p3: f32,
}
