use super::types::{MissionFrame as MissionItemFrame, MissionItem};
use crate::geo::{
    GeoPoint3d, GeoPoint3dMsl, GeoPoint3dRelHome, GeoPoint3dTerrain, quantize_degrees_e7,
};
use serde::{Deserialize, Serialize};

#[allow(non_camel_case_types)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
enum MavCmd {
    MAV_CMD_NAV_WAYPOINT = 16,
    MAV_CMD_NAV_LOITER_UNLIMITED = 17,
    MAV_CMD_NAV_LOITER_TURNS = 18,
    MAV_CMD_NAV_LOITER_TIME = 19,
    MAV_CMD_NAV_RETURN_TO_LAUNCH = 20,
    MAV_CMD_NAV_LAND = 21,
    MAV_CMD_NAV_TAKEOFF = 22,
    MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT = 30,
    MAV_CMD_NAV_LOITER_TO_ALT = 31,
    MAV_CMD_NAV_ARC_WAYPOINT = 36,
    MAV_CMD_NAV_SPLINE_WAYPOINT = 82,
    MAV_CMD_NAV_ALTITUDE_WAIT = 83,
    MAV_CMD_NAV_VTOL_TAKEOFF = 84,
    MAV_CMD_NAV_VTOL_LAND = 85,
    MAV_CMD_NAV_GUIDED_ENABLE = 92,
    MAV_CMD_NAV_DELAY = 93,
    MAV_CMD_NAV_PAYLOAD_PLACE = 94,
    MAV_CMD_DO_JUMP = 177,
    MAV_CMD_DO_CHANGE_SPEED = 178,
    MAV_CMD_DO_SET_HOME = 179,
    MAV_CMD_DO_SET_RELAY = 181,
    MAV_CMD_DO_REPEAT_RELAY = 182,
    MAV_CMD_DO_SET_SERVO = 183,
    MAV_CMD_DO_REPEAT_SERVO = 184,
    MAV_CMD_DO_RETURN_PATH_START = 188,
    MAV_CMD_DO_LAND_START = 189,
    MAV_CMD_DO_GO_AROUND = 191,
    MAV_CMD_DO_PAUSE_CONTINUE = 193,
    MAV_CMD_DO_SET_REVERSE = 194,
    MAV_CMD_DO_SET_ROI_LOCATION = 195,
    MAV_CMD_DO_SET_ROI_NONE = 197,
    MAV_CMD_DO_SET_ROI = 201,
    MAV_CMD_DO_DIGICAM_CONFIGURE = 202,
    MAV_CMD_DO_DIGICAM_CONTROL = 203,
    MAV_CMD_DO_MOUNT_CONTROL = 205,
    MAV_CMD_DO_SET_CAM_TRIGG_DIST = 206,
    MAV_CMD_DO_FENCE_ENABLE = 207,
    MAV_CMD_DO_PARACHUTE = 208,
    MAV_CMD_DO_INVERTED_FLIGHT = 210,
    MAV_CMD_DO_GRIPPER = 211,
    MAV_CMD_DO_AUTOTUNE_ENABLE = 212,
    MAV_CMD_NAV_SET_YAW_SPEED = 213,
    MAV_CMD_DO_SET_RESUME_REPEAT_DIST = 215,
    MAV_CMD_DO_SPRAYER = 216,
    MAV_CMD_DO_SEND_SCRIPT_MESSAGE = 217,
    MAV_CMD_DO_AUX_FUNCTION = 218,
    MAV_CMD_DO_GUIDED_LIMITS = 222,
    MAV_CMD_DO_ENGINE_CONTROL = 223,
    MAV_CMD_CONDITION_DELAY = 112,
    MAV_CMD_CONDITION_DISTANCE = 114,
    MAV_CMD_CONDITION_YAW = 115,
    MAV_CMD_SET_CAMERA_ZOOM = 531,
    MAV_CMD_SET_CAMERA_FOCUS = 532,
    MAV_CMD_SET_CAMERA_SOURCE = 534,
    MAV_CMD_JUMP_TAG = 600,
    MAV_CMD_DO_JUMP_TAG = 601,
    MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW = 1000,
    MAV_CMD_IMAGE_START_CAPTURE = 2000,
    MAV_CMD_IMAGE_STOP_CAPTURE = 2001,
    MAV_CMD_VIDEO_START_CAPTURE = 2500,
    MAV_CMD_VIDEO_STOP_CAPTURE = 2501,
    MAV_CMD_DO_VTOL_TRANSITION = 3000,
    MAV_CMD_DO_WINCH = 42_600,
    MAV_CMD_NAV_SCRIPT_TIME = 42_702,
    MAV_CMD_NAV_ATTITUDE_TIME = 42_703,
}

impl From<MavCmd> for u16 {
    fn from(value: MavCmd) -> Self {
        value as Self
    }
}

use MavCmd::*;

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
/// Typed mission command API item used by plan serialization and validation.
pub enum MissionFrame {
    Global,
    GlobalRelativeAlt,
    GlobalTerrainAlt,
    Mission,
    Other(u8),
}

impl From<MissionFrame> for MissionItemFrame {
    fn from(value: MissionFrame) -> Self {
        match value {
            MissionFrame::Global => MissionItemFrame::GlobalInt,
            MissionFrame::GlobalRelativeAlt => MissionItemFrame::GlobalRelativeAltInt,
            MissionFrame::GlobalTerrainAlt => MissionItemFrame::GlobalTerrainAltInt,
            MissionFrame::Mission => MissionItemFrame::Mission,
            MissionFrame::Other(1) => MissionItemFrame::LocalNed,
            MissionFrame::Other(_) => MissionItemFrame::Other,
        }
    }
}

impl From<MissionItemFrame> for MissionFrame {
    fn from(value: MissionItemFrame) -> Self {
        match value {
            MissionItemFrame::Mission => MissionFrame::Mission,
            MissionItemFrame::GlobalInt => MissionFrame::Global,
            MissionItemFrame::GlobalRelativeAltInt => MissionFrame::GlobalRelativeAlt,
            MissionItemFrame::GlobalTerrainAltInt => MissionFrame::GlobalTerrainAlt,
            MissionItemFrame::LocalNed => MissionFrame::Other(1),
            MissionItemFrame::Other => MissionFrame::Other(0),
        }
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct RawMissionCommand {
    pub command: u16,
    pub frame: MissionFrame,
    pub param1: f32,
    pub param2: f32,
    pub param3: f32,
    pub param4: f32,
    pub x: i32,
    pub y: i32,
    pub z: f32,
}

impl RawMissionCommand {
    fn to_wire(self) -> (u16, MissionFrame, [f32; 4], i32, i32, f32) {
        (
            self.command,
            self.frame,
            [self.param1, self.param2, self.param3, self.param4],
            self.x,
            self.y,
            self.z,
        )
    }
}

macro_rules! mission_commands {
    (
        Nav {
            Data {
                $(
                    $nav_variant:ident($nav_ty:ty) {
                        command: $nav_command:expr,
                        to_wire: $nav_to_wire:path,
                        from_wire: $nav_from_wire:path $(,)?
                    }
                ),+ $(,)?
            }
            Unit {
                $(
                    $nav_unit_variant:ident {
                        command: $nav_unit_command:expr,
                        to_wire: $nav_unit_to_wire:path,
                        from_wire: $nav_unit_from_wire:path $(,)?
                    }
                ),* $(,)?
            }
        }
        Do {
            Data {
                $(
                    $do_variant:ident($do_ty:ty) {
                        command: $do_command:expr,
                        to_wire: $do_to_wire:path,
                        from_wire: $do_from_wire:path $(,)?
                    }
                ),* $(,)?
            }
            Unit {
                $(
                    $do_unit_variant:ident {
                        command: $do_unit_command:expr,
                        to_wire: $do_unit_to_wire:path,
                        from_wire: $do_unit_from_wire:path $(,)?
                    }
                ),* $(,)?
            }
        }
        Condition {
            $(
                $condition_variant:ident($condition_ty:ty) {
                    command: $condition_command:expr,
                    to_wire: $condition_to_wire:path,
                    from_wire: $condition_from_wire:path $(,)?
                }
            ),+ $(,)?
        }
    ) => {
        #[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
        /// Typed mission command API item used by plan serialization and validation.
        pub enum NavCommand {
            $($nav_variant($nav_ty),)+
            $($nav_unit_variant,)*
        }

        #[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
        /// Typed mission command API item used by plan serialization and validation.
        pub enum DoCommand {
            $($do_variant($do_ty),)*
            $($do_unit_variant,)*
        }

        #[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
        /// Typed mission command API item used by plan serialization and validation.
        pub enum ConditionCommand {
            $($condition_variant($condition_ty)),+
        }

        #[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
        /// Typed mission command API item used by plan serialization and validation.
        pub enum MissionCommand {
            Nav(NavCommand),
            Do(DoCommand),
            Condition(ConditionCommand),
            Other(RawMissionCommand),
        }

        impl From<NavCommand> for MissionCommand {
            fn from(value: NavCommand) -> Self {
                Self::Nav(value)
            }
        }

        impl From<DoCommand> for MissionCommand {
            fn from(value: DoCommand) -> Self {
                Self::Do(value)
            }
        }

        impl From<ConditionCommand> for MissionCommand {
            fn from(value: ConditionCommand) -> Self {
                Self::Condition(value)
            }
        }

        impl From<RawMissionCommand> for MissionCommand {
            fn from(value: RawMissionCommand) -> Self {
                Self::Other(value)
            }
        }

        $(
            impl From<$nav_ty> for NavCommand {
                fn from(value: $nav_ty) -> Self {
                    Self::$nav_variant(value)
                }
            }

            impl From<$nav_ty> for MissionCommand {
                fn from(value: $nav_ty) -> Self {
                    Self::Nav(value.into())
                }
            }
        )+

        $(
            impl From<$do_ty> for DoCommand {
                fn from(value: $do_ty) -> Self {
                    Self::$do_variant(value)
                }
            }

            impl From<$do_ty> for MissionCommand {
                fn from(value: $do_ty) -> Self {
                    Self::Do(value.into())
                }
            }
        )*

        $(
            impl From<$condition_ty> for ConditionCommand {
                fn from(value: $condition_ty) -> Self {
                    Self::$condition_variant(value)
                }
            }

            impl From<$condition_ty> for MissionCommand {
                fn from(value: $condition_ty) -> Self {
                    Self::Condition(value.into())
                }
            }
        )+

        impl MissionCommand {
            pub fn from_wire(
                command: u16,
                frame: MissionFrame,
                params: [f32; 4],
                x: i32,
                y: i32,
                z: f32,
            ) -> Self {
                match command {
                    $(
                        code if code == u16::from($nav_command) => {
                            Self::Nav(NavCommand::$nav_variant($nav_from_wire(frame, params, x, y, z)))
                        },
                    )+
                    $(
                        code if code == u16::from($nav_unit_command) => {
                            $nav_unit_from_wire(frame, params, x, y, z);
                            Self::Nav(NavCommand::$nav_unit_variant)
                        },
                    )*
                    $(
                        code if code == u16::from($do_command) => {
                            Self::Do(DoCommand::$do_variant($do_from_wire(frame, params, x, y, z)))
                        },
                    )*
                    $(
                        code if code == u16::from($do_unit_command) => {
                            $do_unit_from_wire(frame, params, x, y, z);
                            Self::Do(DoCommand::$do_unit_variant)
                        },
                    )*
                    $(
                        code if code == u16::from($condition_command) => {
                            Self::Condition(ConditionCommand::$condition_variant(
                                $condition_from_wire(frame, params, x, y, z),
                            ))
                        },
                    )+
                    _ => Self::Other(RawMissionCommand {
                        command,
                        frame,
                        param1: params[0],
                        param2: params[1],
                        param3: params[2],
                        param4: params[3],
                        x,
                        y,
                        z,
                    }),
                }
            }

            pub fn into_wire(self) -> (u16, MissionFrame, [f32; 4], i32, i32, f32) {
                match self {
                    $(
                        Self::Nav(NavCommand::$nav_variant(command_value)) => {
                            let (frame, params, x, y, z) = $nav_to_wire(command_value);
                            (u16::from($nav_command), frame, params, x, y, z)
                        }
                    )+
                    $(
                        Self::Nav(NavCommand::$nav_unit_variant) => {
                            let (frame, params, x, y, z) = $nav_unit_to_wire();
                            (u16::from($nav_unit_command), frame, params, x, y, z)
                        }
                    )*
                    $(
                        Self::Do(DoCommand::$do_variant(command_value)) => {
                            let (frame, params, x, y, z) = $do_to_wire(command_value);
                            (u16::from($do_command), frame, params, x, y, z)
                        }
                    )*
                    $(
                        Self::Do(DoCommand::$do_unit_variant) => {
                            let (frame, params, x, y, z) = $do_unit_to_wire();
                            (u16::from($do_unit_command), frame, params, x, y, z)
                        }
                    )*
                    $(
                        Self::Condition(ConditionCommand::$condition_variant(command_value)) => {
                            let (frame, params, x, y, z) = $condition_to_wire(command_value);
                            (u16::from($condition_command), frame, params, x, y, z)
                        }
                    )+
                    Self::Other(raw) => raw.to_wire(),
                }
            }
        }

        impl<T: Into<MissionCommand>> From<T> for MissionItem {
            fn from(value: T) -> Self {
                Self {
                    command: value.into(),
                    current: false,
                    autocontinue: true,
                }
            }
        }
    };
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
/// Typed mission command API item used by plan serialization and validation.
pub enum LoiterDirection {
    Clockwise,
    CounterClockwise,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
/// Typed mission command API item used by plan serialization and validation.
pub enum YawDirection {
    Clockwise,
    CounterClockwise,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
/// Typed mission command API item used by plan serialization and validation.
pub enum AltChangeAction {
    Neutral,
    Climb,
    Descend,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
/// Typed mission command API item used by plan serialization and validation.
pub enum SpeedType {
    Airspeed,
    Groundspeed,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
/// Typed mission command API item used by plan serialization and validation.
pub enum FenceAction {
    Disable,
    Enable,
    DisableFloor,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
/// Typed mission command API item used by plan serialization and validation.
pub enum ParachuteAction {
    Disable,
    Enable,
    Release,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
/// Typed mission command API item used by plan serialization and validation.
pub enum GripperAction {
    Release,
    Grab,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
/// Typed mission command API item used by plan serialization and validation.
pub enum WinchAction {
    Relax,
    LengthControl,
    RateControl,
}

fn position_to_wire(position: GeoPoint3d) -> (MissionFrame, i32, i32, f32) {
    match position {
        GeoPoint3d::Msl(point) => (
            MissionFrame::Global,
            quantize_degrees_e7(point.latitude_deg),
            quantize_degrees_e7(point.longitude_deg),
            point.altitude_msl_m as f32,
        ),
        GeoPoint3d::RelHome(point) => (
            MissionFrame::GlobalRelativeAlt,
            quantize_degrees_e7(point.latitude_deg),
            quantize_degrees_e7(point.longitude_deg),
            point.relative_alt_m as f32,
        ),
        GeoPoint3d::Terrain(point) => (
            MissionFrame::GlobalTerrainAlt,
            quantize_degrees_e7(point.latitude_deg),
            quantize_degrees_e7(point.longitude_deg),
            point.altitude_terrain_m as f32,
        ),
    }
}

fn position_from_wire(frame: MissionFrame, x: i32, y: i32, z: f32) -> GeoPoint3d {
    let latitude_deg = f64::from(x) / 1e7;
    let longitude_deg = f64::from(y) / 1e7;

    match frame {
        MissionFrame::Global => GeoPoint3d::Msl(GeoPoint3dMsl {
            latitude_deg,
            longitude_deg,
            altitude_msl_m: f64::from(z),
        }),
        MissionFrame::GlobalRelativeAlt => GeoPoint3d::RelHome(GeoPoint3dRelHome {
            latitude_deg,
            longitude_deg,
            relative_alt_m: f64::from(z),
        }),
        MissionFrame::GlobalTerrainAlt => GeoPoint3d::Terrain(GeoPoint3dTerrain {
            latitude_deg,
            longitude_deg,
            altitude_terrain_m: f64::from(z),
        }),
        MissionFrame::Mission | MissionFrame::Other(_) => GeoPoint3d::Msl(GeoPoint3dMsl {
            latitude_deg,
            longitude_deg,
            altitude_msl_m: f64::from(z),
        }),
    }
}

fn position_command_to_wire(
    position: GeoPoint3d,
    params: [f32; 4],
) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    let (frame, x, y, z) = position_to_wire(position);
    (frame, params, x, y, z)
}

fn mission_command_to_wire(
    params: [f32; 4],
    x: i32,
    y: i32,
    z: f32,
) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    (MissionFrame::Mission, params, x, y, z)
}

fn unit_command_to_wire() -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire([0.0, 0.0, 0.0, 0.0], 0, 0, 0.0)
}

fn signed_value(value: f32, direction: LoiterDirection) -> f32 {
    match direction {
        LoiterDirection::Clockwise => value.abs(),
        LoiterDirection::CounterClockwise => -value.abs(),
    }
}

fn direction_from_signed(value: f32) -> LoiterDirection {
    if value.is_sign_negative() {
        LoiterDirection::CounterClockwise
    } else {
        LoiterDirection::Clockwise
    }
}

fn yaw_direction_to_param(direction: YawDirection) -> f32 {
    match direction {
        YawDirection::Clockwise => 1.0,
        YawDirection::CounterClockwise => -1.0,
    }
}

fn yaw_direction_from_param(value: f32) -> YawDirection {
    if value.is_sign_negative() {
        YawDirection::CounterClockwise
    } else {
        YawDirection::Clockwise
    }
}

fn bool_to_param(value: bool) -> f32 {
    if value { 1.0 } else { 0.0 }
}

fn bool_from_param(value: f32) -> bool {
    value > 0.5
}

fn u8_from_param(value: f32) -> u8 {
    value.round() as u8
}

fn i8_from_param(value: f32) -> i8 {
    value.round() as i8
}

fn u16_from_param(value: f32) -> u16 {
    value.round() as u16
}

fn u32_from_param(value: f32) -> u32 {
    value.round() as u32
}

fn speed_type_to_param(speed_type: SpeedType) -> f32 {
    match speed_type {
        SpeedType::Airspeed => 0.0,
        SpeedType::Groundspeed => 1.0,
    }
}

fn speed_type_from_param(value: f32) -> SpeedType {
    match value.round() as i32 {
        0 => SpeedType::Airspeed,
        _ => SpeedType::Groundspeed,
    }
}

fn fence_action_to_param(action: FenceAction) -> f32 {
    match action {
        FenceAction::Disable => 0.0,
        FenceAction::Enable => 1.0,
        FenceAction::DisableFloor => 2.0,
    }
}

fn fence_action_from_param(value: f32) -> FenceAction {
    match value.round() as i32 {
        1 => FenceAction::Enable,
        2 => FenceAction::DisableFloor,
        _ => FenceAction::Disable,
    }
}

fn parachute_action_to_param(action: ParachuteAction) -> f32 {
    match action {
        ParachuteAction::Disable => 0.0,
        ParachuteAction::Enable => 1.0,
        ParachuteAction::Release => 2.0,
    }
}

fn parachute_action_from_param(value: f32) -> ParachuteAction {
    match value.round() as i32 {
        1 => ParachuteAction::Enable,
        2 => ParachuteAction::Release,
        _ => ParachuteAction::Disable,
    }
}

fn gripper_action_to_param(action: GripperAction) -> f32 {
    match action {
        GripperAction::Release => 0.0,
        GripperAction::Grab => 1.0,
    }
}

fn gripper_action_from_param(value: f32) -> GripperAction {
    match value.round() as i32 {
        1 => GripperAction::Grab,
        _ => GripperAction::Release,
    }
}

fn winch_action_to_param(action: WinchAction) -> f32 {
    match action {
        WinchAction::Relax => 0.0,
        WinchAction::LengthControl => 1.0,
        WinchAction::RateControl => 2.0,
    }
}

fn winch_action_from_param(value: f32) -> WinchAction {
    match value.round() as i32 {
        1 => WinchAction::LengthControl,
        2 => WinchAction::RateControl,
        _ => WinchAction::Relax,
    }
}

fn engine_allow_disarmed_to_param(value: bool) -> f32 {
    if value { 1.0 } else { 0.0 }
}

fn engine_allow_disarmed_from_param(value: f32) -> bool {
    (value.round() as u32 & 1) != 0
}

fn alt_change_action_to_param(action: AltChangeAction) -> f32 {
    match action {
        AltChangeAction::Neutral => 0.0,
        AltChangeAction::Climb => 1.0,
        AltChangeAction::Descend => 2.0,
    }
}

fn alt_change_action_from_param(value: f32) -> AltChangeAction {
    match value.round() as i32 {
        1 => AltChangeAction::Climb,
        2 => AltChangeAction::Descend,
        _ => AltChangeAction::Neutral,
    }
}

fn empty_unit_from_wire(_frame: MissionFrame, _params: [f32; 4], _x: i32, _y: i32, _z: f32) {}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct NavWaypoint {
    pub position: GeoPoint3d,
    pub hold_time_s: f32,
    pub acceptance_radius_m: f32,
    pub pass_radius_m: f32,
    pub yaw_deg: f32,
}

fn nav_waypoint_to_wire(command: NavWaypoint) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    position_command_to_wire(
        command.position,
        [
            command.hold_time_s,
            command.acceptance_radius_m,
            command.pass_radius_m,
            command.yaw_deg,
        ],
    )
}

fn nav_waypoint_from_wire(
    frame: MissionFrame,
    params: [f32; 4],
    x: i32,
    y: i32,
    z: f32,
) -> NavWaypoint {
    NavWaypoint {
        position: position_from_wire(frame, x, y, z),
        hold_time_s: params[0],
        acceptance_radius_m: params[1],
        pass_radius_m: params[2],
        yaw_deg: params[3],
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct NavSplineWaypoint {
    pub position: GeoPoint3d,
    pub hold_time_s: f32,
}

fn nav_spline_waypoint_to_wire(
    command: NavSplineWaypoint,
) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    position_command_to_wire(command.position, [command.hold_time_s, 0.0, 0.0, 0.0])
}

fn nav_spline_waypoint_from_wire(
    frame: MissionFrame,
    params: [f32; 4],
    x: i32,
    y: i32,
    z: f32,
) -> NavSplineWaypoint {
    NavSplineWaypoint {
        position: position_from_wire(frame, x, y, z),
        hold_time_s: params[0],
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct NavArcWaypoint {
    pub position: GeoPoint3d,
    pub arc_angle_deg: f32,
    pub direction: LoiterDirection,
}

fn nav_arc_waypoint_to_wire(command: NavArcWaypoint) -> (MissionFrame, [f32; 4], i32, i32, f32) {
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

fn nav_arc_waypoint_from_wire(
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

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct NavTakeoff {
    pub position: GeoPoint3d,
    pub pitch_deg: f32,
}

fn nav_takeoff_to_wire(command: NavTakeoff) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    position_command_to_wire(command.position, [command.pitch_deg, 0.0, 0.0, 0.0])
}

fn nav_takeoff_from_wire(
    frame: MissionFrame,
    params: [f32; 4],
    x: i32,
    y: i32,
    z: f32,
) -> NavTakeoff {
    NavTakeoff {
        position: position_from_wire(frame, x, y, z),
        pitch_deg: params[0],
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct NavLand {
    pub position: GeoPoint3d,
    pub abort_alt_m: f32,
}

fn nav_land_to_wire(command: NavLand) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    position_command_to_wire(command.position, [command.abort_alt_m, 0.0, 0.0, 0.0])
}

fn nav_land_from_wire(frame: MissionFrame, params: [f32; 4], x: i32, y: i32, z: f32) -> NavLand {
    NavLand {
        position: position_from_wire(frame, x, y, z),
        abort_alt_m: params[0],
    }
}

fn return_to_launch_to_wire() -> (MissionFrame, [f32; 4], i32, i32, f32) {
    unit_command_to_wire()
}

fn return_to_launch_from_wire(frame: MissionFrame, params: [f32; 4], x: i32, y: i32, z: f32) {
    empty_unit_from_wire(frame, params, x, y, z);
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct NavLoiterUnlimited {
    pub position: GeoPoint3d,
    pub radius_m: f32,
    pub direction: LoiterDirection,
}

fn nav_loiter_unlimited_to_wire(
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

fn nav_loiter_unlimited_from_wire(
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

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct NavLoiterTurns {
    pub position: GeoPoint3d,
    pub turns: f32,
    pub radius_m: f32,
    pub direction: LoiterDirection,
    pub exit_xtrack: bool,
}

fn nav_loiter_turns_to_wire(command: NavLoiterTurns) -> (MissionFrame, [f32; 4], i32, i32, f32) {
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

fn nav_loiter_turns_from_wire(
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

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct NavLoiterTime {
    pub position: GeoPoint3d,
    pub time_s: f32,
    pub direction: LoiterDirection,
    pub exit_xtrack: bool,
}

fn nav_loiter_time_to_wire(command: NavLoiterTime) -> (MissionFrame, [f32; 4], i32, i32, f32) {
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

fn nav_loiter_time_from_wire(
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

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct NavLoiterToAlt {
    pub position: GeoPoint3d,
    pub radius_m: f32,
    pub direction: LoiterDirection,
    pub exit_xtrack: bool,
}

fn nav_loiter_to_alt_to_wire(command: NavLoiterToAlt) -> (MissionFrame, [f32; 4], i32, i32, f32) {
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

fn nav_loiter_to_alt_from_wire(
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

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct NavContinueAndChangeAlt {
    pub position: GeoPoint3d,
    pub action: AltChangeAction,
}

fn nav_continue_and_change_alt_to_wire(
    command: NavContinueAndChangeAlt,
) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    position_command_to_wire(
        command.position,
        [alt_change_action_to_param(command.action), 0.0, 0.0, 0.0],
    )
}

fn nav_continue_and_change_alt_from_wire(
    frame: MissionFrame,
    params: [f32; 4],
    x: i32,
    y: i32,
    z: f32,
) -> NavContinueAndChangeAlt {
    NavContinueAndChangeAlt {
        position: position_from_wire(frame, x, y, z),
        action: alt_change_action_from_param(params[0]),
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct NavDelay {
    pub seconds: f32,
    pub hour_utc: f32,
    pub min_utc: f32,
    pub sec_utc: f32,
}

fn nav_delay_to_wire(command: NavDelay) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire(
        [
            command.seconds,
            command.hour_utc,
            command.min_utc,
            command.sec_utc,
        ],
        0,
        0,
        0.0,
    )
}

fn nav_delay_from_wire(
    _frame: MissionFrame,
    params: [f32; 4],
    _x: i32,
    _y: i32,
    _z: f32,
) -> NavDelay {
    NavDelay {
        seconds: params[0],
        hour_utc: params[1],
        min_utc: params[2],
        sec_utc: params[3],
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct NavGuidedEnable {
    pub enabled: bool,
}

fn nav_guided_enable_to_wire(command: NavGuidedEnable) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire([bool_to_param(command.enabled), 0.0, 0.0, 0.0], 0, 0, 0.0)
}

fn nav_guided_enable_from_wire(
    _frame: MissionFrame,
    params: [f32; 4],
    _x: i32,
    _y: i32,
    _z: f32,
) -> NavGuidedEnable {
    NavGuidedEnable {
        enabled: bool_from_param(params[0]),
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct NavAltitudeWait {
    pub altitude_m: f32,
    pub descent_rate_mps: f32,
    pub wiggle_time_s: f32,
}

fn nav_altitude_wait_to_wire(command: NavAltitudeWait) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire(
        [
            command.altitude_m,
            command.descent_rate_mps,
            command.wiggle_time_s,
            0.0,
        ],
        0,
        0,
        0.0,
    )
}

fn nav_altitude_wait_from_wire(
    _frame: MissionFrame,
    params: [f32; 4],
    _x: i32,
    _y: i32,
    _z: f32,
) -> NavAltitudeWait {
    NavAltitudeWait {
        altitude_m: params[0],
        descent_rate_mps: params[1],
        wiggle_time_s: params[2],
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct NavVtolTakeoff {
    pub position: GeoPoint3d,
}

fn nav_vtol_takeoff_to_wire(command: NavVtolTakeoff) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    position_command_to_wire(command.position, [0.0, 0.0, 0.0, 0.0])
}

fn nav_vtol_takeoff_from_wire(
    frame: MissionFrame,
    _params: [f32; 4],
    x: i32,
    y: i32,
    z: f32,
) -> NavVtolTakeoff {
    NavVtolTakeoff {
        position: position_from_wire(frame, x, y, z),
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct NavVtolLand {
    pub position: GeoPoint3d,
    pub options: u8,
}

fn nav_vtol_land_to_wire(command: NavVtolLand) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    position_command_to_wire(
        command.position,
        [f32::from(command.options), 0.0, 0.0, 0.0],
    )
}

fn nav_vtol_land_from_wire(
    frame: MissionFrame,
    params: [f32; 4],
    x: i32,
    y: i32,
    z: f32,
) -> NavVtolLand {
    NavVtolLand {
        position: position_from_wire(frame, x, y, z),
        options: params[0] as u8,
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct NavPayloadPlace {
    pub position: GeoPoint3d,
    pub max_descent_m: f32,
}

fn nav_payload_place_to_wire(command: NavPayloadPlace) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    position_command_to_wire(command.position, [command.max_descent_m, 0.0, 0.0, 0.0])
}

fn nav_payload_place_from_wire(
    frame: MissionFrame,
    params: [f32; 4],
    x: i32,
    y: i32,
    z: f32,
) -> NavPayloadPlace {
    NavPayloadPlace {
        position: position_from_wire(frame, x, y, z),
        max_descent_m: params[0],
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct NavSetYawSpeed {
    pub angle_deg: f32,
    pub speed_mps: f32,
    pub relative: bool,
}

fn nav_set_yaw_speed_to_wire(command: NavSetYawSpeed) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire(
        [
            command.angle_deg,
            command.speed_mps,
            bool_to_param(command.relative),
            0.0,
        ],
        0,
        0,
        0.0,
    )
}

fn nav_set_yaw_speed_from_wire(
    _frame: MissionFrame,
    params: [f32; 4],
    _x: i32,
    _y: i32,
    _z: f32,
) -> NavSetYawSpeed {
    NavSetYawSpeed {
        angle_deg: params[0],
        speed_mps: params[1],
        relative: bool_from_param(params[2]),
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct NavScriptTime {
    pub command: u16,
    pub timeout_s: f32,
    pub arg1: f32,
    pub arg2: f32,
    pub arg3: i16,
    pub arg4: i16,
}

fn nav_script_time_to_wire(command: NavScriptTime) -> (MissionFrame, [f32; 4], i32, i32, f32) {
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

fn nav_script_time_from_wire(
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

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct NavAttitudeTime {
    pub time_s: f32,
    pub roll_deg: f32,
    pub pitch_deg: f32,
    pub yaw_deg: f32,
    pub climb_rate_mps: f32,
}

fn nav_attitude_time_to_wire(command: NavAttitudeTime) -> (MissionFrame, [f32; 4], i32, i32, f32) {
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

fn nav_attitude_time_from_wire(
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

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct DoJump {
    pub target_index: u16,
    pub repeat_count: u16,
}

fn do_jump_to_wire(command: DoJump) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire(
        [
            f32::from(command.target_index),
            f32::from(command.repeat_count),
            0.0,
            0.0,
        ],
        0,
        0,
        0.0,
    )
}

fn do_jump_from_wire(_frame: MissionFrame, params: [f32; 4], _x: i32, _y: i32, _z: f32) -> DoJump {
    DoJump {
        target_index: u16_from_param(params[0]),
        repeat_count: u16_from_param(params[1]),
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct DoJumpTag {
    pub tag: u16,
    pub repeat_count: u16,
}

fn do_jump_tag_to_wire(command: DoJumpTag) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire(
        [
            f32::from(command.tag),
            f32::from(command.repeat_count),
            0.0,
            0.0,
        ],
        0,
        0,
        0.0,
    )
}

fn do_jump_tag_from_wire(
    _frame: MissionFrame,
    params: [f32; 4],
    _x: i32,
    _y: i32,
    _z: f32,
) -> DoJumpTag {
    DoJumpTag {
        tag: u16_from_param(params[0]),
        repeat_count: u16_from_param(params[1]),
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct DoTag {
    pub tag: u16,
}

fn do_tag_to_wire(command: DoTag) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire([f32::from(command.tag), 0.0, 0.0, 0.0], 0, 0, 0.0)
}

fn do_tag_from_wire(_frame: MissionFrame, params: [f32; 4], _x: i32, _y: i32, _z: f32) -> DoTag {
    DoTag {
        tag: u16_from_param(params[0]),
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct DoPauseContinue {
    pub pause: bool,
}

fn do_pause_continue_to_wire(command: DoPauseContinue) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire([bool_to_param(!command.pause), 0.0, 0.0, 0.0], 0, 0, 0.0)
}

fn do_pause_continue_from_wire(
    _frame: MissionFrame,
    params: [f32; 4],
    _x: i32,
    _y: i32,
    _z: f32,
) -> DoPauseContinue {
    DoPauseContinue {
        pause: !bool_from_param(params[0]),
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct DoChangeSpeed {
    pub speed_type: SpeedType,
    pub speed_mps: f32,
    pub throttle_pct: f32,
}

fn do_change_speed_to_wire(command: DoChangeSpeed) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire(
        [
            speed_type_to_param(command.speed_type),
            command.speed_mps,
            command.throttle_pct,
            0.0,
        ],
        0,
        0,
        0.0,
    )
}

fn do_change_speed_from_wire(
    _frame: MissionFrame,
    params: [f32; 4],
    _x: i32,
    _y: i32,
    _z: f32,
) -> DoChangeSpeed {
    DoChangeSpeed {
        speed_type: speed_type_from_param(params[0]),
        speed_mps: params[1],
        throttle_pct: params[2],
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct DoSetReverse {
    pub reverse: bool,
}

fn do_set_reverse_to_wire(command: DoSetReverse) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire([bool_to_param(command.reverse), 0.0, 0.0, 0.0], 0, 0, 0.0)
}

fn do_set_reverse_from_wire(
    _frame: MissionFrame,
    params: [f32; 4],
    _x: i32,
    _y: i32,
    _z: f32,
) -> DoSetReverse {
    DoSetReverse {
        reverse: bool_from_param(params[0]),
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct DoSetHome {
    pub position: GeoPoint3d,
    pub use_current: bool,
}

fn do_set_home_to_wire(command: DoSetHome) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    position_command_to_wire(
        command.position,
        [bool_to_param(command.use_current), 0.0, 0.0, 0.0],
    )
}

fn do_set_home_from_wire(
    frame: MissionFrame,
    params: [f32; 4],
    x: i32,
    y: i32,
    z: f32,
) -> DoSetHome {
    DoSetHome {
        position: position_from_wire(frame, x, y, z),
        use_current: bool_from_param(params[0]),
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct DoLandStart {
    pub position: GeoPoint3d,
}

fn do_land_start_to_wire(command: DoLandStart) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    position_command_to_wire(command.position, [0.0, 0.0, 0.0, 0.0])
}

fn do_land_start_from_wire(
    frame: MissionFrame,
    _params: [f32; 4],
    x: i32,
    y: i32,
    z: f32,
) -> DoLandStart {
    DoLandStart {
        position: position_from_wire(frame, x, y, z),
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct DoReturnPathStart {
    pub position: GeoPoint3d,
}

fn do_return_path_start_to_wire(
    command: DoReturnPathStart,
) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    position_command_to_wire(command.position, [0.0, 0.0, 0.0, 0.0])
}

fn do_return_path_start_from_wire(
    frame: MissionFrame,
    _params: [f32; 4],
    x: i32,
    y: i32,
    z: f32,
) -> DoReturnPathStart {
    DoReturnPathStart {
        position: position_from_wire(frame, x, y, z),
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct DoGoAround {
    pub position: GeoPoint3d,
}

fn do_go_around_to_wire(command: DoGoAround) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    position_command_to_wire(command.position, [0.0, 0.0, 0.0, 0.0])
}

fn do_go_around_from_wire(
    frame: MissionFrame,
    _params: [f32; 4],
    x: i32,
    y: i32,
    z: f32,
) -> DoGoAround {
    DoGoAround {
        position: position_from_wire(frame, x, y, z),
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct DoSetRoiLocation {
    pub position: GeoPoint3d,
}

fn do_set_roi_location_to_wire(
    command: DoSetRoiLocation,
) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    position_command_to_wire(command.position, [0.0, 0.0, 0.0, 0.0])
}

fn do_set_roi_location_from_wire(
    frame: MissionFrame,
    _params: [f32; 4],
    x: i32,
    y: i32,
    z: f32,
) -> DoSetRoiLocation {
    DoSetRoiLocation {
        position: position_from_wire(frame, x, y, z),
    }
}

fn do_set_roi_none_to_wire() -> (MissionFrame, [f32; 4], i32, i32, f32) {
    unit_command_to_wire()
}

fn do_set_roi_none_from_wire(frame: MissionFrame, params: [f32; 4], x: i32, y: i32, z: f32) {
    empty_unit_from_wire(frame, params, x, y, z);
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct DoSetRoi {
    pub mode: u8,
    pub position: GeoPoint3d,
}

fn do_set_roi_to_wire(command: DoSetRoi) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    position_command_to_wire(command.position, [f32::from(command.mode), 0.0, 0.0, 0.0])
}

fn do_set_roi_from_wire(frame: MissionFrame, params: [f32; 4], x: i32, y: i32, z: f32) -> DoSetRoi {
    DoSetRoi {
        mode: u8_from_param(params[0]),
        position: position_from_wire(frame, x, y, z),
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct DoMountControl {
    pub pitch_deg: f32,
    pub roll_deg: f32,
    pub yaw_deg: f32,
}

fn do_mount_control_to_wire(command: DoMountControl) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire(
        [command.pitch_deg, command.roll_deg, command.yaw_deg, 0.0],
        0,
        0,
        0.0,
    )
}

fn do_mount_control_from_wire(
    _frame: MissionFrame,
    params: [f32; 4],
    _x: i32,
    _y: i32,
    _z: f32,
) -> DoMountControl {
    DoMountControl {
        pitch_deg: params[0],
        roll_deg: params[1],
        yaw_deg: params[2],
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct DoGimbalManagerPitchYaw {
    pub pitch_deg: f32,
    pub yaw_deg: f32,
    pub pitch_rate_dps: f32,
    pub yaw_rate_dps: f32,
    pub flags: u32,
    pub gimbal_id: u8,
}

fn do_gimbal_manager_pitch_yaw_to_wire(
    command: DoGimbalManagerPitchYaw,
) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire(
        [
            command.pitch_deg,
            command.yaw_deg,
            command.pitch_rate_dps,
            command.yaw_rate_dps,
        ],
        command.flags as i32,
        0,
        f32::from(command.gimbal_id),
    )
}

fn do_gimbal_manager_pitch_yaw_from_wire(
    _frame: MissionFrame,
    params: [f32; 4],
    x: i32,
    _y: i32,
    z: f32,
) -> DoGimbalManagerPitchYaw {
    DoGimbalManagerPitchYaw {
        pitch_deg: params[0],
        yaw_deg: params[1],
        pitch_rate_dps: params[2],
        yaw_rate_dps: params[3],
        flags: x as u32,
        gimbal_id: z.round() as u8,
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct DoCamTriggerDistance {
    pub meters: f32,
    pub trigger_now: bool,
}

fn do_cam_trigger_distance_to_wire(
    command: DoCamTriggerDistance,
) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire(
        [command.meters, 0.0, bool_to_param(command.trigger_now), 0.0],
        0,
        0,
        0.0,
    )
}

fn do_cam_trigger_distance_from_wire(
    _frame: MissionFrame,
    params: [f32; 4],
    _x: i32,
    _y: i32,
    _z: f32,
) -> DoCamTriggerDistance {
    DoCamTriggerDistance {
        meters: params[0],
        trigger_now: bool_from_param(params[2]),
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct DoImageStartCapture {
    pub instance: u8,
    pub interval_s: f32,
    pub total_images: u32,
    pub start_number: u32,
}

fn do_image_start_capture_to_wire(
    command: DoImageStartCapture,
) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire(
        [
            f32::from(command.instance),
            command.interval_s,
            command.total_images as f32,
            command.start_number as f32,
        ],
        0,
        0,
        0.0,
    )
}

fn do_image_start_capture_from_wire(
    _frame: MissionFrame,
    params: [f32; 4],
    _x: i32,
    _y: i32,
    _z: f32,
) -> DoImageStartCapture {
    DoImageStartCapture {
        instance: u8_from_param(params[0]),
        interval_s: params[1],
        total_images: u32_from_param(params[2]),
        start_number: u32_from_param(params[3]),
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct DoImageStopCapture {
    pub instance: u8,
}

fn do_image_stop_capture_to_wire(
    command: DoImageStopCapture,
) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire([f32::from(command.instance), 0.0, 0.0, 0.0], 0, 0, 0.0)
}

fn do_image_stop_capture_from_wire(
    _frame: MissionFrame,
    params: [f32; 4],
    _x: i32,
    _y: i32,
    _z: f32,
) -> DoImageStopCapture {
    DoImageStopCapture {
        instance: u8_from_param(params[0]),
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct DoVideoStartCapture {
    pub stream_id: u8,
}

fn do_video_start_capture_to_wire(
    command: DoVideoStartCapture,
) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire([f32::from(command.stream_id), 0.0, 0.0, 0.0], 0, 0, 0.0)
}

fn do_video_start_capture_from_wire(
    _frame: MissionFrame,
    params: [f32; 4],
    _x: i32,
    _y: i32,
    _z: f32,
) -> DoVideoStartCapture {
    DoVideoStartCapture {
        stream_id: u8_from_param(params[0]),
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct DoVideoStopCapture {
    pub stream_id: u8,
}

fn do_video_stop_capture_to_wire(
    command: DoVideoStopCapture,
) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire([f32::from(command.stream_id), 0.0, 0.0, 0.0], 0, 0, 0.0)
}

fn do_video_stop_capture_from_wire(
    _frame: MissionFrame,
    params: [f32; 4],
    _x: i32,
    _y: i32,
    _z: f32,
) -> DoVideoStopCapture {
    DoVideoStopCapture {
        stream_id: u8_from_param(params[0]),
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct DoSetCameraZoom {
    pub zoom_type: u8,
    pub zoom_value: f32,
}

fn do_set_camera_zoom_to_wire(command: DoSetCameraZoom) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire(
        [f32::from(command.zoom_type), command.zoom_value, 0.0, 0.0],
        0,
        0,
        0.0,
    )
}

fn do_set_camera_zoom_from_wire(
    _frame: MissionFrame,
    params: [f32; 4],
    _x: i32,
    _y: i32,
    _z: f32,
) -> DoSetCameraZoom {
    DoSetCameraZoom {
        zoom_type: u8_from_param(params[0]),
        zoom_value: params[1],
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct DoSetCameraFocus {
    pub focus_type: u8,
    pub focus_value: f32,
}

fn do_set_camera_focus_to_wire(
    command: DoSetCameraFocus,
) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire(
        [f32::from(command.focus_type), command.focus_value, 0.0, 0.0],
        0,
        0,
        0.0,
    )
}

fn do_set_camera_focus_from_wire(
    _frame: MissionFrame,
    params: [f32; 4],
    _x: i32,
    _y: i32,
    _z: f32,
) -> DoSetCameraFocus {
    DoSetCameraFocus {
        focus_type: u8_from_param(params[0]),
        focus_value: params[1],
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct DoSetCameraSource {
    pub instance: u8,
    pub primary: u8,
    pub secondary: u8,
}

fn do_set_camera_source_to_wire(
    command: DoSetCameraSource,
) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire(
        [
            f32::from(command.instance),
            f32::from(command.primary),
            f32::from(command.secondary),
            0.0,
        ],
        0,
        0,
        0.0,
    )
}

fn do_set_camera_source_from_wire(
    _frame: MissionFrame,
    params: [f32; 4],
    _x: i32,
    _y: i32,
    _z: f32,
) -> DoSetCameraSource {
    DoSetCameraSource {
        instance: u8_from_param(params[0]),
        primary: u8_from_param(params[1]),
        secondary: u8_from_param(params[2]),
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct DoDigicamConfigure {
    pub shooting_mode: u8,
    pub shutter_speed: u16,
    pub aperture: f32,
    pub iso: u16,
    pub exposure_type: u8,
    pub cmd_id: u8,
    pub cutoff_time: f32,
}

fn do_digicam_configure_to_wire(
    command: DoDigicamConfigure,
) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire(
        [
            f32::from(command.shooting_mode),
            f32::from(command.shutter_speed),
            command.aperture,
            f32::from(command.iso),
        ],
        i32::from(command.exposure_type),
        i32::from(command.cmd_id),
        command.cutoff_time,
    )
}

fn do_digicam_configure_from_wire(
    _frame: MissionFrame,
    params: [f32; 4],
    x: i32,
    y: i32,
    z: f32,
) -> DoDigicamConfigure {
    DoDigicamConfigure {
        shooting_mode: u8_from_param(params[0]),
        shutter_speed: u16_from_param(params[1]),
        aperture: params[2],
        iso: u16_from_param(params[3]),
        exposure_type: x as u8,
        cmd_id: y as u8,
        cutoff_time: z,
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct DoDigicamControl {
    pub session: u8,
    pub zoom_pos: u8,
    pub zoom_step: i8,
    pub focus_lock: u8,
    pub shooting_cmd: u8,
    pub cmd_id: u8,
}

fn do_digicam_control_to_wire(
    command: DoDigicamControl,
) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire(
        [
            f32::from(command.session),
            f32::from(command.zoom_pos),
            f32::from(command.zoom_step),
            f32::from(command.focus_lock),
        ],
        i32::from(command.shooting_cmd),
        i32::from(command.cmd_id),
        0.0,
    )
}

fn do_digicam_control_from_wire(
    _frame: MissionFrame,
    params: [f32; 4],
    x: i32,
    y: i32,
    _z: f32,
) -> DoDigicamControl {
    DoDigicamControl {
        session: u8_from_param(params[0]),
        zoom_pos: u8_from_param(params[1]),
        zoom_step: i8_from_param(params[2]),
        focus_lock: u8_from_param(params[3]),
        shooting_cmd: x as u8,
        cmd_id: y as u8,
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct DoSetServo {
    pub channel: u16,
    pub pwm: u16,
}

fn do_set_servo_to_wire(command: DoSetServo) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire(
        [f32::from(command.channel), f32::from(command.pwm), 0.0, 0.0],
        0,
        0,
        0.0,
    )
}

fn do_set_servo_from_wire(
    _frame: MissionFrame,
    params: [f32; 4],
    _x: i32,
    _y: i32,
    _z: f32,
) -> DoSetServo {
    DoSetServo {
        channel: u16_from_param(params[0]),
        pwm: u16_from_param(params[1]),
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct DoSetRelay {
    pub number: u8,
    pub state: bool,
}

fn do_set_relay_to_wire(command: DoSetRelay) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire(
        [
            f32::from(command.number),
            bool_to_param(command.state),
            0.0,
            0.0,
        ],
        0,
        0,
        0.0,
    )
}

fn do_set_relay_from_wire(
    _frame: MissionFrame,
    params: [f32; 4],
    _x: i32,
    _y: i32,
    _z: f32,
) -> DoSetRelay {
    DoSetRelay {
        number: u8_from_param(params[0]),
        state: bool_from_param(params[1]),
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct DoRepeatServo {
    pub channel: u16,
    pub pwm: u16,
    pub count: u16,
    pub cycle_time_s: f32,
}

fn do_repeat_servo_to_wire(command: DoRepeatServo) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire(
        [
            f32::from(command.channel),
            f32::from(command.pwm),
            f32::from(command.count),
            command.cycle_time_s,
        ],
        0,
        0,
        0.0,
    )
}

fn do_repeat_servo_from_wire(
    _frame: MissionFrame,
    params: [f32; 4],
    _x: i32,
    _y: i32,
    _z: f32,
) -> DoRepeatServo {
    DoRepeatServo {
        channel: u16_from_param(params[0]),
        pwm: u16_from_param(params[1]),
        count: u16_from_param(params[2]),
        cycle_time_s: params[3],
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct DoRepeatRelay {
    pub number: u8,
    pub count: u16,
    pub cycle_time_s: f32,
}

fn do_repeat_relay_to_wire(command: DoRepeatRelay) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire(
        [
            f32::from(command.number),
            f32::from(command.count),
            command.cycle_time_s,
            0.0,
        ],
        0,
        0,
        0.0,
    )
}

fn do_repeat_relay_from_wire(
    _frame: MissionFrame,
    params: [f32; 4],
    _x: i32,
    _y: i32,
    _z: f32,
) -> DoRepeatRelay {
    DoRepeatRelay {
        number: u8_from_param(params[0]),
        count: u16_from_param(params[1]),
        cycle_time_s: params[2],
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct DoFenceEnable {
    pub action: FenceAction,
}

fn do_fence_enable_to_wire(command: DoFenceEnable) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire(
        [fence_action_to_param(command.action), 0.0, 0.0, 0.0],
        0,
        0,
        0.0,
    )
}

fn do_fence_enable_from_wire(
    _frame: MissionFrame,
    params: [f32; 4],
    _x: i32,
    _y: i32,
    _z: f32,
) -> DoFenceEnable {
    DoFenceEnable {
        action: fence_action_from_param(params[0]),
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct DoParachute {
    pub action: ParachuteAction,
}

fn do_parachute_to_wire(command: DoParachute) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire(
        [parachute_action_to_param(command.action), 0.0, 0.0, 0.0],
        0,
        0,
        0.0,
    )
}

fn do_parachute_from_wire(
    _frame: MissionFrame,
    params: [f32; 4],
    _x: i32,
    _y: i32,
    _z: f32,
) -> DoParachute {
    DoParachute {
        action: parachute_action_from_param(params[0]),
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct DoGripper {
    pub number: u8,
    pub action: GripperAction,
}

fn do_gripper_to_wire(command: DoGripper) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire(
        [
            f32::from(command.number),
            gripper_action_to_param(command.action),
            0.0,
            0.0,
        ],
        0,
        0,
        0.0,
    )
}

fn do_gripper_from_wire(
    _frame: MissionFrame,
    params: [f32; 4],
    _x: i32,
    _y: i32,
    _z: f32,
) -> DoGripper {
    DoGripper {
        number: u8_from_param(params[0]),
        action: gripper_action_from_param(params[1]),
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct DoSprayer {
    pub enabled: bool,
}

fn do_sprayer_to_wire(command: DoSprayer) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire([bool_to_param(command.enabled), 0.0, 0.0, 0.0], 0, 0, 0.0)
}

fn do_sprayer_from_wire(
    _frame: MissionFrame,
    params: [f32; 4],
    _x: i32,
    _y: i32,
    _z: f32,
) -> DoSprayer {
    DoSprayer {
        enabled: bool_from_param(params[0]),
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct DoWinch {
    pub number: u8,
    pub action: WinchAction,
    pub release_length_m: f32,
    pub release_rate_mps: f32,
}

fn do_winch_to_wire(command: DoWinch) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire(
        [
            f32::from(command.number),
            winch_action_to_param(command.action),
            command.release_length_m,
            command.release_rate_mps,
        ],
        0,
        0,
        0.0,
    )
}

fn do_winch_from_wire(
    _frame: MissionFrame,
    params: [f32; 4],
    _x: i32,
    _y: i32,
    _z: f32,
) -> DoWinch {
    DoWinch {
        number: u8_from_param(params[0]),
        action: winch_action_from_param(params[1]),
        release_length_m: params[2],
        release_rate_mps: params[3],
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct DoEngineControl {
    pub start: bool,
    pub cold_start: bool,
    pub height_delay_m: f32,
    pub allow_disarmed: bool,
}

fn do_engine_control_to_wire(command: DoEngineControl) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire(
        [
            bool_to_param(command.start),
            bool_to_param(command.cold_start),
            command.height_delay_m,
            engine_allow_disarmed_to_param(command.allow_disarmed),
        ],
        0,
        0,
        0.0,
    )
}

fn do_engine_control_from_wire(
    _frame: MissionFrame,
    params: [f32; 4],
    _x: i32,
    _y: i32,
    _z: f32,
) -> DoEngineControl {
    DoEngineControl {
        start: bool_from_param(params[0]),
        cold_start: bool_from_param(params[1]),
        height_delay_m: params[2],
        allow_disarmed: engine_allow_disarmed_from_param(params[3]),
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct DoInvertedFlight {
    pub inverted: bool,
}

fn do_inverted_flight_to_wire(
    command: DoInvertedFlight,
) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire([bool_to_param(command.inverted), 0.0, 0.0, 0.0], 0, 0, 0.0)
}

fn do_inverted_flight_from_wire(
    _frame: MissionFrame,
    params: [f32; 4],
    _x: i32,
    _y: i32,
    _z: f32,
) -> DoInvertedFlight {
    DoInvertedFlight {
        inverted: bool_from_param(params[0]),
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct DoAutotuneEnable {
    pub enabled: bool,
}

fn do_autotune_enable_to_wire(
    command: DoAutotuneEnable,
) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire([bool_to_param(command.enabled), 0.0, 0.0, 0.0], 0, 0, 0.0)
}

fn do_autotune_enable_from_wire(
    _frame: MissionFrame,
    params: [f32; 4],
    _x: i32,
    _y: i32,
    _z: f32,
) -> DoAutotuneEnable {
    DoAutotuneEnable {
        enabled: bool_from_param(params[0]),
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct DoVtolTransition {
    pub target_state: u8,
}

fn do_vtol_transition_to_wire(
    command: DoVtolTransition,
) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire([f32::from(command.target_state), 0.0, 0.0, 0.0], 0, 0, 0.0)
}

fn do_vtol_transition_from_wire(
    _frame: MissionFrame,
    params: [f32; 4],
    _x: i32,
    _y: i32,
    _z: f32,
) -> DoVtolTransition {
    DoVtolTransition {
        target_state: u8_from_param(params[0]),
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct DoGuidedLimits {
    pub max_time_s: f32,
    pub min_alt_m: f32,
    pub max_alt_m: f32,
    pub max_horiz_m: f32,
}

fn do_guided_limits_to_wire(command: DoGuidedLimits) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire(
        [
            command.max_time_s,
            command.min_alt_m,
            command.max_alt_m,
            command.max_horiz_m,
        ],
        0,
        0,
        0.0,
    )
}

fn do_guided_limits_from_wire(
    _frame: MissionFrame,
    params: [f32; 4],
    _x: i32,
    _y: i32,
    _z: f32,
) -> DoGuidedLimits {
    DoGuidedLimits {
        max_time_s: params[0],
        min_alt_m: params[1],
        max_alt_m: params[2],
        max_horiz_m: params[3],
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct DoSetResumeRepeatDist {
    pub distance_m: f32,
}

fn do_set_resume_repeat_dist_to_wire(
    command: DoSetResumeRepeatDist,
) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire([command.distance_m, 0.0, 0.0, 0.0], 0, 0, 0.0)
}

fn do_set_resume_repeat_dist_from_wire(
    _frame: MissionFrame,
    params: [f32; 4],
    _x: i32,
    _y: i32,
    _z: f32,
) -> DoSetResumeRepeatDist {
    DoSetResumeRepeatDist {
        distance_m: params[0],
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct DoAuxFunction {
    pub function: u16,
    pub switch_pos: u8,
}

fn do_aux_function_to_wire(command: DoAuxFunction) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire(
        [
            f32::from(command.function),
            f32::from(command.switch_pos),
            0.0,
            0.0,
        ],
        0,
        0,
        0.0,
    )
}

fn do_aux_function_from_wire(
    _frame: MissionFrame,
    params: [f32; 4],
    _x: i32,
    _y: i32,
    _z: f32,
) -> DoAuxFunction {
    DoAuxFunction {
        function: u16_from_param(params[0]),
        switch_pos: u8_from_param(params[1]),
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct DoSendScriptMessage {
    pub id: u16,
    pub p1: f32,
    pub p2: f32,
    pub p3: f32,
}

fn do_send_script_message_to_wire(
    command: DoSendScriptMessage,
) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire(
        [f32::from(command.id), command.p1, command.p2, command.p3],
        0,
        0,
        0.0,
    )
}

fn do_send_script_message_from_wire(
    _frame: MissionFrame,
    params: [f32; 4],
    _x: i32,
    _y: i32,
    _z: f32,
) -> DoSendScriptMessage {
    DoSendScriptMessage {
        id: u16_from_param(params[0]),
        p1: params[1],
        p2: params[2],
        p3: params[3],
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct CondDelay {
    pub delay_s: f32,
}

fn condition_delay_to_wire(command: CondDelay) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire([command.delay_s, 0.0, 0.0, 0.0], 0, 0, 0.0)
}

fn condition_delay_from_wire(
    _frame: MissionFrame,
    params: [f32; 4],
    _x: i32,
    _y: i32,
    _z: f32,
) -> CondDelay {
    CondDelay { delay_s: params[0] }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct CondDistance {
    pub distance_m: f32,
}

fn condition_distance_to_wire(command: CondDistance) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire([command.distance_m, 0.0, 0.0, 0.0], 0, 0, 0.0)
}

fn condition_distance_from_wire(
    _frame: MissionFrame,
    params: [f32; 4],
    _x: i32,
    _y: i32,
    _z: f32,
) -> CondDistance {
    CondDistance {
        distance_m: params[0],
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
/// Typed mission command API item used by plan serialization and validation.
pub struct CondYaw {
    pub angle_deg: f32,
    pub turn_rate_dps: f32,
    pub direction: YawDirection,
    pub relative: bool,
}

fn condition_yaw_to_wire(command: CondYaw) -> (MissionFrame, [f32; 4], i32, i32, f32) {
    mission_command_to_wire(
        [
            command.angle_deg,
            command.turn_rate_dps,
            yaw_direction_to_param(command.direction),
            bool_to_param(command.relative),
        ],
        0,
        0,
        0.0,
    )
}

fn condition_yaw_from_wire(
    _frame: MissionFrame,
    params: [f32; 4],
    _x: i32,
    _y: i32,
    _z: f32,
) -> CondYaw {
    CondYaw {
        angle_deg: params[0],
        turn_rate_dps: params[1],
        direction: yaw_direction_from_param(params[2]),
        relative: bool_from_param(params[3]),
    }
}

mission_commands! {
    Nav {
        Data {
            Waypoint(NavWaypoint) {
                command: MAV_CMD_NAV_WAYPOINT,
                to_wire: nav_waypoint_to_wire,
                from_wire: nav_waypoint_from_wire,
            },
            SplineWaypoint(NavSplineWaypoint) {
                command: MAV_CMD_NAV_SPLINE_WAYPOINT,
                to_wire: nav_spline_waypoint_to_wire,
                from_wire: nav_spline_waypoint_from_wire,
            },
            ArcWaypoint(NavArcWaypoint) {
                command: MAV_CMD_NAV_ARC_WAYPOINT,
                to_wire: nav_arc_waypoint_to_wire,
                from_wire: nav_arc_waypoint_from_wire,
            },
            Takeoff(NavTakeoff) {
                command: MAV_CMD_NAV_TAKEOFF,
                to_wire: nav_takeoff_to_wire,
                from_wire: nav_takeoff_from_wire,
            },
            Land(NavLand) {
                command: MAV_CMD_NAV_LAND,
                to_wire: nav_land_to_wire,
                from_wire: nav_land_from_wire,
            },
            LoiterUnlimited(NavLoiterUnlimited) {
                command: MAV_CMD_NAV_LOITER_UNLIMITED,
                to_wire: nav_loiter_unlimited_to_wire,
                from_wire: nav_loiter_unlimited_from_wire,
            },
            LoiterTurns(NavLoiterTurns) {
                command: MAV_CMD_NAV_LOITER_TURNS,
                to_wire: nav_loiter_turns_to_wire,
                from_wire: nav_loiter_turns_from_wire,
            },
            LoiterTime(NavLoiterTime) {
                command: MAV_CMD_NAV_LOITER_TIME,
                to_wire: nav_loiter_time_to_wire,
                from_wire: nav_loiter_time_from_wire,
            },
            LoiterToAlt(NavLoiterToAlt) {
                command: MAV_CMD_NAV_LOITER_TO_ALT,
                to_wire: nav_loiter_to_alt_to_wire,
                from_wire: nav_loiter_to_alt_from_wire,
            },
            ContinueAndChangeAlt(NavContinueAndChangeAlt) {
                command: MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT,
                to_wire: nav_continue_and_change_alt_to_wire,
                from_wire: nav_continue_and_change_alt_from_wire,
            },
            Delay(NavDelay) {
                command: MAV_CMD_NAV_DELAY,
                to_wire: nav_delay_to_wire,
                from_wire: nav_delay_from_wire,
            },
            GuidedEnable(NavGuidedEnable) {
                command: MAV_CMD_NAV_GUIDED_ENABLE,
                to_wire: nav_guided_enable_to_wire,
                from_wire: nav_guided_enable_from_wire,
            },
            AltitudeWait(NavAltitudeWait) {
                command: MAV_CMD_NAV_ALTITUDE_WAIT,
                to_wire: nav_altitude_wait_to_wire,
                from_wire: nav_altitude_wait_from_wire,
            },
            VtolTakeoff(NavVtolTakeoff) {
                command: MAV_CMD_NAV_VTOL_TAKEOFF,
                to_wire: nav_vtol_takeoff_to_wire,
                from_wire: nav_vtol_takeoff_from_wire,
            },
            VtolLand(NavVtolLand) {
                command: MAV_CMD_NAV_VTOL_LAND,
                to_wire: nav_vtol_land_to_wire,
                from_wire: nav_vtol_land_from_wire,
            },
            PayloadPlace(NavPayloadPlace) {
                command: MAV_CMD_NAV_PAYLOAD_PLACE,
                to_wire: nav_payload_place_to_wire,
                from_wire: nav_payload_place_from_wire,
            },
            SetYawSpeed(NavSetYawSpeed) {
                command: MAV_CMD_NAV_SET_YAW_SPEED,
                to_wire: nav_set_yaw_speed_to_wire,
                from_wire: nav_set_yaw_speed_from_wire,
            },
            ScriptTime(NavScriptTime) {
                command: MAV_CMD_NAV_SCRIPT_TIME,
                to_wire: nav_script_time_to_wire,
                from_wire: nav_script_time_from_wire,
            },
            AttitudeTime(NavAttitudeTime) {
                command: MAV_CMD_NAV_ATTITUDE_TIME,
                to_wire: nav_attitude_time_to_wire,
                from_wire: nav_attitude_time_from_wire,
            }
        }
        Unit {
            ReturnToLaunch {
                command: MAV_CMD_NAV_RETURN_TO_LAUNCH,
                to_wire: return_to_launch_to_wire,
                from_wire: return_to_launch_from_wire,
            }
        }
    }
    Do {
        Data {
            Jump(DoJump) {
                command: MAV_CMD_DO_JUMP,
                to_wire: do_jump_to_wire,
                from_wire: do_jump_from_wire,
            },
            JumpTag(DoJumpTag) {
                command: MAV_CMD_DO_JUMP_TAG,
                to_wire: do_jump_tag_to_wire,
                from_wire: do_jump_tag_from_wire,
            },
            Tag(DoTag) {
                command: MAV_CMD_JUMP_TAG,
                to_wire: do_tag_to_wire,
                from_wire: do_tag_from_wire,
            },
            PauseContinue(DoPauseContinue) {
                command: MAV_CMD_DO_PAUSE_CONTINUE,
                to_wire: do_pause_continue_to_wire,
                from_wire: do_pause_continue_from_wire,
            },
            ChangeSpeed(DoChangeSpeed) {
                command: MAV_CMD_DO_CHANGE_SPEED,
                to_wire: do_change_speed_to_wire,
                from_wire: do_change_speed_from_wire,
            },
            SetReverse(DoSetReverse) {
                command: MAV_CMD_DO_SET_REVERSE,
                to_wire: do_set_reverse_to_wire,
                from_wire: do_set_reverse_from_wire,
            },
            SetHome(DoSetHome) {
                command: MAV_CMD_DO_SET_HOME,
                to_wire: do_set_home_to_wire,
                from_wire: do_set_home_from_wire,
            },
            LandStart(DoLandStart) {
                command: MAV_CMD_DO_LAND_START,
                to_wire: do_land_start_to_wire,
                from_wire: do_land_start_from_wire,
            },
            ReturnPathStart(DoReturnPathStart) {
                command: MAV_CMD_DO_RETURN_PATH_START,
                to_wire: do_return_path_start_to_wire,
                from_wire: do_return_path_start_from_wire,
            },
            GoAround(DoGoAround) {
                command: MAV_CMD_DO_GO_AROUND,
                to_wire: do_go_around_to_wire,
                from_wire: do_go_around_from_wire,
            },
            SetRoiLocation(DoSetRoiLocation) {
                command: MAV_CMD_DO_SET_ROI_LOCATION,
                to_wire: do_set_roi_location_to_wire,
                from_wire: do_set_roi_location_from_wire,
            },
            SetRoi(DoSetRoi) {
                command: MAV_CMD_DO_SET_ROI,
                to_wire: do_set_roi_to_wire,
                from_wire: do_set_roi_from_wire,
            },
            MountControl(DoMountControl) {
                command: MAV_CMD_DO_MOUNT_CONTROL,
                to_wire: do_mount_control_to_wire,
                from_wire: do_mount_control_from_wire,
            },
            GimbalManagerPitchYaw(DoGimbalManagerPitchYaw) {
                command: MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
                to_wire: do_gimbal_manager_pitch_yaw_to_wire,
                from_wire: do_gimbal_manager_pitch_yaw_from_wire,
            },
            CamTriggerDistance(DoCamTriggerDistance) {
                command: MAV_CMD_DO_SET_CAM_TRIGG_DIST,
                to_wire: do_cam_trigger_distance_to_wire,
                from_wire: do_cam_trigger_distance_from_wire,
            },
            ImageStartCapture(DoImageStartCapture) {
                command: MAV_CMD_IMAGE_START_CAPTURE,
                to_wire: do_image_start_capture_to_wire,
                from_wire: do_image_start_capture_from_wire,
            },
            ImageStopCapture(DoImageStopCapture) {
                command: MAV_CMD_IMAGE_STOP_CAPTURE,
                to_wire: do_image_stop_capture_to_wire,
                from_wire: do_image_stop_capture_from_wire,
            },
            VideoStartCapture(DoVideoStartCapture) {
                command: MAV_CMD_VIDEO_START_CAPTURE,
                to_wire: do_video_start_capture_to_wire,
                from_wire: do_video_start_capture_from_wire,
            },
            VideoStopCapture(DoVideoStopCapture) {
                command: MAV_CMD_VIDEO_STOP_CAPTURE,
                to_wire: do_video_stop_capture_to_wire,
                from_wire: do_video_stop_capture_from_wire,
            },
            SetCameraZoom(DoSetCameraZoom) {
                command: MAV_CMD_SET_CAMERA_ZOOM,
                to_wire: do_set_camera_zoom_to_wire,
                from_wire: do_set_camera_zoom_from_wire,
            },
            SetCameraFocus(DoSetCameraFocus) {
                command: MAV_CMD_SET_CAMERA_FOCUS,
                to_wire: do_set_camera_focus_to_wire,
                from_wire: do_set_camera_focus_from_wire,
            },
            SetCameraSource(DoSetCameraSource) {
                command: MAV_CMD_SET_CAMERA_SOURCE,
                to_wire: do_set_camera_source_to_wire,
                from_wire: do_set_camera_source_from_wire,
            },
            DigicamConfigure(DoDigicamConfigure) {
                command: MAV_CMD_DO_DIGICAM_CONFIGURE,
                to_wire: do_digicam_configure_to_wire,
                from_wire: do_digicam_configure_from_wire,
            },
            DigicamControl(DoDigicamControl) {
                command: MAV_CMD_DO_DIGICAM_CONTROL,
                to_wire: do_digicam_control_to_wire,
                from_wire: do_digicam_control_from_wire,
            },
            SetServo(DoSetServo) {
                command: MAV_CMD_DO_SET_SERVO,
                to_wire: do_set_servo_to_wire,
                from_wire: do_set_servo_from_wire,
            },
            SetRelay(DoSetRelay) {
                command: MAV_CMD_DO_SET_RELAY,
                to_wire: do_set_relay_to_wire,
                from_wire: do_set_relay_from_wire,
            },
            RepeatServo(DoRepeatServo) {
                command: MAV_CMD_DO_REPEAT_SERVO,
                to_wire: do_repeat_servo_to_wire,
                from_wire: do_repeat_servo_from_wire,
            },
            RepeatRelay(DoRepeatRelay) {
                command: MAV_CMD_DO_REPEAT_RELAY,
                to_wire: do_repeat_relay_to_wire,
                from_wire: do_repeat_relay_from_wire,
            },
            FenceEnable(DoFenceEnable) {
                command: MAV_CMD_DO_FENCE_ENABLE,
                to_wire: do_fence_enable_to_wire,
                from_wire: do_fence_enable_from_wire,
            },
            Parachute(DoParachute) {
                command: MAV_CMD_DO_PARACHUTE,
                to_wire: do_parachute_to_wire,
                from_wire: do_parachute_from_wire,
            },
            Gripper(DoGripper) {
                command: MAV_CMD_DO_GRIPPER,
                to_wire: do_gripper_to_wire,
                from_wire: do_gripper_from_wire,
            },
            Sprayer(DoSprayer) {
                command: MAV_CMD_DO_SPRAYER,
                to_wire: do_sprayer_to_wire,
                from_wire: do_sprayer_from_wire,
            },
            Winch(DoWinch) {
                command: MAV_CMD_DO_WINCH,
                to_wire: do_winch_to_wire,
                from_wire: do_winch_from_wire,
            },
            EngineControl(DoEngineControl) {
                command: MAV_CMD_DO_ENGINE_CONTROL,
                to_wire: do_engine_control_to_wire,
                from_wire: do_engine_control_from_wire,
            },
            InvertedFlight(DoInvertedFlight) {
                command: MAV_CMD_DO_INVERTED_FLIGHT,
                to_wire: do_inverted_flight_to_wire,
                from_wire: do_inverted_flight_from_wire,
            },
            AutotuneEnable(DoAutotuneEnable) {
                command: MAV_CMD_DO_AUTOTUNE_ENABLE,
                to_wire: do_autotune_enable_to_wire,
                from_wire: do_autotune_enable_from_wire,
            },
            VtolTransition(DoVtolTransition) {
                command: MAV_CMD_DO_VTOL_TRANSITION,
                to_wire: do_vtol_transition_to_wire,
                from_wire: do_vtol_transition_from_wire,
            },
            GuidedLimits(DoGuidedLimits) {
                command: MAV_CMD_DO_GUIDED_LIMITS,
                to_wire: do_guided_limits_to_wire,
                from_wire: do_guided_limits_from_wire,
            },
            SetResumeRepeatDist(DoSetResumeRepeatDist) {
                command: MAV_CMD_DO_SET_RESUME_REPEAT_DIST,
                to_wire: do_set_resume_repeat_dist_to_wire,
                from_wire: do_set_resume_repeat_dist_from_wire,
            },
            AuxFunction(DoAuxFunction) {
                command: MAV_CMD_DO_AUX_FUNCTION,
                to_wire: do_aux_function_to_wire,
                from_wire: do_aux_function_from_wire,
            },
            SendScriptMessage(DoSendScriptMessage) {
                command: MAV_CMD_DO_SEND_SCRIPT_MESSAGE,
                to_wire: do_send_script_message_to_wire,
                from_wire: do_send_script_message_from_wire,
            }
        }
        Unit {
            SetRoiNone {
                command: MAV_CMD_DO_SET_ROI_NONE,
                to_wire: do_set_roi_none_to_wire,
                from_wire: do_set_roi_none_from_wire,
            }
        }
    }
    Condition {
        Delay(CondDelay) {
            command: MAV_CMD_CONDITION_DELAY,
            to_wire: condition_delay_to_wire,
            from_wire: condition_delay_from_wire,
        },
        Distance(CondDistance) {
            command: MAV_CMD_CONDITION_DISTANCE,
            to_wire: condition_distance_to_wire,
            from_wire: condition_distance_from_wire,
        },
        Yaw(CondYaw) {
            command: MAV_CMD_CONDITION_YAW,
            to_wire: condition_yaw_to_wire,
            from_wire: condition_yaw_from_wire,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geo::{GeoPoint3d, GeoPoint3dMsl, GeoPoint3dRelHome, GeoPoint3dTerrain};
    use crate::mission::types::MissionFrame as MissionItemFrame;
    use serde::ser::{Error as _, Impossible, Serializer};
    use std::fmt;

    #[derive(Debug)]
    struct TestSerError(String);

    impl serde::ser::Error for TestSerError {
        fn custom<T: fmt::Display>(msg: T) -> Self {
            Self(msg.to_string())
        }
    }

    impl fmt::Display for TestSerError {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            f.write_str(&self.0)
        }
    }

    impl std::error::Error for TestSerError {}

    struct UnitVariantNameSerializer;

    impl Serializer for UnitVariantNameSerializer {
        type Ok = String;
        type Error = TestSerError;
        type SerializeSeq = Impossible<String, TestSerError>;
        type SerializeTuple = Impossible<String, TestSerError>;
        type SerializeTupleStruct = Impossible<String, TestSerError>;
        type SerializeTupleVariant = Impossible<String, TestSerError>;
        type SerializeMap = Impossible<String, TestSerError>;
        type SerializeStruct = Impossible<String, TestSerError>;
        type SerializeStructVariant = Impossible<String, TestSerError>;

        fn serialize_unit_variant(
            self,
            _name: &'static str,
            _variant_index: u32,
            variant: &'static str,
        ) -> Result<Self::Ok, Self::Error> {
            Ok(variant.to_owned())
        }

        fn serialize_bool(self, _v: bool) -> Result<Self::Ok, Self::Error> {
            Err(TestSerError::custom("unsupported"))
        }
        fn serialize_i8(self, _v: i8) -> Result<Self::Ok, Self::Error> {
            Err(TestSerError::custom("unsupported"))
        }
        fn serialize_i16(self, _v: i16) -> Result<Self::Ok, Self::Error> {
            Err(TestSerError::custom("unsupported"))
        }
        fn serialize_i32(self, _v: i32) -> Result<Self::Ok, Self::Error> {
            Err(TestSerError::custom("unsupported"))
        }
        fn serialize_i64(self, _v: i64) -> Result<Self::Ok, Self::Error> {
            Err(TestSerError::custom("unsupported"))
        }
        fn serialize_u8(self, _v: u8) -> Result<Self::Ok, Self::Error> {
            Err(TestSerError::custom("unsupported"))
        }
        fn serialize_u16(self, _v: u16) -> Result<Self::Ok, Self::Error> {
            Err(TestSerError::custom("unsupported"))
        }
        fn serialize_u32(self, _v: u32) -> Result<Self::Ok, Self::Error> {
            Err(TestSerError::custom("unsupported"))
        }
        fn serialize_u64(self, _v: u64) -> Result<Self::Ok, Self::Error> {
            Err(TestSerError::custom("unsupported"))
        }
        fn serialize_f32(self, _v: f32) -> Result<Self::Ok, Self::Error> {
            Err(TestSerError::custom("unsupported"))
        }
        fn serialize_f64(self, _v: f64) -> Result<Self::Ok, Self::Error> {
            Err(TestSerError::custom("unsupported"))
        }
        fn serialize_char(self, _v: char) -> Result<Self::Ok, Self::Error> {
            Err(TestSerError::custom("unsupported"))
        }
        fn serialize_str(self, _v: &str) -> Result<Self::Ok, Self::Error> {
            Err(TestSerError::custom("unsupported"))
        }
        fn serialize_bytes(self, _v: &[u8]) -> Result<Self::Ok, Self::Error> {
            Err(TestSerError::custom("unsupported"))
        }
        fn serialize_none(self) -> Result<Self::Ok, Self::Error> {
            Err(TestSerError::custom("unsupported"))
        }
        fn serialize_some<T: ?Sized + serde::Serialize>(
            self,
            _value: &T,
        ) -> Result<Self::Ok, Self::Error> {
            Err(TestSerError::custom("unsupported"))
        }
        fn serialize_unit(self) -> Result<Self::Ok, Self::Error> {
            Err(TestSerError::custom("unsupported"))
        }
        fn serialize_unit_struct(self, _name: &'static str) -> Result<Self::Ok, Self::Error> {
            Err(TestSerError::custom("unsupported"))
        }
        fn serialize_newtype_struct<T: ?Sized + serde::Serialize>(
            self,
            _name: &'static str,
            _value: &T,
        ) -> Result<Self::Ok, Self::Error> {
            Err(TestSerError::custom("unsupported"))
        }
        fn serialize_newtype_variant<T: ?Sized + serde::Serialize>(
            self,
            _name: &'static str,
            _variant_index: u32,
            _variant: &'static str,
            _value: &T,
        ) -> Result<Self::Ok, Self::Error> {
            Err(TestSerError::custom("unsupported"))
        }
        fn serialize_seq(self, _len: Option<usize>) -> Result<Self::SerializeSeq, Self::Error> {
            Err(TestSerError::custom("unsupported"))
        }
        fn serialize_tuple(self, _len: usize) -> Result<Self::SerializeTuple, Self::Error> {
            Err(TestSerError::custom("unsupported"))
        }
        fn serialize_tuple_struct(
            self,
            _name: &'static str,
            _len: usize,
        ) -> Result<Self::SerializeTupleStruct, Self::Error> {
            Err(TestSerError::custom("unsupported"))
        }
        fn serialize_tuple_variant(
            self,
            _name: &'static str,
            _variant_index: u32,
            _variant: &'static str,
            _len: usize,
        ) -> Result<Self::SerializeTupleVariant, Self::Error> {
            Err(TestSerError::custom("unsupported"))
        }
        fn serialize_map(self, _len: Option<usize>) -> Result<Self::SerializeMap, Self::Error> {
            Err(TestSerError::custom("unsupported"))
        }
        fn serialize_struct(
            self,
            _name: &'static str,
            _len: usize,
        ) -> Result<Self::SerializeStruct, Self::Error> {
            Err(TestSerError::custom("unsupported"))
        }
        fn serialize_struct_variant(
            self,
            _name: &'static str,
            _variant_index: u32,
            _variant: &'static str,
            _len: usize,
        ) -> Result<Self::SerializeStructVariant, Self::Error> {
            Err(TestSerError::custom("unsupported"))
        }
    }

    fn assert_unit_enum_serde<T>(value: T, wire_name: &str)
    where
        T: serde::Serialize + for<'de> serde::Deserialize<'de> + PartialEq + fmt::Debug,
    {
        let serialized = value
            .serialize(UnitVariantNameSerializer)
            .expect("serialize unit enum");
        assert_eq!(serialized, wire_name);

        let decoded: T = serde::Deserialize::deserialize(serde::de::value::StrDeserializer::<
            serde::de::value::Error,
        >::new(wire_name))
        .expect("deserialize unit enum");
        assert_eq!(decoded, value);
    }

    fn geo_msl(latitude_e7: i32, longitude_e7: i32, altitude_msl_m: f64) -> GeoPoint3d {
        GeoPoint3d::Msl(GeoPoint3dMsl {
            latitude_deg: f64::from(latitude_e7) / 1e7,
            longitude_deg: f64::from(longitude_e7) / 1e7,
            altitude_msl_m,
        })
    }

    fn geo_rel_home(latitude_e7: i32, longitude_e7: i32, relative_alt_m: f64) -> GeoPoint3d {
        GeoPoint3d::RelHome(GeoPoint3dRelHome {
            latitude_deg: f64::from(latitude_e7) / 1e7,
            longitude_deg: f64::from(longitude_e7) / 1e7,
            relative_alt_m,
        })
    }

    fn geo_terrain(latitude_e7: i32, longitude_e7: i32, altitude_terrain_m: f64) -> GeoPoint3d {
        GeoPoint3d::Terrain(GeoPoint3dTerrain {
            latitude_deg: f64::from(latitude_e7) / 1e7,
            longitude_deg: f64::from(longitude_e7) / 1e7,
            altitude_terrain_m,
        })
    }

    fn assert_roundtrip(
        original: MissionCommand,
        expected_wire: (u16, MissionFrame, [f32; 4], i32, i32, f32),
    ) {
        let encoded = original.clone().into_wire();
        assert_eq!(encoded, expected_wire);

        let decoded = MissionCommand::from_wire(
            expected_wire.0,
            expected_wire.1,
            expected_wire.2,
            expected_wire.3,
            expected_wire.4,
            expected_wire.5,
        );
        assert_eq!(decoded, original);
    }

    #[test]
    fn from_chain() {
        let waypoint = NavWaypoint {
            position: geo_rel_home(473_977_420, 85_455_970, 42.0),
            hold_time_s: 5.0,
            acceptance_radius_m: 2.0,
            pass_radius_m: 0.0,
            yaw_deg: 180.0,
        };

        let nav_command: NavCommand = waypoint.clone().into();
        assert_eq!(nav_command, NavCommand::Waypoint(waypoint.clone()));

        let mission_command_from_sub_enum: MissionCommand = nav_command.into();
        assert_eq!(
            mission_command_from_sub_enum,
            MissionCommand::Nav(NavCommand::Waypoint(waypoint.clone()))
        );

        let mission_command_from_struct: MissionCommand = waypoint.clone().into();
        assert_eq!(mission_command_from_struct, mission_command_from_sub_enum);

        let mission_item: MissionItem = waypoint.into();
        let (command, frame, params, x, y, z) = mission_item.command.clone().into_wire();
        assert_eq!(command, u16::from(MAV_CMD_NAV_WAYPOINT));
        assert_eq!(
            MissionItemFrame::from(frame),
            MissionItemFrame::GlobalRelativeAltInt
        );
        assert!(!mission_item.current);
        assert!(mission_item.autocontinue);
        assert_eq!(params[0], 5.0);
        assert_eq!(params[1], 2.0);
        assert_eq!(params[2], 0.0);
        assert_eq!(params[3], 180.0);
        assert_eq!(x, 473_977_420);
        assert_eq!(y, 85_455_970);
        assert_eq!(z, 42.0);
    }

    #[test]
    fn nav_waypoint_roundtrip() {
        let original = MissionCommand::from(NavWaypoint {
            position: geo_rel_home(473_977_420, 85_455_970, 42.0),
            hold_time_s: 5.0,
            acceptance_radius_m: 2.0,
            pass_radius_m: 0.5,
            yaw_deg: 180.0,
        });

        assert_roundtrip(
            original,
            (
                u16::from(MAV_CMD_NAV_WAYPOINT),
                MissionFrame::GlobalRelativeAlt,
                [5.0, 2.0, 0.5, 180.0],
                473_977_420,
                85_455_970,
                42.0,
            ),
        );
    }

    #[test]
    fn all_nav_roundtrip() {
        let cases = [
            (
                MissionCommand::from(NavWaypoint {
                    position: geo_rel_home(473_977_420, 85_455_970, 42.0),
                    hold_time_s: 5.0,
                    acceptance_radius_m: 2.0,
                    pass_radius_m: 0.5,
                    yaw_deg: 180.0,
                }),
                (
                    u16::from(MAV_CMD_NAV_WAYPOINT),
                    MissionFrame::GlobalRelativeAlt,
                    [5.0, 2.0, 0.5, 180.0],
                    473_977_420,
                    85_455_970,
                    42.0,
                ),
            ),
            (
                MissionCommand::from(NavSplineWaypoint {
                    position: geo_msl(473_977_421, 85_455_971, 120.0),
                    hold_time_s: 7.0,
                }),
                (
                    u16::from(MAV_CMD_NAV_SPLINE_WAYPOINT),
                    MissionFrame::Global,
                    [7.0, 0.0, 0.0, 0.0],
                    473_977_421,
                    85_455_971,
                    120.0,
                ),
            ),
            (
                MissionCommand::from(NavArcWaypoint {
                    position: geo_terrain(473_977_422, 85_455_972, 18.0),
                    arc_angle_deg: 90.0,
                    direction: LoiterDirection::CounterClockwise,
                }),
                (
                    u16::from(MAV_CMD_NAV_ARC_WAYPOINT),
                    MissionFrame::GlobalTerrainAlt,
                    [-90.0, 0.0, 0.0, 0.0],
                    473_977_422,
                    85_455_972,
                    18.0,
                ),
            ),
            (
                MissionCommand::from(NavTakeoff {
                    position: geo_terrain(473_977_423, 85_455_973, 60.0),
                    pitch_deg: 12.0,
                }),
                (
                    u16::from(MAV_CMD_NAV_TAKEOFF),
                    MissionFrame::GlobalTerrainAlt,
                    [12.0, 0.0, 0.0, 0.0],
                    473_977_423,
                    85_455_973,
                    60.0,
                ),
            ),
            (
                MissionCommand::from(NavLand {
                    position: geo_msl(473_977_424, 85_455_974, 10.0),
                    abort_alt_m: 25.0,
                }),
                (
                    u16::from(MAV_CMD_NAV_LAND),
                    MissionFrame::Global,
                    [25.0, 0.0, 0.0, 0.0],
                    473_977_424,
                    85_455_974,
                    10.0,
                ),
            ),
            (
                MissionCommand::from(NavCommand::ReturnToLaunch),
                (
                    u16::from(MAV_CMD_NAV_RETURN_TO_LAUNCH),
                    MissionFrame::Mission,
                    [0.0, 0.0, 0.0, 0.0],
                    0,
                    0,
                    0.0,
                ),
            ),
            (
                MissionCommand::from(NavLoiterUnlimited {
                    position: geo_rel_home(473_977_425, 85_455_975, 50.0),
                    radius_m: 35.0,
                    direction: LoiterDirection::CounterClockwise,
                }),
                (
                    u16::from(MAV_CMD_NAV_LOITER_UNLIMITED),
                    MissionFrame::GlobalRelativeAlt,
                    [0.0, 0.0, -35.0, 0.0],
                    473_977_425,
                    85_455_975,
                    50.0,
                ),
            ),
            (
                MissionCommand::from(NavLoiterTurns {
                    position: geo_rel_home(473_977_426, 85_455_976, 55.0),
                    turns: 2.5,
                    radius_m: 20.0,
                    direction: LoiterDirection::Clockwise,
                    exit_xtrack: true,
                }),
                (
                    u16::from(MAV_CMD_NAV_LOITER_TURNS),
                    MissionFrame::GlobalRelativeAlt,
                    [2.5, 0.0, 20.0, 1.0],
                    473_977_426,
                    85_455_976,
                    55.0,
                ),
            ),
            (
                MissionCommand::from(NavLoiterTime {
                    position: geo_msl(473_977_427, 85_455_977, 65.0),
                    time_s: 15.0,
                    direction: LoiterDirection::Clockwise,
                    exit_xtrack: false,
                }),
                (
                    u16::from(MAV_CMD_NAV_LOITER_TIME),
                    MissionFrame::Global,
                    [15.0, 0.0, 0.0, 0.0],
                    473_977_427,
                    85_455_977,
                    65.0,
                ),
            ),
            (
                MissionCommand::from(NavLoiterToAlt {
                    position: geo_terrain(473_977_428, 85_455_978, 75.0),
                    radius_m: 40.0,
                    direction: LoiterDirection::CounterClockwise,
                    exit_xtrack: true,
                }),
                (
                    u16::from(MAV_CMD_NAV_LOITER_TO_ALT),
                    MissionFrame::GlobalTerrainAlt,
                    [0.0, -40.0, 0.0, 1.0],
                    473_977_428,
                    85_455_978,
                    75.0,
                ),
            ),
            (
                MissionCommand::from(NavContinueAndChangeAlt {
                    position: geo_terrain(473_977_429, 85_455_979, 80.0),
                    action: AltChangeAction::Descend,
                }),
                (
                    u16::from(MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT),
                    MissionFrame::GlobalTerrainAlt,
                    [2.0, 0.0, 0.0, 0.0],
                    473_977_429,
                    85_455_979,
                    80.0,
                ),
            ),
            (
                MissionCommand::from(NavDelay {
                    seconds: -1.0,
                    hour_utc: 6.0,
                    min_utc: 7.0,
                    sec_utc: 8.0,
                }),
                (
                    u16::from(MAV_CMD_NAV_DELAY),
                    MissionFrame::Mission,
                    [-1.0, 6.0, 7.0, 8.0],
                    0,
                    0,
                    0.0,
                ),
            ),
            (
                MissionCommand::from(NavGuidedEnable { enabled: true }),
                (
                    u16::from(MAV_CMD_NAV_GUIDED_ENABLE),
                    MissionFrame::Mission,
                    [1.0, 0.0, 0.0, 0.0],
                    0,
                    0,
                    0.0,
                ),
            ),
            (
                MissionCommand::from(NavAltitudeWait {
                    altitude_m: 1500.0,
                    descent_rate_mps: -4.0,
                    wiggle_time_s: 3.0,
                }),
                (
                    u16::from(MAV_CMD_NAV_ALTITUDE_WAIT),
                    MissionFrame::Mission,
                    [1500.0, -4.0, 3.0, 0.0],
                    0,
                    0,
                    0.0,
                ),
            ),
            (
                MissionCommand::from(NavVtolTakeoff {
                    position: geo_rel_home(473_977_430, 85_455_980, 90.0),
                }),
                (
                    u16::from(MAV_CMD_NAV_VTOL_TAKEOFF),
                    MissionFrame::GlobalRelativeAlt,
                    [0.0, 0.0, 0.0, 0.0],
                    473_977_430,
                    85_455_980,
                    90.0,
                ),
            ),
            (
                MissionCommand::from(NavVtolLand {
                    position: geo_msl(473_977_431, 85_455_981, 8.0),
                    options: 2,
                }),
                (
                    u16::from(MAV_CMD_NAV_VTOL_LAND),
                    MissionFrame::Global,
                    [2.0, 0.0, 0.0, 0.0],
                    473_977_431,
                    85_455_981,
                    8.0,
                ),
            ),
            (
                MissionCommand::from(NavPayloadPlace {
                    position: geo_terrain(473_977_432, 85_455_982, 12.0),
                    max_descent_m: 6.0,
                }),
                (
                    u16::from(MAV_CMD_NAV_PAYLOAD_PLACE),
                    MissionFrame::GlobalTerrainAlt,
                    [6.0, 0.0, 0.0, 0.0],
                    473_977_432,
                    85_455_982,
                    12.0,
                ),
            ),
            (
                MissionCommand::from(NavSetYawSpeed {
                    angle_deg: 45.0,
                    speed_mps: 3.5,
                    relative: true,
                }),
                (
                    u16::from(MAV_CMD_NAV_SET_YAW_SPEED),
                    MissionFrame::Mission,
                    [45.0, 3.5, 1.0, 0.0],
                    0,
                    0,
                    0.0,
                ),
            ),
            (
                MissionCommand::from(NavScriptTime {
                    command: 9,
                    timeout_s: 11.0,
                    arg1: 1.5,
                    arg2: -2.5,
                    arg3: 17,
                    arg4: -29,
                }),
                (
                    u16::from(MAV_CMD_NAV_SCRIPT_TIME),
                    MissionFrame::Mission,
                    [9.0, 11.0, 1.5, -2.5],
                    17,
                    -29,
                    0.0,
                ),
            ),
            (
                MissionCommand::from(NavAttitudeTime {
                    time_s: 12.0,
                    roll_deg: 15.0,
                    pitch_deg: -5.0,
                    yaw_deg: 270.0,
                    climb_rate_mps: 3.0,
                }),
                (
                    u16::from(MAV_CMD_NAV_ATTITUDE_TIME),
                    MissionFrame::Mission,
                    [12.0, 15.0, -5.0, 270.0],
                    3,
                    0,
                    0.0,
                ),
            ),
        ];

        for (original, expected_wire) in cases {
            assert_roundtrip(original, expected_wire);
        }
    }

    #[test]
    fn do_change_speed_roundtrip() {
        let original = MissionCommand::from(DoChangeSpeed {
            speed_type: SpeedType::Groundspeed,
            speed_mps: 5.5,
            throttle_pct: 66.0,
        });

        assert_roundtrip(
            original,
            (
                u16::from(MAV_CMD_DO_CHANGE_SPEED),
                MissionFrame::Mission,
                [1.0, 5.5, 66.0, 0.0],
                0,
                0,
                0.0,
            ),
        );
    }

    #[test]
    fn do_set_roi_none_roundtrip() {
        assert_roundtrip(
            MissionCommand::from(DoCommand::SetRoiNone),
            (
                u16::from(MAV_CMD_DO_SET_ROI_NONE),
                MissionFrame::Mission,
                [0.0, 0.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        );
    }

    #[test]
    fn do_set_home_roundtrip_uses_geopoint_frame() {
        let original = MissionCommand::from(DoSetHome {
            position: geo_terrain(473_977_520, 85_456_070, 21.5),
            use_current: false,
        });

        assert_roundtrip(
            original,
            (
                u16::from(MAV_CMD_DO_SET_HOME),
                MissionFrame::GlobalTerrainAlt,
                [0.0, 0.0, 0.0, 0.0],
                473_977_520,
                85_456_070,
                21.5,
            ),
        );
    }

    #[test]
    fn all_do_roundtrip() {
        let cases = [
            (
                MissionCommand::from(DoJump {
                    target_index: 12,
                    repeat_count: 3,
                }),
                (
                    u16::from(MAV_CMD_DO_JUMP),
                    MissionFrame::Mission,
                    [12.0, 3.0, 0.0, 0.0],
                    0,
                    0,
                    0.0,
                ),
            ),
            (
                MissionCommand::from(DoJumpTag {
                    tag: 44,
                    repeat_count: 5,
                }),
                (
                    u16::from(MAV_CMD_DO_JUMP_TAG),
                    MissionFrame::Mission,
                    [44.0, 5.0, 0.0, 0.0],
                    0,
                    0,
                    0.0,
                ),
            ),
            (
                MissionCommand::from(DoTag { tag: 44 }),
                (
                    u16::from(MAV_CMD_JUMP_TAG),
                    MissionFrame::Mission,
                    [44.0, 0.0, 0.0, 0.0],
                    0,
                    0,
                    0.0,
                ),
            ),
            (
                MissionCommand::from(DoPauseContinue { pause: true }),
                (
                    u16::from(MAV_CMD_DO_PAUSE_CONTINUE),
                    MissionFrame::Mission,
                    [0.0, 0.0, 0.0, 0.0],
                    0,
                    0,
                    0.0,
                ),
            ),
            (
                MissionCommand::from(DoChangeSpeed {
                    speed_type: SpeedType::Groundspeed,
                    speed_mps: 5.5,
                    throttle_pct: 66.0,
                }),
                (
                    u16::from(MAV_CMD_DO_CHANGE_SPEED),
                    MissionFrame::Mission,
                    [1.0, 5.5, 66.0, 0.0],
                    0,
                    0,
                    0.0,
                ),
            ),
            (
                MissionCommand::from(DoSetReverse { reverse: true }),
                (
                    u16::from(MAV_CMD_DO_SET_REVERSE),
                    MissionFrame::Mission,
                    [1.0, 0.0, 0.0, 0.0],
                    0,
                    0,
                    0.0,
                ),
            ),
            (
                MissionCommand::from(DoSetHome {
                    position: geo_msl(473_977_501, 85_456_001, 110.0),
                    use_current: false,
                }),
                (
                    u16::from(MAV_CMD_DO_SET_HOME),
                    MissionFrame::Global,
                    [0.0, 0.0, 0.0, 0.0],
                    473_977_501,
                    85_456_001,
                    110.0,
                ),
            ),
            (
                MissionCommand::from(DoLandStart {
                    position: geo_rel_home(473_977_502, 85_456_002, 25.0),
                }),
                (
                    u16::from(MAV_CMD_DO_LAND_START),
                    MissionFrame::GlobalRelativeAlt,
                    [0.0, 0.0, 0.0, 0.0],
                    473_977_502,
                    85_456_002,
                    25.0,
                ),
            ),
            (
                MissionCommand::from(DoReturnPathStart {
                    position: geo_terrain(473_977_503, 85_456_003, 15.0),
                }),
                (
                    u16::from(MAV_CMD_DO_RETURN_PATH_START),
                    MissionFrame::GlobalTerrainAlt,
                    [0.0, 0.0, 0.0, 0.0],
                    473_977_503,
                    85_456_003,
                    15.0,
                ),
            ),
            (
                MissionCommand::from(DoGoAround {
                    position: geo_rel_home(473_977_504, 85_456_004, 35.0),
                }),
                (
                    u16::from(MAV_CMD_DO_GO_AROUND),
                    MissionFrame::GlobalRelativeAlt,
                    [0.0, 0.0, 0.0, 0.0],
                    473_977_504,
                    85_456_004,
                    35.0,
                ),
            ),
            (
                MissionCommand::from(DoSetRoiLocation {
                    position: geo_msl(473_977_505, 85_456_005, 45.0),
                }),
                (
                    u16::from(MAV_CMD_DO_SET_ROI_LOCATION),
                    MissionFrame::Global,
                    [0.0, 0.0, 0.0, 0.0],
                    473_977_505,
                    85_456_005,
                    45.0,
                ),
            ),
            (
                MissionCommand::from(DoCommand::SetRoiNone),
                (
                    u16::from(MAV_CMD_DO_SET_ROI_NONE),
                    MissionFrame::Mission,
                    [0.0, 0.0, 0.0, 0.0],
                    0,
                    0,
                    0.0,
                ),
            ),
            (
                MissionCommand::from(DoSetRoi {
                    mode: 3,
                    position: geo_terrain(473_977_506, 85_456_006, 55.0),
                }),
                (
                    u16::from(MAV_CMD_DO_SET_ROI),
                    MissionFrame::GlobalTerrainAlt,
                    [3.0, 0.0, 0.0, 0.0],
                    473_977_506,
                    85_456_006,
                    55.0,
                ),
            ),
            (
                MissionCommand::from(DoMountControl {
                    pitch_deg: -10.0,
                    roll_deg: 2.5,
                    yaw_deg: 180.0,
                }),
                (
                    u16::from(MAV_CMD_DO_MOUNT_CONTROL),
                    MissionFrame::Mission,
                    [-10.0, 2.5, 180.0, 0.0],
                    0,
                    0,
                    0.0,
                ),
            ),
            (
                MissionCommand::from(DoGimbalManagerPitchYaw {
                    pitch_deg: -5.0,
                    yaw_deg: 90.0,
                    pitch_rate_dps: 3.0,
                    yaw_rate_dps: 4.0,
                    flags: 7,
                    gimbal_id: 5,
                }),
                (
                    u16::from(MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW),
                    MissionFrame::Mission,
                    [-5.0, 90.0, 3.0, 4.0],
                    7,
                    0,
                    5.0,
                ),
            ),
            (
                MissionCommand::from(DoCamTriggerDistance {
                    meters: 12.5,
                    trigger_now: true,
                }),
                (
                    u16::from(MAV_CMD_DO_SET_CAM_TRIGG_DIST),
                    MissionFrame::Mission,
                    [12.5, 0.0, 1.0, 0.0],
                    0,
                    0,
                    0.0,
                ),
            ),
            (
                MissionCommand::from(DoImageStartCapture {
                    instance: 2,
                    interval_s: 4.0,
                    total_images: 6,
                    start_number: 8,
                }),
                (
                    u16::from(MAV_CMD_IMAGE_START_CAPTURE),
                    MissionFrame::Mission,
                    [2.0, 4.0, 6.0, 8.0],
                    0,
                    0,
                    0.0,
                ),
            ),
            (
                MissionCommand::from(DoImageStopCapture { instance: 2 }),
                (
                    u16::from(MAV_CMD_IMAGE_STOP_CAPTURE),
                    MissionFrame::Mission,
                    [2.0, 0.0, 0.0, 0.0],
                    0,
                    0,
                    0.0,
                ),
            ),
            (
                MissionCommand::from(DoVideoStartCapture { stream_id: 3 }),
                (
                    u16::from(MAV_CMD_VIDEO_START_CAPTURE),
                    MissionFrame::Mission,
                    [3.0, 0.0, 0.0, 0.0],
                    0,
                    0,
                    0.0,
                ),
            ),
            (
                MissionCommand::from(DoVideoStopCapture { stream_id: 4 }),
                (
                    u16::from(MAV_CMD_VIDEO_STOP_CAPTURE),
                    MissionFrame::Mission,
                    [4.0, 0.0, 0.0, 0.0],
                    0,
                    0,
                    0.0,
                ),
            ),
            (
                MissionCommand::from(DoSetCameraZoom {
                    zoom_type: 2,
                    zoom_value: 55.0,
                }),
                (
                    u16::from(MAV_CMD_SET_CAMERA_ZOOM),
                    MissionFrame::Mission,
                    [2.0, 55.0, 0.0, 0.0],
                    0,
                    0,
                    0.0,
                ),
            ),
            (
                MissionCommand::from(DoSetCameraFocus {
                    focus_type: 3,
                    focus_value: 9.0,
                }),
                (
                    u16::from(MAV_CMD_SET_CAMERA_FOCUS),
                    MissionFrame::Mission,
                    [3.0, 9.0, 0.0, 0.0],
                    0,
                    0,
                    0.0,
                ),
            ),
            (
                MissionCommand::from(DoSetCameraSource {
                    instance: 1,
                    primary: 2,
                    secondary: 3,
                }),
                (
                    u16::from(MAV_CMD_SET_CAMERA_SOURCE),
                    MissionFrame::Mission,
                    [1.0, 2.0, 3.0, 0.0],
                    0,
                    0,
                    0.0,
                ),
            ),
            (
                MissionCommand::from(DoDigicamConfigure {
                    shooting_mode: 1,
                    shutter_speed: 125,
                    aperture: 2.8,
                    iso: 200,
                    exposure_type: 4,
                    cmd_id: 5,
                    cutoff_time: 0.7,
                }),
                (
                    u16::from(MAV_CMD_DO_DIGICAM_CONFIGURE),
                    MissionFrame::Mission,
                    [1.0, 125.0, 2.8, 200.0],
                    4,
                    5,
                    0.7,
                ),
            ),
            (
                MissionCommand::from(DoDigicamControl {
                    session: 1,
                    zoom_pos: 2,
                    zoom_step: -3,
                    focus_lock: 4,
                    shooting_cmd: 5,
                    cmd_id: 6,
                }),
                (
                    u16::from(MAV_CMD_DO_DIGICAM_CONTROL),
                    MissionFrame::Mission,
                    [1.0, 2.0, -3.0, 4.0],
                    5,
                    6,
                    0.0,
                ),
            ),
            (
                MissionCommand::from(DoSetServo {
                    channel: 9,
                    pwm: 1500,
                }),
                (
                    u16::from(MAV_CMD_DO_SET_SERVO),
                    MissionFrame::Mission,
                    [9.0, 1500.0, 0.0, 0.0],
                    0,
                    0,
                    0.0,
                ),
            ),
            (
                MissionCommand::from(DoSetRelay {
                    number: 2,
                    state: true,
                }),
                (
                    u16::from(MAV_CMD_DO_SET_RELAY),
                    MissionFrame::Mission,
                    [2.0, 1.0, 0.0, 0.0],
                    0,
                    0,
                    0.0,
                ),
            ),
            (
                MissionCommand::from(DoRepeatServo {
                    channel: 10,
                    pwm: 1200,
                    count: 4,
                    cycle_time_s: 2.0,
                }),
                (
                    u16::from(MAV_CMD_DO_REPEAT_SERVO),
                    MissionFrame::Mission,
                    [10.0, 1200.0, 4.0, 2.0],
                    0,
                    0,
                    0.0,
                ),
            ),
            (
                MissionCommand::from(DoRepeatRelay {
                    number: 3,
                    count: 5,
                    cycle_time_s: 1.5,
                }),
                (
                    u16::from(MAV_CMD_DO_REPEAT_RELAY),
                    MissionFrame::Mission,
                    [3.0, 5.0, 1.5, 0.0],
                    0,
                    0,
                    0.0,
                ),
            ),
            (
                MissionCommand::from(DoFenceEnable {
                    action: FenceAction::Enable,
                }),
                (
                    u16::from(MAV_CMD_DO_FENCE_ENABLE),
                    MissionFrame::Mission,
                    [1.0, 0.0, 0.0, 0.0],
                    0,
                    0,
                    0.0,
                ),
            ),
            (
                MissionCommand::from(DoParachute {
                    action: ParachuteAction::Release,
                }),
                (
                    u16::from(MAV_CMD_DO_PARACHUTE),
                    MissionFrame::Mission,
                    [2.0, 0.0, 0.0, 0.0],
                    0,
                    0,
                    0.0,
                ),
            ),
            (
                MissionCommand::from(DoGripper {
                    number: 1,
                    action: GripperAction::Grab,
                }),
                (
                    u16::from(MAV_CMD_DO_GRIPPER),
                    MissionFrame::Mission,
                    [1.0, 1.0, 0.0, 0.0],
                    0,
                    0,
                    0.0,
                ),
            ),
            (
                MissionCommand::from(DoSprayer { enabled: true }),
                (
                    u16::from(MAV_CMD_DO_SPRAYER),
                    MissionFrame::Mission,
                    [1.0, 0.0, 0.0, 0.0],
                    0,
                    0,
                    0.0,
                ),
            ),
            (
                MissionCommand::from(DoWinch {
                    number: 1,
                    action: WinchAction::RateControl,
                    release_length_m: 12.5,
                    release_rate_mps: -1.25,
                }),
                (
                    u16::from(MAV_CMD_DO_WINCH),
                    MissionFrame::Mission,
                    [1.0, 2.0, 12.5, -1.25],
                    0,
                    0,
                    0.0,
                ),
            ),
            (
                MissionCommand::from(DoEngineControl {
                    start: true,
                    cold_start: false,
                    height_delay_m: 6.5,
                    allow_disarmed: true,
                }),
                (
                    u16::from(MAV_CMD_DO_ENGINE_CONTROL),
                    MissionFrame::Mission,
                    [1.0, 0.0, 6.5, 1.0],
                    0,
                    0,
                    0.0,
                ),
            ),
            (
                MissionCommand::from(DoInvertedFlight { inverted: true }),
                (
                    u16::from(MAV_CMD_DO_INVERTED_FLIGHT),
                    MissionFrame::Mission,
                    [1.0, 0.0, 0.0, 0.0],
                    0,
                    0,
                    0.0,
                ),
            ),
            (
                MissionCommand::from(DoAutotuneEnable { enabled: true }),
                (
                    u16::from(MAV_CMD_DO_AUTOTUNE_ENABLE),
                    MissionFrame::Mission,
                    [1.0, 0.0, 0.0, 0.0],
                    0,
                    0,
                    0.0,
                ),
            ),
            (
                MissionCommand::from(DoVtolTransition { target_state: 4 }),
                (
                    u16::from(MAV_CMD_DO_VTOL_TRANSITION),
                    MissionFrame::Mission,
                    [4.0, 0.0, 0.0, 0.0],
                    0,
                    0,
                    0.0,
                ),
            ),
            (
                MissionCommand::from(DoGuidedLimits {
                    max_time_s: 10.0,
                    min_alt_m: 20.0,
                    max_alt_m: 30.0,
                    max_horiz_m: 40.0,
                }),
                (
                    u16::from(MAV_CMD_DO_GUIDED_LIMITS),
                    MissionFrame::Mission,
                    [10.0, 20.0, 30.0, 40.0],
                    0,
                    0,
                    0.0,
                ),
            ),
            (
                MissionCommand::from(DoSetResumeRepeatDist { distance_m: 18.0 }),
                (
                    u16::from(MAV_CMD_DO_SET_RESUME_REPEAT_DIST),
                    MissionFrame::Mission,
                    [18.0, 0.0, 0.0, 0.0],
                    0,
                    0,
                    0.0,
                ),
            ),
            (
                MissionCommand::from(DoAuxFunction {
                    function: 42,
                    switch_pos: 2,
                }),
                (
                    u16::from(MAV_CMD_DO_AUX_FUNCTION),
                    MissionFrame::Mission,
                    [42.0, 2.0, 0.0, 0.0],
                    0,
                    0,
                    0.0,
                ),
            ),
            (
                MissionCommand::from(DoSendScriptMessage {
                    id: 7,
                    p1: 1.25,
                    p2: -2.5,
                    p3: 3.75,
                }),
                (
                    u16::from(MAV_CMD_DO_SEND_SCRIPT_MESSAGE),
                    MissionFrame::Mission,
                    [7.0, 1.25, -2.5, 3.75],
                    0,
                    0,
                    0.0,
                ),
            ),
        ];

        for (original, expected_wire) in cases {
            assert_roundtrip(original, expected_wire);
        }
    }

    #[test]
    fn all_cond_roundtrip() {
        let cases = [
            (
                MissionCommand::from(CondDelay { delay_s: 8.5 }),
                (
                    u16::from(MAV_CMD_CONDITION_DELAY),
                    MissionFrame::Mission,
                    [8.5, 0.0, 0.0, 0.0],
                    0,
                    0,
                    0.0,
                ),
            ),
            (
                MissionCommand::from(CondDistance { distance_m: 125.0 }),
                (
                    u16::from(MAV_CMD_CONDITION_DISTANCE),
                    MissionFrame::Mission,
                    [125.0, 0.0, 0.0, 0.0],
                    0,
                    0,
                    0.0,
                ),
            ),
            (
                MissionCommand::from(CondYaw {
                    angle_deg: 135.0,
                    turn_rate_dps: 20.0,
                    direction: YawDirection::CounterClockwise,
                    relative: true,
                }),
                (
                    u16::from(MAV_CMD_CONDITION_YAW),
                    MissionFrame::Mission,
                    [135.0, 20.0, -1.0, 1.0],
                    0,
                    0,
                    0.0,
                ),
            ),
        ];

        for (original, expected_wire) in cases {
            assert_roundtrip(original, expected_wire);
        }
    }

    #[test]
    fn enum_serde() {
        assert_unit_enum_serde(LoiterDirection::Clockwise, "clockwise");
        assert_unit_enum_serde(LoiterDirection::CounterClockwise, "counter_clockwise");
        assert_unit_enum_serde(YawDirection::Clockwise, "clockwise");
        assert_unit_enum_serde(YawDirection::CounterClockwise, "counter_clockwise");
        assert_unit_enum_serde(SpeedType::Airspeed, "airspeed");
        assert_unit_enum_serde(SpeedType::Groundspeed, "groundspeed");
        assert_unit_enum_serde(AltChangeAction::Neutral, "neutral");
        assert_unit_enum_serde(AltChangeAction::Climb, "climb");
        assert_unit_enum_serde(AltChangeAction::Descend, "descend");
        assert_unit_enum_serde(FenceAction::Disable, "disable");
        assert_unit_enum_serde(FenceAction::Enable, "enable");
        assert_unit_enum_serde(FenceAction::DisableFloor, "disable_floor");
        assert_unit_enum_serde(ParachuteAction::Disable, "disable");
        assert_unit_enum_serde(ParachuteAction::Enable, "enable");
        assert_unit_enum_serde(ParachuteAction::Release, "release");
        assert_unit_enum_serde(GripperAction::Release, "release");
        assert_unit_enum_serde(GripperAction::Grab, "grab");
        assert_unit_enum_serde(WinchAction::Relax, "relax");
        assert_unit_enum_serde(WinchAction::LengthControl, "length_control");
        assert_unit_enum_serde(WinchAction::RateControl, "rate_control");
    }

    #[test]
    fn other_roundtrip_preserves_raw_wire_fields() {
        let raw = RawMissionCommand {
            command: 31_337,
            frame: MissionFrame::Other(77),
            param1: 1.0,
            param2: -2.0,
            param3: 3.5,
            param4: -4.25,
            x: -17,
            y: 28,
            z: 900.5,
        };

        let (command, frame, params, x, y, z) = MissionCommand::Other(raw).into_wire();
        let decoded = MissionCommand::from_wire(command, frame, params, x, y, z);
        assert_eq!(decoded, MissionCommand::Other(raw));
    }

    #[test]
    fn known_command_roundtrip_uses_generated_wire_arms() {
        let original = MissionCommand::from(NavWaypoint {
            position: geo_msl(10, 20, 30.0),
            hold_time_s: 1.0,
            acceptance_radius_m: 2.0,
            pass_radius_m: 3.0,
            yaw_deg: 4.0,
        });

        let (command, frame, params, x, y, z) = original.clone().into_wire();
        let decoded = MissionCommand::from_wire(command, frame, params, x, y, z);
        assert_eq!(decoded, original);
    }
}
