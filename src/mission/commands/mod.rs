use super::types::{MissionFrame as MissionItemFrame, MissionItem};
use serde::{Deserialize, Serialize};

mod condition;
mod do_commands;
mod nav;
mod wire_support;

pub use condition::*;
pub use do_commands::*;
pub use nav::*;

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

/// Typed mission command API item used by plan serialization and validation.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
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

/// Typed mission command API item used by plan serialization and validation.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
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
        /// Typed mission command API item used by plan serialization and validation.
        #[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
        pub enum NavCommand {
            $($nav_variant($nav_ty),)+
            $($nav_unit_variant,)*
        }

        /// Typed mission command API item used by plan serialization and validation.
        #[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
        pub enum DoCommand {
            $($do_variant($do_ty),)*
            $($do_unit_variant,)*
        }

        /// Typed mission command API item used by plan serialization and validation.
        #[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
        pub enum ConditionCommand {
            $($condition_variant($condition_ty)),+
        }

        /// Typed mission command API item used by plan serialization and validation.
        #[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
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
                    autocontinue: true,
                }
            }
        }
    };
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
            to_wire: cond_delay_to_wire,
            from_wire: cond_delay_from_wire,
        },
        Distance(CondDistance) {
            command: MAV_CMD_CONDITION_DISTANCE,
            to_wire: cond_distance_to_wire,
            from_wire: cond_distance_from_wire,
        },
        Yaw(CondYaw) {
            command: MAV_CMD_CONDITION_YAW,
            to_wire: cond_yaw_to_wire,
            from_wire: cond_yaw_from_wire,
        }
    }
}

#[cfg(test)]
mod tests;
