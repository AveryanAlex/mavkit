use super::types::{MissionFrame as MissionItemFrame, MissionItem};
use serde::{Deserialize, Serialize};

mod condition;
mod do_commands;
mod nav;
mod wire_support;

pub use condition::*;
pub use do_commands::*;
pub use nav::*;

/// Typed mission command API item used by plan serialization and validation.
#[cfg_attr(feature = "typescript", derive(specta::Type))]
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
#[cfg_attr(feature = "typescript", derive(specta::Type))]
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
                    $nav_variant:ident($nav_ty:ty)
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
                    $do_variant:ident($do_ty:ty)
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
                $condition_variant:ident($condition_ty:ty)
            ),+ $(,)?
        }
    ) => {
        /// Typed mission command API item used by plan serialization and validation.
        #[cfg_attr(feature = "typescript", derive(specta::Type))]
        #[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
        pub enum NavCommand {
            $($nav_variant($nav_ty),)+
            $($nav_unit_variant,)*
        }

        /// Typed mission command API item used by plan serialization and validation.
        #[cfg_attr(feature = "typescript", derive(specta::Type))]
        #[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
        pub enum DoCommand {
            $($do_variant($do_ty),)*
            $($do_unit_variant,)*
        }

        /// Typed mission command API item used by plan serialization and validation.
        #[cfg_attr(feature = "typescript", derive(specta::Type))]
        #[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
        pub enum ConditionCommand {
            $($condition_variant($condition_ty)),+
        }

        /// Typed mission command API item used by plan serialization and validation.
        #[cfg_attr(feature = "typescript", derive(specta::Type))]
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
                        code if code == <$nav_ty>::COMMAND_ID => {
                            Self::Nav(NavCommand::$nav_variant(<$nav_ty>::from_wire(frame, params, x, y, z)))
                        },
                    )+
                    $(
                        code if code == ($nav_unit_command) as u16 => {
                            $nav_unit_from_wire(frame, params, x, y, z);
                            Self::Nav(NavCommand::$nav_unit_variant)
                        },
                    )*
                    $(
                        code if code == <$do_ty>::COMMAND_ID => {
                            Self::Do(DoCommand::$do_variant(<$do_ty>::from_wire(frame, params, x, y, z)))
                        },
                    )*
                    $(
                        code if code == ($do_unit_command) as u16 => {
                            $do_unit_from_wire(frame, params, x, y, z);
                            Self::Do(DoCommand::$do_unit_variant)
                        },
                    )*
                    $(
                        code if code == <$condition_ty>::COMMAND_ID => {
                            Self::Condition(ConditionCommand::$condition_variant(
                                <$condition_ty>::from_wire(frame, params, x, y, z),
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
                            let (frame, params, x, y, z) = command_value.into_wire();
                            (<$nav_ty>::COMMAND_ID, frame, params, x, y, z)
                        }
                    )+
                    $(
                        Self::Nav(NavCommand::$nav_unit_variant) => {
                            let (frame, params, x, y, z) = $nav_unit_to_wire();
                            (($nav_unit_command) as u16, frame, params, x, y, z)
                        }
                    )*
                    $(
                        Self::Do(DoCommand::$do_variant(command_value)) => {
                            let (frame, params, x, y, z) = command_value.into_wire();
                            (<$do_ty>::COMMAND_ID, frame, params, x, y, z)
                        }
                    )*
                    $(
                        Self::Do(DoCommand::$do_unit_variant) => {
                            let (frame, params, x, y, z) = $do_unit_to_wire();
                            (($do_unit_command) as u16, frame, params, x, y, z)
                        }
                    )*
                    $(
                        Self::Condition(ConditionCommand::$condition_variant(command_value)) => {
                            let (frame, params, x, y, z) = command_value.into_wire();
                            (<$condition_ty>::COMMAND_ID, frame, params, x, y, z)
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
            Waypoint(NavWaypoint),
            SplineWaypoint(NavSplineWaypoint),
            ArcWaypoint(NavArcWaypoint),
            Takeoff(NavTakeoff),
            Land(NavLand),
            LoiterUnlimited(NavLoiterUnlimited),
            LoiterTurns(NavLoiterTurns),
            LoiterTime(NavLoiterTime),
            LoiterToAlt(NavLoiterToAlt),
            ContinueAndChangeAlt(NavContinueAndChangeAlt),
            Delay(NavDelay),
            GuidedEnable(NavGuidedEnable),
            AltitudeWait(NavAltitudeWait),
            VtolTakeoff(NavVtolTakeoff),
            VtolLand(NavVtolLand),
            PayloadPlace(NavPayloadPlace),
            SetYawSpeed(NavSetYawSpeed),
            ScriptTime(NavScriptTime),
            AttitudeTime(NavAttitudeTime)
        }
        Unit {
            ReturnToLaunch {
                command: 20,
                to_wire: return_to_launch_to_wire,
                from_wire: return_to_launch_from_wire,
            }
        }
    }
    Do {
        Data {
            Jump(DoJump),
            JumpTag(DoJumpTag),
            Tag(DoTag),
            PauseContinue(DoPauseContinue),
            ChangeSpeed(DoChangeSpeed),
            SetReverse(DoSetReverse),
            SetHome(DoSetHome),
            LandStart(DoLandStart),
            ReturnPathStart(DoReturnPathStart),
            GoAround(DoGoAround),
            SetRoiLocation(DoSetRoiLocation),
            SetRoi(DoSetRoi),
            MountControl(DoMountControl),
            GimbalManagerPitchYaw(DoGimbalManagerPitchYaw),
            CamTriggerDistance(DoCamTriggerDistance),
            ImageStartCapture(DoImageStartCapture),
            ImageStopCapture(DoImageStopCapture),
            VideoStartCapture(DoVideoStartCapture),
            VideoStopCapture(DoVideoStopCapture),
            SetCameraZoom(DoSetCameraZoom),
            SetCameraFocus(DoSetCameraFocus),
            SetCameraSource(DoSetCameraSource),
            DigicamConfigure(DoDigicamConfigure),
            DigicamControl(DoDigicamControl),
            SetServo(DoSetServo),
            SetRelay(DoSetRelay),
            RepeatServo(DoRepeatServo),
            RepeatRelay(DoRepeatRelay),
            FenceEnable(DoFenceEnable),
            Parachute(DoParachute),
            Gripper(DoGripper),
            Sprayer(DoSprayer),
            Winch(DoWinch),
            EngineControl(DoEngineControl),
            InvertedFlight(DoInvertedFlight),
            AutotuneEnable(DoAutotuneEnable),
            VtolTransition(DoVtolTransition),
            GuidedLimits(DoGuidedLimits),
            SetResumeRepeatDist(DoSetResumeRepeatDist),
            AuxFunction(DoAuxFunction),
            SendScriptMessage(DoSendScriptMessage)
        }
        Unit {
            SetRoiNone {
                command: 197,
                to_wire: do_set_roi_none_to_wire,
                from_wire: do_set_roi_none_from_wire,
            }
        }
    }
    Condition {
        Delay(CondDelay),
        Distance(CondDistance),
        Yaw(CondYaw)
    }
}

#[cfg(test)]
mod tests;
