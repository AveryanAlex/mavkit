use crate::dialect::{self, MavCmd, MavFrame};
use crate::error::VehicleError;
use crate::mission::{MissionType, WireMissionPlan};
use crate::params::{ParamStore, ParamWriteResult};
use crate::raw::CommandAck;
use std::time::Instant;
use tokio::sync::oneshot;
use tokio_util::sync::CancellationToken;

pub(crate) struct RawCommandIntPayload {
    pub(crate) command: MavCmd,
    pub(crate) frame: MavFrame,
    pub(crate) current: u8,
    pub(crate) autocontinue: u8,
    pub(crate) params: [f32; 4],
    pub(crate) x: i32,
    pub(crate) y: i32,
    pub(crate) z: f32,
}

pub(crate) enum Command {
    Arm {
        force: bool,
        reply: oneshot::Sender<Result<(), VehicleError>>,
    },
    Disarm {
        force: bool,
        reply: oneshot::Sender<Result<(), VehicleError>>,
    },
    SetMode {
        custom_mode: u32,
        reply: oneshot::Sender<Result<(), VehicleError>>,
    },
    Long {
        command: MavCmd,
        params: [f32; 7],
        reply: oneshot::Sender<Result<(), VehicleError>>,
    },
    LongRaw {
        command_id: u16,
        params: [f32; 7],
        reply: oneshot::Sender<Result<(), VehicleError>>,
    },
    RawCommandLong {
        command: MavCmd,
        params: [f32; 7],
        reply: oneshot::Sender<Result<(), VehicleError>>,
    },
    RawCommandLongAck {
        command: MavCmd,
        params: [f32; 7],
        reply: oneshot::Sender<Result<CommandAck, VehicleError>>,
    },
    RawCommandInt {
        payload: RawCommandIntPayload,
        reply: oneshot::Sender<Result<CommandAck, VehicleError>>,
    },
    RawSend {
        message: Box<dialect::MavMessage>,
        reply: oneshot::Sender<Result<(), VehicleError>>,
    },
    GuidedGoto {
        lat_e7: i32,
        lon_e7: i32,
        alt_m: f32,
        reply: oneshot::Sender<Result<(), VehicleError>>,
    },
    SetOrigin {
        latitude: i32,
        longitude: i32,
        altitude: i32,
        reply: oneshot::Sender<Result<Instant, VehicleError>>,
    },
    MissionUpload {
        plan: WireMissionPlan,
        reply: oneshot::Sender<Result<(), VehicleError>>,
        cancel: CancellationToken,
    },
    MissionDownload {
        mission_type: MissionType,
        reply: oneshot::Sender<Result<WireMissionPlan, VehicleError>>,
        cancel: CancellationToken,
    },
    MissionClear {
        mission_type: MissionType,
        reply: oneshot::Sender<Result<(), VehicleError>>,
        cancel: CancellationToken,
    },
    MissionSetCurrent {
        seq: u16,
        reply: oneshot::Sender<Result<(), VehicleError>>,
    },
    ParamDownloadAll {
        reply: oneshot::Sender<Result<ParamStore, VehicleError>>,
    },
    ParamWrite {
        name: String,
        value: f32,
        reply: oneshot::Sender<Result<ParamWriteResult, VehicleError>>,
    },
    ParamWriteBatch {
        params: Vec<(String, f32)>,
        reply: oneshot::Sender<Result<Vec<ParamWriteResult>, VehicleError>>,
    },
    Shutdown,
}
