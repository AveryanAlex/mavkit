use crate::error::VehicleError;
use crate::mission::{MissionPlan, MissionType};
use crate::params::{Param, ParamStore, ParamWriteResult};
use mavlink::common::MavCmd;
use tokio::sync::oneshot;

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
    CommandLong {
        command: MavCmd,
        params: [f32; 7],
        reply: oneshot::Sender<Result<(), VehicleError>>,
    },
    GuidedGoto {
        lat_e7: i32,
        lon_e7: i32,
        alt_m: f32,
        reply: oneshot::Sender<Result<(), VehicleError>>,
    },
    MissionUpload {
        plan: MissionPlan,
        reply: oneshot::Sender<Result<(), VehicleError>>,
    },
    MissionDownload {
        mission_type: MissionType,
        reply: oneshot::Sender<Result<MissionPlan, VehicleError>>,
    },
    MissionClear {
        mission_type: MissionType,
        reply: oneshot::Sender<Result<(), VehicleError>>,
    },
    MissionSetCurrent {
        seq: u16,
        reply: oneshot::Sender<Result<(), VehicleError>>,
    },
    MissionCancelTransfer,
    ParamDownloadAll {
        reply: oneshot::Sender<Result<ParamStore, VehicleError>>,
    },
    ParamWrite {
        name: String,
        value: f32,
        reply: oneshot::Sender<Result<Param, VehicleError>>,
    },
    ParamWriteBatch {
        params: Vec<(String, f32)>,
        reply: oneshot::Sender<Result<Vec<ParamWriteResult>, VehicleError>>,
    },
    Shutdown,
}
