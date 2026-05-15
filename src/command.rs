use crate::dialect::{self, MavCmd, MavFrame};
use crate::error::VehicleError;
use crate::mission::{MissionType, WireMissionPlan};
use crate::params::{ParamStore, ParamWriteResult};
use crate::raw::CommandAck;
use crate::time::Instant;
use tokio::sync::{mpsc, oneshot};
use tokio_util::sync::CancellationToken;

pub(crate) struct CommandIntPayload {
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
        cancel: CancellationToken,
    },
    Disarm {
        force: bool,
        reply: oneshot::Sender<Result<(), VehicleError>>,
        cancel: CancellationToken,
    },
    SetMode {
        custom_mode: u32,
        reply: oneshot::Sender<Result<(), VehicleError>>,
        cancel: CancellationToken,
    },
    Long {
        command: MavCmd,
        params: [f32; 7],
        reply: oneshot::Sender<Result<(), VehicleError>>,
        cancel: CancellationToken,
    },
    LongRaw {
        command_id: u16,
        params: [f32; 7],
        reply: oneshot::Sender<Result<(), VehicleError>>,
        cancel: CancellationToken,
    },
    RawCommandLong {
        command: MavCmd,
        params: [f32; 7],
        reply: oneshot::Sender<Result<(), VehicleError>>,
        cancel: CancellationToken,
    },
    RawCommandLongAck {
        command: MavCmd,
        params: [f32; 7],
        reply: oneshot::Sender<Result<CommandAck, VehicleError>>,
        cancel: CancellationToken,
    },
    RawCommandInt {
        payload: CommandIntPayload,
        reply: oneshot::Sender<Result<CommandAck, VehicleError>>,
        cancel: CancellationToken,
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

pub(crate) async fn send_command_int_ack(
    command_tx: mpsc::Sender<Command>,
    payload: CommandIntPayload,
) -> Result<CommandAck, VehicleError> {
    let (tx, rx) = oneshot::channel();
    let cancel = CancellationToken::new();
    let _cancel_on_drop = CommandCancelOnDrop {
        cancel: cancel.clone(),
    };
    command_tx
        .send(Command::RawCommandInt {
            payload,
            reply: tx,
            cancel: cancel.clone(),
        })
        .await
        .map_err(|_| VehicleError::Disconnected)?;

    rx.await.map_err(|_| VehicleError::Disconnected)?
}

pub(crate) async fn send_typed_command_int(
    command_tx: mpsc::Sender<Command>,
    payload: CommandIntPayload,
) -> Result<(), VehicleError> {
    send_command_int_ack(command_tx, payload).await.map(|_| ())
}

struct CommandCancelOnDrop {
    cancel: CancellationToken,
}

impl Drop for CommandCancelOnDrop {
    fn drop(&mut self) {
        self.cancel.cancel();
    }
}
