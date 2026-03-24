mod commands;
mod dispatcher;
mod init;
mod mission;
mod params;
mod state_update;

use commands::{
    handle_arm_disarm, handle_command_int, handle_command_long, handle_command_long_raw,
    handle_guided_goto, handle_raw_send, handle_set_mode, handle_set_origin,
};
use mission::{
    handle_mission_clear, handle_mission_download, handle_mission_set_current,
    handle_mission_upload,
};
use params::{handle_param_download_all, handle_param_write, handle_param_write_batch};
use state_update::update_state;

use crate::command::Command;
use crate::config::VehicleConfig;
use crate::error::VehicleError;
use crate::state::{LinkState, StateWriters};
use dispatcher::{AckCommandDispatcher, WireCommandAck};
#[cfg(test)]
pub(crate) use init::InitUnavailableReason;
pub(crate) use init::{InitManager, InitSnapshot, InitState};

use crate::dialect;
use mavlink::{AsyncMavConnection, MavHeader, Message};
use std::sync::Arc;
use tokio::sync::{broadcast, mpsc};
use tokio_util::sync::CancellationToken;
use tracing::{debug, warn};

const MAGIC_FORCE_ARM_VALUE: f32 = 2989.0;
const MAGIC_FORCE_DISARM_VALUE: f32 = 21196.0;
const ROUTER_CHANNEL_CAPACITY: usize = 256;
const COMMAND_ACK_MESSAGE_ID: u32 = 77;

type RouterMessage = (MavHeader, dialect::MavMessage);
type SharedConnection = Arc<dyn AsyncMavConnection<dialect::MavMessage> + Sync + Send>;

/// Internal tracking of the remote vehicle identity (from heartbeats).
#[derive(Debug, Clone, Copy)]
struct VehicleTarget {
    system_id: u8,
    component_id: u8,
    autopilot: dialect::MavAutopilot,
    vehicle_type: dialect::MavType,
}

/// Bundles the common parameters passed to every command handler.
struct CommandContext {
    pub(crate) connection: SharedConnection,
    pub(crate) writers: Arc<StateWriters>,
    pub(crate) vehicle_target: Option<VehicleTarget>,
    pub(crate) config: VehicleConfig,
    pub(crate) cancel: CancellationToken,
    pub(crate) inbound_rx: broadcast::Receiver<RouterMessage>,
    pub(crate) ack_dispatcher: Arc<AckCommandDispatcher>,
}

struct MessageRouterContext<'a> {
    connection: SharedConnection,
    state_writers: &'a StateWriters,
    router_tx: &'a broadcast::Sender<RouterMessage>,
    ack_dispatcher: &'a AckCommandDispatcher,
    init_manager: &'a InitManager,
    vehicle_target: &'a mut Option<VehicleTarget>,
    cancel: CancellationToken,
}

#[cfg(test)]
pub(crate) async fn run_event_loop(
    connection: Box<dyn AsyncMavConnection<dialect::MavMessage> + Sync + Send>,
    command_rx: mpsc::Receiver<Command>,
    state_writers: StateWriters,
    config: VehicleConfig,
    cancel: CancellationToken,
) {
    let init_manager = InitManager::new(config.clone());
    run_event_loop_with_init(
        connection,
        command_rx,
        state_writers,
        config,
        init_manager,
        cancel,
    )
    .await;
}

pub(crate) async fn run_event_loop_with_init(
    connection: Box<dyn AsyncMavConnection<dialect::MavMessage> + Sync + Send>,
    mut command_rx: mpsc::Receiver<Command>,
    state_writers: StateWriters,
    config: VehicleConfig,
    init_manager: InitManager,
    cancel: CancellationToken,
) {
    let connection: SharedConnection = Arc::from(connection);
    let state_writers = Arc::new(state_writers);
    let (router_tx, _) = broadcast::channel::<RouterMessage>(ROUTER_CHANNEL_CAPACITY);
    let ack_dispatcher = Arc::new(AckCommandDispatcher::new(
        config.gcs_system_id,
        config.gcs_component_id,
    ));
    let mut command_tasks = tokio::task::JoinSet::new();

    let mut vehicle_target: Option<VehicleTarget> = None;

    publish_link_state(state_writers.as_ref(), LinkState::Connected);

    loop {
        tokio::select! {
            biased;

            _ = cancel.cancelled() => {
                debug!("event loop cancelled");
                publish_link_state(state_writers.as_ref(), LinkState::Disconnected);
                break;
            }
            Some(cmd) = command_rx.recv() => {
                match cmd {
                    Command::Shutdown => {
                        debug!("event loop shutdown requested");
                        cancel.cancel();
                        publish_link_state(state_writers.as_ref(), LinkState::Disconnected);
                        break;
                    }
                    cmd => {
                        let mut ctx = CommandContext {
                            connection: connection.clone(),
                            writers: state_writers.clone(),
                            vehicle_target,
                            config: config.clone(),
                            cancel: cancel.clone(),
                            inbound_rx: router_tx.subscribe(),
                            ack_dispatcher: ack_dispatcher.clone(),
                        };
                        command_tasks.spawn(async move {
                            handle_command(cmd, &mut ctx).await;
                        });
                    }
                }
            }
            result = connection.recv_raw() => {
                match result {
                    Ok(raw) => {
                        let header = MavHeader {
                            sequence: raw.sequence(),
                            system_id: raw.system_id(),
                            component_id: raw.component_id(),
                        };
                        let mut router_ctx = MessageRouterContext {
                            connection: connection.clone(),
                            state_writers: state_writers.as_ref(),
                            router_tx: &router_tx,
                            ack_dispatcher: ack_dispatcher.as_ref(),
                            init_manager: &init_manager,
                            vehicle_target: &mut vehicle_target,
                            cancel: cancel.clone(),
                        };
                        match dialect::MavMessage::parse(raw.version(), raw.message_id(), raw.payload()) {
                            Ok(msg) => {
                                route_incoming_message(&mut router_ctx, header, msg).await;
                            }
                            Err(err) => {
                                if raw.message_id() == COMMAND_ACK_MESSAGE_ID
                                    && let Some(ack) = parse_command_ack_payload(raw.payload())
                                {
                                    router_ctx.ack_dispatcher.route_command_ack_raw(header, ack);
                                    continue;
                                }
                                warn!("MAVLink parse error: {err}");
                            }
                        }
                    }
                    Err(err) => {
                        warn!("MAVLink recv error: {err}");
                        publish_link_state(state_writers.as_ref(), LinkState::Error(err.to_string()));
                        break;
                    }
                }
            }
            Some(join_result) = command_tasks.join_next(), if !command_tasks.is_empty() => {
                if let Err(err) = join_result {
                    warn!("command task join error: {err}");
                }
            }
        }
    }

    command_tasks.abort_all();
    while command_tasks.join_next().await.is_some() {}
}

fn publish_link_state(state_writers: &StateWriters, state: LinkState) {
    let _ = state_writers.link_state.send(state.clone());
    let _ = state_writers.link_state_observation.publish(state);
}

fn bridge_vehicle_cancel_to_op(vehicle: &CancellationToken, op: &CancellationToken) {
    let vehicle = vehicle.clone();
    let op = op.clone();
    tokio::spawn(async move {
        vehicle.cancelled().await;
        op.cancel();
    });
}

async fn route_incoming_message(
    ctx: &mut MessageRouterContext<'_>,
    header: MavHeader,
    msg: dialect::MavMessage,
) {
    let _ = ctx.state_writers.raw_message_tx.send((header, msg.clone()));
    let _ = ctx.router_tx.send((header, msg.clone()));

    if let dialect::MavMessage::COMMAND_ACK(ack) = &msg {
        ctx.ack_dispatcher.route_command_ack(header, ack);
    }

    ctx.init_manager.handle_message(&msg);

    update_vehicle_target(ctx.vehicle_target, &header, &msg);
    if matches!(msg, dialect::MavMessage::HEARTBEAT(_))
        && let Some(target) = ctx.vehicle_target
    {
        ctx.init_manager
            .start(ctx.connection.clone(), *target, ctx.cancel.clone());
    }

    update_state(&header, &msg, ctx.state_writers, ctx.vehicle_target);
}

fn parse_command_ack_payload(payload: &[u8]) -> Option<WireCommandAck> {
    if payload.len() < 3 {
        return None;
    }

    let command_id = u16::from_le_bytes([payload[0], payload[1]]);
    let result = payload[2];
    if payload.len() < 10 {
        return Some(WireCommandAck {
            command_id,
            result,
            progress: 0,
            result_param2: 0,
            target_system: 0,
            target_component: 0,
        });
    }

    Some(WireCommandAck {
        command_id,
        result,
        progress: payload[3],
        result_param2: i32::from_le_bytes([payload[4], payload[5], payload[6], payload[7]]),
        target_system: payload[8],
        target_component: payload[9],
    })
}

async fn recv_routed(
    inbound_rx: &mut broadcast::Receiver<RouterMessage>,
) -> Result<RouterMessage, VehicleError> {
    loop {
        match inbound_rx.recv().await {
            Ok(message) => return Ok(message),
            Err(broadcast::error::RecvError::Lagged(_)) => continue,
            Err(broadcast::error::RecvError::Closed) => return Err(VehicleError::Disconnected),
        }
    }
}

fn update_vehicle_target(
    vehicle_target: &mut Option<VehicleTarget>,
    header: &MavHeader,
    message: &dialect::MavMessage,
) {
    if header.system_id == 0 {
        return;
    }

    if let dialect::MavMessage::HEARTBEAT(hb) = message {
        *vehicle_target = Some(VehicleTarget {
            system_id: header.system_id,
            component_id: header.component_id,
            autopilot: hb.autopilot,
            vehicle_type: hb.mavtype,
        });
    } else if vehicle_target.is_none() {
        *vehicle_target = Some(VehicleTarget {
            system_id: header.system_id,
            component_id: header.component_id,
            autopilot: dialect::MavAutopilot::MAV_AUTOPILOT_GENERIC,
            vehicle_type: dialect::MavType::MAV_TYPE_GENERIC,
        });
    }
}

// ---------------------------------------------------------------------------
// Helpers: send message, wait for response
// ---------------------------------------------------------------------------

pub(super) async fn send_message(
    connection: &(dyn AsyncMavConnection<dialect::MavMessage> + Sync + Send),
    config: &VehicleConfig,
    message: dialect::MavMessage,
) -> Result<(), VehicleError> {
    connection
        .send(
            &MavHeader {
                system_id: config.gcs_system_id,
                component_id: config.gcs_component_id,
                sequence: 0,
            },
            &message,
        )
        .await
        .map(|_| ())
        .map_err(|err| VehicleError::Io(std::io::Error::other(err.to_string())))
}

fn get_target(vehicle_target: &Option<VehicleTarget>) -> Result<VehicleTarget, VehicleError> {
    vehicle_target.ok_or(VehicleError::IdentityUnknown)
}

// ---------------------------------------------------------------------------
// Command dispatch
// ---------------------------------------------------------------------------

async fn handle_command(cmd: Command, ctx: &mut CommandContext) {
    match cmd {
        Command::Arm { force, reply } => {
            let result = handle_arm_disarm(true, force, ctx).await;
            let _ = reply.send(result);
        }
        Command::Disarm { force, reply } => {
            let result = handle_arm_disarm(false, force, ctx).await;
            let _ = reply.send(result);
        }
        Command::SetMode { custom_mode, reply } => {
            let result = handle_set_mode(custom_mode, ctx).await;
            let _ = reply.send(result);
        }
        Command::Long {
            command,
            params,
            reply,
        } => {
            let result = handle_command_long(command, params, ctx).await.map(|_| ());
            let _ = reply.send(result);
        }
        Command::LongRaw {
            command_id,
            params,
            reply,
        } => {
            let result = handle_command_long_raw(command_id, params, ctx)
                .await
                .map(|_| ());
            let _ = reply.send(result);
        }
        Command::RawCommandLong {
            command,
            params,
            reply,
        } => {
            let result = handle_command_long(command, params, ctx).await.map(|_| ());
            let _ = reply.send(result);
        }
        Command::RawCommandLongAck {
            command,
            params,
            reply,
        } => {
            let result = handle_command_long(command, params, ctx).await;
            let _ = reply.send(result);
        }
        Command::RawCommandInt { payload, reply } => {
            let result = handle_command_int(payload, ctx).await;
            let _ = reply.send(result);
        }
        Command::RawSend { message, reply } => {
            let result = handle_raw_send(*message, ctx).await;
            let _ = reply.send(result);
        }
        Command::GuidedGoto {
            lat_e7,
            lon_e7,
            alt_m,
            reply,
        } => {
            let result = handle_guided_goto(lat_e7, lon_e7, alt_m, ctx).await;
            let _ = reply.send(result);
        }
        Command::SetOrigin {
            latitude,
            longitude,
            altitude,
            reply,
        } => {
            let result = handle_set_origin(latitude, longitude, altitude, ctx).await;
            let _ = reply.send(result);
        }
        Command::MissionUpload {
            plan,
            reply,
            cancel: op_cancel,
        } => {
            bridge_vehicle_cancel_to_op(&ctx.cancel, &op_cancel);
            let result = handle_mission_upload(plan, ctx, &op_cancel).await;
            let _ = reply.send(result);
        }
        Command::MissionDownload {
            mission_type,
            reply,
            cancel: op_cancel,
        } => {
            bridge_vehicle_cancel_to_op(&ctx.cancel, &op_cancel);
            let result = handle_mission_download(mission_type, ctx, &op_cancel).await;
            let _ = reply.send(result);
        }
        Command::MissionClear {
            mission_type,
            reply,
            cancel: op_cancel,
        } => {
            bridge_vehicle_cancel_to_op(&ctx.cancel, &op_cancel);
            let result = handle_mission_clear(mission_type, ctx, &op_cancel).await;
            let _ = reply.send(result);
        }
        Command::MissionSetCurrent { seq, reply } => {
            let result = handle_mission_set_current(seq, ctx).await;
            let _ = reply.send(result);
        }
        Command::ParamDownloadAll { reply } => {
            let result = handle_param_download_all(ctx).await;
            let _ = reply.send(result);
        }
        Command::ParamWrite { name, value, reply } => {
            let result = handle_param_write(&name, value, ctx).await;
            let _ = reply.send(result);
        }
        Command::ParamWriteBatch { params, reply } => {
            let result = handle_param_write_batch(params, ctx).await;
            let _ = reply.send(result);
        }
        Command::Shutdown => {
            // Handled in the main loop
        }
    }
}

#[cfg(test)]
mod tests;
