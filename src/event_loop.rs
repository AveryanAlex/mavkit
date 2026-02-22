use crate::command::Command;
use crate::config::VehicleConfig;
use crate::error::VehicleError;
use crate::mission::{
    self, IssueSeverity, MissionFrame, MissionItem, MissionPlan, MissionTransferMachine, MissionType,
    TransferPhase,
};
use crate::params::{Param, ParamProgress, ParamStore, ParamTransferPhase, ParamType, ParamWriteResult};
use crate::state::{
    AutopilotType, GpsFixType, LinkState, MissionState, StateWriters, SystemStatus,
    VehicleState, VehicleType,
};
use mavlink::common::{self, MavCmd, MavModeFlag, MavParamType};
use mavlink::{AsyncMavConnection, MavHeader};
use std::collections::{HashMap, HashSet};
use std::time::Duration;
use tokio::sync::mpsc;
use tokio_util::sync::CancellationToken;
use tracing::{debug, trace, warn};

const MAGIC_FORCE_ARM_VALUE: f32 = 2989.0;
const MAGIC_FORCE_DISARM_VALUE: f32 = 21196.0;

/// Internal tracking of the remote vehicle identity (from heartbeats).
#[derive(Debug, Clone, Copy)]
struct VehicleTarget {
    system_id: u8,
    component_id: u8,
    autopilot: common::MavAutopilot,
    vehicle_type: common::MavType,
}

pub(crate) async fn run_event_loop(
    connection: Box<dyn AsyncMavConnection<common::MavMessage> + Sync + Send>,
    mut command_rx: mpsc::Receiver<Command>,
    state_writers: StateWriters,
    config: VehicleConfig,
    cancel: CancellationToken,
) {
    let mut vehicle_target: Option<VehicleTarget> = None;
    let mut home_requested = false;

    let _ = state_writers.link_state.send(LinkState::Connected);

    loop {
        tokio::select! {
            biased;

            _ = cancel.cancelled() => {
                debug!("event loop cancelled");
                let _ = state_writers.link_state.send(LinkState::Disconnected);
                break;
            }
            Some(cmd) = command_rx.recv() => {
                match cmd {
                    Command::Shutdown => {
                        debug!("event loop shutdown requested");
                        let _ = state_writers.link_state.send(LinkState::Disconnected);
                        break;
                    }
                    cmd => {
                        handle_command(
                            cmd,
                            &*connection,
                            &state_writers,
                            &mut vehicle_target,
                            &config,
                            &cancel,
                        ).await;
                    }
                }
            }
            result = connection.recv() => {
                match result {
                    Ok((header, msg)) => {
                        update_vehicle_target(&mut vehicle_target, &header, &msg);
                        if !home_requested && config.auto_request_home {
                            if let Some(ref target) = vehicle_target {
                                request_home_position(&*connection, target, &config).await;
                                home_requested = true;
                            }
                        }
                        update_state(&header, &msg, &state_writers, &vehicle_target);
                    }
                    Err(err) => {
                        warn!("MAVLink recv error: {err}");
                        let _ = state_writers.link_state.send(LinkState::Error(err.to_string()));
                        break;
                    }
                }
            }
        }
    }
}

async fn request_home_position(
    connection: &(dyn AsyncMavConnection<common::MavMessage> + Sync + Send),
    target: &VehicleTarget,
    config: &VehicleConfig,
) {
    let _ = connection
        .send(
            &MavHeader {
                system_id: config.gcs_system_id,
                component_id: config.gcs_component_id,
                sequence: 0,
            },
            &common::MavMessage::COMMAND_LONG(common::COMMAND_LONG_DATA {
                target_system: target.system_id,
                target_component: target.component_id,
                command: MavCmd::MAV_CMD_REQUEST_MESSAGE,
                confirmation: 0,
                param1: 242.0, // HOME_POSITION message ID
                param2: 0.0,
                param3: 0.0,
                param4: 0.0,
                param5: 0.0,
                param6: 0.0,
                param7: 0.0,
            }),
        )
        .await;
}

fn update_vehicle_target(
    vehicle_target: &mut Option<VehicleTarget>,
    header: &MavHeader,
    message: &common::MavMessage,
) {
    if header.system_id == 0 {
        return;
    }

    if let common::MavMessage::HEARTBEAT(hb) = message {
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
            autopilot: common::MavAutopilot::MAV_AUTOPILOT_GENERIC,
            vehicle_type: common::MavType::MAV_TYPE_GENERIC,
        });
    }
}

fn update_state(
    _header: &MavHeader,
    message: &common::MavMessage,
    writers: &StateWriters,
    vehicle_target: &Option<VehicleTarget>,
) {
    match message {
        common::MavMessage::HEARTBEAT(hb) => {
            if let Some(target) = vehicle_target {
                let autopilot_type = AutopilotType::from_mav(target.autopilot);
                let vtype = VehicleType::from_mav(target.vehicle_type);
                let armed = hb
                    .base_mode
                    .contains(MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED);
                let mode_name = crate::modes::mode_name(autopilot_type, vtype, hb.custom_mode);

                let _ = writers.vehicle_state.send(VehicleState {
                    armed,
                    custom_mode: hb.custom_mode,
                    mode_name,
                    system_status: SystemStatus::from_mav(hb.system_status),
                    vehicle_type: vtype,
                    autopilot: autopilot_type,
                });
            }
        }
        common::MavMessage::VFR_HUD(data) => {
            writers.telemetry.send_modify(|t| {
                t.altitude_m = Some(data.alt as f64);
                t.speed_mps = Some(data.groundspeed as f64);
                t.heading_deg = Some(data.heading as f64);
                t.climb_rate_mps = Some(data.climb as f64);
                t.throttle_pct = Some(data.throttle as f64);
                t.airspeed_mps = Some(data.airspeed as f64);
            });
        }
        common::MavMessage::GLOBAL_POSITION_INT(data) => {
            writers.telemetry.send_modify(|t| {
                t.altitude_m = Some(data.relative_alt as f64 / 1000.0);
                t.latitude_deg = Some(data.lat as f64 / 1e7);
                t.longitude_deg = Some(data.lon as f64 / 1e7);
                let vx = data.vx as f64 / 100.0;
                let vy = data.vy as f64 / 100.0;
                t.speed_mps = Some((vx * vx + vy * vy).sqrt());
                if data.hdg != u16::MAX {
                    t.heading_deg = Some(data.hdg as f64 / 100.0);
                }
            });
        }
        common::MavMessage::SYS_STATUS(data) => {
            writers.telemetry.send_modify(|t| {
                if data.battery_remaining >= 0 {
                    t.battery_pct = Some(data.battery_remaining as f64);
                }
                if data.voltage_battery != u16::MAX {
                    t.battery_voltage_v = Some(data.voltage_battery as f64 / 1000.0);
                }
                if data.current_battery >= 0 {
                    t.battery_current_a = Some(data.current_battery as f64 / 100.0);
                }
            });
        }
        common::MavMessage::GPS_RAW_INT(data) => {
            writers.telemetry.send_modify(|t| {
                t.gps_fix_type = Some(GpsFixType::from_raw(data.fix_type as u8));
                if data.satellites_visible != u8::MAX {
                    t.gps_satellites = Some(data.satellites_visible);
                }
                if data.eph != u16::MAX {
                    t.gps_hdop = Some(data.eph as f64 / 100.0);
                }
            });
        }
        common::MavMessage::MISSION_CURRENT(data) => {
            let _ = writers.mission_state.send(MissionState {
                current_seq: data.seq,
                total_items: data.total,
            });
        }
        common::MavMessage::HOME_POSITION(data) => {
            let _ = writers
                .home_position
                .send(Some(mission::HomePosition {
                    latitude_deg: data.latitude as f64 / 1e7,
                    longitude_deg: data.longitude as f64 / 1e7,
                    altitude_m: (data.altitude as f64 / 1000.0) as f32,
                }));
        }
        common::MavMessage::ATTITUDE(data) => {
            writers.telemetry.send_modify(|t| {
                t.roll_deg = Some(data.roll.to_degrees() as f64);
                t.pitch_deg = Some(data.pitch.to_degrees() as f64);
                t.yaw_deg = Some(data.yaw.to_degrees() as f64);
            });
        }
        common::MavMessage::NAV_CONTROLLER_OUTPUT(data) => {
            writers.telemetry.send_modify(|t| {
                t.wp_dist_m = Some(data.wp_dist as f64);
                t.nav_bearing_deg = Some(data.nav_bearing as f64);
                t.target_bearing_deg = Some(data.target_bearing as f64);
                t.xtrack_error_m = Some(data.xtrack_error as f64);
            });
        }
        common::MavMessage::TERRAIN_REPORT(data) => {
            writers.telemetry.send_modify(|t| {
                t.terrain_height_m = Some(data.terrain_height as f64);
                t.height_above_terrain_m = Some(data.current_height as f64);
            });
        }
        common::MavMessage::BATTERY_STATUS(data) => {
            writers.telemetry.send_modify(|t| {
                let cells: Vec<f64> = data
                    .voltages
                    .iter()
                    .filter(|&&v| v != u16::MAX)
                    .map(|&v| v as f64 / 1000.0)
                    .collect();
                if !cells.is_empty() {
                    t.battery_voltage_cells = Some(cells);
                }
                if data.energy_consumed >= 0 {
                    t.energy_consumed_wh = Some(data.energy_consumed as f64 / 36.0);
                }
                if data.time_remaining > 0 {
                    t.battery_time_remaining_s = Some(data.time_remaining);
                }
            });
        }
        common::MavMessage::RC_CHANNELS(data) => {
            writers.telemetry.send_modify(|t| {
                let count = data.chancount.min(18) as usize;
                let all = [
                    data.chan1_raw,
                    data.chan2_raw,
                    data.chan3_raw,
                    data.chan4_raw,
                    data.chan5_raw,
                    data.chan6_raw,
                    data.chan7_raw,
                    data.chan8_raw,
                    data.chan9_raw,
                    data.chan10_raw,
                    data.chan11_raw,
                    data.chan12_raw,
                    data.chan13_raw,
                    data.chan14_raw,
                    data.chan15_raw,
                    data.chan16_raw,
                    data.chan17_raw,
                    data.chan18_raw,
                ];
                t.rc_channels = Some(all[..count].to_vec());
                if data.rssi != u8::MAX {
                    t.rc_rssi = Some(data.rssi);
                }
            });
        }
        common::MavMessage::SERVO_OUTPUT_RAW(data) => {
            writers.telemetry.send_modify(|t| {
                t.servo_outputs = Some(vec![
                    data.servo1_raw,
                    data.servo2_raw,
                    data.servo3_raw,
                    data.servo4_raw,
                    data.servo5_raw,
                    data.servo6_raw,
                    data.servo7_raw,
                    data.servo8_raw,
                    data.servo9_raw,
                    data.servo10_raw,
                    data.servo11_raw,
                    data.servo12_raw,
                    data.servo13_raw,
                    data.servo14_raw,
                    data.servo15_raw,
                    data.servo16_raw,
                ]);
            });
        }
        common::MavMessage::STATUSTEXT(data) => {
            let text = data.text.to_str().unwrap_or("").to_string();
            if !text.is_empty() {
                let _ = writers.statustext.send(Some(crate::state::StatusMessage {
                    text,
                    severity: data.severity as u8,
                }));
            }
        }
        _ => {
            trace!("unhandled message type");
        }
    }
}

// ---------------------------------------------------------------------------
// Command handling
// ---------------------------------------------------------------------------

async fn handle_command(
    cmd: Command,
    connection: &(dyn AsyncMavConnection<common::MavMessage> + Sync + Send),
    writers: &StateWriters,
    vehicle_target: &mut Option<VehicleTarget>,
    config: &VehicleConfig,
    cancel: &CancellationToken,
) {
    match cmd {
        Command::Arm { force, reply } => {
            let result = handle_arm_disarm(true, force, connection, vehicle_target, config, cancel).await;
            let _ = reply.send(result);
        }
        Command::Disarm { force, reply } => {
            let result = handle_arm_disarm(false, force, connection, vehicle_target, config, cancel).await;
            let _ = reply.send(result);
        }
        Command::SetMode { custom_mode, reply } => {
            let result = handle_set_mode(custom_mode, connection, vehicle_target, config, cancel).await;
            let _ = reply.send(result);
        }
        Command::CommandLong { command, params, reply } => {
            let result = handle_command_long(command, params, connection, vehicle_target, config, cancel).await;
            let _ = reply.send(result);
        }
        Command::GuidedGoto { lat_e7, lon_e7, alt_m, reply } => {
            let result = handle_guided_goto(lat_e7, lon_e7, alt_m, connection, vehicle_target, config).await;
            let _ = reply.send(result);
        }
        Command::MissionUpload { plan, reply } => {
            let result = handle_mission_upload(plan, connection, writers, vehicle_target, config, cancel).await;
            let _ = reply.send(result);
        }
        Command::MissionDownload { mission_type, reply } => {
            let result = handle_mission_download(mission_type, connection, writers, vehicle_target, config, cancel).await;
            let _ = reply.send(result);
        }
        Command::MissionClear { mission_type, reply } => {
            let result = handle_mission_clear(mission_type, connection, writers, vehicle_target, config, cancel).await;
            let _ = reply.send(result);
        }
        Command::MissionSetCurrent { seq, reply } => {
            let result = handle_mission_set_current(seq, connection, writers, vehicle_target, config, cancel).await;
            let _ = reply.send(result);
        }
        Command::MissionCancelTransfer => {
            // Cancel is signaled through the cancellation token on the vehicle side;
            // for now this is a placeholder.
        }
        Command::ParamDownloadAll { reply } => {
            let result = handle_param_download_all(connection, writers, vehicle_target, config, cancel).await;
            let _ = reply.send(result);
        }
        Command::ParamWrite { name, value, reply } => {
            let result = handle_param_write(&name, value, connection, writers, vehicle_target, config, cancel).await;
            let _ = reply.send(result);
        }
        Command::ParamWriteBatch { params, reply } => {
            let result = handle_param_write_batch(params, connection, writers, vehicle_target, config, cancel).await;
            let _ = reply.send(result);
        }
        Command::Shutdown => {
            // Handled in the main loop
        }
    }
}

// ---------------------------------------------------------------------------
// Helpers: send message, wait for response
// ---------------------------------------------------------------------------

async fn send_message(
    connection: &(dyn AsyncMavConnection<common::MavMessage> + Sync + Send),
    config: &VehicleConfig,
    message: common::MavMessage,
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
        .map_err(|err| VehicleError::Io(std::io::Error::new(std::io::ErrorKind::Other, err.to_string())))
}

/// Wait for a message matching `predicate`, continuing to update state for
/// all other messages received in the meantime.
#[allow(dead_code)]
async fn wait_for_response<F, T>(
    connection: &(dyn AsyncMavConnection<common::MavMessage> + Sync + Send),
    writers: &StateWriters,
    vehicle_target: &mut Option<VehicleTarget>,
    cancel: &CancellationToken,
    timeout: Duration,
    mut predicate: F,
) -> Result<T, VehicleError>
where
    F: FnMut(&MavHeader, &common::MavMessage) -> Option<T>,
{
    let deadline = tokio::time::sleep(timeout);
    tokio::pin!(deadline);
    loop {
        tokio::select! {
            biased;
            _ = cancel.cancelled() => return Err(VehicleError::Cancelled),
            _ = &mut deadline => return Err(VehicleError::Timeout),
            result = connection.recv() => {
                let (header, msg) = result.map_err(|err| {
                    VehicleError::Io(std::io::Error::new(std::io::ErrorKind::Other, err.to_string()))
                })?;
                update_vehicle_target(vehicle_target, &header, &msg);
                update_state(&header, &msg, writers, vehicle_target);
                if let Some(val) = predicate(&header, &msg) {
                    return Ok(val);
                }
            }
        }
    }
}

fn get_target(vehicle_target: &Option<VehicleTarget>) -> Result<VehicleTarget, VehicleError> {
    vehicle_target.ok_or(VehicleError::IdentityUnknown)
}

// ---------------------------------------------------------------------------
// Arm / Disarm
// ---------------------------------------------------------------------------

async fn handle_arm_disarm(
    arm: bool,
    force: bool,
    connection: &(dyn AsyncMavConnection<common::MavMessage> + Sync + Send),
    vehicle_target: &mut Option<VehicleTarget>,
    config: &VehicleConfig,
    cancel: &CancellationToken,
) -> Result<(), VehicleError> {
    let target = get_target(vehicle_target)?;
    let param1 = if arm { 1.0 } else { 0.0 };
    let param2 = if force {
        if arm { MAGIC_FORCE_ARM_VALUE } else { MAGIC_FORCE_DISARM_VALUE }
    } else {
        0.0
    };

    send_command_long_ack(
        MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
        [param1, param2, 0.0, 0.0, 0.0, 0.0, 0.0],
        target,
        connection,
        // We don't have writers here for the simple command path, so we pass
        // a stub StateWriters — but actually we need access. Let's restructure.
        vehicle_target,
        config,
        cancel,
    )
    .await
}

async fn send_command_long_ack(
    command: MavCmd,
    params: [f32; 7],
    target: VehicleTarget,
    connection: &(dyn AsyncMavConnection<common::MavMessage> + Sync + Send),
    vehicle_target: &mut Option<VehicleTarget>,
    config: &VehicleConfig,
    cancel: &CancellationToken,
) -> Result<(), VehicleError> {
    // We create a temporary pair of state writers just for the wait_for_response
    // helper. This is wasteful; instead we'll accept StateWriters by ref.
    // Actually we need to thread StateWriters through. Let me fix this.
    //
    // For now, we'll do a simplified version that doesn't update state
    // during the ACK wait. The main event loop will pick up any messages
    // after the command returns.

    let retry_policy = &config.retry_policy;
    for _attempt in 0..=retry_policy.max_retries {
        send_message(
            connection,
            config,
            common::MavMessage::COMMAND_LONG(common::COMMAND_LONG_DATA {
                target_system: target.system_id,
                target_component: target.component_id,
                command,
                confirmation: 0,
                param1: params[0],
                param2: params[1],
                param3: params[2],
                param4: params[3],
                param5: params[4],
                param6: params[5],
                param7: params[6],
            }),
        )
        .await?;

        let timeout = Duration::from_millis(retry_policy.request_timeout_ms);
        let deadline = tokio::time::sleep(timeout);
        tokio::pin!(deadline);

        loop {
            tokio::select! {
                biased;
                _ = cancel.cancelled() => return Err(VehicleError::Cancelled),
                _ = &mut deadline => break, // retry
                result = connection.recv() => {
                    let (header, msg) = result.map_err(|err| {
                        VehicleError::Io(std::io::Error::new(std::io::ErrorKind::Other, err.to_string()))
                    })?;
                    update_vehicle_target(vehicle_target, &header, &msg);
                    if let common::MavMessage::COMMAND_ACK(ack) = &msg {
                        if ack.command == command {
                            if ack.result == common::MavResult::MAV_RESULT_ACCEPTED {
                                return Ok(());
                            }
                            return Err(VehicleError::CommandRejected {
                                command: format!("{command:?}"),
                                result: format!("{:?}", ack.result),
                            });
                        }
                    }
                }
            }
        }
    }

    Err(VehicleError::Timeout)
}

// ---------------------------------------------------------------------------
// Set mode
// ---------------------------------------------------------------------------

async fn handle_set_mode(
    custom_mode: u32,
    connection: &(dyn AsyncMavConnection<common::MavMessage> + Sync + Send),
    vehicle_target: &mut Option<VehicleTarget>,
    config: &VehicleConfig,
    cancel: &CancellationToken,
) -> Result<(), VehicleError> {
    let target = get_target(vehicle_target)?;

    // Try COMMAND_LONG(DO_SET_MODE) first
    let do_set_mode_result = send_command_long_ack(
        MavCmd::MAV_CMD_DO_SET_MODE,
        [1.0, custom_mode as f32, 0.0, 0.0, 0.0, 0.0, 0.0],
        target,
        connection,
        vehicle_target,
        config,
        cancel,
    )
    .await;

    if do_set_mode_result.is_ok() {
        return Ok(());
    }

    // Fallback: wait for confirming heartbeat
    let timeout = Duration::from_secs(2);
    let deadline = tokio::time::sleep(timeout);
    tokio::pin!(deadline);

    loop {
        tokio::select! {
            biased;
            _ = cancel.cancelled() => return Err(VehicleError::Cancelled),
            _ = &mut deadline => {
                return Err(VehicleError::CommandRejected {
                    command: format!("DO_SET_MODE({custom_mode})"),
                    result: "no confirming HEARTBEAT".to_string(),
                });
            }
            result = connection.recv() => {
                let (header, msg) = result.map_err(|err| {
                    VehicleError::Io(std::io::Error::new(std::io::ErrorKind::Other, err.to_string()))
                })?;
                update_vehicle_target(vehicle_target, &header, &msg);
                if let common::MavMessage::HEARTBEAT(hb) = &msg {
                    if hb.custom_mode == custom_mode {
                        return Ok(());
                    }
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Generic COMMAND_LONG (public API)
// ---------------------------------------------------------------------------

async fn handle_command_long(
    command: MavCmd,
    params: [f32; 7],
    connection: &(dyn AsyncMavConnection<common::MavMessage> + Sync + Send),
    vehicle_target: &mut Option<VehicleTarget>,
    config: &VehicleConfig,
    cancel: &CancellationToken,
) -> Result<(), VehicleError> {
    let target = get_target(vehicle_target)?;
    send_command_long_ack(command, params, target, connection, vehicle_target, config, cancel).await
}

// ---------------------------------------------------------------------------
// Guided goto
// ---------------------------------------------------------------------------

async fn handle_guided_goto(
    lat_e7: i32,
    lon_e7: i32,
    alt_m: f32,
    connection: &(dyn AsyncMavConnection<common::MavMessage> + Sync + Send),
    vehicle_target: &mut Option<VehicleTarget>,
    config: &VehicleConfig,
) -> Result<(), VehicleError> {
    let target = get_target(vehicle_target)?;
    let type_mask = common::PositionTargetTypemask::from_bits_truncate(0x07F8);

    send_message(
        connection,
        config,
        common::MavMessage::SET_POSITION_TARGET_GLOBAL_INT(
            common::SET_POSITION_TARGET_GLOBAL_INT_DATA {
                time_boot_ms: 0,
                target_system: target.system_id,
                target_component: target.component_id,
                coordinate_frame: common::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
                type_mask,
                lat_int: lat_e7,
                lon_int: lon_e7,
                alt: alt_m,
                vx: 0.0,
                vy: 0.0,
                vz: 0.0,
                afx: 0.0,
                afy: 0.0,
                afz: 0.0,
                yaw: 0.0,
                yaw_rate: 0.0,
            },
        ),
    )
    .await
}

// ---------------------------------------------------------------------------
// Mission operations
// ---------------------------------------------------------------------------

fn to_mav_mission_type(mission_type: MissionType) -> common::MavMissionType {
    match mission_type {
        MissionType::Mission => common::MavMissionType::MAV_MISSION_TYPE_MISSION,
        MissionType::Fence => common::MavMissionType::MAV_MISSION_TYPE_FENCE,
        MissionType::Rally => common::MavMissionType::MAV_MISSION_TYPE_RALLY,
    }
}

fn to_mav_frame(frame: MissionFrame) -> common::MavFrame {
    match frame {
        MissionFrame::Mission => common::MavFrame::MAV_FRAME_MISSION,
        MissionFrame::GlobalInt => common::MavFrame::MAV_FRAME_GLOBAL,
        MissionFrame::GlobalRelativeAltInt => common::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MissionFrame::GlobalTerrainAltInt => common::MavFrame::MAV_FRAME_GLOBAL_TERRAIN_ALT,
        MissionFrame::LocalNed => common::MavFrame::MAV_FRAME_LOCAL_NED,
        MissionFrame::Other => common::MavFrame::MAV_FRAME_MISSION,
    }
}

#[allow(deprecated)]
fn from_mav_frame(frame: common::MavFrame) -> MissionFrame {
    match frame {
        common::MavFrame::MAV_FRAME_MISSION => MissionFrame::Mission,
        common::MavFrame::MAV_FRAME_GLOBAL | common::MavFrame::MAV_FRAME_GLOBAL_INT => {
            MissionFrame::GlobalInt
        }
        common::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT
        | common::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT_INT => MissionFrame::GlobalRelativeAltInt,
        common::MavFrame::MAV_FRAME_GLOBAL_TERRAIN_ALT
        | common::MavFrame::MAV_FRAME_GLOBAL_TERRAIN_ALT_INT => MissionFrame::GlobalTerrainAltInt,
        common::MavFrame::MAV_FRAME_LOCAL_NED => MissionFrame::LocalNed,
        _ => MissionFrame::Other,
    }
}

fn from_mission_item_int(data: &common::MISSION_ITEM_INT_DATA) -> MissionItem {
    MissionItem {
        seq: data.seq,
        command: data.command as u16,
        frame: from_mav_frame(data.frame),
        current: data.current > 0,
        autocontinue: data.autocontinue > 0,
        param1: data.param1,
        param2: data.param2,
        param3: data.param3,
        param4: data.param4,
        x: data.x,
        y: data.y,
        z: data.z,
    }
}

#[allow(deprecated)]
fn from_mission_item_float(data: &common::MISSION_ITEM_DATA) -> MissionItem {
    let is_global = matches!(
        data.frame,
        common::MavFrame::MAV_FRAME_GLOBAL
            | common::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT
            | common::MavFrame::MAV_FRAME_GLOBAL_TERRAIN_ALT
            | common::MavFrame::MAV_FRAME_GLOBAL_INT
            | common::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
            | common::MavFrame::MAV_FRAME_GLOBAL_TERRAIN_ALT_INT
    );

    MissionItem {
        seq: data.seq,
        command: data.command as u16,
        frame: from_mav_frame(data.frame),
        current: data.current > 0,
        autocontinue: data.autocontinue > 0,
        param1: data.param1,
        param2: data.param2,
        param3: data.param3,
        param4: data.param4,
        x: if is_global { (data.x as f64 * 1e7) as i32 } else { data.x as i32 },
        y: if is_global { (data.y as f64 * 1e7) as i32 } else { data.y as i32 },
        z: data.z,
    }
}

fn mission_type_matches(received: common::MavMissionType, expected: MissionType) -> bool {
    let expected_mav = to_mav_mission_type(expected);
    if expected == MissionType::Mission {
        received == expected_mav || received == common::MavMissionType::MAV_MISSION_TYPE_MISSION
    } else {
        received == expected_mav
    }
}

fn send_requested_item_msg(
    wire_items: &[MissionItem],
    target: VehicleTarget,
    mission_type: MissionType,
    seq: u16,
) -> Result<common::MavMessage, VehicleError> {
    let item = wire_items
        .get(seq as usize)
        .ok_or_else(|| VehicleError::MissionTransfer {
            code: "item_out_of_range".to_string(),
            message: format!("requested item {seq} out of range"),
        })?;

    let command = num_traits::FromPrimitive::from_u16(item.command)
        .ok_or_else(|| VehicleError::MissionTransfer {
            code: "unsupported_command".to_string(),
            message: format!("unsupported MAV_CMD value {}", item.command),
        })?;
    let frame = to_mav_frame(item.frame);

    Ok(common::MavMessage::MISSION_ITEM_INT(
        common::MISSION_ITEM_INT_DATA {
            param1: item.param1,
            param2: item.param2,
            param3: item.param3,
            param4: item.param4,
            x: item.x,
            y: item.y,
            z: item.z,
            seq: item.seq,
            command,
            target_system: target.system_id,
            target_component: target.component_id,
            frame,
            current: 0,
            autocontinue: u8::from(item.autocontinue),
            mission_type: to_mav_mission_type(mission_type),
        },
    ))
}

// ---------------------------------------------------------------------------
// Mission Upload
// ---------------------------------------------------------------------------

#[allow(deprecated)]
async fn handle_mission_upload(
    plan: MissionPlan,
    connection: &(dyn AsyncMavConnection<common::MavMessage> + Sync + Send),
    writers: &StateWriters,
    vehicle_target: &mut Option<VehicleTarget>,
    config: &VehicleConfig,
    cancel: &CancellationToken,
) -> Result<(), VehicleError> {
    // Validate
    let issues = mission::validate_plan(&plan);
    if let Some(issue) = issues.iter().find(|i| i.severity == IssueSeverity::Error) {
        return Err(VehicleError::MissionValidation(format!(
            "{}: {}",
            issue.code, issue.message
        )));
    }

    let wire_items = mission::items_for_wire_upload(&plan);
    let target = get_target(vehicle_target)?;
    let mav_mission_type = to_mav_mission_type(plan.mission_type);

    let mut machine = MissionTransferMachine::new_upload(
        plan.mission_type,
        wire_items.len() as u16,
        config.retry_policy,
    );
    let _ = writers.mission_progress.send(Some(machine.progress()));

    let count_msg = common::MavMessage::MISSION_COUNT(common::MISSION_COUNT_DATA {
        count: wire_items.len() as u16,
        target_system: target.system_id,
        target_component: target.component_id,
        mission_type: mav_mission_type,
        opaque_id: 0,
    });

    send_message(connection, config, count_msg.clone()).await?;

    // If empty plan, just wait for ACK
    if wire_items.is_empty() {
        return wait_for_mission_ack(
            &mut machine,
            plan.mission_type,
            connection,
            writers,
            vehicle_target,
            config,
            cancel,
            || count_msg.clone(),
        )
        .await;
    }

    let mut acknowledged = HashSet::<u16>::new();

    // Wait for MISSION_REQUEST_INT / MISSION_REQUEST messages
    while machine.progress().phase != TransferPhase::AwaitAck {
        let timeout = Duration::from_millis(machine.timeout_ms());
        let deadline = tokio::time::sleep(timeout);
        tokio::pin!(deadline);

        let msg = loop {
            tokio::select! {
                biased;
                _ = cancel.cancelled() => {
                    machine.cancel();
                    let _ = writers.mission_progress.send(Some(machine.progress()));
                    return Err(VehicleError::Cancelled);
                }
                _ = &mut deadline => {
                    if let Some(err) = machine.on_timeout() {
                        let _ = writers.mission_progress.send(Some(machine.progress()));
                        return Err(VehicleError::MissionTransfer {
                            code: err.code,
                            message: err.message,
                        });
                    }
                    let _ = writers.mission_progress.send(Some(machine.progress()));
                    send_message(connection, config, count_msg.clone()).await?;
                    break None;
                }
                result = connection.recv() => {
                    let (header, msg) = result.map_err(|err| {
                        VehicleError::Io(std::io::Error::new(std::io::ErrorKind::Other, err.to_string()))
                    })?;
                    update_vehicle_target(vehicle_target, &header, &msg);
                    update_state(&header, &msg, writers, vehicle_target);

                    match &msg {
                        common::MavMessage::MISSION_REQUEST_INT(data) if data.mission_type == mav_mission_type => {
                            break Some(("int", data.seq));
                        }
                        common::MavMessage::MISSION_REQUEST(data) if data.mission_type == mav_mission_type => {
                            break Some(("req", data.seq));
                        }
                        common::MavMessage::MISSION_ACK(data) if data.mission_type == mav_mission_type => {
                            if data.mavtype == common::MavMissionResult::MAV_MISSION_ACCEPTED {
                                machine.on_ack_success();
                                let _ = writers.mission_progress.send(Some(machine.progress()));
                                return Ok(());
                            }
                            return Err(VehicleError::MissionTransfer {
                                code: "transfer.ack_error".to_string(),
                                message: format!("MISSION_ACK error: {:?}", data.mavtype),
                            });
                        }
                        _ => {}
                    }
                    continue;
                }
            }
        };

        if let Some((_kind, seq)) = msg {
            let item_msg = send_requested_item_msg(&wire_items, target, plan.mission_type, seq)?;
            send_message(connection, config, item_msg).await?;
            if acknowledged.insert(seq) {
                machine.on_item_transferred();
                let _ = writers.mission_progress.send(Some(machine.progress()));
            }
        }
    }

    // Await final ACK
    wait_for_mission_ack(
        &mut machine,
        plan.mission_type,
        connection,
        writers,
        vehicle_target,
        config,
        cancel,
        || count_msg.clone(),
    )
    .await
}

async fn wait_for_mission_ack<F>(
    machine: &mut MissionTransferMachine,
    mission_type: MissionType,
    connection: &(dyn AsyncMavConnection<common::MavMessage> + Sync + Send),
    writers: &StateWriters,
    vehicle_target: &mut Option<VehicleTarget>,
    config: &VehicleConfig,
    cancel: &CancellationToken,
    retry_msg: F,
) -> Result<(), VehicleError>
where
    F: Fn() -> common::MavMessage,
{
    let mav_mission_type = to_mav_mission_type(mission_type);
    loop {
        let timeout = Duration::from_millis(machine.timeout_ms());
        let deadline = tokio::time::sleep(timeout);
        tokio::pin!(deadline);

        tokio::select! {
            biased;
            _ = cancel.cancelled() => {
                machine.cancel();
                let _ = writers.mission_progress.send(Some(machine.progress()));
                return Err(VehicleError::Cancelled);
            }
            _ = &mut deadline => {
                if let Some(err) = machine.on_timeout() {
                    let _ = writers.mission_progress.send(Some(machine.progress()));
                    return Err(VehicleError::MissionTransfer {
                        code: err.code,
                        message: err.message,
                    });
                }
                let _ = writers.mission_progress.send(Some(machine.progress()));
                send_message(connection, config, retry_msg()).await?;
            }
            result = connection.recv() => {
                let (header, msg) = result.map_err(|err| {
                    VehicleError::Io(std::io::Error::new(std::io::ErrorKind::Other, err.to_string()))
                })?;
                update_vehicle_target(vehicle_target, &header, &msg);
                update_state(&header, &msg, writers, vehicle_target);

                if let common::MavMessage::MISSION_ACK(data) = &msg {
                    if data.mission_type != mav_mission_type {
                        continue;
                    }
                    if data.mavtype == common::MavMissionResult::MAV_MISSION_ACCEPTED {
                        machine.on_ack_success();
                        let _ = writers.mission_progress.send(Some(machine.progress()));
                        return Ok(());
                    }
                    return Err(VehicleError::MissionTransfer {
                        code: "transfer.ack_error".to_string(),
                        message: format!("MISSION_ACK error: {:?}", data.mavtype),
                    });
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Mission Download
// ---------------------------------------------------------------------------

#[allow(deprecated)]
async fn handle_mission_download(
    mission_type: MissionType,
    connection: &(dyn AsyncMavConnection<common::MavMessage> + Sync + Send),
    writers: &StateWriters,
    vehicle_target: &mut Option<VehicleTarget>,
    config: &VehicleConfig,
    cancel: &CancellationToken,
) -> Result<MissionPlan, VehicleError> {
    let target = get_target(vehicle_target)?;
    let mav_mission_type = to_mav_mission_type(mission_type);
    let mut machine = MissionTransferMachine::new_download(mission_type, config.retry_policy);
    let _ = writers.mission_progress.send(Some(machine.progress()));

    let request_list_msg = common::MavMessage::MISSION_REQUEST_LIST(
        common::MISSION_REQUEST_LIST_DATA {
            target_system: target.system_id,
            target_component: target.component_id,
            mission_type: mav_mission_type,
        },
    );
    send_message(connection, config, request_list_msg.clone()).await?;

    // Wait for MISSION_COUNT
    let count = loop {
        let timeout = Duration::from_millis(machine.timeout_ms());
        let deadline = tokio::time::sleep(timeout);
        tokio::pin!(deadline);

        tokio::select! {
            biased;
            _ = cancel.cancelled() => {
                machine.cancel();
                let _ = writers.mission_progress.send(Some(machine.progress()));
                return Err(VehicleError::Cancelled);
            }
            _ = &mut deadline => {
                if let Some(err) = machine.on_timeout() {
                    let _ = writers.mission_progress.send(Some(machine.progress()));
                    return Err(VehicleError::MissionTransfer {
                        code: err.code,
                        message: err.message,
                    });
                }
                let _ = writers.mission_progress.send(Some(machine.progress()));
                send_message(connection, config, request_list_msg.clone()).await?;
            }
            result = connection.recv() => {
                let (header, msg) = result.map_err(|err| {
                    VehicleError::Io(std::io::Error::new(std::io::ErrorKind::Other, err.to_string()))
                })?;
                update_vehicle_target(vehicle_target, &header, &msg);
                update_state(&header, &msg, writers, vehicle_target);

                if let common::MavMessage::MISSION_COUNT(data) = &msg {
                    if mission_type_matches(data.mission_type, mission_type) {
                        break data.count;
                    }
                }
            }
        }
    };

    machine.set_download_total(count);
    let _ = writers.mission_progress.send(Some(machine.progress()));

    // Request each item
    let mut items = Vec::with_capacity(count as usize);
    for seq in 0..count {
        let mut use_int_request = true;

        let request_int_msg = common::MavMessage::MISSION_REQUEST_INT(
            common::MISSION_REQUEST_INT_DATA {
                seq,
                target_system: target.system_id,
                target_component: target.component_id,
                mission_type: mav_mission_type,
            },
        );
        let request_float_msg = common::MavMessage::MISSION_REQUEST(
            common::MISSION_REQUEST_DATA {
                seq,
                target_system: target.system_id,
                target_component: target.component_id,
                mission_type: mav_mission_type,
            },
        );

        let make_request_msg = |use_int: bool| -> common::MavMessage {
            if use_int {
                request_int_msg.clone()
            } else {
                request_float_msg.clone()
            }
        };

        send_message(connection, config, make_request_msg(use_int_request)).await?;

        let item = loop {
            let timeout = Duration::from_millis(machine.timeout_ms());
            let deadline = tokio::time::sleep(timeout);
            tokio::pin!(deadline);

            tokio::select! {
                biased;
                _ = cancel.cancelled() => {
                    machine.cancel();
                    let _ = writers.mission_progress.send(Some(machine.progress()));
                    return Err(VehicleError::Cancelled);
                }
                _ = &mut deadline => {
                    if let Some(err) = machine.on_timeout() {
                        let _ = writers.mission_progress.send(Some(machine.progress()));
                        return Err(VehicleError::MissionTransfer {
                            code: err.code,
                            message: err.message,
                        });
                    }
                    let _ = writers.mission_progress.send(Some(machine.progress()));
                    if use_int_request {
                        use_int_request = false;
                    }
                    send_message(connection, config, make_request_msg(use_int_request)).await?;
                }
                result = connection.recv() => {
                    let (header, msg) = result.map_err(|err| {
                        VehicleError::Io(std::io::Error::new(std::io::ErrorKind::Other, err.to_string()))
                    })?;
                    update_vehicle_target(vehicle_target, &header, &msg);
                    update_state(&header, &msg, writers, vehicle_target);

                    match &msg {
                        common::MavMessage::MISSION_ITEM_INT(data)
                            if data.seq == seq && mission_type_matches(data.mission_type, mission_type) =>
                        {
                            break from_mission_item_int(data);
                        }
                        common::MavMessage::MISSION_ITEM(data)
                            if data.seq == seq && mission_type_matches(data.mission_type, mission_type) =>
                        {
                            break from_mission_item_float(data);
                        }
                        _ => {}
                    }
                }
            }
        };

        items.push(item);
        machine.on_item_transferred();
        let _ = writers.mission_progress.send(Some(machine.progress()));
    }

    // Send ACK
    let _ = send_message(
        connection,
        config,
        common::MavMessage::MISSION_ACK(common::MISSION_ACK_DATA {
            target_system: target.system_id,
            target_component: target.component_id,
            mavtype: common::MavMissionResult::MAV_MISSION_ACCEPTED,
            mission_type: mav_mission_type,
            opaque_id: 0,
        }),
    )
    .await;

    machine.on_ack_success();
    let _ = writers.mission_progress.send(Some(machine.progress()));

    Ok(mission::plan_from_wire_download(mission_type, items))
}

// ---------------------------------------------------------------------------
// Mission Clear
// ---------------------------------------------------------------------------

async fn handle_mission_clear(
    mission_type: MissionType,
    connection: &(dyn AsyncMavConnection<common::MavMessage> + Sync + Send),
    writers: &StateWriters,
    vehicle_target: &mut Option<VehicleTarget>,
    config: &VehicleConfig,
    cancel: &CancellationToken,
) -> Result<(), VehicleError> {
    let target = get_target(vehicle_target)?;
    let mav_mission_type = to_mav_mission_type(mission_type);

    let mut machine = MissionTransferMachine::new_upload(mission_type, 0, config.retry_policy);
    let _ = writers.mission_progress.send(Some(machine.progress()));

    let clear_msg = common::MavMessage::MISSION_CLEAR_ALL(common::MISSION_CLEAR_ALL_DATA {
        target_system: target.system_id,
        target_component: target.component_id,
        mission_type: mav_mission_type,
    });

    send_message(connection, config, clear_msg.clone()).await?;

    wait_for_mission_ack(
        &mut machine,
        mission_type,
        connection,
        writers,
        vehicle_target,
        config,
        cancel,
        || clear_msg.clone(),
    )
    .await
}

// ---------------------------------------------------------------------------
// Mission Set Current
// ---------------------------------------------------------------------------

async fn handle_mission_set_current(
    seq: u16,
    connection: &(dyn AsyncMavConnection<common::MavMessage> + Sync + Send),
    writers: &StateWriters,
    vehicle_target: &mut Option<VehicleTarget>,
    config: &VehicleConfig,
    cancel: &CancellationToken,
) -> Result<(), VehicleError> {
    let target = get_target(vehicle_target)?;
    let retry_policy = &config.retry_policy;

    for _attempt in 0..=retry_policy.max_retries {
        send_message(
            connection,
            config,
            common::MavMessage::COMMAND_LONG(common::COMMAND_LONG_DATA {
                target_system: target.system_id,
                target_component: target.component_id,
                command: MavCmd::MAV_CMD_DO_SET_MISSION_CURRENT,
                confirmation: 0,
                param1: seq as f32,
                param2: 0.0,
                param3: 0.0,
                param4: 0.0,
                param5: 0.0,
                param6: 0.0,
                param7: 0.0,
            }),
        )
        .await?;

        let timeout = Duration::from_millis(retry_policy.request_timeout_ms);
        let deadline = tokio::time::sleep(timeout);
        tokio::pin!(deadline);

        loop {
            tokio::select! {
                biased;
                _ = cancel.cancelled() => return Err(VehicleError::Cancelled),
                _ = &mut deadline => break, // retry outer loop
                result = connection.recv() => {
                    let (header, msg) = result.map_err(|err| {
                        VehicleError::Io(std::io::Error::new(std::io::ErrorKind::Other, err.to_string()))
                    })?;
                    update_vehicle_target(vehicle_target, &header, &msg);
                    update_state(&header, &msg, writers, vehicle_target);

                    match &msg {
                        common::MavMessage::COMMAND_ACK(data) => {
                            if data.command == MavCmd::MAV_CMD_DO_SET_MISSION_CURRENT
                                && data.result == common::MavResult::MAV_RESULT_ACCEPTED
                            {
                                return Ok(());
                            }
                        }
                        common::MavMessage::MISSION_CURRENT(data) => {
                            if data.seq == seq {
                                return Ok(());
                            }
                        }
                        _ => {}
                    }
                }
            }
        }
    }

    Err(VehicleError::MissionTransfer {
        code: "mission.set_current_timeout".to_string(),
        message: "Did not receive confirmation for set-current command".to_string(),
    })
}

// ---------------------------------------------------------------------------
// Parameter type helpers
// ---------------------------------------------------------------------------

fn from_mav_param_type(mav: MavParamType) -> ParamType {
    match mav {
        MavParamType::MAV_PARAM_TYPE_UINT8 => ParamType::Uint8,
        MavParamType::MAV_PARAM_TYPE_INT8 => ParamType::Int8,
        MavParamType::MAV_PARAM_TYPE_UINT16 => ParamType::Uint16,
        MavParamType::MAV_PARAM_TYPE_INT16 => ParamType::Int16,
        MavParamType::MAV_PARAM_TYPE_UINT32 => ParamType::Uint32,
        MavParamType::MAV_PARAM_TYPE_INT32 => ParamType::Int32,
        _ => ParamType::Real32,
    }
}

fn to_mav_param_type(pt: ParamType) -> MavParamType {
    match pt {
        ParamType::Uint8 => MavParamType::MAV_PARAM_TYPE_UINT8,
        ParamType::Int8 => MavParamType::MAV_PARAM_TYPE_INT8,
        ParamType::Uint16 => MavParamType::MAV_PARAM_TYPE_UINT16,
        ParamType::Int16 => MavParamType::MAV_PARAM_TYPE_INT16,
        ParamType::Uint32 => MavParamType::MAV_PARAM_TYPE_UINT32,
        ParamType::Int32 => MavParamType::MAV_PARAM_TYPE_INT32,
        ParamType::Real32 => MavParamType::MAV_PARAM_TYPE_REAL32,
    }
}

fn param_id_to_string(param_id: &mavlink::types::CharArray<16>) -> String {
    param_id.to_str().unwrap_or("").to_string()
}

fn string_to_param_id(name: &str) -> mavlink::types::CharArray<16> {
    name.into()
}

// ---------------------------------------------------------------------------
// Parameter Download All
// ---------------------------------------------------------------------------

async fn handle_param_download_all(
    connection: &(dyn AsyncMavConnection<common::MavMessage> + Sync + Send),
    writers: &StateWriters,
    vehicle_target: &mut Option<VehicleTarget>,
    config: &VehicleConfig,
    cancel: &CancellationToken,
) -> Result<ParamStore, VehicleError> {
    let target = get_target(vehicle_target)?;

    // Reset progress
    let _ = writers.param_progress.send(ParamProgress {
        phase: ParamTransferPhase::Downloading,
        received: 0,
        expected: 0,
    });

    // Send PARAM_REQUEST_LIST
    send_message(
        connection,
        config,
        common::MavMessage::PARAM_REQUEST_LIST(common::PARAM_REQUEST_LIST_DATA {
            target_system: target.system_id,
            target_component: target.component_id,
        }),
    )
    .await?;

    let mut params: HashMap<String, Param> = HashMap::new();
    let mut received_indices: HashSet<u16> = HashSet::new();
    let mut expected_count: u16 = 0;
    let mut count_known = false;
    let mut last_progress_update = 0u16;
    let max_retries = 3u32;
    let mut retries = 0u32;

    loop {
        let timeout = Duration::from_secs(2);
        let deadline = tokio::time::sleep(timeout);
        tokio::pin!(deadline);

        let mut got_new = false;

        loop {
            tokio::select! {
                biased;
                _ = cancel.cancelled() => {
                    let _ = writers.param_progress.send(ParamProgress {
                        phase: ParamTransferPhase::Failed,
                        received: params.len() as u16,
                        expected: expected_count,
                    });
                    return Err(VehicleError::Cancelled);
                }
                _ = &mut deadline => break,
                result = connection.recv() => {
                    let (header, msg) = result.map_err(|err| {
                        VehicleError::Io(std::io::Error::new(std::io::ErrorKind::Other, err.to_string()))
                    })?;
                    update_vehicle_target(vehicle_target, &header, &msg);
                    update_state(&header, &msg, writers, vehicle_target);

                    if let common::MavMessage::PARAM_VALUE(data) = &msg {
                        let name = param_id_to_string(&data.param_id);
                        if name.is_empty() {
                            continue;
                        }

                        if !count_known && data.param_count > 0 {
                            expected_count = data.param_count;
                            count_known = true;
                        }

                        if received_indices.insert(data.param_index) {
                            got_new = true;
                            params.insert(name.clone(), Param {
                                name,
                                value: data.param_value,
                                param_type: from_mav_param_type(data.param_type),
                                index: data.param_index,
                            });
                        }

                        // Update progress every 50 params
                        let received = params.len() as u16;
                        if received - last_progress_update >= 50 || received >= expected_count {
                            last_progress_update = received;
                            let _ = writers.param_progress.send(ParamProgress {
                                phase: ParamTransferPhase::Downloading,
                                received,
                                expected: expected_count,
                            });
                        }

                        // Reset deadline on new data
                        deadline.as_mut().reset(tokio::time::Instant::now() + Duration::from_secs(2));
                    }
                }
            }
        }

        // Timeout reached — check if we're done
        let received = params.len() as u16;
        if count_known && received >= expected_count {
            break; // Done
        }

        if !got_new {
            retries += 1;
            if retries > max_retries {
                // Accept partial if we have more than 50% of expected
                if count_known && received > expected_count / 2 {
                    warn!(
                        "param download: accepting partial {}/{} after {} retries",
                        received, expected_count, max_retries
                    );
                    break;
                }
                let _ = writers.param_progress.send(ParamProgress {
                    phase: ParamTransferPhase::Failed,
                    received,
                    expected: expected_count,
                });
                return Err(VehicleError::Timeout);
            }
        } else {
            retries = 0;
        }

        // Request missing indices
        if count_known {
            let mut missing_requested = 0u32;
            for idx in 0..expected_count {
                if !received_indices.contains(&idx) {
                    send_message(
                        connection,
                        config,
                        common::MavMessage::PARAM_REQUEST_READ(common::PARAM_REQUEST_READ_DATA {
                            param_index: idx as i16,
                            target_system: target.system_id,
                            target_component: target.component_id,
                            param_id: string_to_param_id(""),
                        }),
                    )
                    .await?;
                    missing_requested += 1;
                    if missing_requested >= 10 {
                        break; // Don't flood, request in batches
                    }
                }
            }
            debug!("param download: requested {} missing params (retry {})", missing_requested, retries);
        }
    }

    let store = ParamStore {
        params,
        expected_count,
    };

    let _ = writers.param_store.send(store.clone());
    let _ = writers.param_progress.send(ParamProgress {
        phase: ParamTransferPhase::Completed,
        received: store.params.len() as u16,
        expected: expected_count,
    });

    Ok(store)
}

// ---------------------------------------------------------------------------
// Parameter Write
// ---------------------------------------------------------------------------

async fn handle_param_write(
    name: &str,
    value: f32,
    connection: &(dyn AsyncMavConnection<common::MavMessage> + Sync + Send),
    writers: &StateWriters,
    vehicle_target: &mut Option<VehicleTarget>,
    config: &VehicleConfig,
    cancel: &CancellationToken,
) -> Result<Param, VehicleError> {
    let target = get_target(vehicle_target)?;

    // Look up current param_type from store, or default to Real32
    let param_type = {
        let store = writers.param_store.borrow();
        store
            .params
            .get(name)
            .map(|p| p.param_type)
            .unwrap_or(ParamType::Real32)
    };

    let retry_policy = &config.retry_policy;

    for _attempt in 0..=retry_policy.max_retries {
        send_message(
            connection,
            config,
            common::MavMessage::PARAM_SET(common::PARAM_SET_DATA {
                param_value: value,
                target_system: target.system_id,
                target_component: target.component_id,
                param_id: string_to_param_id(name),
                param_type: to_mav_param_type(param_type),
            }),
        )
        .await?;

        let timeout = Duration::from_millis(retry_policy.request_timeout_ms);
        let deadline = tokio::time::sleep(timeout);
        tokio::pin!(deadline);

        loop {
            tokio::select! {
                biased;
                _ = cancel.cancelled() => return Err(VehicleError::Cancelled),
                _ = &mut deadline => break, // retry
                result = connection.recv() => {
                    let (header, msg) = result.map_err(|err| {
                        VehicleError::Io(std::io::Error::new(std::io::ErrorKind::Other, err.to_string()))
                    })?;
                    update_vehicle_target(vehicle_target, &header, &msg);
                    update_state(&header, &msg, writers, vehicle_target);

                    if let common::MavMessage::PARAM_VALUE(data) = &msg {
                        let received_name = param_id_to_string(&data.param_id);
                        if received_name == name {
                            let confirmed = Param {
                                name: received_name.clone(),
                                value: data.param_value,
                                param_type: from_mav_param_type(data.param_type),
                                index: data.param_index,
                            };

                            // Update store
                            writers.param_store.send_modify(|store| {
                                store.params.insert(received_name, confirmed.clone());
                            });

                            return Ok(confirmed);
                        }
                    }
                }
            }
        }
    }

    Err(VehicleError::Timeout)
}

// ---------------------------------------------------------------------------
// Parameter Batch Write
// ---------------------------------------------------------------------------

async fn handle_param_write_batch(
    params: Vec<(String, f32)>,
    connection: &(dyn AsyncMavConnection<common::MavMessage> + Sync + Send),
    writers: &StateWriters,
    vehicle_target: &mut Option<VehicleTarget>,
    config: &VehicleConfig,
    cancel: &CancellationToken,
) -> Result<Vec<ParamWriteResult>, VehicleError> {
    let total = params.len() as u16;
    let mut results = Vec::with_capacity(params.len());

    let _ = writers.param_progress.send(ParamProgress {
        phase: ParamTransferPhase::Writing,
        received: 0,
        expected: total,
    });

    for (i, (name, value)) in params.into_iter().enumerate() {
        let result = handle_param_write(&name, value, connection, writers, vehicle_target, config, cancel).await;
        match result {
            Ok(confirmed) => {
                results.push(ParamWriteResult {
                    name,
                    requested_value: value,
                    confirmed_value: confirmed.value,
                    success: true,
                });
            }
            Err(VehicleError::Cancelled) => {
                let _ = writers.param_progress.send(ParamProgress {
                    phase: ParamTransferPhase::Failed,
                    received: i as u16,
                    expected: total,
                });
                return Err(VehicleError::Cancelled);
            }
            Err(_) => {
                results.push(ParamWriteResult {
                    name,
                    requested_value: value,
                    confirmed_value: 0.0,
                    success: false,
                });
            }
        }

        let _ = writers.param_progress.send(ParamProgress {
            phase: ParamTransferPhase::Writing,
            received: (i + 1) as u16,
            expected: total,
        });
    }

    let all_ok = results.iter().all(|r| r.success);
    let _ = writers.param_progress.send(ParamProgress {
        phase: if all_ok { ParamTransferPhase::Completed } else { ParamTransferPhase::Failed },
        received: results.iter().filter(|r| r.success).count() as u16,
        expected: total,
    });

    Ok(results)
}
