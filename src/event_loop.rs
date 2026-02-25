use crate::command::Command;
use crate::config::VehicleConfig;
use crate::error::VehicleError;
use crate::mission::{
    self, IssueSeverity, MissionFrame, MissionItem, MissionPlan, MissionTransferMachine,
    MissionType, TransferPhase,
};
use crate::params::{
    Param, ParamProgress, ParamStore, ParamTransferPhase, ParamType, ParamWriteResult,
};
use crate::state::{
    AutopilotType, GpsFixType, LinkState, MissionState, StateWriters, SystemStatus, VehicleState,
    VehicleType, set_if_changed,
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
const HOME_POSITION_MSG_ID: f32 = 242.0;

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
                        if !home_requested
                            && config.auto_request_home
                            && let Some(ref target) = vehicle_target
                        {
                            request_home_position(&*connection, target, &config).await;
                            home_requested = true;
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
                param1: HOME_POSITION_MSG_ID,
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
                    system_id: target.system_id,
                    component_id: target.component_id,
                });
            }
        }
        common::MavMessage::VFR_HUD(data) => {
            let alt = Some(data.alt as f64);
            let spd = Some(data.groundspeed as f64);
            let hdg = Some(data.heading as f64);
            let climb = Some(data.climb as f64);
            let thr = Some(data.throttle as f64);
            let air = Some(data.airspeed as f64);
            writers.telemetry.send_if_modified(|t| {
                let mut c = false;
                c |= set_if_changed(&mut t.altitude_m, alt);
                c |= set_if_changed(&mut t.speed_mps, spd);
                c |= set_if_changed(&mut t.heading_deg, hdg);
                c |= set_if_changed(&mut t.climb_rate_mps, climb);
                c |= set_if_changed(&mut t.throttle_pct, thr);
                c |= set_if_changed(&mut t.airspeed_mps, air);
                c
            });
            writers.position.send_if_modified(|p| {
                let mut c = false;
                c |= set_if_changed(&mut p.altitude_m, alt);
                c |= set_if_changed(&mut p.speed_mps, spd);
                c |= set_if_changed(&mut p.heading_deg, hdg);
                c |= set_if_changed(&mut p.climb_rate_mps, climb);
                c |= set_if_changed(&mut p.throttle_pct, thr);
                c |= set_if_changed(&mut p.airspeed_mps, air);
                c
            });
        }
        common::MavMessage::GLOBAL_POSITION_INT(data) => {
            let alt = Some(data.relative_alt as f64 / 1000.0);
            let lat = Some(data.lat as f64 / 1e7);
            let lon = Some(data.lon as f64 / 1e7);
            let vx = data.vx as f64 / 100.0;
            let vy = data.vy as f64 / 100.0;
            let spd = Some((vx * vx + vy * vy).sqrt());
            let hdg = if data.hdg != u16::MAX {
                Some(data.hdg as f64 / 100.0)
            } else {
                None
            };
            writers.telemetry.send_if_modified(|t| {
                let mut c = false;
                c |= set_if_changed(&mut t.altitude_m, alt);
                c |= set_if_changed(&mut t.latitude_deg, lat);
                c |= set_if_changed(&mut t.longitude_deg, lon);
                c |= set_if_changed(&mut t.speed_mps, spd);
                if let Some(h) = hdg {
                    c |= set_if_changed(&mut t.heading_deg, Some(h));
                }
                c
            });
            writers.position.send_if_modified(|p| {
                let mut c = false;
                c |= set_if_changed(&mut p.altitude_m, alt);
                c |= set_if_changed(&mut p.latitude_deg, lat);
                c |= set_if_changed(&mut p.longitude_deg, lon);
                c |= set_if_changed(&mut p.speed_mps, spd);
                if let Some(h) = hdg {
                    c |= set_if_changed(&mut p.heading_deg, Some(h));
                }
                c
            });
        }
        common::MavMessage::SYS_STATUS(data) => {
            let pct = if data.battery_remaining >= 0 {
                Some(data.battery_remaining as f64)
            } else {
                None
            };
            let volt = if data.voltage_battery != u16::MAX {
                Some(data.voltage_battery as f64 / 1000.0)
            } else {
                None
            };
            let cur = if data.current_battery >= 0 {
                Some(data.current_battery as f64 / 100.0)
            } else {
                None
            };
            writers.telemetry.send_if_modified(|t| {
                let mut c = false;
                if let Some(v) = pct {
                    c |= set_if_changed(&mut t.battery_pct, Some(v));
                }
                if let Some(v) = volt {
                    c |= set_if_changed(&mut t.battery_voltage_v, Some(v));
                }
                if let Some(v) = cur {
                    c |= set_if_changed(&mut t.battery_current_a, Some(v));
                }
                c
            });
            writers.battery.send_if_modified(|b| {
                let mut c = false;
                if let Some(v) = pct {
                    c |= set_if_changed(&mut b.remaining_pct, Some(v));
                }
                if let Some(v) = volt {
                    c |= set_if_changed(&mut b.voltage_v, Some(v));
                }
                if let Some(v) = cur {
                    c |= set_if_changed(&mut b.current_a, Some(v));
                }
                c
            });
        }
        common::MavMessage::GPS_RAW_INT(data) => {
            let fix = Some(GpsFixType::from_raw(data.fix_type as u8));
            let sats = if data.satellites_visible != u8::MAX {
                Some(data.satellites_visible)
            } else {
                None
            };
            let hdop = if data.eph != u16::MAX {
                Some(data.eph as f64 / 100.0)
            } else {
                None
            };
            writers.telemetry.send_if_modified(|t| {
                let mut c = false;
                c |= set_if_changed(&mut t.gps_fix_type, fix);
                if let Some(v) = sats {
                    c |= set_if_changed(&mut t.gps_satellites, Some(v));
                }
                if let Some(v) = hdop {
                    c |= set_if_changed(&mut t.gps_hdop, Some(v));
                }
                c
            });
            writers.gps.send_if_modified(|g| {
                let mut c = false;
                c |= set_if_changed(&mut g.fix_type, fix);
                if let Some(v) = sats {
                    c |= set_if_changed(&mut g.satellites, Some(v));
                }
                if let Some(v) = hdop {
                    c |= set_if_changed(&mut g.hdop, Some(v));
                }
                c
            });
        }
        common::MavMessage::MISSION_CURRENT(data) => {
            let _ = writers.mission_state.send(MissionState {
                current_seq: data.seq,
                total_items: data.total,
            });
        }
        common::MavMessage::HOME_POSITION(data) => {
            let _ = writers.home_position.send(Some(mission::HomePosition {
                latitude_deg: data.latitude as f64 / 1e7,
                longitude_deg: data.longitude as f64 / 1e7,
                altitude_m: (data.altitude as f64 / 1000.0) as f32,
            }));
        }
        common::MavMessage::ATTITUDE(data) => {
            let roll = Some(data.roll.to_degrees() as f64);
            let pitch = Some(data.pitch.to_degrees() as f64);
            let yaw = Some(data.yaw.to_degrees() as f64);
            writers.telemetry.send_if_modified(|t| {
                let mut c = false;
                c |= set_if_changed(&mut t.roll_deg, roll);
                c |= set_if_changed(&mut t.pitch_deg, pitch);
                c |= set_if_changed(&mut t.yaw_deg, yaw);
                c
            });
            writers.attitude.send_if_modified(|a| {
                let mut c = false;
                c |= set_if_changed(&mut a.roll_deg, roll);
                c |= set_if_changed(&mut a.pitch_deg, pitch);
                c |= set_if_changed(&mut a.yaw_deg, yaw);
                c
            });
        }
        common::MavMessage::NAV_CONTROLLER_OUTPUT(data) => {
            let wp = Some(data.wp_dist as f64);
            let nav = Some(data.nav_bearing as f64);
            let tgt = Some(data.target_bearing as f64);
            let xt = Some(data.xtrack_error as f64);
            writers.telemetry.send_if_modified(|t| {
                let mut c = false;
                c |= set_if_changed(&mut t.wp_dist_m, wp);
                c |= set_if_changed(&mut t.nav_bearing_deg, nav);
                c |= set_if_changed(&mut t.target_bearing_deg, tgt);
                c |= set_if_changed(&mut t.xtrack_error_m, xt);
                c
            });
            writers.navigation.send_if_modified(|n| {
                let mut c = false;
                c |= set_if_changed(&mut n.wp_dist_m, wp);
                c |= set_if_changed(&mut n.nav_bearing_deg, nav);
                c |= set_if_changed(&mut n.target_bearing_deg, tgt);
                c |= set_if_changed(&mut n.xtrack_error_m, xt);
                c
            });
        }
        common::MavMessage::TERRAIN_REPORT(data) => {
            let th = Some(data.terrain_height as f64);
            let hat = Some(data.current_height as f64);
            writers.telemetry.send_if_modified(|t| {
                let mut c = false;
                c |= set_if_changed(&mut t.terrain_height_m, th);
                c |= set_if_changed(&mut t.height_above_terrain_m, hat);
                c
            });
            writers.terrain.send_if_modified(|tr| {
                let mut c = false;
                c |= set_if_changed(&mut tr.terrain_height_m, th);
                c |= set_if_changed(&mut tr.height_above_terrain_m, hat);
                c
            });
        }
        common::MavMessage::BATTERY_STATUS(data) => {
            let cells: Vec<f64> = data
                .voltages
                .iter()
                .filter(|&&v| v != u16::MAX)
                .map(|&v| v as f64 / 1000.0)
                .collect();
            let cells_opt = if !cells.is_empty() { Some(cells) } else { None };
            let energy = if data.energy_consumed >= 0 {
                Some(data.energy_consumed as f64 / 36.0)
            } else {
                None
            };
            let remaining = if data.time_remaining > 0 {
                Some(data.time_remaining)
            } else {
                None
            };
            writers.telemetry.send_if_modified(|t| {
                let mut c = false;
                if let Some(ref v) = cells_opt {
                    c |= set_if_changed(&mut t.battery_voltage_cells, Some(v.clone()));
                }
                if let Some(v) = energy {
                    c |= set_if_changed(&mut t.energy_consumed_wh, Some(v));
                }
                if let Some(v) = remaining {
                    c |= set_if_changed(&mut t.battery_time_remaining_s, Some(v));
                }
                c
            });
            writers.battery.send_if_modified(|b| {
                let mut c = false;
                if let Some(ref v) = cells_opt {
                    c |= set_if_changed(&mut b.voltage_cells, Some(v.clone()));
                }
                if let Some(v) = energy {
                    c |= set_if_changed(&mut b.energy_consumed_wh, Some(v));
                }
                if let Some(v) = remaining {
                    c |= set_if_changed(&mut b.time_remaining_s, Some(v));
                }
                c
            });
        }
        common::MavMessage::RC_CHANNELS(data) => {
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
            let ch = Some(all[..count].to_vec());
            let rssi = if data.rssi != u8::MAX {
                Some(data.rssi)
            } else {
                None
            };
            writers.telemetry.send_if_modified(|t| {
                let mut c = false;
                c |= set_if_changed(&mut t.rc_channels, ch.clone());
                if let Some(v) = rssi {
                    c |= set_if_changed(&mut t.rc_rssi, Some(v));
                }
                c
            });
            writers.rc_channels.send_if_modified(|rc| {
                let mut c = false;
                c |= set_if_changed(&mut rc.channels, ch.clone());
                if let Some(v) = rssi {
                    c |= set_if_changed(&mut rc.rssi, Some(v));
                }
                c
            });
        }
        common::MavMessage::SERVO_OUTPUT_RAW(data) => {
            let servos = Some(vec![
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
            writers
                .telemetry
                .send_if_modified(|t| set_if_changed(&mut t.servo_outputs, servos.clone()));
            writers
                .rc_channels
                .send_if_modified(|rc| set_if_changed(&mut rc.servo_outputs, servos.clone()));
        }
        common::MavMessage::STATUSTEXT(data) => {
            let text = data.text.to_str().unwrap_or("").to_string();
            if !text.is_empty() {
                let _ = writers.statustext.send(Some(crate::state::StatusMessage {
                    text,
                    severity: crate::state::MavSeverity::from_mav(data.severity),
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
            let result = handle_arm_disarm(
                true,
                force,
                connection,
                writers,
                vehicle_target,
                config,
                cancel,
            )
            .await;
            let _ = reply.send(result);
        }
        Command::Disarm { force, reply } => {
            let result = handle_arm_disarm(
                false,
                force,
                connection,
                writers,
                vehicle_target,
                config,
                cancel,
            )
            .await;
            let _ = reply.send(result);
        }
        Command::SetMode { custom_mode, reply } => {
            let result = handle_set_mode(
                custom_mode,
                connection,
                writers,
                vehicle_target,
                config,
                cancel,
            )
            .await;
            let _ = reply.send(result);
        }
        Command::Long {
            command,
            params,
            reply,
        } => {
            let result = handle_command_long(
                command,
                params,
                connection,
                writers,
                vehicle_target,
                config,
                cancel,
            )
            .await;
            let _ = reply.send(result);
        }
        Command::GuidedGoto {
            lat_e7,
            lon_e7,
            alt_m,
            reply,
        } => {
            let result =
                handle_guided_goto(lat_e7, lon_e7, alt_m, connection, vehicle_target, config).await;
            let _ = reply.send(result);
        }
        Command::MissionUpload { plan, reply } => {
            let result =
                handle_mission_upload(plan, connection, writers, vehicle_target, config, cancel)
                    .await;
            let _ = reply.send(result);
        }
        Command::MissionDownload {
            mission_type,
            reply,
        } => {
            let result = handle_mission_download(
                mission_type,
                connection,
                writers,
                vehicle_target,
                config,
                cancel,
            )
            .await;
            let _ = reply.send(result);
        }
        Command::MissionClear {
            mission_type,
            reply,
        } => {
            let result = handle_mission_clear(
                mission_type,
                connection,
                writers,
                vehicle_target,
                config,
                cancel,
            )
            .await;
            let _ = reply.send(result);
        }
        Command::MissionSetCurrent { seq, reply } => {
            let result = handle_mission_set_current(
                seq,
                connection,
                writers,
                vehicle_target,
                config,
                cancel,
            )
            .await;
            let _ = reply.send(result);
        }
        Command::MissionCancelTransfer => {
            // Transfer cancellation currently relies on outer cancellation.
        }
        Command::ParamDownloadAll { reply } => {
            let result =
                handle_param_download_all(connection, writers, vehicle_target, config, cancel)
                    .await;
            let _ = reply.send(result);
        }
        Command::ParamWrite { name, value, reply } => {
            let result = handle_param_write(
                &name,
                value,
                connection,
                writers,
                vehicle_target,
                config,
                cancel,
            )
            .await;
            let _ = reply.send(result);
        }
        Command::ParamWriteBatch { params, reply } => {
            let result = handle_param_write_batch(
                params,
                connection,
                writers,
                vehicle_target,
                config,
                cancel,
            )
            .await;
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
        .map_err(|err| VehicleError::Io(std::io::Error::other(err.to_string())))
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
                let (header, msg) =
                    result.map_err(|err| VehicleError::Io(std::io::Error::other(err.to_string())))?;
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
    writers: &StateWriters,
    vehicle_target: &mut Option<VehicleTarget>,
    config: &VehicleConfig,
    cancel: &CancellationToken,
) -> Result<(), VehicleError> {
    let target = get_target(vehicle_target)?;
    let param1 = if arm { 1.0 } else { 0.0 };
    let param2 = if force {
        if arm {
            MAGIC_FORCE_ARM_VALUE
        } else {
            MAGIC_FORCE_DISARM_VALUE
        }
    } else {
        0.0
    };

    send_command_long_ack(
        MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
        [param1, param2, 0.0, 0.0, 0.0, 0.0, 0.0],
        target,
        connection,
        writers,
        vehicle_target,
        config,
        cancel,
    )
    .await
}

#[allow(clippy::too_many_arguments)]
async fn send_command_long_ack(
    command: MavCmd,
    params: [f32; 7],
    target: VehicleTarget,
    connection: &(dyn AsyncMavConnection<common::MavMessage> + Sync + Send),
    writers: &StateWriters,
    vehicle_target: &mut Option<VehicleTarget>,
    config: &VehicleConfig,
    cancel: &CancellationToken,
) -> Result<(), VehicleError> {
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
                    let (header, msg) = result
                        .map_err(|err| VehicleError::Io(std::io::Error::other(err.to_string())))?;
                    update_vehicle_target(vehicle_target, &header, &msg);
                    update_state(&header, &msg, writers, vehicle_target);
                    if let common::MavMessage::COMMAND_ACK(ack) = &msg
                        && ack.command == command
                    {
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

    Err(VehicleError::Timeout)
}

// ---------------------------------------------------------------------------
// Set mode
// ---------------------------------------------------------------------------

async fn handle_set_mode(
    custom_mode: u32,
    connection: &(dyn AsyncMavConnection<common::MavMessage> + Sync + Send),
    writers: &StateWriters,
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
        writers,
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
                let (header, msg) =
                    result.map_err(|err| VehicleError::Io(std::io::Error::other(err.to_string())))?;
                update_vehicle_target(vehicle_target, &header, &msg);
                update_state(&header, &msg, writers, vehicle_target);
                if let common::MavMessage::HEARTBEAT(hb) = &msg
                    && hb.custom_mode == custom_mode
                {
                    return Ok(());
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
    writers: &StateWriters,
    vehicle_target: &mut Option<VehicleTarget>,
    config: &VehicleConfig,
    cancel: &CancellationToken,
) -> Result<(), VehicleError> {
    let target = get_target(vehicle_target)?;
    send_command_long_ack(
        command,
        params,
        target,
        connection,
        writers,
        vehicle_target,
        config,
        cancel,
    )
    .await
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
        x: if is_global {
            (data.x as f64 * 1e7) as i32
        } else {
            data.x as i32
        },
        y: if is_global {
            (data.y as f64 * 1e7) as i32
        } else {
            data.y as i32
        },
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

    let command = num_traits::FromPrimitive::from_u16(item.command).ok_or_else(|| {
        VehicleError::MissionTransfer {
            code: "unsupported_command".to_string(),
            message: format!("unsupported MAV_CMD value {}", item.command),
        }
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
                    let (header, msg) = result
                        .map_err(|err| VehicleError::Io(std::io::Error::other(err.to_string())))?;
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

#[allow(clippy::too_many_arguments)]
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
                let (header, msg) =
                    result.map_err(|err| VehicleError::Io(std::io::Error::other(err.to_string())))?;
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

    let request_list_msg =
        common::MavMessage::MISSION_REQUEST_LIST(common::MISSION_REQUEST_LIST_DATA {
            target_system: target.system_id,
            target_component: target.component_id,
            mission_type: mav_mission_type,
        });
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
                let (header, msg) =
                    result.map_err(|err| VehicleError::Io(std::io::Error::other(err.to_string())))?;
                update_vehicle_target(vehicle_target, &header, &msg);
                update_state(&header, &msg, writers, vehicle_target);

                if let common::MavMessage::MISSION_COUNT(data) = &msg
                    && mission_type_matches(data.mission_type, mission_type)
                {
                    break data.count;
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

        let request_int_msg =
            common::MavMessage::MISSION_REQUEST_INT(common::MISSION_REQUEST_INT_DATA {
                seq,
                target_system: target.system_id,
                target_component: target.component_id,
                mission_type: mav_mission_type,
            });
        let request_float_msg = common::MavMessage::MISSION_REQUEST(common::MISSION_REQUEST_DATA {
            seq,
            target_system: target.system_id,
            target_component: target.component_id,
            mission_type: mav_mission_type,
        });

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
                    let (header, msg) = result
                        .map_err(|err| VehicleError::Io(std::io::Error::other(err.to_string())))?;
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
                    let (header, msg) = result
                        .map_err(|err| VehicleError::Io(std::io::Error::other(err.to_string())))?;
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
                    let (header, msg) = result
                        .map_err(|err| VehicleError::Io(std::io::Error::other(err.to_string())))?;
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
            debug!(
                "param download: requested {} missing params (retry {})",
                missing_requested, retries
            );
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
                    let (header, msg) = result
                        .map_err(|err| VehicleError::Io(std::io::Error::other(err.to_string())))?;
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
        let result = handle_param_write(
            &name,
            value,
            connection,
            writers,
            vehicle_target,
            config,
            cancel,
        )
        .await;
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
        phase: if all_ok {
            ParamTransferPhase::Completed
        } else {
            ParamTransferPhase::Failed
        },
        received: results.iter().filter(|r| r.success).count() as u16,
        expected: total,
    });

    Ok(results)
}

// ===========================================================================
// Unit tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::{self, LinkState, create_channels};
    use mavlink::common::{self, MavModeFlag, MavState};
    use mavlink::{MavHeader, MavlinkVersion};
    use std::sync::{Arc, Mutex};
    use tokio::sync::{mpsc, oneshot};

    // -----------------------------------------------------------------------
    // Mock AsyncMavConnection
    // -----------------------------------------------------------------------

    type SentMessages = Arc<Mutex<Vec<(MavHeader, common::MavMessage)>>>;

    /// Messages the mock should yield from `recv()`.  When the queue is empty
    /// the mock blocks forever (the test drives progress via commands/cancel).
    struct MockConnection {
        recv_rx: tokio::sync::Mutex<mpsc::Receiver<(MavHeader, common::MavMessage)>>,
        sent: SentMessages,
    }

    impl MockConnection {
        fn new(rx: mpsc::Receiver<(MavHeader, common::MavMessage)>) -> (Self, SentMessages) {
            let sent = Arc::new(Mutex::new(Vec::new()));
            (
                Self {
                    recv_rx: tokio::sync::Mutex::new(rx),
                    sent: sent.clone(),
                },
                sent,
            )
        }
    }

    #[async_trait::async_trait]
    impl AsyncMavConnection<common::MavMessage> for MockConnection {
        async fn recv(
            &self,
        ) -> Result<(MavHeader, common::MavMessage), mavlink::error::MessageReadError> {
            let mut rx = self.recv_rx.lock().await;
            match rx.recv().await {
                Some(msg) => Ok(msg),
                None => Err(mavlink::error::MessageReadError::Io(std::io::Error::new(
                    std::io::ErrorKind::ConnectionReset,
                    "mock connection closed",
                ))),
            }
        }

        async fn recv_raw(
            &self,
        ) -> Result<mavlink::MAVLinkMessageRaw, mavlink::error::MessageReadError> {
            unimplemented!("recv_raw not used in event_loop tests")
        }

        async fn send(
            &self,
            header: &MavHeader,
            data: &common::MavMessage,
        ) -> Result<usize, mavlink::error::MessageWriteError> {
            self.sent.lock().unwrap().push((*header, data.clone()));
            Ok(0)
        }

        fn set_protocol_version(&mut self, _version: MavlinkVersion) {}
        fn protocol_version(&self) -> MavlinkVersion {
            MavlinkVersion::V2
        }
        fn set_allow_recv_any_version(&mut self, _allow: bool) {}
        fn allow_recv_any_version(&self) -> bool {
            true
        }
    }

    // -----------------------------------------------------------------------
    // Test helpers
    // -----------------------------------------------------------------------

    fn default_header() -> MavHeader {
        MavHeader {
            system_id: 1,
            component_id: 1,
            sequence: 0,
        }
    }

    fn heartbeat_msg(armed: bool, custom_mode: u32) -> common::MavMessage {
        let mut base_mode = MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        if armed {
            base_mode |= MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED;
        }
        common::MavMessage::HEARTBEAT(common::HEARTBEAT_DATA {
            custom_mode,
            mavtype: common::MavType::MAV_TYPE_QUADROTOR,
            autopilot: common::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode,
            system_status: MavState::MAV_STATE_STANDBY,
            mavlink_version: 3,
        })
    }

    fn fast_config() -> VehicleConfig {
        VehicleConfig {
            retry_policy: crate::mission::RetryPolicy {
                request_timeout_ms: 50,
                item_timeout_ms: 50,
                max_retries: 1,
            },
            auto_request_home: false,
            ..VehicleConfig::default()
        }
    }

    fn ack_msg(command: MavCmd, result: common::MavResult) -> common::MavMessage {
        common::MavMessage::COMMAND_ACK(common::COMMAND_ACK_DATA {
            command,
            result,
            progress: 0,
            result_param2: 0,
            target_system: 0,
            target_component: 0,
        })
    }

    // -----------------------------------------------------------------------
    // update_vehicle_target tests
    // -----------------------------------------------------------------------

    #[test]
    fn target_set_from_heartbeat() {
        let mut target: Option<VehicleTarget> = None;
        let header = default_header();
        let msg = heartbeat_msg(false, 0);
        update_vehicle_target(&mut target, &header, &msg);
        let t = target.unwrap();
        assert_eq!(t.system_id, 1);
        assert_eq!(t.component_id, 1);
        assert_eq!(
            t.autopilot,
            common::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA
        );
        assert_eq!(t.vehicle_type, common::MavType::MAV_TYPE_QUADROTOR);
    }

    #[test]
    fn target_set_generic_from_non_heartbeat() {
        let mut target: Option<VehicleTarget> = None;
        let header = MavHeader {
            system_id: 5,
            component_id: 10,
            sequence: 0,
        };
        let msg = common::MavMessage::VFR_HUD(common::VFR_HUD_DATA {
            airspeed: 0.0,
            groundspeed: 0.0,
            heading: 0,
            throttle: 0,
            alt: 0.0,
            climb: 0.0,
        });
        update_vehicle_target(&mut target, &header, &msg);
        let t = target.unwrap();
        assert_eq!(t.system_id, 5);
        assert_eq!(t.component_id, 10);
        assert_eq!(t.autopilot, common::MavAutopilot::MAV_AUTOPILOT_GENERIC);
    }

    #[test]
    fn target_ignored_for_system_id_zero() {
        let mut target: Option<VehicleTarget> = None;
        let header = MavHeader {
            system_id: 0,
            component_id: 1,
            sequence: 0,
        };
        let msg = heartbeat_msg(false, 0);
        update_vehicle_target(&mut target, &header, &msg);
        assert!(target.is_none());
    }

    #[test]
    fn heartbeat_updates_existing_target() {
        let mut target = Some(VehicleTarget {
            system_id: 1,
            component_id: 1,
            autopilot: common::MavAutopilot::MAV_AUTOPILOT_GENERIC,
            vehicle_type: common::MavType::MAV_TYPE_GENERIC,
        });
        let header = MavHeader {
            system_id: 2,
            component_id: 3,
            sequence: 0,
        };
        let msg = heartbeat_msg(true, 4);
        update_vehicle_target(&mut target, &header, &msg);
        let t = target.unwrap();
        assert_eq!(t.system_id, 2);
        assert_eq!(t.component_id, 3);
        assert_eq!(t.vehicle_type, common::MavType::MAV_TYPE_QUADROTOR);
    }

    #[test]
    fn non_heartbeat_does_not_overwrite_existing_target() {
        let original = VehicleTarget {
            system_id: 1,
            component_id: 1,
            autopilot: common::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
            vehicle_type: common::MavType::MAV_TYPE_QUADROTOR,
        };
        let mut target = Some(original);
        let header = MavHeader {
            system_id: 99,
            component_id: 99,
            sequence: 0,
        };
        let msg = common::MavMessage::VFR_HUD(common::VFR_HUD_DATA {
            airspeed: 0.0,
            groundspeed: 10.0,
            heading: 90,
            throttle: 50,
            alt: 100.0,
            climb: 1.0,
        });
        update_vehicle_target(&mut target, &header, &msg);
        let t = target.unwrap();
        assert_eq!(t.system_id, 1); // unchanged
    }

    // -----------------------------------------------------------------------
    // update_state tests
    // -----------------------------------------------------------------------

    #[test]
    fn heartbeat_updates_vehicle_state() {
        let (writers, channels) = create_channels();
        let target = Some(VehicleTarget {
            system_id: 1,
            component_id: 1,
            autopilot: common::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
            vehicle_type: common::MavType::MAV_TYPE_QUADROTOR,
        });
        let header = default_header();
        let msg = heartbeat_msg(true, 4);
        update_state(&header, &msg, &writers, &target);

        let vs = channels.vehicle_state.borrow();
        assert!(vs.armed);
        assert_eq!(vs.custom_mode, 4);
        assert_eq!(vs.system_id, 1);
    }

    #[test]
    fn vfr_hud_updates_telemetry() {
        let (writers, channels) = create_channels();
        let target: Option<VehicleTarget> = None;
        let header = default_header();
        let msg = common::MavMessage::VFR_HUD(common::VFR_HUD_DATA {
            airspeed: 12.5,
            groundspeed: 10.0,
            heading: 180,
            throttle: 55,
            alt: 100.0,
            climb: 2.5,
        });
        update_state(&header, &msg, &writers, &target);

        let t = channels.telemetry.borrow();
        assert_eq!(t.altitude_m, Some(100.0));
        assert_eq!(t.speed_mps, Some(10.0));
        assert_eq!(t.heading_deg, Some(180.0));
        assert_eq!(t.climb_rate_mps, Some(2.5));
        assert_eq!(t.throttle_pct, Some(55.0));
        assert_eq!(t.airspeed_mps, Some(12.5));
    }

    #[test]
    fn global_position_int_updates_telemetry() {
        let (writers, channels) = create_channels();
        let target: Option<VehicleTarget> = None;
        let header = default_header();
        let msg = common::MavMessage::GLOBAL_POSITION_INT(common::GLOBAL_POSITION_INT_DATA {
            time_boot_ms: 0,
            lat: 473_977_420, // ~47.3977420
            lon: 85_455_940,  // ~8.5455940
            alt: 0,
            relative_alt: 50_000, // 50m
            vx: 100,              // 1 m/s
            vy: 0,
            vz: 0,
            hdg: 27000, // 270.00 degrees
        });
        update_state(&header, &msg, &writers, &target);

        let t = channels.telemetry.borrow();
        assert_eq!(t.altitude_m, Some(50.0));
        assert!((t.latitude_deg.unwrap() - 47.397742).abs() < 0.0001);
        assert!((t.longitude_deg.unwrap() - 8.545594).abs() < 0.0001);
        assert_eq!(t.heading_deg, Some(270.0));
    }

    #[test]
    fn sys_status_updates_battery() {
        let (writers, channels) = create_channels();
        let target: Option<VehicleTarget> = None;
        let header = default_header();
        let msg = common::MavMessage::SYS_STATUS(common::SYS_STATUS_DATA {
            onboard_control_sensors_present: common::MavSysStatusSensor::empty(),
            onboard_control_sensors_enabled: common::MavSysStatusSensor::empty(),
            onboard_control_sensors_health: common::MavSysStatusSensor::empty(),
            load: 0,
            voltage_battery: 12600, // 12.6V
            current_battery: 500,   // 5.0A
            battery_remaining: 75,
            drop_rate_comm: 0,
            errors_comm: 0,
            errors_count1: 0,
            errors_count2: 0,
            errors_count3: 0,
            errors_count4: 0,
            onboard_control_sensors_present_extended: common::MavSysStatusSensorExtended::empty(),
            onboard_control_sensors_enabled_extended: common::MavSysStatusSensorExtended::empty(),
            onboard_control_sensors_health_extended: common::MavSysStatusSensorExtended::empty(),
        });
        update_state(&header, &msg, &writers, &target);

        let t = channels.telemetry.borrow();
        assert_eq!(t.battery_pct, Some(75.0));
        assert!((t.battery_voltage_v.unwrap() - 12.6).abs() < 0.001);
        assert!((t.battery_current_a.unwrap() - 5.0).abs() < 0.001);
    }

    #[test]
    fn gps_raw_int_updates_telemetry() {
        let (writers, channels) = create_channels();
        let target: Option<VehicleTarget> = None;
        let header = default_header();
        let msg = common::MavMessage::GPS_RAW_INT(common::GPS_RAW_INT_DATA {
            time_usec: 0,
            fix_type: common::GpsFixType::GPS_FIX_TYPE_3D_FIX,
            lat: 0,
            lon: 0,
            alt: 0,
            eph: 150, // 1.50 HDOP
            epv: u16::MAX,
            vel: 0,
            cog: 0,
            satellites_visible: 12,
            alt_ellipsoid: 0,
            h_acc: 0,
            v_acc: 0,
            vel_acc: 0,
            hdg_acc: 0,
            yaw: 0,
        });
        update_state(&header, &msg, &writers, &target);

        let t = channels.telemetry.borrow();
        assert_eq!(t.gps_satellites, Some(12));
        assert!((t.gps_hdop.unwrap() - 1.5).abs() < 0.01);
        assert_eq!(t.gps_fix_type, Some(state::GpsFixType::Fix3d));
    }

    #[test]
    fn mission_current_updates_mission_state() {
        let (writers, channels) = create_channels();
        let target: Option<VehicleTarget> = None;
        let header = default_header();
        let msg = common::MavMessage::MISSION_CURRENT(common::MISSION_CURRENT_DATA {
            seq: 3,
            total: 10,
            mission_state: common::MissionState::MISSION_STATE_ACTIVE,
            mission_mode: 0,
            mission_id: 0,
            fence_id: 0,
            rally_points_id: 0,
        });
        update_state(&header, &msg, &writers, &target);

        let ms = channels.mission_state.borrow();
        assert_eq!(ms.current_seq, 3);
        assert_eq!(ms.total_items, 10);
    }

    #[test]
    fn home_position_updates_state() {
        let (writers, channels) = create_channels();
        let target: Option<VehicleTarget> = None;
        let header = default_header();
        let msg = common::MavMessage::HOME_POSITION(common::HOME_POSITION_DATA {
            latitude: 473_977_420,
            longitude: 85_455_940,
            altitude: 500_000, // 500m
            x: 0.0,
            y: 0.0,
            z: 0.0,
            q: [0.0; 4],
            approach_x: 0.0,
            approach_y: 0.0,
            approach_z: 0.0,
            time_usec: 0,
        });
        update_state(&header, &msg, &writers, &target);

        let hp = channels.home_position.borrow();
        let hp = hp.as_ref().unwrap();
        assert!((hp.latitude_deg - 47.397742).abs() < 0.0001);
        assert!((hp.longitude_deg - 8.545594).abs() < 0.0001);
        assert!((hp.altitude_m - 500.0).abs() < 0.1);
    }

    #[test]
    fn attitude_updates_telemetry() {
        let (writers, channels) = create_channels();
        let target: Option<VehicleTarget> = None;
        let header = default_header();
        let msg = common::MavMessage::ATTITUDE(common::ATTITUDE_DATA {
            time_boot_ms: 0,
            roll: std::f32::consts::FRAC_PI_6,   // 30 degrees
            pitch: -std::f32::consts::FRAC_PI_4, // -45 degrees
            yaw: std::f32::consts::PI,           // 180 degrees
            rollspeed: 0.0,
            pitchspeed: 0.0,
            yawspeed: 0.0,
        });
        update_state(&header, &msg, &writers, &target);

        let t = channels.telemetry.borrow();
        assert!((t.roll_deg.unwrap() - 30.0).abs() < 0.1);
        assert!((t.pitch_deg.unwrap() - (-45.0)).abs() < 0.1);
        assert!((t.yaw_deg.unwrap() - 180.0).abs() < 0.1);
    }

    #[test]
    fn statustext_updates_state() {
        let (writers, channels) = create_channels();
        let target: Option<VehicleTarget> = None;
        let header = default_header();
        let msg = common::MavMessage::STATUSTEXT(common::STATUSTEXT_DATA {
            severity: common::MavSeverity::MAV_SEVERITY_WARNING,
            text: "PreArm: Check fence".into(),
            id: 0,
            chunk_seq: 0,
        });
        update_state(&header, &msg, &writers, &target);

        let st = channels.statustext.borrow();
        let st = st.as_ref().unwrap();
        assert!(st.text.starts_with("PreArm"));
        assert_eq!(st.severity, state::MavSeverity::Warning);
    }

    #[test]
    fn nav_controller_output_updates_telemetry() {
        let (writers, channels) = create_channels();
        let target: Option<VehicleTarget> = None;
        let header = default_header();
        let msg = common::MavMessage::NAV_CONTROLLER_OUTPUT(common::NAV_CONTROLLER_OUTPUT_DATA {
            nav_roll: 0.0,
            nav_pitch: 0.0,
            nav_bearing: 90,
            target_bearing: 95,
            wp_dist: 150,
            alt_error: 0.0,
            aspd_error: 0.0,
            xtrack_error: 1.5,
        });
        update_state(&header, &msg, &writers, &target);

        let t = channels.telemetry.borrow();
        assert_eq!(t.wp_dist_m, Some(150.0));
        assert_eq!(t.nav_bearing_deg, Some(90.0));
        assert_eq!(t.target_bearing_deg, Some(95.0));
        assert_eq!(t.xtrack_error_m, Some(1.5));
    }

    #[test]
    fn terrain_report_updates_telemetry() {
        let (writers, channels) = create_channels();
        let target: Option<VehicleTarget> = None;
        let header = default_header();
        let msg = common::MavMessage::TERRAIN_REPORT(common::TERRAIN_REPORT_DATA {
            lat: 0,
            lon: 0,
            spacing: 0,
            terrain_height: 250.0,
            current_height: 50.0,
            pending: 0,
            loaded: 0,
        });
        update_state(&header, &msg, &writers, &target);

        let t = channels.telemetry.borrow();
        assert_eq!(t.terrain_height_m, Some(250.0));
        assert_eq!(t.height_above_terrain_m, Some(50.0));
    }

    // -----------------------------------------------------------------------
    // run_event_loop tests
    // -----------------------------------------------------------------------

    #[tokio::test]
    async fn shutdown_command_stops_loop() {
        let (msg_tx, msg_rx) = mpsc::channel(16);
        let (conn, _sent) = MockConnection::new(msg_rx);

        let (cmd_tx, cmd_rx) = mpsc::channel(16);
        let (writers, channels) = create_channels();
        let cancel = CancellationToken::new();

        // Send shutdown
        cmd_tx.send(Command::Shutdown).await.unwrap();

        run_event_loop(Box::new(conn), cmd_rx, writers, fast_config(), cancel).await;

        // Loop should have exited and set link state to Disconnected
        assert_eq!(*channels.link_state.borrow(), LinkState::Disconnected);
        drop(msg_tx); // keep sender alive until here
    }

    #[tokio::test]
    async fn cancel_stops_loop() {
        let (_msg_tx, msg_rx) = mpsc::channel(16);
        let (conn, _sent) = MockConnection::new(msg_rx);

        let (_cmd_tx, cmd_rx) = mpsc::channel(16);
        let (writers, channels) = create_channels();
        let cancel = CancellationToken::new();

        cancel.cancel();

        run_event_loop(Box::new(conn), cmd_rx, writers, fast_config(), cancel).await;

        assert_eq!(*channels.link_state.borrow(), LinkState::Disconnected);
    }

    #[tokio::test]
    async fn recv_error_stops_loop_with_error_state() {
        let (msg_tx, msg_rx) = mpsc::channel(16);
        let (conn, _sent) = MockConnection::new(msg_rx);

        let (_cmd_tx, cmd_rx) = mpsc::channel(16);
        let (writers, channels) = create_channels();
        let cancel = CancellationToken::new();

        // Drop the sender to make recv return an error
        drop(msg_tx);

        run_event_loop(Box::new(conn), cmd_rx, writers, fast_config(), cancel).await;

        let link = channels.link_state.borrow().clone();
        match link {
            LinkState::Error(_) => {} // expected
            other => panic!("expected LinkState::Error, got {other:?}"),
        }
    }

    #[tokio::test]
    async fn heartbeat_message_updates_state_through_loop() {
        let (msg_tx, msg_rx) = mpsc::channel(16);
        let (conn, _sent) = MockConnection::new(msg_rx);

        let (cmd_tx, cmd_rx) = mpsc::channel(16);
        let (writers, channels) = create_channels();
        let cancel = CancellationToken::new();

        let handle = tokio::spawn(async move {
            run_event_loop(Box::new(conn), cmd_rx, writers, fast_config(), cancel).await;
        });

        // Send heartbeat and wait for it to be processed before shutdown
        msg_tx
            .send((default_header(), heartbeat_msg(true, 5)))
            .await
            .unwrap();
        tokio::time::sleep(Duration::from_millis(20)).await;

        cmd_tx.send(Command::Shutdown).await.unwrap();
        handle.await.unwrap();

        let vs = channels.vehicle_state.borrow();
        assert!(vs.armed);
        assert_eq!(vs.custom_mode, 5);
    }

    #[tokio::test]
    async fn link_state_set_to_connected_on_start() {
        let (_msg_tx, msg_rx) = mpsc::channel(16);
        let (conn, _sent) = MockConnection::new(msg_rx);

        let (cmd_tx, cmd_rx) = mpsc::channel(16);
        let (writers, channels) = create_channels();
        let cancel = CancellationToken::new();

        // Verify initial state is Connecting
        assert_eq!(*channels.link_state.borrow(), LinkState::Connecting);

        cmd_tx.send(Command::Shutdown).await.unwrap();

        run_event_loop(Box::new(conn), cmd_rx, writers, fast_config(), cancel).await;

        // After loop finishes it sets Disconnected, but it was Connected during run
        assert_eq!(*channels.link_state.borrow(), LinkState::Disconnected);
    }

    // -----------------------------------------------------------------------
    // Command dispatch tests (via run_event_loop)
    // -----------------------------------------------------------------------

    #[tokio::test]
    async fn arm_command_sends_command_long_and_returns_ack() {
        let (msg_tx, msg_rx) = mpsc::channel(64);
        let (conn, sent) = MockConnection::new(msg_rx);

        let (cmd_tx, cmd_rx) = mpsc::channel(16);
        let (writers, _channels) = create_channels();
        let cancel = CancellationToken::new();

        let handle = tokio::spawn(async move {
            run_event_loop(Box::new(conn), cmd_rx, writers, fast_config(), cancel).await;
        });

        // First, establish vehicle target with a heartbeat
        msg_tx
            .send((default_header(), heartbeat_msg(false, 0)))
            .await
            .unwrap();
        // Give the loop time to process
        tokio::time::sleep(Duration::from_millis(10)).await;

        // Send arm command
        let (reply_tx, reply_rx) = oneshot::channel();
        cmd_tx
            .send(Command::Arm {
                force: false,
                reply: reply_tx,
            })
            .await
            .unwrap();

        // Give loop time to send the COMMAND_LONG
        tokio::time::sleep(Duration::from_millis(10)).await;

        // Send ACK response
        msg_tx
            .send((
                default_header(),
                ack_msg(
                    MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
                    common::MavResult::MAV_RESULT_ACCEPTED,
                ),
            ))
            .await
            .unwrap();

        let result = reply_rx.await.unwrap();
        assert!(result.is_ok());

        // Verify COMMAND_LONG was sent
        {
            let sent_msgs = sent.lock().unwrap();
            let arm_msgs: Vec<_> = sent_msgs
                .iter()
                .filter(|(_, msg)| matches!(msg, common::MavMessage::COMMAND_LONG(d) if d.command == MavCmd::MAV_CMD_COMPONENT_ARM_DISARM))
                .collect();
            assert!(!arm_msgs.is_empty());

            // Verify param1 = 1.0 (arm) and param2 = 0.0 (not forced)
            if let common::MavMessage::COMMAND_LONG(data) = &arm_msgs[0].1 {
                assert_eq!(data.param1, 1.0);
                assert_eq!(data.param2, 0.0);
            }
        }

        // Shutdown
        cmd_tx.send(Command::Shutdown).await.unwrap();
        handle.await.unwrap();
    }

    #[tokio::test]
    async fn force_arm_uses_magic_value() {
        let (msg_tx, msg_rx) = mpsc::channel(64);
        let (conn, sent) = MockConnection::new(msg_rx);

        let (cmd_tx, cmd_rx) = mpsc::channel(16);
        let (writers, _channels) = create_channels();
        let cancel = CancellationToken::new();

        let handle = tokio::spawn(async move {
            run_event_loop(Box::new(conn), cmd_rx, writers, fast_config(), cancel).await;
        });

        // Establish target
        msg_tx
            .send((default_header(), heartbeat_msg(false, 0)))
            .await
            .unwrap();
        tokio::time::sleep(Duration::from_millis(10)).await;

        let (reply_tx, reply_rx) = oneshot::channel();
        cmd_tx
            .send(Command::Arm {
                force: true,
                reply: reply_tx,
            })
            .await
            .unwrap();
        tokio::time::sleep(Duration::from_millis(10)).await;

        msg_tx
            .send((
                default_header(),
                ack_msg(
                    MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
                    common::MavResult::MAV_RESULT_ACCEPTED,
                ),
            ))
            .await
            .unwrap();

        let result = reply_rx.await.unwrap();
        assert!(result.is_ok());

        {
            let sent_msgs = sent.lock().unwrap();
            let arm_msg = sent_msgs
                .iter()
                .find(|(_, msg)| matches!(msg, common::MavMessage::COMMAND_LONG(d) if d.command == MavCmd::MAV_CMD_COMPONENT_ARM_DISARM))
                .unwrap();
            if let common::MavMessage::COMMAND_LONG(data) = &arm_msg.1 {
                assert_eq!(data.param2, MAGIC_FORCE_ARM_VALUE);
            }
        }

        cmd_tx.send(Command::Shutdown).await.unwrap();
        handle.await.unwrap();
    }

    #[tokio::test]
    async fn arm_without_target_returns_identity_unknown() {
        let (_msg_tx, msg_rx) = mpsc::channel(64);
        let (conn, _sent) = MockConnection::new(msg_rx);

        let (cmd_tx, cmd_rx) = mpsc::channel(16);
        let (writers, _channels) = create_channels();
        let cancel = CancellationToken::new();

        let handle = tokio::spawn(async move {
            run_event_loop(Box::new(conn), cmd_rx, writers, fast_config(), cancel).await;
        });

        // Send arm without establishing target first
        let (reply_tx, reply_rx) = oneshot::channel();
        cmd_tx
            .send(Command::Arm {
                force: false,
                reply: reply_tx,
            })
            .await
            .unwrap();

        let result = reply_rx.await.unwrap();
        assert!(matches!(result, Err(VehicleError::IdentityUnknown)));

        cmd_tx.send(Command::Shutdown).await.unwrap();
        handle.await.unwrap();
    }

    #[tokio::test]
    async fn command_rejected_returns_error() {
        let (msg_tx, msg_rx) = mpsc::channel(64);
        let (conn, _sent) = MockConnection::new(msg_rx);

        let (cmd_tx, cmd_rx) = mpsc::channel(16);
        let (writers, _channels) = create_channels();
        let cancel = CancellationToken::new();

        let handle = tokio::spawn(async move {
            run_event_loop(Box::new(conn), cmd_rx, writers, fast_config(), cancel).await;
        });

        // Establish target
        msg_tx
            .send((default_header(), heartbeat_msg(false, 0)))
            .await
            .unwrap();
        tokio::time::sleep(Duration::from_millis(10)).await;

        let (reply_tx, reply_rx) = oneshot::channel();
        cmd_tx
            .send(Command::Arm {
                force: false,
                reply: reply_tx,
            })
            .await
            .unwrap();
        tokio::time::sleep(Duration::from_millis(10)).await;

        // Send DENIED
        msg_tx
            .send((
                default_header(),
                ack_msg(
                    MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
                    common::MavResult::MAV_RESULT_DENIED,
                ),
            ))
            .await
            .unwrap();

        let result = reply_rx.await.unwrap();
        assert!(matches!(result, Err(VehicleError::CommandRejected { .. })));

        cmd_tx.send(Command::Shutdown).await.unwrap();
        handle.await.unwrap();
    }

    #[tokio::test]
    async fn set_mode_via_command_long_ack() {
        let (msg_tx, msg_rx) = mpsc::channel(64);
        let (conn, sent) = MockConnection::new(msg_rx);

        let (cmd_tx, cmd_rx) = mpsc::channel(16);
        let (writers, _channels) = create_channels();
        let cancel = CancellationToken::new();

        let handle = tokio::spawn(async move {
            run_event_loop(Box::new(conn), cmd_rx, writers, fast_config(), cancel).await;
        });

        // Establish target
        msg_tx
            .send((default_header(), heartbeat_msg(false, 0)))
            .await
            .unwrap();
        tokio::time::sleep(Duration::from_millis(10)).await;

        let (reply_tx, reply_rx) = oneshot::channel();
        cmd_tx
            .send(Command::SetMode {
                custom_mode: 4,
                reply: reply_tx,
            })
            .await
            .unwrap();
        tokio::time::sleep(Duration::from_millis(10)).await;

        // Send ACK
        msg_tx
            .send((
                default_header(),
                ack_msg(
                    MavCmd::MAV_CMD_DO_SET_MODE,
                    common::MavResult::MAV_RESULT_ACCEPTED,
                ),
            ))
            .await
            .unwrap();

        let result = reply_rx.await.unwrap();
        assert!(result.is_ok());

        // Verify DO_SET_MODE was sent with correct custom_mode
        {
            let sent_msgs = sent.lock().unwrap();
            let mode_msg = sent_msgs
                .iter()
                .find(|(_, msg)| matches!(msg, common::MavMessage::COMMAND_LONG(d) if d.command == MavCmd::MAV_CMD_DO_SET_MODE))
                .unwrap();
            if let common::MavMessage::COMMAND_LONG(data) = &mode_msg.1 {
                assert_eq!(data.param2 as u32, 4);
            }
        }

        cmd_tx.send(Command::Shutdown).await.unwrap();
        handle.await.unwrap();
    }

    #[tokio::test]
    async fn guided_goto_sends_set_position_target() {
        let (msg_tx, msg_rx) = mpsc::channel(64);
        let (conn, sent) = MockConnection::new(msg_rx);

        let (cmd_tx, cmd_rx) = mpsc::channel(16);
        let (writers, _channels) = create_channels();
        let cancel = CancellationToken::new();

        let handle = tokio::spawn(async move {
            run_event_loop(Box::new(conn), cmd_rx, writers, fast_config(), cancel).await;
        });

        // Establish target
        msg_tx
            .send((default_header(), heartbeat_msg(false, 0)))
            .await
            .unwrap();
        tokio::time::sleep(Duration::from_millis(10)).await;

        let (reply_tx, reply_rx) = oneshot::channel();
        cmd_tx
            .send(Command::GuidedGoto {
                lat_e7: 473_977_420,
                lon_e7: 85_455_940,
                alt_m: 100.0,
                reply: reply_tx,
            })
            .await
            .unwrap();

        let result = reply_rx.await.unwrap();
        assert!(result.is_ok());

        {
            let sent_msgs = sent.lock().unwrap();
            let goto_msg = sent_msgs
                .iter()
                .find(|(_, msg)| {
                    matches!(msg, common::MavMessage::SET_POSITION_TARGET_GLOBAL_INT(_))
                })
                .unwrap();
            if let common::MavMessage::SET_POSITION_TARGET_GLOBAL_INT(data) = &goto_msg.1 {
                assert_eq!(data.lat_int, 473_977_420);
                assert_eq!(data.lon_int, 85_455_940);
                assert_eq!(data.alt, 100.0);
            }
        }

        cmd_tx.send(Command::Shutdown).await.unwrap();
        handle.await.unwrap();
    }

    #[tokio::test]
    async fn arm_timeout_returns_error() {
        let (msg_tx, msg_rx) = mpsc::channel(64);
        let (conn, _sent) = MockConnection::new(msg_rx);

        let (cmd_tx, cmd_rx) = mpsc::channel(16);
        let (writers, _channels) = create_channels();
        let cancel = CancellationToken::new();

        let handle = tokio::spawn(async move {
            run_event_loop(Box::new(conn), cmd_rx, writers, fast_config(), cancel).await;
        });

        // Establish target
        msg_tx
            .send((default_header(), heartbeat_msg(false, 0)))
            .await
            .unwrap();
        tokio::time::sleep(Duration::from_millis(10)).await;

        let (reply_tx, reply_rx) = oneshot::channel();
        cmd_tx
            .send(Command::Arm {
                force: false,
                reply: reply_tx,
            })
            .await
            .unwrap();

        // Don't send any ACK — let it timeout
        let result = tokio::time::timeout(Duration::from_secs(5), reply_rx)
            .await
            .expect("reply should arrive before outer timeout")
            .unwrap();
        assert!(matches!(result, Err(VehicleError::Timeout)));

        cmd_tx.send(Command::Shutdown).await.unwrap();
        handle.await.unwrap();
    }

    #[tokio::test]
    async fn disarm_force_uses_magic_disarm_value() {
        let (msg_tx, msg_rx) = mpsc::channel(64);
        let (conn, sent) = MockConnection::new(msg_rx);

        let (cmd_tx, cmd_rx) = mpsc::channel(16);
        let (writers, _channels) = create_channels();
        let cancel = CancellationToken::new();

        let handle = tokio::spawn(async move {
            run_event_loop(Box::new(conn), cmd_rx, writers, fast_config(), cancel).await;
        });

        msg_tx
            .send((default_header(), heartbeat_msg(true, 0)))
            .await
            .unwrap();
        tokio::time::sleep(Duration::from_millis(10)).await;

        let (reply_tx, reply_rx) = oneshot::channel();
        cmd_tx
            .send(Command::Disarm {
                force: true,
                reply: reply_tx,
            })
            .await
            .unwrap();
        tokio::time::sleep(Duration::from_millis(10)).await;

        msg_tx
            .send((
                default_header(),
                ack_msg(
                    MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
                    common::MavResult::MAV_RESULT_ACCEPTED,
                ),
            ))
            .await
            .unwrap();

        let result = reply_rx.await.unwrap();
        assert!(result.is_ok());

        {
            let sent_msgs = sent.lock().unwrap();
            let disarm_msg = sent_msgs
                .iter()
                .find(|(_, msg)| matches!(msg, common::MavMessage::COMMAND_LONG(d) if d.command == MavCmd::MAV_CMD_COMPONENT_ARM_DISARM))
                .unwrap();
            if let common::MavMessage::COMMAND_LONG(data) = &disarm_msg.1 {
                assert_eq!(data.param1, 0.0); // disarm
                assert_eq!(data.param2, MAGIC_FORCE_DISARM_VALUE);
            }
        }

        cmd_tx.send(Command::Shutdown).await.unwrap();
        handle.await.unwrap();
    }

    // -----------------------------------------------------------------------
    // Helper function tests
    // -----------------------------------------------------------------------

    #[test]
    fn get_target_returns_error_when_none() {
        let target: Option<VehicleTarget> = None;
        assert!(matches!(
            get_target(&target),
            Err(VehicleError::IdentityUnknown)
        ));
    }

    #[test]
    fn get_target_returns_value_when_some() {
        let target = Some(VehicleTarget {
            system_id: 1,
            component_id: 1,
            autopilot: common::MavAutopilot::MAV_AUTOPILOT_GENERIC,
            vehicle_type: common::MavType::MAV_TYPE_GENERIC,
        });
        let t = get_target(&target).unwrap();
        assert_eq!(t.system_id, 1);
    }

    #[test]
    fn to_mav_mission_type_roundtrip() {
        assert_eq!(
            to_mav_mission_type(MissionType::Mission),
            common::MavMissionType::MAV_MISSION_TYPE_MISSION
        );
        assert_eq!(
            to_mav_mission_type(MissionType::Fence),
            common::MavMissionType::MAV_MISSION_TYPE_FENCE
        );
        assert_eq!(
            to_mav_mission_type(MissionType::Rally),
            common::MavMissionType::MAV_MISSION_TYPE_RALLY
        );
    }

    #[test]
    fn frame_conversion_roundtrip() {
        let frames = [
            (MissionFrame::Mission, common::MavFrame::MAV_FRAME_MISSION),
            (MissionFrame::GlobalInt, common::MavFrame::MAV_FRAME_GLOBAL),
            (
                MissionFrame::GlobalRelativeAltInt,
                common::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
            ),
            (
                MissionFrame::GlobalTerrainAltInt,
                common::MavFrame::MAV_FRAME_GLOBAL_TERRAIN_ALT,
            ),
            (
                MissionFrame::LocalNed,
                common::MavFrame::MAV_FRAME_LOCAL_NED,
            ),
        ];
        for (kit_frame, mav_frame) in frames {
            assert_eq!(to_mav_frame(kit_frame), mav_frame);
            assert_eq!(from_mav_frame(mav_frame), kit_frame);
        }
    }

    #[test]
    fn from_mission_item_int_converts_correctly() {
        let data = common::MISSION_ITEM_INT_DATA {
            param1: 1.0,
            param2: 2.0,
            param3: 3.0,
            param4: 4.0,
            x: 473_977_420,
            y: 85_455_940,
            z: 100.0,
            seq: 0,
            command: common::MavCmd::MAV_CMD_NAV_WAYPOINT,
            target_system: 1,
            target_component: 1,
            frame: common::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
            current: 1,
            autocontinue: 1,
            mission_type: common::MavMissionType::MAV_MISSION_TYPE_MISSION,
        };
        let item = from_mission_item_int(&data);
        assert_eq!(item.seq, 0);
        assert_eq!(item.x, 473_977_420);
        assert_eq!(item.y, 85_455_940);
        assert_eq!(item.z, 100.0);
        assert!(item.current);
        assert!(item.autocontinue);
        assert_eq!(item.frame, MissionFrame::GlobalRelativeAltInt);
    }

    #[test]
    fn param_type_conversions() {
        use mavlink::common::MavParamType;
        let types = [
            (
                MavParamType::MAV_PARAM_TYPE_UINT8,
                crate::params::ParamType::Uint8,
            ),
            (
                MavParamType::MAV_PARAM_TYPE_INT8,
                crate::params::ParamType::Int8,
            ),
            (
                MavParamType::MAV_PARAM_TYPE_UINT16,
                crate::params::ParamType::Uint16,
            ),
            (
                MavParamType::MAV_PARAM_TYPE_INT16,
                crate::params::ParamType::Int16,
            ),
            (
                MavParamType::MAV_PARAM_TYPE_UINT32,
                crate::params::ParamType::Uint32,
            ),
            (
                MavParamType::MAV_PARAM_TYPE_INT32,
                crate::params::ParamType::Int32,
            ),
            (
                MavParamType::MAV_PARAM_TYPE_REAL32,
                crate::params::ParamType::Real32,
            ),
        ];
        for (mav, kit) in types {
            assert_eq!(from_mav_param_type(mav), kit);
            assert_eq!(to_mav_param_type(kit), mav);
        }
    }

    #[test]
    fn param_id_string_roundtrip() {
        let name = "BATT_CAPACITY";
        let id = string_to_param_id(name);
        let back = param_id_to_string(&id);
        assert_eq!(back, name);
    }

    #[test]
    fn mission_type_matches_works() {
        // Mission type matches itself and the generic MISSION type
        assert!(mission_type_matches(
            common::MavMissionType::MAV_MISSION_TYPE_MISSION,
            MissionType::Mission
        ));
        assert!(!mission_type_matches(
            common::MavMissionType::MAV_MISSION_TYPE_FENCE,
            MissionType::Mission
        ));
        assert!(mission_type_matches(
            common::MavMissionType::MAV_MISSION_TYPE_FENCE,
            MissionType::Fence
        ));
        assert!(mission_type_matches(
            common::MavMissionType::MAV_MISSION_TYPE_RALLY,
            MissionType::Rally
        ));
        assert!(!mission_type_matches(
            common::MavMissionType::MAV_MISSION_TYPE_RALLY,
            MissionType::Fence
        ));
    }

    #[tokio::test]
    async fn auto_request_home_sends_request_message() {
        let (msg_tx, msg_rx) = mpsc::channel(64);
        let (conn, sent) = MockConnection::new(msg_rx);

        let (cmd_tx, cmd_rx) = mpsc::channel(16);
        let (writers, _channels) = create_channels();
        let cancel = CancellationToken::new();

        let mut config = fast_config();
        config.auto_request_home = true;

        let handle = tokio::spawn(async move {
            run_event_loop(Box::new(conn), cmd_rx, writers, config, cancel).await;
        });

        // Send heartbeat to establish target — this should trigger home request
        msg_tx
            .send((default_header(), heartbeat_msg(false, 0)))
            .await
            .unwrap();
        tokio::time::sleep(Duration::from_millis(20)).await;

        // Verify REQUEST_MESSAGE was sent for HOME_POSITION (msg id 242)
        {
            let sent_msgs = sent.lock().unwrap();
            let home_req = sent_msgs.iter().find(|(_, msg)| {
                matches!(
                    msg,
                    common::MavMessage::COMMAND_LONG(d) if d.command == MavCmd::MAV_CMD_REQUEST_MESSAGE && d.param1 == 242.0
                )
            });
            assert!(home_req.is_some(), "should have requested home position");
        }

        cmd_tx.send(Command::Shutdown).await.unwrap();
        handle.await.unwrap();
    }

    #[tokio::test]
    async fn auto_request_home_only_sent_once() {
        let (msg_tx, msg_rx) = mpsc::channel(64);
        let (conn, sent) = MockConnection::new(msg_rx);

        let (cmd_tx, cmd_rx) = mpsc::channel(16);
        let (writers, _channels) = create_channels();
        let cancel = CancellationToken::new();

        let mut config = fast_config();
        config.auto_request_home = true;

        let handle = tokio::spawn(async move {
            run_event_loop(Box::new(conn), cmd_rx, writers, config, cancel).await;
        });

        // Send two heartbeats
        msg_tx
            .send((default_header(), heartbeat_msg(false, 0)))
            .await
            .unwrap();
        tokio::time::sleep(Duration::from_millis(10)).await;
        msg_tx
            .send((default_header(), heartbeat_msg(false, 0)))
            .await
            .unwrap();
        tokio::time::sleep(Duration::from_millis(10)).await;

        {
            let sent_msgs = sent.lock().unwrap();
            let home_reqs: Vec<_> = sent_msgs
                .iter()
                .filter(|(_, msg)| {
                    matches!(
                        msg,
                        common::MavMessage::COMMAND_LONG(d) if d.command == MavCmd::MAV_CMD_REQUEST_MESSAGE && d.param1 == 242.0
                    )
                })
                .collect();
            assert_eq!(home_reqs.len(), 1, "home should only be requested once");
        }

        cmd_tx.send(Command::Shutdown).await.unwrap();
        handle.await.unwrap();
    }

    #[test]
    fn battery_status_updates_telemetry() {
        let (writers, channels) = create_channels();
        let target: Option<VehicleTarget> = None;
        let header = default_header();

        let mut voltages = [u16::MAX; 10];
        voltages[0] = 4200; // 4.2V
        voltages[1] = 4150; // 4.15V
        voltages[2] = 4100; // 4.1V

        let msg = common::MavMessage::BATTERY_STATUS(common::BATTERY_STATUS_DATA {
            current_consumed: 0,
            energy_consumed: 7200, // 200 Wh
            temperature: 0,
            voltages,
            current_battery: 0,
            id: 0,
            battery_function: common::MavBatteryFunction::MAV_BATTERY_FUNCTION_UNKNOWN,
            mavtype: common::MavBatteryType::MAV_BATTERY_TYPE_UNKNOWN,
            battery_remaining: 0,
            time_remaining: 1800, // 30 minutes
            charge_state: common::MavBatteryChargeState::MAV_BATTERY_CHARGE_STATE_OK,
            voltages_ext: [0; 4],
            mode: common::MavBatteryMode::MAV_BATTERY_MODE_UNKNOWN,
            fault_bitmask: common::MavBatteryFault::empty(),
        });
        update_state(&header, &msg, &writers, &target);

        let t = channels.telemetry.borrow();
        let cells = t.battery_voltage_cells.as_ref().unwrap();
        assert_eq!(cells.len(), 3);
        assert!((cells[0] - 4.2).abs() < 0.001);
        assert!((cells[1] - 4.15).abs() < 0.001);
        assert_eq!(t.battery_time_remaining_s, Some(1800));
        // energy_consumed: 7200 / 36.0 = 200.0 Wh
        assert!((t.energy_consumed_wh.unwrap() - 200.0).abs() < 0.01);
    }

    #[test]
    fn rc_channels_updates_telemetry() {
        let (writers, channels) = create_channels();
        let target: Option<VehicleTarget> = None;
        let header = default_header();
        let msg = common::MavMessage::RC_CHANNELS(common::RC_CHANNELS_DATA {
            time_boot_ms: 0,
            chancount: 4,
            chan1_raw: 1500,
            chan2_raw: 1500,
            chan3_raw: 1000,
            chan4_raw: 1500,
            chan5_raw: 0,
            chan6_raw: 0,
            chan7_raw: 0,
            chan8_raw: 0,
            chan9_raw: 0,
            chan10_raw: 0,
            chan11_raw: 0,
            chan12_raw: 0,
            chan13_raw: 0,
            chan14_raw: 0,
            chan15_raw: 0,
            chan16_raw: 0,
            chan17_raw: 0,
            chan18_raw: 0,
            rssi: 200,
        });
        update_state(&header, &msg, &writers, &target);

        let t = channels.telemetry.borrow();
        let rc = t.rc_channels.as_ref().unwrap();
        assert_eq!(rc.len(), 4);
        assert_eq!(rc[2], 1000);
        assert_eq!(t.rc_rssi, Some(200));
    }

    #[test]
    fn servo_output_updates_telemetry() {
        let (writers, channels) = create_channels();
        let target: Option<VehicleTarget> = None;
        let header = default_header();
        let msg = common::MavMessage::SERVO_OUTPUT_RAW(common::SERVO_OUTPUT_RAW_DATA {
            time_usec: 0,
            port: 0,
            servo1_raw: 1100,
            servo2_raw: 1200,
            servo3_raw: 1300,
            servo4_raw: 1400,
            servo5_raw: 0,
            servo6_raw: 0,
            servo7_raw: 0,
            servo8_raw: 0,
            servo9_raw: 0,
            servo10_raw: 0,
            servo11_raw: 0,
            servo12_raw: 0,
            servo13_raw: 0,
            servo14_raw: 0,
            servo15_raw: 0,
            servo16_raw: 0,
        });
        update_state(&header, &msg, &writers, &target);

        let t = channels.telemetry.borrow();
        let servos = t.servo_outputs.as_ref().unwrap();
        assert_eq!(servos.len(), 16);
        assert_eq!(servos[0], 1100);
        assert_eq!(servos[3], 1400);
    }

    #[test]
    fn empty_statustext_ignored() {
        let (writers, channels) = create_channels();
        let target: Option<VehicleTarget> = None;
        let header = default_header();
        let msg = common::MavMessage::STATUSTEXT(common::STATUSTEXT_DATA {
            severity: common::MavSeverity::MAV_SEVERITY_INFO,
            text: "".into(),
            id: 0,
            chunk_seq: 0,
        });
        update_state(&header, &msg, &writers, &target);

        let st = channels.statustext.borrow();
        assert!(st.is_none());
    }

    #[test]
    fn sys_status_sentinel_values_ignored() {
        let (writers, channels) = create_channels();
        let target: Option<VehicleTarget> = None;
        let header = default_header();
        // Use sentinel values that should not update telemetry
        let msg = common::MavMessage::SYS_STATUS(common::SYS_STATUS_DATA {
            onboard_control_sensors_present: common::MavSysStatusSensor::empty(),
            onboard_control_sensors_enabled: common::MavSysStatusSensor::empty(),
            onboard_control_sensors_health: common::MavSysStatusSensor::empty(),
            load: 0,
            voltage_battery: u16::MAX, // sentinel
            current_battery: -1,       // sentinel
            battery_remaining: -1,     // sentinel
            drop_rate_comm: 0,
            errors_comm: 0,
            errors_count1: 0,
            errors_count2: 0,
            errors_count3: 0,
            errors_count4: 0,
            onboard_control_sensors_present_extended: common::MavSysStatusSensorExtended::empty(),
            onboard_control_sensors_enabled_extended: common::MavSysStatusSensorExtended::empty(),
            onboard_control_sensors_health_extended: common::MavSysStatusSensorExtended::empty(),
        });
        update_state(&header, &msg, &writers, &target);

        let t = channels.telemetry.borrow();
        assert_eq!(t.battery_pct, None);
        assert_eq!(t.battery_voltage_v, None);
        assert_eq!(t.battery_current_a, None);
    }

    #[test]
    fn gps_sentinel_values_ignored() {
        let (writers, channels) = create_channels();
        let target: Option<VehicleTarget> = None;
        let header = default_header();
        let msg = common::MavMessage::GPS_RAW_INT(common::GPS_RAW_INT_DATA {
            time_usec: 0,
            fix_type: common::GpsFixType::GPS_FIX_TYPE_NO_FIX,
            lat: 0,
            lon: 0,
            alt: 0,
            eph: u16::MAX, // sentinel
            epv: u16::MAX,
            vel: 0,
            cog: 0,
            satellites_visible: u8::MAX, // sentinel
            alt_ellipsoid: 0,
            h_acc: 0,
            v_acc: 0,
            vel_acc: 0,
            hdg_acc: 0,
            yaw: 0,
        });
        update_state(&header, &msg, &writers, &target);

        let t = channels.telemetry.borrow();
        assert_eq!(t.gps_satellites, None);
        assert_eq!(t.gps_hdop, None);
    }

    #[test]
    fn global_position_int_hdg_max_ignored() {
        let (writers, channels) = create_channels();
        let target: Option<VehicleTarget> = None;
        let header = default_header();
        let msg = common::MavMessage::GLOBAL_POSITION_INT(common::GLOBAL_POSITION_INT_DATA {
            time_boot_ms: 0,
            lat: 0,
            lon: 0,
            alt: 0,
            relative_alt: 10_000,
            vx: 0,
            vy: 0,
            vz: 0,
            hdg: u16::MAX, // sentinel — should not update heading
        });
        update_state(&header, &msg, &writers, &target);

        let t = channels.telemetry.borrow();
        assert_eq!(t.heading_deg, None);
    }
}
