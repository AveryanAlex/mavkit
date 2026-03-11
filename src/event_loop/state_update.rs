use super::VehicleTarget;
use crate::mission;
use crate::state::{
    set_if_changed, AutopilotType, GpsFixType, MagCalReport, MagCalStatus, MissionState,
    SensorHealth, StateWriters, SystemStatus, VehicleState, VehicleType,
};
use mavlink::common::{self, MavModeFlag};
use mavlink::MavHeader;
use tracing::trace;

/// Maximum number of RC channels in the RC_CHANNELS MAVLink message.
const RC_CHANNELS_MAX: usize = 18;

pub(super) fn update_state(
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
                    heartbeat_received: true,
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
            let new_health = SensorHealth::from_bitmasks(
                data.onboard_control_sensors_present.bits(),
                data.onboard_control_sensors_enabled.bits(),
                data.onboard_control_sensors_health.bits(),
            );
            writers
                .sensor_health
                .send_if_modified(|h| set_if_changed(h, new_health.clone()));
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
            let count = (data.chancount as usize).min(RC_CHANNELS_MAX);
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
        common::MavMessage::MAG_CAL_REPORT(data) => {
            let _ = writers.mag_cal_report.send(Some(MagCalReport {
                compass_id: data.compass_id,
                status: MagCalStatus::from_mav(data.cal_status),
                fitness: data.fitness,
                ofs_x: data.ofs_x,
                ofs_y: data.ofs_y,
                ofs_z: data.ofs_z,
                autosaved: data.autosaved != 0,
            }));
        }
        _ => {
            trace!("unhandled message type");
        }
    }
}
