use super::VehicleTarget;
use crate::dialect::{self, MavModeFlag};
use crate::geo::GeoPoint3dMsl;
use crate::mission;
use crate::state::{
    AutopilotType, GpsFixType, MagCalProgress, MagCalReport, MagCalStatus, SensorHealth,
    StateWriters, SystemStatus, VehicleState, VehicleType, WireMissionState, set_if_changed,
};
use crate::telemetry::{
    EulerAttitude, GlobalPosition, GpsQuality, GuidanceState, TelemetryMessageKind,
    TerrainClearance, WaypointProgress, infer_timestamp_from_time_boot_ms,
    infer_timestamp_from_time_usec, sensor_health::sensor_health_summary_from_sys_status,
};
use mavlink::MavHeader;
use tracing::trace;

/// Maximum number of RC channels in the RC_CHANNELS MAVLink message.
const RC_CHANNELS_MAX: usize = 18;

pub(super) fn update_state(
    _header: &MavHeader,
    message: &dialect::MavMessage,
    writers: &StateWriters,
    vehicle_target: &Option<VehicleTarget>,
) {
    match message {
        dialect::MavMessage::HEARTBEAT(hb) => {
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

                writers.telemetry_metrics.armed.publish(
                    armed,
                    TelemetryMessageKind::Heartbeat,
                    None,
                );
            }
        }
        dialect::MavMessage::VFR_HUD(data) => {
            writers
                .telemetry_metrics
                .message_writers
                .vfr_hud
                .publish(data.clone(), None);

            writers.telemetry_metrics.position_groundspeed_mps.publish(
                f64::from(data.groundspeed),
                TelemetryMessageKind::VfrHud,
                None,
            );
            writers.telemetry_metrics.position_airspeed_mps.publish(
                f64::from(data.airspeed),
                TelemetryMessageKind::VfrHud,
                None,
            );
            writers.telemetry_metrics.position_climb_rate_mps.publish(
                f64::from(data.climb),
                TelemetryMessageKind::VfrHud,
                None,
            );
            writers.telemetry_metrics.position_heading_deg.publish(
                f64::from(data.heading),
                TelemetryMessageKind::VfrHud,
                None,
            );
            writers.telemetry_metrics.position_throttle_pct.publish(
                f64::from(data.throttle),
                TelemetryMessageKind::VfrHud,
                None,
            );
        }
        dialect::MavMessage::GLOBAL_POSITION_INT(data) => {
            let vx = f64::from(data.vx) / 100.0;
            let vy = f64::from(data.vy) / 100.0;

            let vehicle_time = infer_timestamp_from_time_boot_ms(
                TelemetryMessageKind::GlobalPositionInt,
                data.time_boot_ms,
            );
            writers
                .telemetry_metrics
                .message_writers
                .global_position_int
                .publish(data.clone(), vehicle_time.clone());
            writers.telemetry_metrics.position_global.publish(
                GlobalPosition {
                    latitude_deg: f64::from(data.lat) / 1e7,
                    longitude_deg: f64::from(data.lon) / 1e7,
                    altitude_msl_m: f64::from(data.alt) / 1000.0,
                    relative_alt_m: f64::from(data.relative_alt) / 1000.0,
                },
                TelemetryMessageKind::GlobalPositionInt,
                vehicle_time.clone(),
            );

            let ground_speed = (vx * vx + vy * vy).sqrt();
            writers.telemetry_metrics.position_groundspeed_mps.publish(
                ground_speed,
                TelemetryMessageKind::GlobalPositionInt,
                vehicle_time.clone(),
            );

            if data.hdg != u16::MAX {
                writers.telemetry_metrics.position_heading_deg.publish(
                    f64::from(data.hdg) / 100.0,
                    TelemetryMessageKind::GlobalPositionInt,
                    vehicle_time.clone(),
                );
            }

            writers.telemetry_metrics.position_climb_rate_mps.publish(
                -(f64::from(data.vz) / 100.0),
                TelemetryMessageKind::GlobalPositionInt,
                vehicle_time,
            );
        }
        dialect::MavMessage::LOCAL_POSITION_NED(data) => {
            writers
                .telemetry_metrics
                .message_writers
                .local_position_ned
                .publish(
                    data.clone(),
                    infer_timestamp_from_time_boot_ms(
                        TelemetryMessageKind::LocalPositionNed,
                        data.time_boot_ms,
                    ),
                );
        }
        dialect::MavMessage::SYS_STATUS(data) => {
            writers
                .telemetry_metrics
                .message_writers
                .sys_status
                .publish(data.clone(), None);
            let pct = if data.battery_remaining >= 0 {
                Some(f64::from(data.battery_remaining))
            } else {
                None
            };
            let volt = if data.voltage_battery != u16::MAX {
                Some(f64::from(data.voltage_battery) / 1000.0)
            } else {
                None
            };
            let cur = if data.current_battery >= 0 {
                Some(f64::from(data.current_battery) / 100.0)
            } else {
                None
            };

            writers
                .telemetry_metrics
                .publish_battery_from_sys_status(pct, volt, cur);

            let new_health = SensorHealth::from_bitmasks(
                data.onboard_control_sensors_present.bits(),
                data.onboard_control_sensors_enabled.bits(),
                data.onboard_control_sensors_health.bits(),
            );
            writers
                .sensor_health
                .send_if_modified(|h| set_if_changed(h, new_health.clone()));
            writers.telemetry_metrics.sensor_health.publish(
                sensor_health_summary_from_sys_status(
                    data.onboard_control_sensors_present,
                    data.onboard_control_sensors_enabled,
                    data.onboard_control_sensors_health,
                ),
                TelemetryMessageKind::SysStatus,
                None,
            );
        }
        dialect::MavMessage::GPS_RAW_INT(data) => {
            let sats = if data.satellites_visible != u8::MAX {
                Some(data.satellites_visible)
            } else {
                None
            };
            let hdop = if data.eph != u16::MAX {
                Some(f64::from(data.eph) / 100.0)
            } else {
                None
            };

            let vehicle_time =
                infer_timestamp_from_time_usec(TelemetryMessageKind::GpsRawInt, data.time_usec);
            writers
                .telemetry_metrics
                .message_writers
                .gps_raw_int
                .publish(data.clone(), vehicle_time.clone());
            writers.telemetry_metrics.gps_quality.publish(
                GpsQuality {
                    fix_type: GpsFixType::from_raw(data.fix_type as u8),
                    satellites: sats,
                    hdop,
                },
                TelemetryMessageKind::GpsRawInt,
                vehicle_time.clone(),
            );
            writers.telemetry_metrics.gps_position_msl.publish(
                GeoPoint3dMsl {
                    latitude_deg: f64::from(data.lat) / 1e7,
                    longitude_deg: f64::from(data.lon) / 1e7,
                    altitude_msl_m: f64::from(data.alt) / 1000.0,
                },
                TelemetryMessageKind::GpsRawInt,
                vehicle_time,
            );
        }
        dialect::MavMessage::MISSION_CURRENT(data) => {
            // MISSION_CURRENT always reports Mission type execution state
            let _ = writers.mission_state.send(WireMissionState::from_wire(
                mission::MissionType::Mission,
                data.seq,
                data.total,
            ));
        }
        dialect::MavMessage::HOME_POSITION(data) => {
            let _ = writers.home_position.send(Some(mission::HomePosition {
                latitude_deg: f64::from(data.latitude) / 1e7,
                longitude_deg: f64::from(data.longitude) / 1e7,
                altitude_m: f64::from(data.altitude) / 1000.0,
            }));

            let vehicle_time =
                infer_timestamp_from_time_usec(TelemetryMessageKind::HomePosition, data.time_usec);
            writers
                .telemetry_metrics
                .message_writers
                .home_position
                .publish(data.clone(), vehicle_time.clone());
            writers.telemetry_metrics.home.publish(
                GeoPoint3dMsl {
                    latitude_deg: f64::from(data.latitude) / 1e7,
                    longitude_deg: f64::from(data.longitude) / 1e7,
                    altitude_msl_m: f64::from(data.altitude) / 1000.0,
                },
                TelemetryMessageKind::HomePosition,
                vehicle_time,
            );
        }
        dialect::MavMessage::GPS_GLOBAL_ORIGIN(data) => {
            let vehicle_time = infer_timestamp_from_time_usec(
                TelemetryMessageKind::GpsGlobalOrigin,
                data.time_usec,
            );
            writers
                .telemetry_metrics
                .message_writers
                .gps_global_origin
                .publish(data.clone(), vehicle_time.clone());
            writers.telemetry_metrics.origin.publish(
                GeoPoint3dMsl {
                    latitude_deg: f64::from(data.latitude) / 1e7,
                    longitude_deg: f64::from(data.longitude) / 1e7,
                    altitude_msl_m: f64::from(data.altitude) / 1000.0,
                },
                TelemetryMessageKind::GpsGlobalOrigin,
                vehicle_time,
            );
        }
        dialect::MavMessage::ATTITUDE(data) => {
            let vehicle_time = infer_timestamp_from_time_boot_ms(
                TelemetryMessageKind::Attitude,
                data.time_boot_ms,
            );
            writers
                .telemetry_metrics
                .message_writers
                .attitude
                .publish(data.clone(), vehicle_time.clone());
            writers.telemetry_metrics.attitude_euler.publish(
                EulerAttitude {
                    roll_deg: f64::from(data.roll.to_degrees()),
                    pitch_deg: f64::from(data.pitch.to_degrees()),
                    yaw_deg: f64::from(data.yaw.to_degrees()),
                },
                TelemetryMessageKind::Attitude,
                vehicle_time,
            );
        }
        dialect::MavMessage::NAV_CONTROLLER_OUTPUT(data) => {
            writers
                .telemetry_metrics
                .message_writers
                .nav_controller_output
                .publish(data.clone(), None);

            writers.telemetry_metrics.navigation_waypoint.publish(
                WaypointProgress {
                    distance_m: f64::from(data.wp_dist),
                    bearing_deg: f64::from(data.nav_bearing),
                },
                TelemetryMessageKind::NavControllerOutput,
                None,
            );
            writers.telemetry_metrics.navigation_guidance.publish(
                GuidanceState {
                    bearing_deg: f64::from(data.target_bearing),
                    cross_track_error_m: f64::from(data.xtrack_error),
                },
                TelemetryMessageKind::NavControllerOutput,
                None,
            );
        }
        dialect::MavMessage::TERRAIN_REPORT(data) => {
            writers
                .telemetry_metrics
                .message_writers
                .terrain_report
                .publish(data.clone(), None);

            writers.telemetry_metrics.terrain_clearance.publish(
                TerrainClearance {
                    terrain_height_m: f64::from(data.terrain_height),
                    height_above_terrain_m: f64::from(data.current_height),
                },
                TelemetryMessageKind::TerrainReport,
                None,
            );
        }
        dialect::MavMessage::BATTERY_STATUS(data) => {
            writers
                .telemetry_metrics
                .message_writers
                .battery_status
                .publish(data.id, data.clone(), None);
            let cells: Vec<f64> = data
                .voltages
                .iter()
                .filter(|&&v| v != u16::MAX)
                .map(|&v| f64::from(v) / 1000.0)
                .collect();
            let cells_opt = if !cells.is_empty() { Some(cells) } else { None };
            let energy = if data.energy_consumed >= 0 {
                Some(f64::from(data.energy_consumed) / 36.0)
            } else {
                None
            };
            let remaining = if data.time_remaining > 0 {
                Some(data.time_remaining)
            } else {
                None
            };

            if data.id == 0 {
                let remaining_pct = if data.battery_remaining >= 0 {
                    Some(f64::from(data.battery_remaining))
                } else {
                    None
                };
                let current_a = if data.current_battery >= 0 {
                    Some(f64::from(data.current_battery) / 100.0)
                } else {
                    None
                };

                writers
                    .telemetry_metrics
                    .publish_battery_from_primary_status(
                        remaining_pct,
                        cells_opt,
                        current_a,
                        energy,
                        remaining,
                    );
            }
        }
        dialect::MavMessage::RC_CHANNELS(data) => {
            let count = (data.chancount as usize).min(RC_CHANNELS_MAX);
            let rssi = if data.rssi != u8::MAX {
                Some(data.rssi)
            } else {
                None
            };

            let vehicle_time = infer_timestamp_from_time_boot_ms(
                TelemetryMessageKind::RcChannels,
                data.time_boot_ms,
            );
            writers
                .telemetry_metrics
                .message_writers
                .rc_channels
                .publish(data.clone(), vehicle_time.clone());
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

            for (index, value) in all.iter().enumerate().take(count) {
                writers.telemetry_metrics.rc_channels_pwm_us[index].publish(
                    *value,
                    TelemetryMessageKind::RcChannels,
                    vehicle_time.clone(),
                );
            }

            for index in count..RC_CHANNELS_MAX {
                writers.telemetry_metrics.rc_channels_pwm_us[index].clear();
            }

            if let Some(rssi_pct) = rssi {
                writers.telemetry_metrics.rc_rssi_pct.publish(
                    rssi_pct,
                    TelemetryMessageKind::RcChannels,
                    vehicle_time,
                );
            }
        }
        dialect::MavMessage::SERVO_OUTPUT_RAW(data) => {
            let vehicle_time = infer_timestamp_from_time_usec(
                TelemetryMessageKind::ServoOutputRaw,
                u64::from(data.time_usec),
            );
            writers
                .telemetry_metrics
                .message_writers
                .servo_output_raw
                .publish(data.port, data.clone(), vehicle_time.clone());
            let values = [
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
            ];
            for (index, value) in values.iter().enumerate() {
                writers.telemetry_metrics.actuator_servo_pwm_us[index].publish(
                    *value,
                    TelemetryMessageKind::ServoOutputRaw,
                    vehicle_time.clone(),
                );
            }
        }
        dialect::MavMessage::STATUSTEXT(data) => {
            if let Some(status_text) = writers
                .telemetry_metrics
                .message_writers
                .status_text
                .ingest(_header, data, None)
            {
                let _ = writers.statustext.send(Some(crate::state::StatusMessage {
                    text: status_text.text,
                    severity: crate::state::MavSeverity::from_mav(status_text.severity),
                }));
            }
        }
        dialect::MavMessage::MAG_CAL_REPORT(data) => {
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
        dialect::MavMessage::MAG_CAL_PROGRESS(data) => {
            let _ = writers.mag_cal_progress.send(Some(MagCalProgress {
                compass_id: data.compass_id,
                completion_pct: data.completion_pct,
                status: MagCalStatus::from_mav(data.cal_status),
                attempt: data.attempt,
            }));
        }
        _ => {
            trace!("unhandled message type");
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dialect;
    use crate::state::create_channels;
    use crate::telemetry::TelemetryHandle;
    use crate::types::SensorHealthState;
    use std::time::Duration;

    fn exact_50_char_text(value: &str) -> mavlink::types::CharArray<50> {
        assert_eq!(value.len(), 50, "test chunk must be exactly 50 chars");
        value.into()
    }

    fn default_header() -> MavHeader {
        MavHeader {
            system_id: 1,
            component_id: 1,
            sequence: 0,
        }
    }

    #[test]
    fn metric_source_map_global_position_and_vfr_hud() {
        let (writers, channels) = create_channels();
        let telemetry = TelemetryHandle::new(&channels.telemetry_handles);
        let target: Option<VehicleTarget> = None;

        let global_position =
            dialect::MavMessage::GLOBAL_POSITION_INT(dialect::GLOBAL_POSITION_INT_DATA {
                time_boot_ms: 250,
                lat: 473_977_420,
                lon: 85_455_940,
                alt: 510_000,
                relative_alt: 50_000,
                vx: 120,
                vy: 50,
                vz: -80,
                hdg: 27_000,
            });
        update_state(&default_header(), &global_position, &writers, &target);

        let global_sample = telemetry.position().global().latest().unwrap();
        assert_eq!(
            global_sample.source,
            crate::TelemetryMessageKind::GlobalPositionInt
        );
        assert!((global_sample.value.latitude_deg - 47.397742).abs() < 0.0001);
        assert!((global_sample.value.longitude_deg - 8.545594).abs() < 0.0001);
        assert!((global_sample.value.altitude_msl_m - 510.0).abs() < 0.001);
        assert!((global_sample.value.relative_alt_m - 50.0).abs() < 0.001);

        let vfr_hud = dialect::MavMessage::VFR_HUD(dialect::VFR_HUD_DATA {
            airspeed: 12.5,
            groundspeed: 10.0,
            heading: 180,
            throttle: 55,
            alt: 100.0,
            climb: 2.5,
        });
        update_state(&default_header(), &vfr_hud, &writers, &target);

        assert_eq!(
            telemetry
                .position()
                .groundspeed_mps()
                .latest()
                .unwrap()
                .source,
            crate::TelemetryMessageKind::VfrHud
        );
        assert_eq!(
            telemetry
                .position()
                .groundspeed_mps()
                .latest()
                .unwrap()
                .value,
            10.0
        );
        assert_eq!(
            telemetry.position().heading_deg().latest().unwrap().value,
            180.0
        );
        assert_eq!(
            telemetry
                .position()
                .climb_rate_mps()
                .latest()
                .unwrap()
                .value,
            2.5
        );
        assert_eq!(
            telemetry.position().airspeed_mps().latest().unwrap().value,
            12.5
        );
        assert_eq!(
            telemetry.position().throttle_pct().latest().unwrap().value,
            55.0
        );
    }

    #[test]
    fn battery_voltage_prefers_primary_battery_status_with_sys_status_fallback() {
        let (writers, channels) = create_channels();
        let telemetry = TelemetryHandle::new(&channels.telemetry_handles);
        let target: Option<VehicleTarget> = None;

        let sys_status = dialect::MavMessage::SYS_STATUS(dialect::SYS_STATUS_DATA {
            onboard_control_sensors_present: dialect::MavSysStatusSensor::empty(),
            onboard_control_sensors_enabled: dialect::MavSysStatusSensor::empty(),
            onboard_control_sensors_health: dialect::MavSysStatusSensor::empty(),
            load: 0,
            voltage_battery: 12_600,
            current_battery: 520,
            battery_remaining: 71,
            drop_rate_comm: 0,
            errors_comm: 0,
            errors_count1: 0,
            errors_count2: 0,
            errors_count3: 0,
            errors_count4: 0,
            onboard_control_sensors_present_extended: dialect::MavSysStatusSensorExtended::empty(),
            onboard_control_sensors_enabled_extended: dialect::MavSysStatusSensorExtended::empty(),
            onboard_control_sensors_health_extended: dialect::MavSysStatusSensorExtended::empty(),
        });
        update_state(&default_header(), &sys_status, &writers, &target);

        let sys_fallback_sample = telemetry.battery().voltage_v().latest().unwrap();
        assert_eq!(
            sys_fallback_sample.source,
            crate::TelemetryMessageKind::SysStatus
        );
        assert!((sys_fallback_sample.value - 12.6).abs() < 0.001);

        let mut ignored_instance_cells = [u16::MAX; 10];
        ignored_instance_cells[0] = 4300;
        ignored_instance_cells[1] = 4200;
        ignored_instance_cells[2] = 4100;
        let battery_status_non_primary =
            dialect::MavMessage::BATTERY_STATUS(dialect::BATTERY_STATUS_DATA {
                current_consumed: -1,
                energy_consumed: -1,
                temperature: i16::MAX,
                voltages: ignored_instance_cells,
                current_battery: 600,
                id: 1,
                battery_function: dialect::MavBatteryFunction::MAV_BATTERY_FUNCTION_UNKNOWN,
                mavtype: dialect::MavBatteryType::MAV_BATTERY_TYPE_UNKNOWN,
                battery_remaining: 68,
                time_remaining: 0,
                charge_state: dialect::MavBatteryChargeState::MAV_BATTERY_CHARGE_STATE_OK,
                voltages_ext: [0; 4],
                mode: dialect::MavBatteryMode::MAV_BATTERY_MODE_UNKNOWN,
                fault_bitmask: dialect::MavBatteryFault::empty(),
            });
        update_state(
            &default_header(),
            &battery_status_non_primary,
            &writers,
            &target,
        );

        let still_sys_sample = telemetry.battery().voltage_v().latest().unwrap();
        assert_eq!(
            still_sys_sample.source,
            crate::TelemetryMessageKind::SysStatus
        );
        assert!((still_sys_sample.value - 12.6).abs() < 0.001);

        let mut cells = [u16::MAX; 10];
        cells[0] = 4200;
        cells[1] = 4150;
        cells[2] = 4100;
        let battery_status_primary =
            dialect::MavMessage::BATTERY_STATUS(dialect::BATTERY_STATUS_DATA {
                current_consumed: 0,
                energy_consumed: 7200,
                temperature: 0,
                voltages: cells,
                current_battery: 550,
                id: 0,
                battery_function: dialect::MavBatteryFunction::MAV_BATTERY_FUNCTION_UNKNOWN,
                mavtype: dialect::MavBatteryType::MAV_BATTERY_TYPE_UNKNOWN,
                battery_remaining: 66,
                time_remaining: 1800,
                charge_state: dialect::MavBatteryChargeState::MAV_BATTERY_CHARGE_STATE_OK,
                voltages_ext: [0; 4],
                mode: dialect::MavBatteryMode::MAV_BATTERY_MODE_UNKNOWN,
                fault_bitmask: dialect::MavBatteryFault::empty(),
            });
        update_state(
            &default_header(),
            &battery_status_primary,
            &writers,
            &target,
        );

        let primary_sample = telemetry.battery().voltage_v().latest().unwrap();
        assert_eq!(
            primary_sample.source,
            crate::TelemetryMessageKind::BatteryStatus
        );
        assert!((primary_sample.value - 12.45).abs() < 0.001);

        let sys_status_new = dialect::MavMessage::SYS_STATUS(dialect::SYS_STATUS_DATA {
            voltage_battery: 11_100,
            current_battery: 250,
            battery_remaining: 51,
            ..dialect::SYS_STATUS_DATA::default()
        });
        update_state(&default_header(), &sys_status_new, &writers, &target);

        let after_primary_seen_sample = telemetry.battery().voltage_v().latest().unwrap();
        assert_eq!(
            after_primary_seen_sample.source,
            crate::TelemetryMessageKind::BatteryStatus
        );
        assert!((after_primary_seen_sample.value - 12.45).abs() < 0.001);
    }

    #[test]
    fn gps_hdop_conversion_uses_centi_units_and_sentinel() {
        let (writers, channels) = create_channels();
        let telemetry = TelemetryHandle::new(&channels.telemetry_handles);
        let target: Option<VehicleTarget> = None;

        let with_hdop = dialect::MavMessage::GPS_RAW_INT(dialect::GPS_RAW_INT_DATA {
            time_usec: 1_700_000_000_000_000,
            fix_type: dialect::GpsFixType::GPS_FIX_TYPE_3D_FIX,
            lat: 473_977_420,
            lon: 85_455_940,
            alt: 510_000,
            eph: 150,
            epv: u16::MAX,
            vel: u16::MAX,
            cog: u16::MAX,
            satellites_visible: 10,
            alt_ellipsoid: 0,
            h_acc: 0,
            v_acc: 0,
            vel_acc: 0,
            hdg_acc: 0,
            yaw: 0,
        });
        update_state(&default_header(), &with_hdop, &writers, &target);

        let hdop_sample = telemetry.gps().quality().latest().unwrap();
        assert_eq!(hdop_sample.source, crate::TelemetryMessageKind::GpsRawInt);
        assert_eq!(hdop_sample.value.hdop, Some(1.5));

        let sentinel_hdop = dialect::MavMessage::GPS_RAW_INT(dialect::GPS_RAW_INT_DATA {
            eph: u16::MAX,
            ..dialect::GPS_RAW_INT_DATA::default()
        });
        update_state(&default_header(), &sentinel_hdop, &writers, &target);

        let sentinel_sample = telemetry.gps().quality().latest().unwrap();
        assert_eq!(sentinel_sample.value.hdop, None);
    }

    #[test]
    fn grouped_metric_handles_refresh_from_expected_sources() {
        let (writers, channels) = create_channels();
        let telemetry = TelemetryHandle::new(&channels.telemetry_handles);
        let target: Option<VehicleTarget> = None;

        let gps_raw = dialect::MavMessage::GPS_RAW_INT(dialect::GPS_RAW_INT_DATA {
            time_usec: 1_700_000_000_000_000,
            fix_type: dialect::GpsFixType::GPS_FIX_TYPE_3D_FIX,
            lat: 473_977_420,
            lon: 85_455_940,
            alt: 510_000,
            eph: 150,
            satellites_visible: 12,
            ..dialect::GPS_RAW_INT_DATA::default()
        });
        update_state(&default_header(), &gps_raw, &writers, &target);

        let gps_quality = telemetry.gps().quality().latest().unwrap();
        assert_eq!(gps_quality.source, crate::TelemetryMessageKind::GpsRawInt);
        assert_eq!(gps_quality.value.hdop, Some(1.5));

        let gps_position = telemetry.gps().position_msl().latest().unwrap();
        assert_eq!(gps_position.source, crate::TelemetryMessageKind::GpsRawInt);
        assert!((gps_position.value.latitude_deg - 47.397742).abs() < 0.0001);
        assert!((gps_position.value.longitude_deg - 8.545594).abs() < 0.0001);
        assert!((gps_position.value.altitude_msl_m - 510.0).abs() < 0.001);

        let attitude = dialect::MavMessage::ATTITUDE(dialect::ATTITUDE_DATA {
            time_boot_ms: 250,
            roll: 0.1,
            pitch: -0.2,
            yaw: 1.3,
            ..dialect::ATTITUDE_DATA::default()
        });
        update_state(&default_header(), &attitude, &writers, &target);

        let euler = telemetry.attitude().euler().latest().unwrap();
        assert_eq!(euler.source, crate::TelemetryMessageKind::Attitude);
        assert!((euler.value.roll_deg - f64::from(0.1f32.to_degrees())).abs() < 0.001);
        assert!((euler.value.pitch_deg - f64::from((-0.2f32).to_degrees())).abs() < 0.001);
        assert!((euler.value.yaw_deg - f64::from(1.3f32.to_degrees())).abs() < 0.001);

        let nav_output =
            dialect::MavMessage::NAV_CONTROLLER_OUTPUT(dialect::NAV_CONTROLLER_OUTPUT_DATA {
                wp_dist: 33,
                nav_bearing: 210,
                target_bearing: 205,
                xtrack_error: 4.5,
                ..dialect::NAV_CONTROLLER_OUTPUT_DATA::default()
            });
        update_state(&default_header(), &nav_output, &writers, &target);

        let waypoint = telemetry.navigation().waypoint().latest().unwrap();
        assert_eq!(
            waypoint.source,
            crate::TelemetryMessageKind::NavControllerOutput
        );
        assert_eq!(waypoint.value.distance_m, 33.0);
        assert_eq!(waypoint.value.bearing_deg, 210.0);

        let guidance = telemetry.navigation().guidance().latest().unwrap();
        assert_eq!(
            guidance.source,
            crate::TelemetryMessageKind::NavControllerOutput
        );
        assert_eq!(guidance.value.bearing_deg, 205.0);
        assert!((guidance.value.cross_track_error_m - 4.5).abs() < 0.001);

        let terrain_report = dialect::MavMessage::TERRAIN_REPORT(dialect::TERRAIN_REPORT_DATA {
            terrain_height: 120.5,
            current_height: 32.0,
            ..dialect::TERRAIN_REPORT_DATA::default()
        });
        update_state(&default_header(), &terrain_report, &writers, &target);

        let terrain = telemetry.terrain().clearance().latest().unwrap();
        assert_eq!(terrain.source, crate::TelemetryMessageKind::TerrainReport);
        assert!((terrain.value.terrain_height_m - 120.5).abs() < 0.001);
        assert!((terrain.value.height_above_terrain_m - 32.0).abs() < 0.001);

        let mut cell_mv = [u16::MAX; 10];
        cell_mv[0] = 4200;
        cell_mv[1] = 4150;
        cell_mv[2] = 4100;
        let primary_battery = dialect::MavMessage::BATTERY_STATUS(dialect::BATTERY_STATUS_DATA {
            id: 0,
            voltages: cell_mv,
            ..dialect::BATTERY_STATUS_DATA::default()
        });
        update_state(&default_header(), &primary_battery, &writers, &target);

        let cells = telemetry.battery().cells().latest().unwrap();
        assert_eq!(cells.source, crate::TelemetryMessageKind::BatteryStatus);
        assert_eq!(cells.value.voltages_v, vec![4.2, 4.15, 4.1]);
    }

    #[test]
    fn home_and_origin_messages_update_grouped_geo_observations() {
        let (writers, channels) = create_channels();
        let telemetry = TelemetryHandle::new(&channels.telemetry_handles);
        let target: Option<VehicleTarget> = None;

        let home = dialect::MavMessage::HOME_POSITION(dialect::HOME_POSITION_DATA {
            latitude: 473_977_420,
            longitude: 85_455_940,
            altitude: 510_000,
            time_usec: 1_700_000_000_000_000,
            ..dialect::HOME_POSITION_DATA::default()
        });
        update_state(&default_header(), &home, &writers, &target);

        let home_sample = telemetry.home().latest().unwrap();
        assert_eq!(
            home_sample.source,
            crate::TelemetryMessageKind::HomePosition
        );
        assert!((home_sample.value.latitude_deg - 47.397742).abs() < 0.0001);
        assert!((home_sample.value.longitude_deg - 8.545594).abs() < 0.0001);
        assert!((home_sample.value.altitude_msl_m - 510.0).abs() < 0.001);

        let origin = dialect::MavMessage::GPS_GLOBAL_ORIGIN(dialect::GPS_GLOBAL_ORIGIN_DATA {
            latitude: 473_981_230,
            longitude: 85_463_210,
            altitude: 505_500,
            time_usec: 1_700_000_010_000_000,
        });
        update_state(&default_header(), &origin, &writers, &target);

        let origin_sample = telemetry.origin().latest().unwrap();
        assert_eq!(
            origin_sample.source,
            crate::TelemetryMessageKind::GpsGlobalOrigin
        );
        assert!((origin_sample.value.latitude_deg - 47.398123).abs() < 0.0001);
        assert!((origin_sample.value.longitude_deg - 8.546321).abs() < 0.0001);
        assert!((origin_sample.value.altitude_msl_m - 505.5).abs() < 0.001);
    }

    #[test]
    fn message_handles_receive_state_update_publications() {
        let (writers, channels) = create_channels();
        let telemetry = TelemetryHandle::new(&channels.telemetry_handles);
        let target: Option<VehicleTarget> = None;

        let vfr_hud = dialect::MavMessage::VFR_HUD(dialect::VFR_HUD_DATA {
            groundspeed: 10.0,
            ..dialect::VFR_HUD_DATA::default()
        });
        update_state(&default_header(), &vfr_hud, &writers, &target);
        assert_eq!(
            telemetry
                .messages()
                .vfr_hud()
                .latest()
                .unwrap()
                .value
                .groundspeed,
            10.0
        );

        let local_position =
            dialect::MavMessage::LOCAL_POSITION_NED(dialect::LOCAL_POSITION_NED_DATA {
                time_boot_ms: 250,
                x: 1.0,
                y: 2.0,
                z: -3.0,
                ..dialect::LOCAL_POSITION_NED_DATA::default()
            });
        update_state(&default_header(), &local_position, &writers, &target);
        let local_sample = telemetry.messages().local_position_ned().latest().unwrap();
        assert_eq!(local_sample.value.x, 1.0);
        assert_eq!(local_sample.value.y, 2.0);
        assert_eq!(local_sample.value.z, -3.0);

        let battery_status = dialect::MavMessage::BATTERY_STATUS(dialect::BATTERY_STATUS_DATA {
            id: 2,
            battery_remaining: 61,
            ..dialect::BATTERY_STATUS_DATA::default()
        });
        update_state(&default_header(), &battery_status, &writers, &target);
        assert_eq!(
            telemetry
                .messages()
                .battery_status(2)
                .latest()
                .unwrap()
                .value
                .battery_remaining,
            61
        );

        let servo_output = dialect::MavMessage::SERVO_OUTPUT_RAW(dialect::SERVO_OUTPUT_RAW_DATA {
            port: 1,
            servo1_raw: 1100,
            ..dialect::SERVO_OUTPUT_RAW_DATA::default()
        });
        update_state(&default_header(), &servo_output, &writers, &target);
        assert_eq!(
            telemetry
                .messages()
                .servo_output_raw(1)
                .latest()
                .unwrap()
                .value
                .servo1_raw,
            1100
        );

        let status_text = dialect::MavMessage::STATUSTEXT(dialect::STATUSTEXT_DATA {
            text: "PreArm: Check fence".into(),
            ..dialect::STATUSTEXT_DATA::default()
        });
        update_state(&default_header(), &status_text, &writers, &target);
        assert_eq!(
            telemetry
                .messages()
                .status_text()
                .latest()
                .unwrap()
                .value
                .text,
            "PreArm: Check fence"
        );
    }

    #[tokio::test]
    async fn statustext_reassembly_combines_chunks_by_source_and_id() {
        let (writers, channels) = create_channels();
        let telemetry = TelemetryHandle::new(&channels.telemetry_handles);
        let target: Option<VehicleTarget> = None;
        let mut subscription = telemetry.messages().status_text().subscribe();
        let header = MavHeader {
            system_id: 42,
            component_id: 99,
            sequence: 7,
        };

        update_state(
            &header,
            &dialect::MavMessage::STATUSTEXT(dialect::STATUSTEXT_DATA {
                severity: dialect::MavSeverity::MAV_SEVERITY_WARNING,
                text: exact_50_char_text("01234567890123456789012345678901234567890123456789"),
                id: 5,
                chunk_seq: 0,
            }),
            &writers,
            &target,
        );

        assert!(
            tokio::time::timeout(Duration::from_millis(25), subscription.recv())
                .await
                .is_err()
        );

        update_state(
            &header,
            &dialect::MavMessage::STATUSTEXT(dialect::STATUSTEXT_DATA {
                severity: dialect::MavSeverity::MAV_SEVERITY_WARNING,
                text: "abcdefghij".into(),
                id: 5,
                chunk_seq: 1,
            }),
            &writers,
            &target,
        );

        let sample = tokio::time::timeout(Duration::from_millis(250), subscription.recv())
            .await
            .expect("assembled status text should be emitted")
            .expect("assembled status text sample should exist");
        assert_eq!(
            sample.value.text,
            "01234567890123456789012345678901234567890123456789abcdefghij"
        );
        assert_eq!(sample.value.id, 5);
    }

    #[tokio::test]
    async fn statustext_flushes_incomplete_chunks_after_timeout() {
        let (writers, channels) = create_channels();
        let telemetry = TelemetryHandle::new(&channels.telemetry_handles);
        let target: Option<VehicleTarget> = None;
        let mut subscription = telemetry.messages().status_text().subscribe();
        let header = MavHeader {
            system_id: 21,
            component_id: 2,
            sequence: 3,
        };

        update_state(
            &header,
            &dialect::MavMessage::STATUSTEXT(dialect::STATUSTEXT_DATA {
                severity: dialect::MavSeverity::MAV_SEVERITY_ERROR,
                text: exact_50_char_text("98765432109876543210987654321098765432109876543210"),
                id: 9,
                chunk_seq: 0,
            }),
            &writers,
            &target,
        );

        tokio::task::yield_now().await;
        assert!(
            tokio::time::timeout(Duration::from_millis(1), subscription.recv())
                .await
                .is_err()
        );

        tokio::time::sleep(Duration::from_millis(2_100)).await;

        let sample = tokio::time::timeout(Duration::from_millis(50), subscription.recv())
            .await
            .expect("timed flush should emit pending status text")
            .expect("flushed status text sample should exist");
        assert_eq!(
            sample.value.text,
            "98765432109876543210987654321098765432109876543210"
        );
        assert_eq!(sample.value.id, 9);
    }

    #[test]
    fn sensor_health_mapping_sets_all_ten_summary_fields() {
        let (writers, channels) = create_channels();
        let telemetry = TelemetryHandle::new(&channels.telemetry_handles);
        let target: Option<VehicleTarget> = None;

        let present = dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_GYRO
            | dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_ACCEL
            | dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_MAG
            | dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE
            | dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_GPS
            | dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE
            | dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_RC_RECEIVER
            | dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_BATTERY
            | dialect::MavSysStatusSensor::MAV_SYS_STATUS_TERRAIN
            | dialect::MavSysStatusSensor::MAV_SYS_STATUS_GEOFENCE;
        let enabled = dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_GYRO
            | dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_ACCEL
            | dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_MAG
            | dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE
            | dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_GPS
            | dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_BATTERY
            | dialect::MavSysStatusSensor::MAV_SYS_STATUS_GEOFENCE;
        let healthy = dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_GYRO
            | dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_ACCEL
            | dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE
            | dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_GPS
            | dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_BATTERY
            | dialect::MavSysStatusSensor::MAV_SYS_STATUS_GEOFENCE;

        update_state(
            &default_header(),
            &dialect::MavMessage::SYS_STATUS(dialect::SYS_STATUS_DATA {
                onboard_control_sensors_present: present,
                onboard_control_sensors_enabled: enabled,
                onboard_control_sensors_health: healthy,
                ..dialect::SYS_STATUS_DATA::default()
            }),
            &writers,
            &target,
        );

        let summary = telemetry.sensor_health().latest().unwrap().value;
        assert_eq!(summary.gyro, SensorHealthState::Healthy);
        assert_eq!(summary.accel, SensorHealthState::Healthy);
        assert_eq!(summary.mag, SensorHealthState::Unhealthy);
        assert_eq!(summary.baro, SensorHealthState::Healthy);
        assert_eq!(summary.gps, SensorHealthState::Healthy);
        assert_eq!(summary.airspeed, SensorHealthState::Disabled);
        assert_eq!(summary.rc_receiver, SensorHealthState::Disabled);
        assert_eq!(summary.battery, SensorHealthState::Healthy);
        assert_eq!(summary.terrain, SensorHealthState::Disabled);
        assert_eq!(summary.geofence, SensorHealthState::Healthy);
    }

    #[test]
    fn mag_cal_progress_message_updates_progress_channel() {
        let (writers, channels) = create_channels();
        let target: Option<VehicleTarget> = None;

        assert_eq!(*channels.mag_cal_progress.borrow(), None);

        update_state(
            &default_header(),
            &dialect::MavMessage::MAG_CAL_PROGRESS(dialect::MAG_CAL_PROGRESS_DATA {
                compass_id: 2,
                completion_pct: 73,
                cal_status: dialect::MagCalStatus::MAG_CAL_RUNNING_STEP_ONE,
                attempt: 4,
                ..dialect::MAG_CAL_PROGRESS_DATA::default()
            }),
            &writers,
            &target,
        );

        assert_eq!(
            *channels.mag_cal_progress.borrow(),
            Some(MagCalProgress {
                compass_id: 2,
                completion_pct: 73,
                status: MagCalStatus::RunningStepOne,
                attempt: 4,
            })
        );
    }
}
