use mavlink::MavHeader;

use crate::dialect;
use crate::error::VehicleError;

use super::dynamics::{
    degrees_to_e7, horizontal_speed_mps, meters_per_second_to_cms, meters_per_second_to_cms_u16,
    meters_to_millimeters, normalized_heading_deg, radians_to_cdeg,
};
use super::state::{
    ACCEPTANCE_RADIUS_M, AUTOPILOT_VERSION_MESSAGE_ID, AVAILABLE_MODES_MESSAGE_ID,
    DEFAULT_COMPONENT_ID, DEFAULT_SYSTEM_ID, GPS_GLOBAL_ORIGIN_MESSAGE_ID,
    HOME_POSITION_MESSAGE_ID, NavSource, NavTarget, SimulatorCore, StreamSchedule, auto_mode,
    guided_mode, health_sensors, profile_modes, profile_vehicle_type,
};

const DEFAULT_TELEMETRY_MESSAGE_IDS: [u32; 12] = [
    33,
    24,
    30,
    74,
    1,
    147,
    65,
    36,
    62,
    HOME_POSITION_MESSAGE_ID,
    GPS_GLOBAL_ORIGIN_MESSAGE_ID,
    42,
];

impl SimulatorCore {
    #[allow(
        deprecated,
        reason = "the demo simulator still accepts legacy MAVLink messages used by existing MAVKit paths"
    )]
    pub(crate) async fn handle_inbound_message(
        &mut self,
        message: dialect::MavMessage,
    ) -> Result<(), VehicleError> {
        match message {
            dialect::MavMessage::COMMAND_LONG(data) => self.handle_command_long(data).await,
            dialect::MavMessage::COMMAND_INT(data) => self.handle_command_int(data).await,
            dialect::MavMessage::PARAM_REQUEST_LIST(_data) => {
                self.handle_param_request_list().await
            }
            dialect::MavMessage::PARAM_REQUEST_READ(data) => {
                self.handle_param_request_read(data).await
            }
            dialect::MavMessage::PARAM_SET(data) => self.handle_param_set(data).await,
            dialect::MavMessage::SET_POSITION_TARGET_GLOBAL_INT(data) => {
                self.handle_set_position_target_global_int(data).await
            }
            dialect::MavMessage::MISSION_COUNT(data) => self.handle_mission_count(data).await,
            dialect::MavMessage::MISSION_ITEM_INT(data) => self.handle_mission_item_int(data).await,
            dialect::MavMessage::MISSION_REQUEST_LIST(data) => {
                self.handle_mission_request_list(data).await
            }
            dialect::MavMessage::MISSION_REQUEST_INT(data) => {
                self.handle_mission_request_int(data).await
            }
            dialect::MavMessage::MISSION_REQUEST(data) => self.handle_mission_request(data).await,
            dialect::MavMessage::MISSION_CLEAR_ALL(data) => {
                self.handle_mission_clear_all(data).await
            }
            dialect::MavMessage::MISSION_ACK(_data) => Ok(()),
            dialect::MavMessage::SET_GPS_GLOBAL_ORIGIN(data) => {
                self.snapshot.home.latitude_deg = f64::from(data.latitude) / 1e7;
                self.snapshot.home.longitude_deg = f64::from(data.longitude) / 1e7;
                self.snapshot.home.altitude_m = f64::from(data.altitude) / 1000.0;
                self.snapshot.relative_alt_m =
                    self.snapshot.altitude_msl_m - self.snapshot.home.altitude_m;
                self.publish_snapshot();
                self.emit_gps_global_origin().await
            }
            _ => Ok(()),
        }
    }

    #[allow(
        deprecated,
        reason = "the demo simulator still handles MAV_CMD_GET_HOME_POSITION because MAVKit currently sends it during init"
    )]
    async fn handle_command_long(
        &mut self,
        data: dialect::COMMAND_LONG_DATA,
    ) -> Result<(), VehicleError> {
        match data.command {
            dialect::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM => {
                self.snapshot.armed = data.param1 >= 0.5;
                if !self.snapshot.armed {
                    self.velocity = super::state::VelocityNed {
                        north_mps: 0.0,
                        east_mps: 0.0,
                        down_mps: 0.0,
                    };
                    self.mission_speed_override_mps = None;
                }
                self.publish_snapshot();
                self.send_command_ack(data.command, dialect::MavResult::MAV_RESULT_ACCEPTED)
                    .await?;
                self.emit_heartbeat().await
            }
            dialect::MavCmd::MAV_CMD_DO_SET_MODE => {
                self.snapshot.custom_mode = data.param2.max(0.0) as u32;
                if self.snapshot.custom_mode != guided_mode(self.config.profile)
                    && self
                        .nav_target
                        .is_some_and(|target| target.source == NavSource::Guided)
                {
                    self.nav_target = None;
                }
                if self.snapshot.custom_mode != auto_mode(self.config.profile) {
                    self.mission_completed = false;
                }
                self.publish_snapshot();
                self.send_command_ack(data.command, dialect::MavResult::MAV_RESULT_ACCEPTED)
                    .await?;
                self.emit_heartbeat().await?;
                self.emit_current_mode().await
            }
            dialect::MavCmd::MAV_CMD_REQUEST_MESSAGE => {
                self.send_command_ack(data.command, dialect::MavResult::MAV_RESULT_ACCEPTED)
                    .await?;
                self.emit_message_by_id(data.param1.max(0.0) as u32).await
            }
            dialect::MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL => {
                let message_id = data.param1.max(0.0) as u32;
                self.stream_schedules
                    .insert(message_id, StreamSchedule::new(data.param2 as i64));
                self.send_command_ack(data.command, dialect::MavResult::MAV_RESULT_ACCEPTED)
                    .await
            }
            dialect::MavCmd::MAV_CMD_GET_HOME_POSITION => {
                self.send_command_ack(data.command, dialect::MavResult::MAV_RESULT_ACCEPTED)
                    .await?;
                self.emit_home_position().await
            }
            dialect::MavCmd::MAV_CMD_DO_SET_HOME => {
                if data.param1 >= 0.5 {
                    self.snapshot.home.latitude_deg = self.snapshot.latitude_deg;
                    self.snapshot.home.longitude_deg = self.snapshot.longitude_deg;
                    self.snapshot.home.altitude_m = self.snapshot.altitude_msl_m;
                }
                self.snapshot.relative_alt_m =
                    self.snapshot.altitude_msl_m - self.snapshot.home.altitude_m;
                self.publish_snapshot();
                self.send_command_ack(data.command, dialect::MavResult::MAV_RESULT_ACCEPTED)
                    .await?;
                self.emit_home_and_origin().await
            }
            dialect::MavCmd::MAV_CMD_DO_SET_MISSION_CURRENT => {
                self.snapshot.mission_current_wire_seq = data.param1.max(0.0) as u16;
                self.mission_completed = false;
                self.nav_target = None;
                self.pending_reached_wire_seq = None;
                self.restore_mission_runtime_state();
                self.publish_snapshot();
                self.send_command_ack(data.command, dialect::MavResult::MAV_RESULT_ACCEPTED)
                    .await?;
                self.emit_mission_current().await
            }
            _ => {
                self.send_command_ack(data.command, dialect::MavResult::MAV_RESULT_UNSUPPORTED)
                    .await
            }
        }
    }

    async fn handle_command_int(
        &mut self,
        data: dialect::COMMAND_INT_DATA,
    ) -> Result<(), VehicleError> {
        if data.command == dialect::MavCmd::MAV_CMD_DO_SET_HOME {
            self.snapshot.home.latitude_deg = f64::from(data.x) / 1e7;
            self.snapshot.home.longitude_deg = f64::from(data.y) / 1e7;
            self.snapshot.home.altitude_m = f64::from(data.z);
            self.snapshot.relative_alt_m =
                self.snapshot.altitude_msl_m - self.snapshot.home.altitude_m;
            self.publish_snapshot();
            self.send_command_ack(data.command, dialect::MavResult::MAV_RESULT_ACCEPTED)
                .await?;
            self.emit_home_and_origin().await
        } else {
            self.send_command_ack(data.command, dialect::MavResult::MAV_RESULT_UNSUPPORTED)
                .await
        }
    }

    #[allow(
        deprecated,
        reason = "the demo simulator accepts legacy relative-alt target frames still used by MAVLink clients"
    )]
    async fn handle_set_position_target_global_int(
        &mut self,
        data: dialect::SET_POSITION_TARGET_GLOBAL_INT_DATA,
    ) -> Result<(), VehicleError> {
        let altitude_msl_m = match data.coordinate_frame {
            dialect::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT
            | dialect::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT_INT => {
                self.snapshot.home.altitude_m + f64::from(data.alt)
            }
            _ => f64::from(data.alt),
        };

        self.nav_target = Some(NavTarget {
            source: NavSource::Guided,
            latitude_deg: f64::from(data.lat_int) / 1e7,
            longitude_deg: f64::from(data.lon_int) / 1e7,
            altitude_msl_m,
            acceptance_radius_m: ACCEPTANCE_RADIUS_M,
            wire_seq: None,
            disarm_on_reach: false,
        });
        Ok(())
    }

    pub(crate) async fn emit_message_by_id(&mut self, message_id: u32) -> Result<(), VehicleError> {
        match message_id {
            AUTOPILOT_VERSION_MESSAGE_ID => self.emit_autopilot_version().await,
            AVAILABLE_MODES_MESSAGE_ID => self.emit_available_modes().await,
            GPS_GLOBAL_ORIGIN_MESSAGE_ID => self.emit_gps_global_origin().await,
            HOME_POSITION_MESSAGE_ID => self.emit_home_position().await,
            0 => self.emit_heartbeat().await,
            1 => self.emit_sys_status().await,
            24 => self.emit_gps_raw_int().await,
            30 => self.emit_attitude().await,
            33 => self.emit_global_position_int().await,
            36 => self.emit_servo_output_raw().await,
            42 => self.emit_mission_current().await,
            62 => self.emit_nav_controller_output().await,
            65 => self.emit_rc_channels().await,
            74 => self.emit_vfr_hud().await,
            147 => self.emit_battery_status().await,
            _ => Ok(()),
        }
    }

    async fn emit_available_modes(&mut self) -> Result<(), VehicleError> {
        let modes = profile_modes(self.config.profile);
        for (index, (custom_mode, name)) in modes.iter().enumerate() {
            self.send_message(dialect::MavMessage::AVAILABLE_MODES(
                dialect::AVAILABLE_MODES_DATA {
                    custom_mode: *custom_mode,
                    properties: dialect::MavModeProperty::empty(),
                    number_modes: modes.len() as u8,
                    mode_index: index as u8,
                    standard_mode: dialect::MavStandardMode::MAV_STANDARD_MODE_NON_STANDARD,
                    mode_name: (*name).into(),
                },
            ))
            .await?;
        }
        Ok(())
    }

    async fn emit_autopilot_version(&mut self) -> Result<(), VehicleError> {
        self.send_message(dialect::MavMessage::AUTOPILOT_VERSION(
            dialect::AUTOPILOT_VERSION_DATA {
                flight_sw_version: 0x0405_0000,
                uid: 1,
                ..dialect::AUTOPILOT_VERSION_DATA::default()
            },
        ))
        .await
    }

    pub(crate) async fn emit_heartbeat(&mut self) -> Result<(), VehicleError> {
        let mut base_mode = dialect::MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        if self.snapshot.armed {
            base_mode |= dialect::MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED;
        }

        self.send_message(dialect::MavMessage::HEARTBEAT(dialect::HEARTBEAT_DATA {
            custom_mode: self.snapshot.custom_mode,
            mavtype: profile_vehicle_type(self.config.profile),
            autopilot: dialect::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode,
            system_status: if self.snapshot.armed {
                dialect::MavState::MAV_STATE_ACTIVE
            } else {
                dialect::MavState::MAV_STATE_STANDBY
            },
            mavlink_version: 3,
        }))
        .await
    }

    pub(crate) async fn emit_current_mode(&mut self) -> Result<(), VehicleError> {
        self.send_message(dialect::MavMessage::CURRENT_MODE(
            dialect::CURRENT_MODE_DATA {
                custom_mode: self.snapshot.custom_mode,
                intended_custom_mode: 0,
                standard_mode: dialect::MavStandardMode::MAV_STANDARD_MODE_NON_STANDARD,
            },
        ))
        .await
    }

    pub(crate) async fn emit_telemetry_burst(&mut self) -> Result<(), VehicleError> {
        for message_id in DEFAULT_TELEMETRY_MESSAGE_IDS {
            self.emit_message_by_id(message_id).await?;
        }
        Ok(())
    }

    pub(crate) async fn emit_default_telemetry_burst(&mut self) -> Result<(), VehicleError> {
        for message_id in DEFAULT_TELEMETRY_MESSAGE_IDS {
            if !self.stream_schedules.contains_key(&message_id) {
                self.emit_message_by_id(message_id).await?;
            }
        }
        Ok(())
    }

    async fn emit_global_position_int(&mut self) -> Result<(), VehicleError> {
        self.send_message(dialect::MavMessage::GLOBAL_POSITION_INT(
            dialect::GLOBAL_POSITION_INT_DATA {
                time_boot_ms: self.snapshot.time_boot_ms,
                lat: degrees_to_e7(self.snapshot.latitude_deg),
                lon: degrees_to_e7(self.snapshot.longitude_deg),
                alt: meters_to_millimeters(self.snapshot.altitude_msl_m),
                relative_alt: meters_to_millimeters(self.snapshot.relative_alt_m),
                vx: meters_per_second_to_cms(self.velocity.north_mps),
                vy: meters_per_second_to_cms(self.velocity.east_mps),
                vz: meters_per_second_to_cms(self.velocity.down_mps),
                hdg: radians_to_cdeg(self.snapshot.yaw_rad),
            },
        ))
        .await
    }

    async fn emit_gps_raw_int(&mut self) -> Result<(), VehicleError> {
        self.send_message(dialect::MavMessage::GPS_RAW_INT(
            dialect::GPS_RAW_INT_DATA {
                time_usec: u64::from(self.snapshot.time_boot_ms) * 1000,
                fix_type: dialect::GpsFixType::GPS_FIX_TYPE_3D_FIX,
                lat: degrees_to_e7(self.snapshot.latitude_deg),
                lon: degrees_to_e7(self.snapshot.longitude_deg),
                alt: meters_to_millimeters(self.snapshot.altitude_msl_m),
                eph: 120,
                epv: 180,
                vel: meters_per_second_to_cms_u16(horizontal_speed_mps(self.velocity)),
                cog: radians_to_cdeg(self.snapshot.yaw_rad),
                satellites_visible: 14,
                alt_ellipsoid: meters_to_millimeters(self.snapshot.altitude_msl_m),
                h_acc: 500,
                v_acc: 800,
                vel_acc: 100,
                hdg_acc: 100,
                yaw: radians_to_cdeg(self.snapshot.yaw_rad),
            },
        ))
        .await
    }

    async fn emit_attitude(&mut self) -> Result<(), VehicleError> {
        self.send_message(dialect::MavMessage::ATTITUDE(dialect::ATTITUDE_DATA {
            time_boot_ms: self.snapshot.time_boot_ms,
            roll: self.snapshot.roll_rad,
            pitch: self.snapshot.pitch_rad,
            yaw: self.snapshot.yaw_rad,
            rollspeed: 0.0,
            pitchspeed: 0.0,
            yawspeed: 0.02,
        }))
        .await
    }

    async fn emit_vfr_hud(&mut self) -> Result<(), VehicleError> {
        self.send_message(dialect::MavMessage::VFR_HUD(dialect::VFR_HUD_DATA {
            airspeed: horizontal_speed_mps(self.velocity) as f32,
            groundspeed: horizontal_speed_mps(self.velocity) as f32,
            heading: normalized_heading_deg(self.snapshot.yaw_rad),
            throttle: if self.snapshot.armed { 40 } else { 0 },
            alt: self.snapshot.relative_alt_m as f32,
            climb: -self.velocity.down_mps as f32,
        }))
        .await
    }

    async fn emit_battery_status(&mut self) -> Result<(), VehicleError> {
        self.send_message(dialect::MavMessage::BATTERY_STATUS(
            dialect::BATTERY_STATUS_DATA {
                id: 0,
                battery_function: dialect::MavBatteryFunction::MAV_BATTERY_FUNCTION_ALL,
                mavtype: dialect::MavBatteryType::MAV_BATTERY_TYPE_LIPO,
                temperature: i16::MAX,
                voltages: self.power.cell_voltages_mv(),
                current_battery: self.power.current_ca(),
                current_consumed: self.power.consumed_mah(),
                energy_consumed: self.power.consumed_hj(),
                battery_remaining: self.power.remaining_pct_i8(),
                time_remaining: self.power.time_remaining_s(),
                charge_state: self.battery_charge_state(),
                voltages_ext: [0; 4],
                mode: dialect::MavBatteryMode::MAV_BATTERY_MODE_UNKNOWN,
                fault_bitmask: dialect::MavBatteryFault::empty(),
            },
        ))
        .await
    }

    async fn emit_rc_channels(&mut self) -> Result<(), VehicleError> {
        let throttle = if self.snapshot.armed {
            (1100.0 + self.power.load_dpercent() as f32 * 0.5)
                .round()
                .clamp(1100.0, 1700.0) as u16
        } else {
            1000
        };

        self.send_message(dialect::MavMessage::RC_CHANNELS(
            dialect::RC_CHANNELS_DATA {
                time_boot_ms: self.snapshot.time_boot_ms,
                chancount: 8,
                chan1_raw: 1500,
                chan2_raw: 1500,
                chan3_raw: throttle,
                chan4_raw: 1500,
                chan5_raw: if self.snapshot.armed { 1800 } else { 1000 },
                chan6_raw: 1500,
                chan7_raw: 1500,
                chan8_raw: 1500,
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
                rssi: 210,
            },
        ))
        .await
    }

    async fn emit_servo_output_raw(&mut self) -> Result<(), VehicleError> {
        let actuator = if self.snapshot.armed {
            (1100.0 + self.power.load_dpercent() as f32 * 0.55)
                .round()
                .clamp(1100.0, 1900.0) as u16
        } else {
            1000
        };

        self.send_message(dialect::MavMessage::SERVO_OUTPUT_RAW(
            dialect::SERVO_OUTPUT_RAW_DATA {
                time_usec: self.snapshot.time_boot_ms.saturating_mul(1000),
                port: 0,
                servo1_raw: actuator,
                servo2_raw: actuator,
                servo3_raw: actuator,
                servo4_raw: actuator,
                servo5_raw: 1500,
                servo6_raw: 1500,
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
            },
        ))
        .await
    }

    async fn emit_nav_controller_output(&mut self) -> Result<(), VehicleError> {
        let (target_bearing, wp_dist, alt_error) = if let Some(target) = self.active_nav_target() {
            let (north_m, east_m) = super::dynamics::lat_lon_delta_m(
                self.snapshot.latitude_deg,
                self.snapshot.longitude_deg,
                target.latitude_deg,
                target.longitude_deg,
            );
            (
                normalized_heading_deg(east_m.atan2(north_m) as f32),
                (north_m * north_m + east_m * east_m).sqrt().round() as u16,
                (target.altitude_msl_m - self.snapshot.altitude_msl_m) as f32,
            )
        } else {
            (normalized_heading_deg(self.snapshot.yaw_rad), 0, 0.0)
        };

        self.send_message(dialect::MavMessage::NAV_CONTROLLER_OUTPUT(
            dialect::NAV_CONTROLLER_OUTPUT_DATA {
                nav_roll: self.snapshot.roll_rad.to_degrees(),
                nav_pitch: self.snapshot.pitch_rad.to_degrees(),
                nav_bearing: normalized_heading_deg(self.snapshot.yaw_rad),
                target_bearing,
                wp_dist,
                alt_error,
                aspd_error: 0.0,
                xtrack_error: 0.0,
            },
        ))
        .await
    }

    async fn emit_sys_status(&mut self) -> Result<(), VehicleError> {
        let sensors = health_sensors();
        self.send_message(dialect::MavMessage::SYS_STATUS(dialect::SYS_STATUS_DATA {
            onboard_control_sensors_present: sensors,
            onboard_control_sensors_enabled: sensors,
            onboard_control_sensors_health: sensors,
            load: self.power.load_dpercent(),
            voltage_battery: self.power.voltage_mv(),
            current_battery: self.power.current_ca(),
            battery_remaining: self.power.remaining_pct_i8(),
            drop_rate_comm: 0,
            errors_comm: 0,
            errors_count1: 0,
            errors_count2: 0,
            errors_count3: 0,
            errors_count4: 0,
            onboard_control_sensors_present_extended: dialect::MavSysStatusSensorExtended::empty(),
            onboard_control_sensors_enabled_extended: dialect::MavSysStatusSensorExtended::empty(),
            onboard_control_sensors_health_extended: dialect::MavSysStatusSensorExtended::empty(),
        }))
        .await
    }

    pub(crate) async fn emit_home_and_origin(&mut self) -> Result<(), VehicleError> {
        self.emit_home_position().await?;
        self.emit_gps_global_origin().await
    }

    async fn emit_home_position(&mut self) -> Result<(), VehicleError> {
        self.send_message(dialect::MavMessage::HOME_POSITION(
            dialect::HOME_POSITION_DATA {
                latitude: degrees_to_e7(self.snapshot.home.latitude_deg),
                longitude: degrees_to_e7(self.snapshot.home.longitude_deg),
                altitude: meters_to_millimeters(self.snapshot.home.altitude_m),
                x: 0.0,
                y: 0.0,
                z: 0.0,
                q: [0.0; 4],
                approach_x: 0.0,
                approach_y: 0.0,
                approach_z: 0.0,
                time_usec: u64::from(self.snapshot.time_boot_ms) * 1000,
            },
        ))
        .await
    }

    async fn emit_gps_global_origin(&mut self) -> Result<(), VehicleError> {
        self.send_message(dialect::MavMessage::GPS_GLOBAL_ORIGIN(
            dialect::GPS_GLOBAL_ORIGIN_DATA {
                latitude: degrees_to_e7(self.snapshot.home.latitude_deg),
                longitude: degrees_to_e7(self.snapshot.home.longitude_deg),
                altitude: meters_to_millimeters(self.snapshot.home.altitude_m),
                time_usec: u64::from(self.snapshot.time_boot_ms) * 1000,
            },
        ))
        .await
    }

    pub(crate) async fn emit_mission_current(&mut self) -> Result<(), VehicleError> {
        self.send_message(dialect::MavMessage::MISSION_CURRENT(
            dialect::MISSION_CURRENT_DATA {
                seq: self.snapshot.mission_current_wire_seq,
                total: self.snapshot.mission_total_wire_items,
                mission_state: if self.mission_completed {
                    dialect::MissionState::MISSION_STATE_COMPLETE
                } else if self.snapshot.mission_total_wire_items > 0 {
                    dialect::MissionState::MISSION_STATE_ACTIVE
                } else {
                    dialect::MissionState::MISSION_STATE_NOT_STARTED
                },
                mission_mode: 0,
                mission_id: 0,
                fence_id: 0,
                rally_points_id: 0,
            },
        ))
        .await
    }

    pub(crate) async fn emit_mission_item_reached(&mut self, seq: u16) -> Result<(), VehicleError> {
        self.send_message(dialect::MavMessage::MISSION_ITEM_REACHED(
            dialect::MISSION_ITEM_REACHED_DATA { seq },
        ))
        .await
    }

    pub(crate) async fn emit_statustext(&mut self, text: String) -> Result<(), VehicleError> {
        self.send_message(dialect::MavMessage::STATUSTEXT(dialect::STATUSTEXT_DATA {
            severity: dialect::MavSeverity::MAV_SEVERITY_INFO,
            text: text.as_str().into(),
            ..dialect::STATUSTEXT_DATA::default()
        }))
        .await
    }

    async fn send_command_ack(
        &mut self,
        command: dialect::MavCmd,
        result: dialect::MavResult,
    ) -> Result<(), VehicleError> {
        self.send_message(dialect::MavMessage::COMMAND_ACK(
            dialect::COMMAND_ACK_DATA {
                command,
                result,
                progress: 0,
                result_param2: 0,
                target_system: 0,
                target_component: 0,
            },
        ))
        .await
    }

    pub(crate) async fn send_mission_ack(
        &mut self,
        mission_type: dialect::MavMissionType,
        result: dialect::MavMissionResult,
    ) -> Result<(), VehicleError> {
        self.send_message(dialect::MavMessage::MISSION_ACK(
            dialect::MISSION_ACK_DATA {
                target_system: 0,
                target_component: 0,
                mavtype: result,
                mission_type,
                opaque_id: 0,
            },
        ))
        .await
    }

    pub(crate) async fn send_message(
        &mut self,
        message: dialect::MavMessage,
    ) -> Result<(), VehicleError> {
        let header = MavHeader {
            system_id: DEFAULT_SYSTEM_ID,
            component_id: DEFAULT_COMPONENT_ID,
            sequence: self.header_sequence,
        };
        self.header_sequence = self.header_sequence.wrapping_add(1);
        self.outbound_tx
            .send((header, message))
            .await
            .map_err(|_| VehicleError::Disconnected)
    }
}

#[cfg(test)]
mod tests {
    use mavlink::Message;
    use tokio::sync::{mpsc, watch};

    use super::*;
    use crate::mission::HomePosition;
    use crate::sim::state::{DemoVehicleConfig, default_mode};
    use crate::sim::{DemoClock, DemoProfile, DemoVehicleSnapshot};

    fn sim_core_with_tick_hz(
        tick_hz: u32,
    ) -> (
        SimulatorCore,
        mpsc::Receiver<(MavHeader, dialect::MavMessage)>,
    ) {
        let home = HomePosition {
            latitude_deg: 42.0,
            longitude_deg: -71.0,
            altitude_m: 100.0,
        };
        let snapshot = DemoVehicleSnapshot {
            time_boot_ms: 0,
            armed: false,
            custom_mode: default_mode(DemoProfile::ArduCopter),
            home: home.clone(),
            latitude_deg: home.latitude_deg,
            longitude_deg: home.longitude_deg,
            altitude_msl_m: home.altitude_m,
            relative_alt_m: 0.0,
            roll_rad: 0.0,
            pitch_rad: 0.0,
            yaw_rad: 0.0,
            mission_current_wire_seq: 0,
            mission_total_wire_items: 0,
        };
        let (outbound_tx, outbound_rx) = mpsc::channel(128);
        let (snapshot_tx, _snapshot_rx) = watch::channel(snapshot.clone());
        let sim = SimulatorCore::new(
            DemoVehicleConfig {
                profile: DemoProfile::ArduCopter,
                clock: DemoClock::Manual,
                tick_hz,
                home,
            },
            outbound_tx,
            snapshot_tx,
            snapshot,
        );

        (sim, outbound_rx)
    }

    fn sim_core() -> (
        SimulatorCore,
        mpsc::Receiver<(MavHeader, dialect::MavMessage)>,
    ) {
        sim_core_with_tick_hz(1)
    }

    fn command_long(command: dialect::MavCmd, param1: f32, param2: f32) -> dialect::MavMessage {
        dialect::MavMessage::COMMAND_LONG(dialect::COMMAND_LONG_DATA {
            target_system: DEFAULT_SYSTEM_ID,
            target_component: DEFAULT_COMPONENT_ID,
            command,
            confirmation: 0,
            param1,
            param2,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
        })
    }

    fn drain_message_ids(
        outbound_rx: &mut mpsc::Receiver<(MavHeader, dialect::MavMessage)>,
    ) -> Vec<u32> {
        let mut message_ids = Vec::new();
        while let Ok((_header, message)) = outbound_rx.try_recv() {
            message_ids.push(message.message_id());
        }
        message_ids
    }

    fn count_message_id(message_ids: &[u32], expected: u32) -> usize {
        message_ids
            .iter()
            .filter(|message_id| **message_id == expected)
            .count()
    }

    #[tokio::test]
    async fn emits_dynamic_sys_status_and_battery_status_values() {
        let (mut sim, mut outbound_rx) = sim_core();
        sim.snapshot.armed = true;
        sim.velocity = super::super::state::VelocityNed {
            north_mps: 4.0,
            east_mps: 0.0,
            down_mps: -2.0,
        };
        sim.advance_power_model();

        sim.emit_message_by_id(1).await.unwrap();
        let sys_status = match outbound_rx.recv().await.unwrap().1 {
            dialect::MavMessage::SYS_STATUS(data) => data,
            _ => panic!("expected SYS_STATUS"),
        };

        sim.emit_message_by_id(147).await.unwrap();
        let battery_status = match outbound_rx.recv().await.unwrap().1 {
            dialect::MavMessage::BATTERY_STATUS(data) => data,
            _ => panic!("expected BATTERY_STATUS"),
        };

        assert_eq!(sys_status.voltage_battery, sim.power.voltage_mv());
        assert_eq!(sys_status.current_battery, sim.power.current_ca());
        assert_eq!(sys_status.battery_remaining, sim.power.remaining_pct_i8());
        assert_ne!(sys_status.voltage_battery, 12_400);
        assert_ne!(sys_status.current_battery, 450);
        assert_eq!(battery_status.voltages, sim.power.cell_voltages_mv());
        assert_eq!(battery_status.current_battery, sim.power.current_ca());
        assert_eq!(
            battery_status.battery_remaining,
            sim.power.remaining_pct_i8()
        );
    }

    #[tokio::test]
    async fn telemetry_burst_includes_raw_power_and_output_messages() {
        let (mut sim, mut outbound_rx) = sim_core();

        sim.emit_telemetry_burst().await.unwrap();
        let mut seen = std::collections::BTreeSet::new();
        while let Ok((_header, message)) = outbound_rx.try_recv() {
            seen.insert(message.message_id());
        }

        assert!(seen.contains(&147));
        assert!(seen.contains(&65));
        assert!(seen.contains(&36));
    }

    #[tokio::test]
    async fn configured_stream_emits_on_tick_cadence() {
        let (mut sim, mut outbound_rx) = sim_core_with_tick_hz(10);

        sim.handle_inbound_message(command_long(
            dialect::MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
            AUTOPILOT_VERSION_MESSAGE_ID as f32,
            300_000.0,
        ))
        .await
        .unwrap();
        drain_message_ids(&mut outbound_rx);

        let mut autopilot_version_counts = Vec::new();
        for _ in 0..6 {
            sim.advance_one_tick().await.unwrap();
            let message_ids = drain_message_ids(&mut outbound_rx);
            autopilot_version_counts
                .push(count_message_id(&message_ids, AUTOPILOT_VERSION_MESSAGE_ID));
        }

        assert_eq!(autopilot_version_counts, vec![0, 0, 1, 0, 0, 1]);
    }

    #[tokio::test]
    async fn disabled_configured_default_stream_is_not_emitted_on_ticks() {
        let (mut sim, mut outbound_rx) = sim_core();

        sim.handle_inbound_message(command_long(
            dialect::MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
            30.0,
            1_000.0,
        ))
        .await
        .unwrap();
        drain_message_ids(&mut outbound_rx);
        sim.advance_one_tick().await.unwrap();
        let enabled_tick_ids = drain_message_ids(&mut outbound_rx);
        assert_eq!(count_message_id(&enabled_tick_ids, 30), 1);

        sim.handle_inbound_message(command_long(
            dialect::MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
            30.0,
            0.0,
        ))
        .await
        .unwrap();
        drain_message_ids(&mut outbound_rx);
        sim.advance_one_tick().await.unwrap();
        let disabled_tick_ids = drain_message_ids(&mut outbound_rx);

        assert_eq!(count_message_id(&disabled_tick_ids, 30), 0);
        assert!(disabled_tick_ids.contains(&33));
    }

    #[tokio::test]
    async fn request_message_emits_immediately_even_when_stream_disabled() {
        let (mut sim, mut outbound_rx) = sim_core();

        sim.handle_inbound_message(command_long(
            dialect::MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
            AUTOPILOT_VERSION_MESSAGE_ID as f32,
            -1.0,
        ))
        .await
        .unwrap();
        drain_message_ids(&mut outbound_rx);

        sim.handle_inbound_message(command_long(
            dialect::MavCmd::MAV_CMD_REQUEST_MESSAGE,
            AUTOPILOT_VERSION_MESSAGE_ID as f32,
            0.0,
        ))
        .await
        .unwrap();
        let message_ids = drain_message_ids(&mut outbound_rx);

        assert_eq!(
            count_message_id(&message_ids, AUTOPILOT_VERSION_MESSAGE_ID),
            1
        );
    }
}
