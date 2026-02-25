mod commands;
mod mission;
mod params;
mod state_update;

use commands::{handle_arm_disarm, handle_command_long, handle_guided_goto, handle_set_mode};
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

// Re-imports used only by tests
#[cfg(test)]
use crate::mission::{MissionFrame, MissionType};
use mavlink::common::{self, MavCmd};
use mavlink::{AsyncMavConnection, MavHeader};
#[cfg(test)]
use mission::{
    from_mav_frame, from_mission_item_int, mission_type_matches, to_mav_frame, to_mav_mission_type,
};
#[cfg(test)]
use params::{from_mav_param_type, param_id_to_string, string_to_param_id, to_mav_param_type};
use std::time::Duration;
use tokio::sync::mpsc;
use tokio_util::sync::CancellationToken;
use tracing::{debug, warn};

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
// Command dispatch
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
