use super::mission::{
    from_mav_frame, from_mission_item_int, mission_type_matches, to_mav_frame, to_mav_mission_type,
};
use super::params::{
    from_mav_param_type, param_id_to_string, string_to_param_id, to_mav_param_type,
};
use super::*;
use crate::dialect::{self, MavCmd, MavModeFlag, MavState};
use crate::mission::{MissionFrame, MissionType};
use crate::state::{self, LinkState, create_channels};
use mavlink::{MAVLinkMessageRaw, MAVLinkV2MessageRaw, MavHeader, MavlinkVersion};
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::sync::{mpsc, oneshot};

// -----------------------------------------------------------------------
// Mock AsyncMavConnection
// -----------------------------------------------------------------------

type SentMessages = Arc<Mutex<Vec<(MavHeader, dialect::MavMessage)>>>;

/// Messages the mock should yield from `recv()`.  When the queue is empty
/// the mock blocks forever (the test drives progress via commands/cancel).
struct MockConnection {
    recv_rx: tokio::sync::Mutex<mpsc::Receiver<(MavHeader, dialect::MavMessage)>>,
    sent: SentMessages,
}

impl MockConnection {
    fn new(rx: mpsc::Receiver<(MavHeader, dialect::MavMessage)>) -> (Self, SentMessages) {
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
impl AsyncMavConnection<dialect::MavMessage> for MockConnection {
    async fn recv(
        &self,
    ) -> Result<(MavHeader, dialect::MavMessage), mavlink::error::MessageReadError> {
        let mut rx = self.recv_rx.lock().await;
        match rx.recv().await {
            Some(msg) => Ok(msg),
            None => Err(mavlink::error::MessageReadError::Io(std::io::Error::new(
                std::io::ErrorKind::ConnectionReset,
                "mock connection closed",
            ))),
        }
    }

    async fn recv_raw(&self) -> Result<MAVLinkMessageRaw, mavlink::error::MessageReadError> {
        let (header, message) = self.recv().await?;
        let mut raw = MAVLinkV2MessageRaw::new();
        raw.serialize_message(header, &message);
        Ok(MAVLinkMessageRaw::V2(raw))
    }

    async fn send(
        &self,
        header: &MavHeader,
        data: &dialect::MavMessage,
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

fn heartbeat_msg(armed: bool, custom_mode: u32) -> dialect::MavMessage {
    let mut base_mode = MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    if armed {
        base_mode |= MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED;
    }
    dialect::MavMessage::HEARTBEAT(dialect::HEARTBEAT_DATA {
        custom_mode,
        mavtype: dialect::MavType::MAV_TYPE_QUADROTOR,
        autopilot: dialect::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode,
        system_status: MavState::MAV_STATE_STANDBY,
        mavlink_version: 3,
    })
}

fn fast_config() -> VehicleConfig {
    VehicleConfig {
        command_timeout: Duration::from_millis(50),
        command_completion_timeout: Duration::from_millis(250),
        retry_policy: crate::mission::RetryPolicy {
            request_timeout_ms: 50,
            item_timeout_ms: 50,
            max_retries: 2,
        },
        auto_request_home: false,
        ..VehicleConfig::default()
    }
}

fn ack_msg(command: MavCmd, result: dialect::MavResult) -> dialect::MavMessage {
    dialect::MavMessage::COMMAND_ACK(dialect::COMMAND_ACK_DATA {
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
        dialect::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA
    );
    assert_eq!(t.vehicle_type, dialect::MavType::MAV_TYPE_QUADROTOR);
}

#[test]
fn target_set_generic_from_non_heartbeat() {
    let mut target: Option<VehicleTarget> = None;
    let header = MavHeader {
        system_id: 5,
        component_id: 10,
        sequence: 0,
    };
    let msg = dialect::MavMessage::VFR_HUD(dialect::VFR_HUD_DATA {
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
    assert_eq!(t.autopilot, dialect::MavAutopilot::MAV_AUTOPILOT_GENERIC);
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
        autopilot: dialect::MavAutopilot::MAV_AUTOPILOT_GENERIC,
        vehicle_type: dialect::MavType::MAV_TYPE_GENERIC,
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
    assert_eq!(t.vehicle_type, dialect::MavType::MAV_TYPE_QUADROTOR);
}

#[test]
fn non_heartbeat_does_not_overwrite_existing_target() {
    let original = VehicleTarget {
        system_id: 1,
        component_id: 1,
        autopilot: dialect::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
        vehicle_type: dialect::MavType::MAV_TYPE_QUADROTOR,
    };
    let mut target = Some(original);
    let header = MavHeader {
        system_id: 99,
        component_id: 99,
        sequence: 0,
    };
    let msg = dialect::MavMessage::VFR_HUD(dialect::VFR_HUD_DATA {
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
        autopilot: dialect::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
        vehicle_type: dialect::MavType::MAV_TYPE_QUADROTOR,
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
    let msg = dialect::MavMessage::VFR_HUD(dialect::VFR_HUD_DATA {
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
    let msg = dialect::MavMessage::GLOBAL_POSITION_INT(dialect::GLOBAL_POSITION_INT_DATA {
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
    let msg = dialect::MavMessage::SYS_STATUS(dialect::SYS_STATUS_DATA {
        onboard_control_sensors_present: dialect::MavSysStatusSensor::empty(),
        onboard_control_sensors_enabled: dialect::MavSysStatusSensor::empty(),
        onboard_control_sensors_health: dialect::MavSysStatusSensor::empty(),
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
        onboard_control_sensors_present_extended: dialect::MavSysStatusSensorExtended::empty(),
        onboard_control_sensors_enabled_extended: dialect::MavSysStatusSensorExtended::empty(),
        onboard_control_sensors_health_extended: dialect::MavSysStatusSensorExtended::empty(),
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
    let msg = dialect::MavMessage::GPS_RAW_INT(dialect::GPS_RAW_INT_DATA {
        time_usec: 0,
        fix_type: dialect::GpsFixType::GPS_FIX_TYPE_3D_FIX,
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
    let msg = dialect::MavMessage::MISSION_CURRENT(dialect::MISSION_CURRENT_DATA {
        seq: 3,
        total: 10,
        mission_state: dialect::MissionState::MISSION_STATE_ACTIVE,
        mission_mode: 0,
        mission_id: 0,
        fence_id: 0,
        rally_points_id: 0,
    });
    update_state(&header, &msg, &writers, &target);

    let ms = channels.mission_state.borrow();
    // Wire seq 3 → semantic seq 2 (Mission type subtracts 1 for home)
    assert_eq!(ms.current_seq, Some(2));
    // Wire total 10 → semantic 9 (excludes home)
    assert_eq!(ms.total_items, 9);
}

#[test]
fn home_position_updates_state() {
    let (writers, channels) = create_channels();
    let target: Option<VehicleTarget> = None;
    let header = default_header();
    let msg = dialect::MavMessage::HOME_POSITION(dialect::HOME_POSITION_DATA {
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
    let msg = dialect::MavMessage::ATTITUDE(dialect::ATTITUDE_DATA {
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
    let msg = dialect::MavMessage::STATUSTEXT(dialect::STATUSTEXT_DATA {
        severity: dialect::MavSeverity::MAV_SEVERITY_WARNING,
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
    let msg = dialect::MavMessage::NAV_CONTROLLER_OUTPUT(dialect::NAV_CONTROLLER_OUTPUT_DATA {
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
    let msg = dialect::MavMessage::TERRAIN_REPORT(dialect::TERRAIN_REPORT_DATA {
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
async fn heartbeat_processing() {
    let (msg_tx, msg_rx) = mpsc::channel(16);
    let (conn, _sent) = MockConnection::new(msg_rx);

    let (cmd_tx, cmd_rx) = mpsc::channel(16);
    let (writers, channels) = create_channels();
    let cancel = CancellationToken::new();

    let handle = tokio::spawn(async move {
        run_event_loop(Box::new(conn), cmd_rx, writers, fast_config(), cancel).await;
    });

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
async fn mock_outbound_capture() {
    let (msg_tx, msg_rx) = mpsc::channel(64);
    let (conn, sent) = MockConnection::new(msg_rx);

    let (cmd_tx, cmd_rx) = mpsc::channel(16);
    let (writers, _channels) = create_channels();
    let cancel = CancellationToken::new();

    let handle = tokio::spawn(async move {
        run_event_loop(Box::new(conn), cmd_rx, writers, fast_config(), cancel).await;
    });

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
    msg_tx
        .send((
            default_header(),
            ack_msg(
                MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
                dialect::MavResult::MAV_RESULT_ACCEPTED,
            ),
        ))
        .await
        .unwrap();

    let result = reply_rx.await.unwrap();
    assert!(result.is_ok());

    let has_arm_command = {
        let sent_msgs = sent.lock().unwrap();
        sent_msgs.iter().any(|(_, msg)| {
            matches!(
                        msg,
            dialect::MavMessage::COMMAND_LONG(d)
                            if d.command == MavCmd::MAV_CMD_COMPONENT_ARM_DISARM
                    )
        })
    };
    assert!(
        has_arm_command,
        "MockConnection must capture outbound arm COMMAND_LONG"
    );

    cmd_tx.send(Command::Shutdown).await.unwrap();
    handle.await.unwrap();
}

#[tokio::test]
async fn router_forwards_ack_to_raw_subscribers_during_command_wait() {
    let (msg_tx, msg_rx) = mpsc::channel(64);
    let (conn, _sent) = MockConnection::new(msg_rx);

    let (cmd_tx, cmd_rx) = mpsc::channel(16);
    let (writers, channels) = create_channels();
    let cancel = CancellationToken::new();
    let mut raw_rx = channels.raw_message_tx.subscribe();

    let handle = tokio::spawn(async move {
        run_event_loop(Box::new(conn), cmd_rx, writers, fast_config(), cancel).await;
    });

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

    msg_tx
        .send((
            default_header(),
            ack_msg(
                MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
                dialect::MavResult::MAV_RESULT_ACCEPTED,
            ),
        ))
        .await
        .unwrap();

    let ack_seen = tokio::time::timeout(Duration::from_millis(250), async {
        loop {
            let (_, msg) = raw_rx
                .recv()
                .await
                .expect("raw message channel should stay open while loop runs");
            if matches!(msg, dialect::MavMessage::COMMAND_ACK(_)) {
                return true;
            }
        }
    })
    .await
    .unwrap_or(false);

    let result = reply_rx.await.unwrap();
    assert!(result.is_ok());
    assert!(
        ack_seen,
        "single-ingress router should broadcast ACKs to raw subscribers"
    );

    cmd_tx.send(Command::Shutdown).await.unwrap();
    handle.await.unwrap();
}

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
                dialect::MavResult::MAV_RESULT_ACCEPTED,
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
            .filter(|(_, msg)| matches!(msg, dialect::MavMessage::COMMAND_LONG(d) if d.command == MavCmd::MAV_CMD_COMPONENT_ARM_DISARM))
                .collect();
        assert!(!arm_msgs.is_empty());

        // Verify param1 = 1.0 (arm) and param2 = 0.0 (not forced)
        if let dialect::MavMessage::COMMAND_LONG(data) = &arm_msgs[0].1 {
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
                dialect::MavResult::MAV_RESULT_ACCEPTED,
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
            .find(|(_, msg)| matches!(msg, dialect::MavMessage::COMMAND_LONG(d) if d.command == MavCmd::MAV_CMD_COMPONENT_ARM_DISARM))
                .unwrap();
        if let dialect::MavMessage::COMMAND_LONG(data) = &arm_msg.1 {
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
                dialect::MavResult::MAV_RESULT_DENIED,
            ),
        ))
        .await
        .unwrap();

    let result = reply_rx.await.unwrap();
    assert!(matches!(
        result,
        Err(VehicleError::CommandRejected {
            command,
            result: crate::error::CommandResult::Denied,
        }) if command == MavCmd::MAV_CMD_COMPONENT_ARM_DISARM as u16
    ));

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
                dialect::MavResult::MAV_RESULT_ACCEPTED,
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
            .find(|(_, msg)| matches!(msg, dialect::MavMessage::COMMAND_LONG(d) if d.command == MavCmd::MAV_CMD_DO_SET_MODE))
                .unwrap();
        if let dialect::MavMessage::COMMAND_LONG(data) = &mode_msg.1 {
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
            .find(|(_, msg)| matches!(msg, dialect::MavMessage::SET_POSITION_TARGET_GLOBAL_INT(_)))
            .unwrap();
        if let dialect::MavMessage::SET_POSITION_TARGET_GLOBAL_INT(data) = &goto_msg.1 {
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
async fn parallel_different_keys_commands_can_complete() {
    let (msg_tx, msg_rx) = mpsc::channel(64);
    let (conn, _sent) = MockConnection::new(msg_rx);

    let (cmd_tx, cmd_rx) = mpsc::channel(16);
    let (writers, _channels) = create_channels();
    let cancel = CancellationToken::new();

    let handle = tokio::spawn(async move {
        run_event_loop(Box::new(conn), cmd_rx, writers, fast_config(), cancel).await;
    });

    msg_tx
        .send((default_header(), heartbeat_msg(false, 0)))
        .await
        .unwrap();
    tokio::time::sleep(Duration::from_millis(10)).await;

    let (arm_reply_tx, arm_reply_rx) = oneshot::channel();
    cmd_tx
        .send(Command::Arm {
            force: false,
            reply: arm_reply_tx,
        })
        .await
        .unwrap();

    let (mode_reply_tx, mode_reply_rx) = oneshot::channel();
    cmd_tx
        .send(Command::SetMode {
            custom_mode: 4,
            reply: mode_reply_tx,
        })
        .await
        .unwrap();

    tokio::time::sleep(Duration::from_millis(10)).await;

    msg_tx
        .send((
            default_header(),
            ack_msg(
                MavCmd::MAV_CMD_DO_SET_MODE,
                dialect::MavResult::MAV_RESULT_ACCEPTED,
            ),
        ))
        .await
        .unwrap();
    msg_tx
        .send((
            default_header(),
            ack_msg(
                MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
                dialect::MavResult::MAV_RESULT_ACCEPTED,
            ),
        ))
        .await
        .unwrap();

    assert!(arm_reply_rx.await.unwrap().is_ok());
    assert!(mode_reply_rx.await.unwrap().is_ok());

    cmd_tx.send(Command::Shutdown).await.unwrap();
    handle.await.unwrap();
}

#[tokio::test]
async fn same_key_conflict_returns_operation_conflict() {
    let (msg_tx, msg_rx) = mpsc::channel(64);
    let (conn, _sent) = MockConnection::new(msg_rx);

    let (cmd_tx, cmd_rx) = mpsc::channel(16);
    let (writers, _channels) = create_channels();
    let cancel = CancellationToken::new();

    let handle = tokio::spawn(async move {
        run_event_loop(Box::new(conn), cmd_rx, writers, fast_config(), cancel).await;
    });

    msg_tx
        .send((default_header(), heartbeat_msg(false, 0)))
        .await
        .unwrap();
    tokio::time::sleep(Duration::from_millis(10)).await;

    let (reply1_tx, reply1_rx) = oneshot::channel();
    cmd_tx
        .send(Command::Arm {
            force: false,
            reply: reply1_tx,
        })
        .await
        .unwrap();

    let (reply2_tx, reply2_rx) = oneshot::channel();
    cmd_tx
        .send(Command::Arm {
            force: false,
            reply: reply2_tx,
        })
        .await
        .unwrap();

    let second_result = tokio::time::timeout(Duration::from_millis(200), reply2_rx)
        .await
        .expect("second same-key command should fail fast")
        .unwrap();
    assert!(matches!(
        second_result,
        Err(VehicleError::OperationConflict { .. })
    ));

    msg_tx
        .send((
            default_header(),
            ack_msg(
                MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
                dialect::MavResult::MAV_RESULT_ACCEPTED,
            ),
        ))
        .await
        .unwrap();

    assert!(reply1_rx.await.unwrap().is_ok());

    cmd_tx.send(Command::Shutdown).await.unwrap();
    handle.await.unwrap();
}

#[tokio::test]
async fn in_progress_ack_stops_retries_and_waits_terminal_ack() {
    let (msg_tx, msg_rx) = mpsc::channel(64);
    let (conn, sent) = MockConnection::new(msg_rx);

    let (cmd_tx, cmd_rx) = mpsc::channel(16);
    let (writers, _channels) = create_channels();
    let cancel = CancellationToken::new();

    let handle = tokio::spawn(async move {
        run_event_loop(Box::new(conn), cmd_rx, writers, fast_config(), cancel).await;
    });

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

    msg_tx
        .send((
            default_header(),
            ack_msg(
                MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
                dialect::MavResult::MAV_RESULT_IN_PROGRESS,
            ),
        ))
        .await
        .unwrap();

    tokio::time::sleep(Duration::from_millis(90)).await;
    let arm_sends_after_in_progress = {
        let sent_msgs = sent.lock().unwrap();
        sent_msgs
            .iter()
            .filter(|(_, msg)| {
                matches!(
                                msg,
                dialect::MavMessage::COMMAND_LONG(data)
                                    if data.command == MavCmd::MAV_CMD_COMPONENT_ARM_DISARM
                            )
            })
            .count()
    };
    assert_eq!(arm_sends_after_in_progress, 1);

    msg_tx
        .send((
            default_header(),
            ack_msg(
                MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
                dialect::MavResult::MAV_RESULT_ACCEPTED,
            ),
        ))
        .await
        .unwrap();

    assert!(reply_rx.await.unwrap().is_ok());

    cmd_tx.send(Command::Shutdown).await.unwrap();
    handle.await.unwrap();
}

#[tokio::test]
async fn command_long_retries_increment_confirmation_field() {
    let (msg_tx, msg_rx) = mpsc::channel(64);
    let (conn, sent) = MockConnection::new(msg_rx);

    let (cmd_tx, cmd_rx) = mpsc::channel(16);
    let (writers, _channels) = create_channels();
    let cancel = CancellationToken::new();

    let handle = tokio::spawn(async move {
        run_event_loop(Box::new(conn), cmd_rx, writers, fast_config(), cancel).await;
    });

    msg_tx
        .send((default_header(), heartbeat_msg(false, 0)))
        .await
        .unwrap();
    tokio::time::sleep(Duration::from_millis(10)).await;

    let (reply_tx, reply_rx) = oneshot::channel();
    cmd_tx
        .send(Command::Long {
            command: MavCmd::MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            params: [3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            reply: reply_tx,
        })
        .await
        .unwrap();

    let confirmations = tokio::time::timeout(Duration::from_millis(250), async {
        loop {
            let values = {
                let sent_msgs = sent.lock().unwrap();
                sent_msgs
                    .iter()
                    .filter_map(|(_, msg)| match msg {
                        dialect::MavMessage::COMMAND_LONG(data)
                            if data.command == MavCmd::MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN =>
                        {
                            Some(data.confirmation)
                        }
                        _ => None,
                    })
                    .collect::<Vec<_>>()
            };

            if values.len() >= 2 {
                break values;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    })
    .await
    .expect("expected at least one retry send");

    assert_eq!(confirmations[0], 0);
    assert_eq!(confirmations[1], 1);

    msg_tx
        .send((
            default_header(),
            ack_msg(
                MavCmd::MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                dialect::MavResult::MAV_RESULT_ACCEPTED,
            ),
        ))
        .await
        .unwrap();

    assert!(reply_rx.await.unwrap().is_ok());

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
                dialect::MavResult::MAV_RESULT_ACCEPTED,
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
            .find(|(_, msg)| matches!(msg, dialect::MavMessage::COMMAND_LONG(d) if d.command == MavCmd::MAV_CMD_COMPONENT_ARM_DISARM))
                .unwrap();
        if let dialect::MavMessage::COMMAND_LONG(data) = &disarm_msg.1 {
            assert_eq!(data.param1, 0.0); // disarm
            assert_eq!(data.param2, MAGIC_FORCE_DISARM_VALUE);
        }
    }

    cmd_tx.send(Command::Shutdown).await.unwrap();
    handle.await.unwrap();
}

#[tokio::test]
async fn command_long_reboot_to_bootloader_sends_preflight_reboot_shutdown() {
    let (msg_tx, msg_rx) = mpsc::channel(64);
    let (conn, sent) = MockConnection::new(msg_rx);

    let (cmd_tx, cmd_rx) = mpsc::channel(16);
    let (writers, _channels) = create_channels();
    let cancel = CancellationToken::new();

    let handle = tokio::spawn(async move {
        run_event_loop(Box::new(conn), cmd_rx, writers, fast_config(), cancel).await;
    });

    msg_tx
        .send((default_header(), heartbeat_msg(false, 0)))
        .await
        .unwrap();
    tokio::time::sleep(Duration::from_millis(10)).await;

    let (reply_tx, reply_rx) = oneshot::channel();
    cmd_tx
        .send(Command::Long {
            command: MavCmd::MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            params: [3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            reply: reply_tx,
        })
        .await
        .unwrap();
    tokio::time::sleep(Duration::from_millis(10)).await;

    msg_tx
        .send((
            default_header(),
            ack_msg(
                MavCmd::MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                dialect::MavResult::MAV_RESULT_ACCEPTED,
            ),
        ))
        .await
        .unwrap();

    let result = reply_rx.await.unwrap();
    assert!(result.is_ok());

    {
        let sent_msgs = sent.lock().unwrap();
        let reboot_msg = sent_msgs
                .iter()
            .find(|(_, msg)| matches!(msg, dialect::MavMessage::COMMAND_LONG(d) if d.command == MavCmd::MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN))
                .unwrap();
        if let dialect::MavMessage::COMMAND_LONG(data) = &reboot_msg.1 {
            assert_eq!(data.param1, 3.0);
            assert_eq!(data.param2, 0.0);
            assert_eq!(data.param3, 0.0);
            assert_eq!(data.param4, 0.0);
            assert_eq!(data.param5, 0.0);
            assert_eq!(data.param6, 0.0);
            assert_eq!(data.param7, 0.0);
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
        autopilot: dialect::MavAutopilot::MAV_AUTOPILOT_GENERIC,
        vehicle_type: dialect::MavType::MAV_TYPE_GENERIC,
    });
    let t = get_target(&target).unwrap();
    assert_eq!(t.system_id, 1);
}

#[test]
fn to_mav_mission_type_roundtrip() {
    assert_eq!(
        to_mav_mission_type(MissionType::Mission),
        dialect::MavMissionType::MAV_MISSION_TYPE_MISSION
    );
    assert_eq!(
        to_mav_mission_type(MissionType::Fence),
        dialect::MavMissionType::MAV_MISSION_TYPE_FENCE
    );
    assert_eq!(
        to_mav_mission_type(MissionType::Rally),
        dialect::MavMissionType::MAV_MISSION_TYPE_RALLY
    );
}

#[test]
fn frame_conversion_roundtrip() {
    let frames = [
        (MissionFrame::Mission, dialect::MavFrame::MAV_FRAME_MISSION),
        (MissionFrame::GlobalInt, dialect::MavFrame::MAV_FRAME_GLOBAL),
        (
            MissionFrame::GlobalRelativeAltInt,
            dialect::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
        ),
        (
            MissionFrame::GlobalTerrainAltInt,
            dialect::MavFrame::MAV_FRAME_GLOBAL_TERRAIN_ALT,
        ),
        (
            MissionFrame::LocalNed,
            dialect::MavFrame::MAV_FRAME_LOCAL_NED,
        ),
    ];
    for (kit_frame, mav_frame) in frames {
        assert_eq!(to_mav_frame(kit_frame), mav_frame);
        assert_eq!(from_mav_frame(mav_frame), kit_frame);
    }
}

#[test]
fn from_mission_item_int_converts_correctly() {
    let data = dialect::MISSION_ITEM_INT_DATA {
        param1: 1.0,
        param2: 2.0,
        param3: 3.0,
        param4: 4.0,
        x: 473_977_420,
        y: 85_455_940,
        z: 100.0,
        seq: 0,
        command: dialect::MavCmd::MAV_CMD_NAV_WAYPOINT,
        target_system: 1,
        target_component: 1,
        frame: dialect::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
        current: 1,
        autocontinue: 1,
        mission_type: dialect::MavMissionType::MAV_MISSION_TYPE_MISSION,
    };
    let item = from_mission_item_int(&data);
    let (_, frame, _params, x, y, z) = item.command.clone().into_wire();
    assert_eq!(item.seq, 0);
    assert_eq!(x, 473_977_420);
    assert_eq!(y, 85_455_940);
    assert_eq!(z, 100.0);
    assert!(item.current);
    assert!(item.autocontinue);
    assert_eq!(
        frame,
        crate::mission::commands::MissionFrame::GlobalRelativeAlt
    );
}

#[test]
fn param_type_conversions() {
    use crate::dialect::MavParamType;
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
        dialect::MavMissionType::MAV_MISSION_TYPE_MISSION,
        MissionType::Mission
    ));
    assert!(!mission_type_matches(
        dialect::MavMissionType::MAV_MISSION_TYPE_FENCE,
        MissionType::Mission
    ));
    assert!(mission_type_matches(
        dialect::MavMissionType::MAV_MISSION_TYPE_FENCE,
        MissionType::Fence
    ));
    assert!(mission_type_matches(
        dialect::MavMissionType::MAV_MISSION_TYPE_RALLY,
        MissionType::Rally
    ));
    assert!(!mission_type_matches(
        dialect::MavMissionType::MAV_MISSION_TYPE_RALLY,
        MissionType::Fence
    ));
}

#[allow(deprecated)]
#[tokio::test]
async fn auto_request_home_sends_get_home_position_command() {
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

    {
        let sent_msgs = sent.lock().unwrap();
        let home_req = sent_msgs.iter().find(|(_, msg)| {
            matches!(
                        msg,
            dialect::MavMessage::COMMAND_LONG(d) if d.command == MavCmd::MAV_CMD_GET_HOME_POSITION
                    )
        });
        assert!(home_req.is_some(), "should have requested home position");
    }

    cmd_tx.send(Command::Shutdown).await.unwrap();
    handle.await.unwrap();
}

#[allow(deprecated)]
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
        dialect::MavMessage::COMMAND_LONG(d) if d.command == MavCmd::MAV_CMD_GET_HOME_POSITION
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

    let msg = dialect::MavMessage::BATTERY_STATUS(dialect::BATTERY_STATUS_DATA {
        current_consumed: 0,
        energy_consumed: 7200, // 200 Wh
        temperature: 0,
        voltages,
        current_battery: 0,
        id: 0,
        battery_function: dialect::MavBatteryFunction::MAV_BATTERY_FUNCTION_UNKNOWN,
        mavtype: dialect::MavBatteryType::MAV_BATTERY_TYPE_UNKNOWN,
        battery_remaining: 0,
        time_remaining: 1800, // 30 minutes
        charge_state: dialect::MavBatteryChargeState::MAV_BATTERY_CHARGE_STATE_OK,
        voltages_ext: [0; 4],
        mode: dialect::MavBatteryMode::MAV_BATTERY_MODE_UNKNOWN,
        fault_bitmask: dialect::MavBatteryFault::empty(),
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
    let msg = dialect::MavMessage::RC_CHANNELS(dialect::RC_CHANNELS_DATA {
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
    let msg = dialect::MavMessage::SERVO_OUTPUT_RAW(dialect::SERVO_OUTPUT_RAW_DATA {
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
    let msg = dialect::MavMessage::STATUSTEXT(dialect::STATUSTEXT_DATA {
        severity: dialect::MavSeverity::MAV_SEVERITY_INFO,
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
    let msg = dialect::MavMessage::SYS_STATUS(dialect::SYS_STATUS_DATA {
        onboard_control_sensors_present: dialect::MavSysStatusSensor::empty(),
        onboard_control_sensors_enabled: dialect::MavSysStatusSensor::empty(),
        onboard_control_sensors_health: dialect::MavSysStatusSensor::empty(),
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
        onboard_control_sensors_present_extended: dialect::MavSysStatusSensorExtended::empty(),
        onboard_control_sensors_enabled_extended: dialect::MavSysStatusSensorExtended::empty(),
        onboard_control_sensors_health_extended: dialect::MavSysStatusSensorExtended::empty(),
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
    let msg = dialect::MavMessage::GPS_RAW_INT(dialect::GPS_RAW_INT_DATA {
        time_usec: 0,
        fix_type: dialect::GpsFixType::GPS_FIX_TYPE_NO_FIX,
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
    let msg = dialect::MavMessage::GLOBAL_POSITION_INT(dialect::GLOBAL_POSITION_INT_DATA {
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

#[test]
fn sys_status_updates_sensor_health() {
    let (writers, channels) = create_channels();
    let target: Option<VehicleTarget> = None;
    let header = default_header();
    let present = dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_GYRO
        | dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_GPS
        | dialect::MavSysStatusSensor::MAV_SYS_STATUS_PREARM_CHECK;
    let enabled = present;
    let health = dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_GYRO
        | dialect::MavSysStatusSensor::MAV_SYS_STATUS_PREARM_CHECK;
    // GPS present+enabled but NOT healthy
    let msg = dialect::MavMessage::SYS_STATUS(dialect::SYS_STATUS_DATA {
        onboard_control_sensors_present: present,
        onboard_control_sensors_enabled: enabled,
        onboard_control_sensors_health: health,
        load: 0,
        voltage_battery: u16::MAX,
        current_battery: -1,
        battery_remaining: -1,
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
    update_state(&header, &msg, &writers, &target);

    let sh = channels.sensor_health.borrow();
    assert!(sh.pre_arm_good);
    let gps = sh
        .sensors
        .iter()
        .find(|(id, _)| *id == state::SensorId::Gps)
        .unwrap();
    assert_eq!(gps.1, state::SensorStatus::Unhealthy);
}

#[tokio::test]
async fn set_current_semantic_zero_sends_wire_seq_one() {
    let (msg_tx, msg_rx) = mpsc::channel(64);
    let (conn, sent) = MockConnection::new(msg_rx);

    let (cmd_tx, cmd_rx) = mpsc::channel(16);
    let (writers, _channels) = create_channels();
    let cancel = CancellationToken::new();

    let handle = tokio::spawn(async move {
        run_event_loop(Box::new(conn), cmd_rx, writers, fast_config(), cancel).await;
    });

    // Establish vehicle target
    msg_tx
        .send((default_header(), heartbeat_msg(false, 0)))
        .await
        .unwrap();
    tokio::time::sleep(Duration::from_millis(10)).await;

    // Send semantic set_current(0) — first visible waypoint
    let (reply_tx, reply_rx) = oneshot::channel();
    cmd_tx
        .send(Command::MissionSetCurrent {
            seq: 0,
            reply: reply_tx,
        })
        .await
        .unwrap();
    tokio::time::sleep(Duration::from_millis(10)).await;

    // Respond with COMMAND_ACK
    msg_tx
        .send((
            default_header(),
            ack_msg(
                MavCmd::MAV_CMD_DO_SET_MISSION_CURRENT,
                dialect::MavResult::MAV_RESULT_ACCEPTED,
            ),
        ))
        .await
        .unwrap();

    let result = reply_rx.await.unwrap();
    assert!(result.is_ok());

    // Verify the sent COMMAND_LONG has wire seq 1 (not semantic 0)
    {
        let sent_msgs = sent.lock().unwrap();
        let set_current_msgs: Vec<_> = sent_msgs
            .iter()
            .filter(|(_, msg)| {
                matches!(msg, dialect::MavMessage::COMMAND_LONG(d)
                        if d.command == MavCmd::MAV_CMD_DO_SET_MISSION_CURRENT)
            })
            .collect();
        assert!(
            !set_current_msgs.is_empty(),
            "expected COMMAND_LONG for DO_SET_MISSION_CURRENT"
        );

        if let dialect::MavMessage::COMMAND_LONG(data) = &set_current_msgs[0].1 {
            assert_eq!(
                data.param1, 1.0,
                "semantic seq 0 must become wire seq 1 (param1)"
            );
        }
    }

    cmd_tx.send(Command::Shutdown).await.unwrap();
    handle.await.unwrap();
}

#[tokio::test]
async fn set_current_semantic_four_sends_wire_seq_five() {
    let (msg_tx, msg_rx) = mpsc::channel(64);
    let (conn, sent) = MockConnection::new(msg_rx);

    let (cmd_tx, cmd_rx) = mpsc::channel(16);
    let (writers, _channels) = create_channels();
    let cancel = CancellationToken::new();

    let handle = tokio::spawn(async move {
        run_event_loop(Box::new(conn), cmd_rx, writers, fast_config(), cancel).await;
    });

    msg_tx
        .send((default_header(), heartbeat_msg(false, 0)))
        .await
        .unwrap();
    tokio::time::sleep(Duration::from_millis(10)).await;

    let (reply_tx, reply_rx) = oneshot::channel();
    cmd_tx
        .send(Command::MissionSetCurrent {
            seq: 4,
            reply: reply_tx,
        })
        .await
        .unwrap();
    tokio::time::sleep(Duration::from_millis(10)).await;

    // Respond with MISSION_CURRENT confirming wire seq 5
    msg_tx
        .send((
            default_header(),
            dialect::MavMessage::MISSION_CURRENT(dialect::MISSION_CURRENT_DATA {
                seq: 5,
                total: 10,
                mission_state: dialect::MissionState::MISSION_STATE_ACTIVE,
                mission_mode: 0,
                mission_id: 0,
                fence_id: 0,
                rally_points_id: 0,
            }),
        ))
        .await
        .unwrap();

    let result = reply_rx.await.unwrap();
    assert!(result.is_ok());

    {
        let sent_msgs = sent.lock().unwrap();
        let set_current_msgs: Vec<_> = sent_msgs
            .iter()
            .filter(|(_, msg)| {
                matches!(msg, dialect::MavMessage::COMMAND_LONG(d)
                        if d.command == MavCmd::MAV_CMD_DO_SET_MISSION_CURRENT)
            })
            .collect();
        assert!(!set_current_msgs.is_empty());

        if let dialect::MavMessage::COMMAND_LONG(data) = &set_current_msgs[0].1 {
            assert_eq!(
                data.param1, 5.0,
                "semantic seq 4 must become wire seq 5 (param1)"
            );
        }
    }

    cmd_tx.send(Command::Shutdown).await.unwrap();
    handle.await.unwrap();
}

#[test]
fn mag_cal_report_updates_channel() {
    let (writers, channels) = create_channels();
    let target: Option<VehicleTarget> = None;
    let header = default_header();
    let msg = dialect::MavMessage::MAG_CAL_REPORT(dialect::MAG_CAL_REPORT_DATA {
        fitness: 0.05,
        ofs_x: 1.0,
        ofs_y: 2.0,
        ofs_z: 3.0,
        diag_x: 0.0,
        diag_y: 0.0,
        diag_z: 0.0,
        offdiag_x: 0.0,
        offdiag_y: 0.0,
        offdiag_z: 0.0,
        compass_id: 0,
        cal_mask: 0x01,
        cal_status: dialect::MagCalStatus::MAG_CAL_SUCCESS,
        autosaved: 1,
        ..dialect::MAG_CAL_REPORT_DATA::DEFAULT
    });
    update_state(&header, &msg, &writers, &target);

    let report = channels.mag_cal_report.borrow();
    let report = report.as_ref().unwrap();
    assert_eq!(report.compass_id, 0);
    assert_eq!(report.status, state::MagCalStatus::Success);
    assert!(report.autosaved);
    assert!((report.fitness - 0.05).abs() < 0.001);
    assert_eq!(report.ofs_x, 1.0);
}

mod init {
    use super::*;
    use std::sync::Arc;

    fn init_target() -> VehicleTarget {
        VehicleTarget {
            system_id: 1,
            component_id: 1,
            autopilot: dialect::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
            vehicle_type: dialect::MavType::MAV_TYPE_QUADROTOR,
        }
    }

    #[tokio::test]
    async fn autopilot_version_request_sent_after_first_heartbeat() {
        let (msg_tx, msg_rx) = mpsc::channel(64);
        let (conn, sent) = MockConnection::new(msg_rx);

        let (cmd_tx, cmd_rx) = mpsc::channel(16);
        let (writers, _channels) = create_channels();
        let cancel = CancellationToken::new();

        let handle = tokio::spawn(async move {
            run_event_loop(Box::new(conn), cmd_rx, writers, fast_config(), cancel).await;
        });

        msg_tx
            .send((default_header(), heartbeat_msg(false, 0)))
            .await
            .unwrap();
        tokio::time::sleep(Duration::from_millis(200)).await;

        let has_autopilot_version_request = {
            let sent_msgs = sent.lock().unwrap();
            sent_msgs.iter().any(|(_, msg)| {
                matches!(
                                msg,
                dialect::MavMessage::COMMAND_LONG(data)
                                    if data.command == MavCmd::MAV_CMD_REQUEST_MESSAGE
                                        && data.param1 == 148.0
                            )
            })
        };
        assert!(
            has_autopilot_version_request,
            "heartbeat should trigger an AUTOPILOT_VERSION request"
        );

        cmd_tx.send(Command::Shutdown).await.unwrap();
        handle.await.unwrap();
    }

    #[tokio::test]
    async fn autopilot_version_silence_exhausts_into_unavailable() {
        let (_msg_tx, msg_rx) = mpsc::channel(16);
        let (conn, sent) = MockConnection::new(msg_rx);
        let connection: SharedConnection = Arc::new(conn);

        let mut config = fast_config();
        config.init_policy.autopilot_version.max_attempts = 3;
        config.init_policy.autopilot_version.budget = Duration::from_millis(90);

        let manager = crate::event_loop::init::InitManager::new(config.clone());
        manager.start(connection, init_target(), CancellationToken::new());

        tokio::time::sleep(Duration::from_millis(160)).await;

        let snapshot = manager.snapshot();
        assert!(matches!(
            snapshot.autopilot_version,
            crate::event_loop::init::InitState::Unavailable {
                reason: crate::event_loop::init::InitUnavailableReason::SilenceBudgetExhausted
            }
        ));

        let sent_msgs = sent.lock().unwrap();
        let request_count = sent_msgs
                .iter()
                .filter(|(_, msg)| matches!(
                    msg,
        dialect::MavMessage::COMMAND_LONG(data)
                        if data.command == MavCmd::MAV_CMD_REQUEST_MESSAGE && data.param1 == 148.0
                ))
                .count();
        assert_eq!(request_count, 3, "retry budget should cap attempts");
    }

    #[tokio::test]
    async fn autopilot_version_response_transitions_to_available() {
        let (_msg_tx, msg_rx) = mpsc::channel(16);
        let (conn, _sent) = MockConnection::new(msg_rx);
        let connection: SharedConnection = Arc::new(conn);

        let mut config = fast_config();
        config.init_policy.autopilot_version.budget = Duration::from_millis(120);

        let manager = crate::event_loop::init::InitManager::new(config);
        manager.start(connection, init_target(), CancellationToken::new());
        manager.handle_message(&dialect::MavMessage::AUTOPILOT_VERSION(
            dialect::AUTOPILOT_VERSION_DATA {
                flight_sw_version: 0x01020304,
                uid: 42,
                ..dialect::AUTOPILOT_VERSION_DATA::default()
            },
        ));

        tokio::time::sleep(Duration::from_millis(20)).await;

        let snapshot = manager.snapshot();
        match snapshot.autopilot_version {
            crate::event_loop::init::InitState::Available(version) => {
                assert_eq!(version.flight_sw_version, 0x01020304);
                assert_eq!(version.uid, 42);
            }
            other => panic!("expected available autopilot version, got {other:?}"),
        }
    }

    #[tokio::test]
    async fn home_silence_does_not_transition_to_unavailable() {
        let (_msg_tx, msg_rx) = mpsc::channel(16);
        let (conn, _sent) = MockConnection::new(msg_rx);
        let connection: SharedConnection = Arc::new(conn);

        let mut config = fast_config();
        config.auto_request_home = true;
        config.init_policy.home.max_attempts = 1;
        config.init_policy.home.budget = Duration::from_millis(50);

        let manager = crate::event_loop::init::InitManager::new(config);
        manager.start(connection, init_target(), CancellationToken::new());

        tokio::time::sleep(Duration::from_millis(90)).await;

        let snapshot = manager.snapshot();
        assert!(matches!(
            snapshot.home_position,
            crate::event_loop::init::InitState::Unknown
        ));
    }

    #[tokio::test]
    async fn origin_silence_does_not_transition_to_unavailable() {
        let (_msg_tx, msg_rx) = mpsc::channel(16);
        let (conn, _sent) = MockConnection::new(msg_rx);
        let connection: SharedConnection = Arc::new(conn);

        let mut config = fast_config();
        config.init_policy.origin.max_attempts = 1;
        config.init_policy.origin.budget = Duration::from_millis(50);

        let manager = crate::event_loop::init::InitManager::new(config);
        manager.start(connection, init_target(), CancellationToken::new());

        tokio::time::sleep(Duration::from_millis(90)).await;

        let snapshot = manager.snapshot();
        assert!(matches!(
            snapshot.gps_global_origin,
            crate::event_loop::init::InitState::Unknown
        ));
    }

    #[tokio::test]
    async fn init_policy_override_limits_available_modes_attempts() {
        let (_msg_tx, msg_rx) = mpsc::channel(16);
        let (conn, sent) = MockConnection::new(msg_rx);
        let connection: SharedConnection = Arc::new(conn);

        let mut config = fast_config();
        config.init_policy.available_modes.max_attempts = 1;
        config.init_policy.available_modes.budget = Duration::from_millis(40);

        let manager = crate::event_loop::init::InitManager::new(config);
        manager.start(connection, init_target(), CancellationToken::new());

        tokio::time::sleep(Duration::from_millis(80)).await;

        let snapshot = manager.snapshot();
        assert!(matches!(
            snapshot.available_modes,
            crate::event_loop::init::InitState::Unavailable {
                reason: crate::event_loop::init::InitUnavailableReason::SilenceBudgetExhausted
            }
        ));

        let sent_msgs = sent.lock().unwrap();
        let request_count = sent_msgs
                .iter()
                .filter(|(_, msg)| matches!(
                    msg,
        dialect::MavMessage::COMMAND_LONG(data)
                        if data.command == MavCmd::MAV_CMD_REQUEST_MESSAGE && data.param1 == 435.0
                ))
                .count();
        assert_eq!(request_count, 1, "override should reduce retry attempts");
    }
}
