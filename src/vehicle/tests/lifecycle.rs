use super::support::*;
use crate::dialect;
use crate::{AutopilotType, Vehicle, VehicleError, VehicleIdentity, VehicleType};
#[cfg(feature = "stream")]
use mavlink::async_peek_reader::AsyncPeekReader;
#[cfg(feature = "byte-connection")]
use mavlink::peek_reader::PeekReader;
#[cfg(feature = "byte-connection")]
use mavlink::{MavHeader, MavlinkVersion, ReadVersion};
#[cfg(feature = "stream")]
use mavlink::{MavlinkVersion, ReadVersion, read_versioned_msg_async, write_versioned_msg_async};
#[cfg(feature = "byte-connection")]
use mavlink::{read_versioned_msg, write_versioned_msg};
use std::sync::Arc;
use std::time::Duration;
#[cfg(feature = "stream")]
use tokio::io::{duplex, split};
use tokio::sync::{mpsc, watch};
use tokio::time::timeout;

#[test]
fn send_sync_bounds() {
    fn assert_bounds<T: Clone + Send + Sync>() {}
    assert_bounds::<Vehicle>();
}

#[tokio::test]
async fn link_state_transitions_to_connected_after_connect() {
    let (vehicle, _msg_tx) = connect_mock_vehicle().await;

    assert_eq!(
        vehicle.link().state().latest(),
        Some(crate::LinkState::Connected)
    );
}

#[tokio::test]
async fn disconnect_sends_shutdown_command() {
    let (vehicle, mut command_rx) = test_vehicle_with_command_rx();
    let disconnect_task = tokio::spawn(async move { vehicle.disconnect().await });

    let command = command_rx
        .recv()
        .await
        .expect("disconnect should send a shutdown command");

    assert!(matches!(command, crate::command::Command::Shutdown));
    assert!(matches!(
        disconnect_task.await.expect("task should complete"),
        Err(VehicleError::Disconnected)
    ));
}

#[tokio::test]
async fn connect_waits_for_first_heartbeat_and_populates_identity() {
    let (msg_tx, msg_rx) = mpsc::channel(16);
    let (conn, _sent) = crate::test_support::MockConnection::new(msg_rx);

    let connect_task =
        tokio::spawn(async move { Vehicle::from_connection(Box::new(conn), fast_config()).await });

    tokio::time::sleep(Duration::from_millis(20)).await;
    assert!(
        !connect_task.is_finished(),
        "connect should stay pending until the first heartbeat arrives"
    );

    msg_tx
        .send((
            crate::test_support::default_header(),
            heartbeat_msg(true, 5),
        ))
        .await
        .expect("heartbeat should be delivered");

    let vehicle = timeout(Duration::from_millis(250), connect_task)
        .await
        .expect("connect should finish after the heartbeat")
        .expect("connect task should join")
        .expect("connect should succeed");

    assert_eq!(
        vehicle.identity(),
        VehicleIdentity {
            system_id: 1,
            component_id: 1,
            autopilot: AutopilotType::ArduPilotMega,
            vehicle_type: VehicleType::Quadrotor,
        }
    );
    assert!(vehicle.available_modes().current().latest().is_some());

    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
}

#[tokio::test]
async fn connect_reports_transport_failure_before_first_heartbeat() {
    let (msg_tx, msg_rx) = mpsc::channel(16);
    let (conn, _sent) = crate::test_support::MockConnection::new(msg_rx);

    let connect_task =
        tokio::spawn(async move { Vehicle::from_connection(Box::new(conn), fast_config()).await });

    drop(msg_tx);

    let result = timeout(Duration::from_millis(250), connect_task)
        .await
        .expect("connect should fail when the transport closes")
        .expect("connect task should join");

    let err = match result {
        Ok(_) => panic!("connect should not succeed without a heartbeat"),
        Err(err) => err,
    };

    assert!(
        matches!(err, VehicleError::ConnectionFailed(message) if message.contains("mock connection closed"))
    );
}

#[cfg(feature = "stream")]
#[tokio::test]
async fn from_stream_parts_waits_for_first_heartbeat_and_populates_identity() {
    let (client, server) = duplex(1024);
    let (client_read, client_write) = split(client);
    let (_server_read, mut server_write) = split(server);

    let connect_task = tokio::spawn(async move {
        Vehicle::from_stream_parts(client_read, client_write, fast_config()).await
    });

    tokio::time::sleep(Duration::from_millis(20)).await;
    assert!(
        !connect_task.is_finished(),
        "connect should stay pending until the first heartbeat arrives"
    );

    write_versioned_msg_async(
        &mut server_write,
        MavlinkVersion::V2,
        crate::test_support::default_header(),
        &heartbeat_msg(true, 5),
    )
    .await
    .expect("heartbeat should be written to the stream peer");

    let vehicle = timeout(Duration::from_millis(250), connect_task)
        .await
        .expect("connect should finish after the heartbeat")
        .expect("connect task should join")
        .expect("connect should succeed");

    assert_eq!(
        vehicle.identity(),
        VehicleIdentity {
            system_id: 1,
            component_id: 1,
            autopilot: AutopilotType::ArduPilotMega,
            vehicle_type: VehicleType::Quadrotor,
        }
    );
    assert!(vehicle.available_modes().current().latest().is_some());

    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
}

#[cfg(feature = "stream")]
#[tokio::test]
async fn from_stream_parts_supports_command_round_trip() {
    let (client, server) = duplex(1024);
    let (client_read, client_write) = split(client);
    let (server_read, mut server_write) = split(server);
    let mut server_read = AsyncPeekReader::new(server_read);

    let connect_task = tokio::spawn(async move {
        Vehicle::from_stream_parts(client_read, client_write, fast_config()).await
    });

    write_versioned_msg_async(
        &mut server_write,
        MavlinkVersion::V2,
        crate::test_support::default_header(),
        &heartbeat_msg(false, 7),
    )
    .await
    .expect("heartbeat should be written to the stream peer");

    let vehicle = timeout(Duration::from_millis(250), connect_task)
        .await
        .expect("connect should finish after the heartbeat")
        .expect("connect task should join")
        .expect("connect should succeed");

    let arm_task = {
        let vehicle = vehicle.clone();
        tokio::spawn(async move { vehicle.arm().await })
    };

    let arm_command = timeout(Duration::from_millis(250), async {
        loop {
            let (_, message) = read_versioned_msg_async::<dialect::MavMessage, _>(
                &mut server_read,
                ReadVersion::Any,
            )
            .await
            .expect("outgoing command should be readable from the stream peer");

            match message {
                dialect::MavMessage::COMMAND_LONG(data)
                    if data.command == dialect::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM =>
                {
                    break data;
                }
                _ => {}
            }
        }
    })
    .await
    .expect("arm command should be observed before timeout");

    assert_eq!(arm_command.param1, 1.0);

    write_versioned_msg_async(
        &mut server_write,
        MavlinkVersion::V2,
        crate::test_support::default_header(),
        &ack_msg(
            dialect::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
            dialect::MavResult::MAV_RESULT_ACCEPTED,
        ),
    )
    .await
    .expect("ack should be written to the stream peer");

    assert!(arm_task.await.expect("arm task should join").is_ok());

    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
}

#[cfg(feature = "byte-connection")]
fn encode_byte_connection_frame(header: MavHeader, message: &dialect::MavMessage) -> Vec<u8> {
    let mut bytes = Vec::new();
    write_versioned_msg(&mut bytes, MavlinkVersion::V2, header, message)
        .expect("frame should encode");
    bytes
}

#[cfg(feature = "byte-connection")]
fn decode_byte_connection_frame(bytes: Vec<u8>) -> (MavHeader, dialect::MavMessage) {
    let mut reader = PeekReader::new(std::io::Cursor::new(bytes));
    read_versioned_msg::<dialect::MavMessage, _>(&mut reader, ReadVersion::Any)
        .expect("frame should decode")
}

#[cfg(feature = "byte-connection")]
#[tokio::test]
async fn from_byte_connection_exposes_bridge_before_first_heartbeat() {
    let (bridge, vehicle_future) = Vehicle::from_byte_connection(
        fast_config(),
        crate::byte_connection::ByteConnectionConfig::default(),
    );

    bridge
        .push_inbound(encode_byte_connection_frame(
            crate::test_support::default_header(),
            &heartbeat_msg(true, 5),
        ))
        .await
        .expect("heartbeat should be delivered through the byte bridge");

    let vehicle = timeout(Duration::from_millis(250), vehicle_future)
        .await
        .expect("vehicle future should finish after the queued heartbeat")
        .expect("vehicle should connect");

    assert_eq!(
        vehicle.identity(),
        VehicleIdentity {
            system_id: 1,
            component_id: 1,
            autopilot: AutopilotType::ArduPilotMega,
            vehicle_type: VehicleType::Quadrotor,
        }
    );
    assert!(vehicle.available_modes().current().latest().is_some());

    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
}

#[cfg(feature = "byte-connection")]
#[tokio::test]
async fn from_byte_connection_supports_command_round_trip() {
    let (bridge, vehicle_future) = Vehicle::from_byte_connection(
        fast_config(),
        crate::byte_connection::ByteConnectionConfig::default(),
    );
    let vehicle_task = tokio::spawn(vehicle_future);

    bridge
        .push_inbound(encode_byte_connection_frame(
            crate::test_support::default_header(),
            &heartbeat_msg(false, 7),
        ))
        .await
        .expect("heartbeat should be delivered through the byte bridge");

    let vehicle = timeout(Duration::from_millis(250), vehicle_task)
        .await
        .expect("vehicle future should finish after the heartbeat")
        .expect("vehicle task should join")
        .expect("vehicle should connect");

    let arm_task = {
        let vehicle = vehicle.clone();
        tokio::spawn(async move { vehicle.arm().await })
    };

    let (_, outbound) = timeout(Duration::from_millis(250), async {
        loop {
            let outbound = bridge
                .next_outbound()
                .await
                .expect("outbound frame should be available before close");
            let decoded = decode_byte_connection_frame(outbound);

            match &decoded.1 {
                dialect::MavMessage::COMMAND_LONG(data)
                    if data.command == dialect::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM =>
                {
                    break decoded;
                }
                _ => {}
            }
        }
    })
    .await
    .expect("arm command should be observed before timeout");

    let arm_command = match outbound {
        dialect::MavMessage::COMMAND_LONG(data) => data,
        other => panic!("expected COMMAND_LONG, got {other:?}"),
    };
    assert_eq!(arm_command.param1, 1.0);

    bridge
        .push_inbound(encode_byte_connection_frame(
            crate::test_support::default_header(),
            &ack_msg(
                dialect::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
                dialect::MavResult::MAV_RESULT_ACCEPTED,
            ),
        ))
        .await
        .expect("ack should be delivered through the byte bridge");

    assert!(arm_task.await.expect("arm task should join").is_ok());

    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
}

#[tokio::test]
async fn connect_disconnect_waits_for_disconnected_state_and_pending_waits() {
    let (vehicle, msg_tx) = connect_mock_vehicle().await;
    let pending_wait = tokio::spawn(wait_for_metric_disconnect(vehicle.telemetry().home()));

    vehicle
        .disconnect()
        .await
        .expect("disconnect should wait for the lifecycle to finish closing");

    assert_eq!(
        *vehicle.inner.stores.link_state.borrow(),
        crate::state::LinkState::Disconnected,
        "disconnect should not return until link state is disconnected"
    );

    let wait_result = timeout(Duration::from_millis(100), pending_wait)
        .await
        .expect("pending metric wait should resolve promptly on disconnect")
        .expect("wait task should join");
    assert!(matches!(wait_result, Err(VehicleError::Disconnected)));

    drop(msg_tx);
}

#[tokio::test]
async fn dropping_last_vehicle_clone_resolves_pending_waits_with_disconnected() {
    let (vehicle, msg_tx) = connect_mock_vehicle().await;
    let pending_wait = tokio::spawn(wait_for_metric_disconnect(vehicle.telemetry().home()));

    drop(vehicle);

    let wait_result = timeout(Duration::from_millis(100), pending_wait)
        .await
        .expect("dropping the last vehicle clone should resolve pending waits promptly")
        .expect("wait task should join");
    assert!(matches!(wait_result, Err(VehicleError::Disconnected)));

    drop(msg_tx);
}

#[tokio::test]
async fn disconnect_send_failure_is_ok_when_shutdown_already_completed() {
    let (mut vehicle, command_rx) = test_vehicle_with_command_rx();
    drop(command_rx);

    let (_shutdown_complete_tx, shutdown_complete_rx) = watch::channel(true);
    Arc::get_mut(&mut vehicle.inner)
        .expect("vehicle should have unique inner ownership in this test")
        .shutdown_complete = Some(shutdown_complete_rx);

    vehicle
        .disconnect()
        .await
        .expect("disconnect should be idempotent when shutdown is already complete");
}

#[tokio::test]
async fn disconnect_closes_connection_scoped_observations_for_remaining_clones() {
    let (vehicle, msg_tx) = connect_mock_vehicle().await;
    let mission_vehicle = vehicle.clone();
    let params_vehicle = vehicle.clone();
    let fence_vehicle = vehicle.clone();
    let rally_vehicle = vehicle.clone();
    let mag_cal_progress_vehicle = vehicle.clone();
    let mag_cal_report_vehicle = vehicle.clone();
    let firmware_vehicle = vehicle.clone();
    let hardware_vehicle = vehicle.clone();
    let unique_ids_vehicle = vehicle.clone();
    let modes_vehicle = vehicle.clone();
    let support_vehicle = vehicle.clone();

    let mission_closed = tokio::spawn(wait_for_subscription_disconnect(
        mission_vehicle.mission().subscribe(),
    ));
    let params_closed = tokio::spawn(wait_for_subscription_disconnect(
        params_vehicle.params().subscribe(),
    ));
    let fence_closed = tokio::spawn(wait_for_subscription_disconnect(
        fence_vehicle.fence().subscribe(),
    ));
    let rally_closed = tokio::spawn(wait_for_subscription_disconnect(
        rally_vehicle.rally().subscribe(),
    ));
    let mag_cal_progress_closed = tokio::spawn(wait_for_subscription_disconnect(
        mag_cal_progress_vehicle
            .ardupilot()
            .mag_cal_progress()
            .subscribe(),
    ));
    let mag_cal_report_closed = tokio::spawn(wait_for_subscription_disconnect(
        mag_cal_report_vehicle
            .ardupilot()
            .mag_cal_report()
            .subscribe(),
    ));
    let firmware_wait =
        tokio::spawn(async move { firmware_vehicle.info().firmware().wait().await });
    let hardware_wait =
        tokio::spawn(async move { hardware_vehicle.info().hardware().wait().await });
    let unique_ids_wait =
        tokio::spawn(async move { unique_ids_vehicle.info().unique_ids().wait().await });
    let modes_current_closed = tokio::spawn(wait_for_subscription_disconnect(
        modes_vehicle.available_modes().current().subscribe(),
    ));
    let support_command_int_closed = tokio::spawn(wait_for_subscription_disconnect(
        support_vehicle.support().command_int().subscribe(),
    ));

    vehicle
        .disconnect()
        .await
        .expect("disconnect should close all connection-scoped observation subscriptions");

    timeout(Duration::from_millis(250), mission_closed)
        .await
        .expect("mission subscription should close")
        .expect("mission close waiter should join");
    timeout(Duration::from_millis(250), params_closed)
        .await
        .expect("params subscription should close")
        .expect("params close waiter should join");
    timeout(Duration::from_millis(250), fence_closed)
        .await
        .expect("fence subscription should close")
        .expect("fence close waiter should join");
    timeout(Duration::from_millis(250), rally_closed)
        .await
        .expect("rally subscription should close")
        .expect("rally close waiter should join");
    timeout(Duration::from_millis(250), mag_cal_progress_closed)
        .await
        .expect("mag_cal_progress subscription should close")
        .expect("mag_cal_progress close waiter should join");
    timeout(Duration::from_millis(250), mag_cal_report_closed)
        .await
        .expect("mag_cal_report subscription should close")
        .expect("mag_cal_report close waiter should join");
    assert!(matches!(
        timeout(Duration::from_millis(250), firmware_wait)
            .await
            .expect("firmware wait should resolve")
            .expect("firmware wait task should join"),
        Err(VehicleError::Disconnected)
    ));
    assert!(matches!(
        timeout(Duration::from_millis(250), hardware_wait)
            .await
            .expect("hardware wait should resolve")
            .expect("hardware wait task should join"),
        Err(VehicleError::Disconnected)
    ));
    assert!(matches!(
        timeout(Duration::from_millis(250), unique_ids_wait)
            .await
            .expect("unique_ids wait should resolve")
            .expect("unique_ids wait task should join"),
        Err(VehicleError::Disconnected)
    ));
    timeout(Duration::from_millis(250), modes_current_closed)
        .await
        .expect("modes current subscription should close")
        .expect("modes current close waiter should join");
    timeout(Duration::from_millis(250), support_command_int_closed)
        .await
        .expect("support command_int subscription should close")
        .expect("support command_int close waiter should join");

    drop(msg_tx);
}

#[tokio::test]
async fn disconnect_closes_indexed_message_families() {
    let (vehicle, msg_tx) = connect_mock_vehicle().await;

    let battery = vehicle.telemetry().messages().battery_status(2);
    let servo = vehicle.telemetry().messages().servo_output_raw(1);
    let mut battery_sub = battery.subscribe();
    let mut servo_sub = servo.subscribe();

    msg_tx
        .send((
            crate::test_support::default_header(),
            dialect::MavMessage::BATTERY_STATUS(dialect::BATTERY_STATUS_DATA {
                id: 2,
                battery_remaining: 61,
                ..dialect::BATTERY_STATUS_DATA::default()
            }),
        ))
        .await
        .expect("battery status should be delivered");
    msg_tx
        .send((
            crate::test_support::default_header(),
            dialect::MavMessage::SERVO_OUTPUT_RAW(dialect::SERVO_OUTPUT_RAW_DATA {
                port: 1,
                servo1_raw: 1100,
                ..dialect::SERVO_OUTPUT_RAW_DATA::default()
            }),
        ))
        .await
        .expect("servo output raw should be delivered");

    assert_eq!(
        timeout(Duration::from_millis(250), battery_sub.recv())
            .await
            .expect("battery subscription should receive initial sample")
            .expect("battery sample should be present")
            .value
            .battery_remaining,
        61
    );
    assert_eq!(
        timeout(Duration::from_millis(250), servo_sub.recv())
            .await
            .expect("servo subscription should receive initial sample")
            .expect("servo sample should be present")
            .value
            .servo1_raw,
        1100
    );

    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");

    assert!(matches!(
        battery.wait_timeout(Duration::from_millis(50)).await,
        Err(VehicleError::Disconnected)
    ));
    assert!(matches!(
        servo.wait_timeout(Duration::from_millis(50)).await,
        Err(VehicleError::Disconnected)
    ));

    let post_disconnect_battery = vehicle.telemetry().messages().battery_status(7);
    let post_disconnect_servo = vehicle.telemetry().messages().servo_output_raw(3);
    let mut post_disconnect_battery_sub = post_disconnect_battery.subscribe();
    let mut post_disconnect_servo_sub = post_disconnect_servo.subscribe();

    assert!(matches!(
        post_disconnect_battery
            .wait_timeout(Duration::from_millis(50))
            .await,
        Err(VehicleError::Disconnected)
    ));
    assert!(matches!(
        post_disconnect_servo
            .wait_timeout(Duration::from_millis(50))
            .await,
        Err(VehicleError::Disconnected)
    ));

    assert_eq!(
        timeout(Duration::from_millis(250), battery_sub.recv())
            .await
            .expect("battery subscription should close after disconnect"),
        None
    );
    assert_eq!(
        timeout(Duration::from_millis(250), servo_sub.recv())
            .await
            .expect("servo subscription should close after disconnect"),
        None
    );
    assert_eq!(
        timeout(
            Duration::from_millis(250),
            post_disconnect_battery_sub.recv()
        )
        .await
        .expect("new battery subscription should close after disconnect"),
        None
    );
    assert_eq!(
        timeout(Duration::from_millis(250), post_disconnect_servo_sub.recv())
            .await
            .expect("new servo subscription should close after disconnect"),
        None
    );

    drop(msg_tx);
}

#[tokio::test]
async fn disconnect_closes_status_text_with_pending_partial_flush() {
    let (vehicle, msg_tx) = connect_mock_vehicle().await;
    let status_text = vehicle.telemetry().messages().status_text();
    let mut status_text_sub = status_text.subscribe();

    msg_tx
        .send((
            crate::test_support::default_header(),
            dialect::MavMessage::STATUSTEXT(dialect::STATUSTEXT_DATA {
                severity: dialect::MavSeverity::MAV_SEVERITY_WARNING,
                text: exact_50_char_text("01234567890123456789012345678901234567890123456789"),
                id: 77,
                chunk_seq: 0,
            }),
        ))
        .await
        .expect("partial status text should be delivered");

    tokio::task::yield_now().await;

    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");

    assert!(matches!(
        status_text.wait_timeout(Duration::from_millis(50)).await,
        Err(VehicleError::Disconnected)
    ));
    assert_eq!(
        timeout(Duration::from_millis(250), status_text_sub.recv())
            .await
            .expect("status text subscription should close after disconnect"),
        None
    );

    drop(msg_tx);
}

fn exact_50_char_text(text: &str) -> mavlink::types::CharArray<50> {
    assert_eq!(text.len(), 50, "status text chunk must be exactly 50 bytes");
    text.into()
}

async fn wait_for_subscription_disconnect<T: Clone + Send + Sync + 'static>(
    mut sub: crate::observation::ObservationSubscription<T>,
) {
    while sub.recv().await.is_some() {}
}
