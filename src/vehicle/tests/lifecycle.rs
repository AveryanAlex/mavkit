use super::support::*;
#[cfg(feature = "stream")]
use crate::dialect;
use crate::{AutopilotType, Vehicle, VehicleError, VehicleIdentity, VehicleType};
#[cfg(feature = "stream")]
use mavlink::async_peek_reader::AsyncPeekReader;
#[cfg(feature = "stream")]
use mavlink::{MavlinkVersion, ReadVersion, read_versioned_msg_async, write_versioned_msg_async};
use std::time::Duration;
#[cfg(feature = "stream")]
use tokio::io::{duplex, split};
use tokio::sync::mpsc;
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
