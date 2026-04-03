use super::support::*;
use crate::VehicleError;
use crate::dialect;
use crate::geo::{GeoPoint3dMsl, quantize_degrees_e7};
use crate::vehicle::actions::quantize_meters_mm;
use std::time::Duration;

#[tokio::test]
async fn arm_ack_flow() {
    let (vehicle, msg_tx, sent) = connect_mock_vehicle_with_sent().await;

    let arm_task = {
        let vehicle = vehicle.clone();
        tokio::spawn(async move { vehicle.arm().await })
    };

    tokio::time::sleep(Duration::from_millis(10)).await;
    msg_tx
        .send((
            crate::test_support::default_header(),
            ack_msg(
                dialect::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
                dialect::MavResult::MAV_RESULT_ACCEPTED,
            ),
        ))
        .await
        .expect("arm ack should be delivered");

    assert!(arm_task.await.expect("arm task should join").is_ok());

    let arm_sent = sent
        .lock()
        .expect("sent messages lock should not poison")
        .iter()
        .find_map(|(_, msg)| match msg {
            dialect::MavMessage::COMMAND_LONG(data)
                if data.command == dialect::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM =>
            {
                Some(data.clone())
            }
            _ => None,
        })
        .expect("arm command should be sent");

    assert_eq!(arm_sent.param1, 1.0);
    assert_eq!(arm_sent.param2, 0.0);

    let disarm_task = {
        let vehicle = vehicle.clone();
        tokio::spawn(async move { vehicle.disarm().await })
    };

    tokio::time::sleep(Duration::from_millis(10)).await;
    msg_tx
        .send((
            crate::test_support::default_header(),
            ack_msg(
                dialect::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
                dialect::MavResult::MAV_RESULT_DENIED,
            ),
        ))
        .await
        .expect("disarm ack should be delivered");

    assert!(matches!(
        disarm_task.await.expect("disarm task should join"),
        Err(VehicleError::CommandRejected {
            command,
            result: crate::error::CommandResult::Denied,
    }) if command == dialect::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM as u16
    ));

    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
}

#[tokio::test]
async fn set_mode_waits_for_observation() {
    let (vehicle, msg_tx, _) = connect_mock_vehicle_with_sent().await;

    let set_mode_ack_only = {
        let vehicle = vehicle.clone();
        tokio::spawn(async move { vehicle.set_mode_no_wait(4).await })
    };

    tokio::time::sleep(Duration::from_millis(10)).await;
    msg_tx
        .send((
            crate::test_support::default_header(),
            ack_msg(
                dialect::MavCmd::MAV_CMD_DO_SET_MODE,
                dialect::MavResult::MAV_RESULT_ACCEPTED,
            ),
        ))
        .await
        .expect("set mode ack should be delivered");

    assert!(
        set_mode_ack_only
            .await
            .expect("set mode ack-only task should join")
            .is_ok()
    );

    let set_mode_waiting = {
        let vehicle = vehicle.clone();
        tokio::spawn(async move { vehicle.set_mode(6).await })
    };

    tokio::time::sleep(Duration::from_millis(10)).await;
    msg_tx
        .send((
            crate::test_support::default_header(),
            ack_msg(
                dialect::MavCmd::MAV_CMD_DO_SET_MODE,
                dialect::MavResult::MAV_RESULT_ACCEPTED,
            ),
        ))
        .await
        .expect("set mode wait ack should be delivered");

    tokio::time::sleep(Duration::from_millis(40)).await;
    assert!(
        !set_mode_waiting.is_finished(),
        "set_mode(..., true) should wait for observed mode after ACK"
    );

    msg_tx
        .send((
            crate::test_support::default_header(),
            heartbeat_msg(false, 6),
        ))
        .await
        .expect("confirming heartbeat should be delivered");

    assert!(
        set_mode_waiting
            .await
            .expect("set mode wait task should join")
            .is_ok()
    );

    let rejected = {
        let vehicle = vehicle.clone();
        tokio::spawn(async move { vehicle.set_mode_no_wait(5).await })
    };

    tokio::time::sleep(Duration::from_millis(10)).await;
    msg_tx
        .send((
            crate::test_support::default_header(),
            ack_msg(
                dialect::MavCmd::MAV_CMD_DO_SET_MODE,
                dialect::MavResult::MAV_RESULT_DENIED,
            ),
        ))
        .await
        .expect("rejected mode ack should be delivered");

    assert!(matches!(
        rejected.await.expect("rejected mode task should join"),
        Err(VehicleError::CommandRejected {
            command,
            result: crate::error::CommandResult::Denied,
    }) if command == dialect::MavCmd::MAV_CMD_DO_SET_MODE as u16
    ));

    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
}

#[tokio::test]
async fn set_mode_rejects_large_custom_mode() {
    let (vehicle, _rx) = connect_mock_vehicle().await;
    let result = vehicle.set_mode_no_wait(16_777_217).await;
    assert!(matches!(result, Err(VehicleError::InvalidParameter(_))));
}

#[tokio::test]
async fn set_mode_by_name_resolve() {
    let (vehicle, msg_tx, sent) = connect_mock_vehicle_with_sent().await;

    let static_lookup = {
        let vehicle = vehicle.clone();
        tokio::spawn(async move { vehicle.set_mode_by_name_no_wait("guided").await })
    };

    tokio::time::sleep(Duration::from_millis(10)).await;
    msg_tx
        .send((
            crate::test_support::default_header(),
            ack_msg(
                dialect::MavCmd::MAV_CMD_DO_SET_MODE,
                dialect::MavResult::MAV_RESULT_ACCEPTED,
            ),
        ))
        .await
        .expect("static lookup ack should be delivered");

    assert!(
        static_lookup
            .await
            .expect("static lookup task should join")
            .is_ok()
    );

    let static_mode = sent
        .lock()
        .expect("sent messages lock should not poison")
        .iter()
        .rev()
        .find_map(|(_, msg)| match msg {
            dialect::MavMessage::COMMAND_LONG(data)
                if data.command == dialect::MavCmd::MAV_CMD_DO_SET_MODE =>
            {
                Some(data.param2)
            }
            _ => None,
        })
        .expect("static mode command should be sent") as u32;
    assert_eq!(static_mode, 4);

    msg_tx
        .send((
            crate::test_support::default_header(),
            available_modes_msg(1, 1, 27, "AUTO_RTL"),
        ))
        .await
        .expect("available modes update should be delivered");
    tokio::time::sleep(Duration::from_millis(20)).await;

    let dynamic_lookup = {
        let vehicle = vehicle.clone();
        tokio::spawn(async move { vehicle.set_mode_by_name_no_wait("AUTO_RTL").await })
    };

    tokio::time::sleep(Duration::from_millis(10)).await;
    msg_tx
        .send((
            crate::test_support::default_header(),
            ack_msg(
                dialect::MavCmd::MAV_CMD_DO_SET_MODE,
                dialect::MavResult::MAV_RESULT_ACCEPTED,
            ),
        ))
        .await
        .expect("dynamic lookup ack should be delivered");

    assert!(
        dynamic_lookup
            .await
            .expect("dynamic lookup task should join")
            .is_ok()
    );

    let dynamic_mode = sent
        .lock()
        .expect("sent messages lock should not poison")
        .iter()
        .rev()
        .find_map(|(_, msg)| match msg {
            dialect::MavMessage::COMMAND_LONG(data)
                if data.command == dialect::MavCmd::MAV_CMD_DO_SET_MODE =>
            {
                Some(data.param2)
            }
            _ => None,
        })
        .expect("dynamic mode command should be sent") as u32;
    assert_eq!(dynamic_mode, 27);

    assert!(matches!(
        vehicle.set_mode_by_name_no_wait("NOT_A_MODE").await,
        Err(VehicleError::ModeNotAvailable(name)) if name == "NOT_A_MODE"
    ));

    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
}

#[tokio::test]
async fn set_home_ack_flow() {
    let (vehicle, msg_tx, sent) = connect_mock_vehicle_with_sent().await;
    let point = GeoPoint3dMsl {
        latitude_deg: 47.397742,
        longitude_deg: 8.545594,
        altitude_msl_m: 510.5,
    };

    let task = {
        let vehicle = vehicle.clone();
        let point = point.clone();
        tokio::spawn(async move { vehicle.set_home(point).await })
    };

    tokio::time::sleep(Duration::from_millis(10)).await;
    msg_tx
        .send((
            crate::test_support::default_header(),
            ack_msg(
                dialect::MavCmd::MAV_CMD_DO_SET_HOME,
                dialect::MavResult::MAV_RESULT_ACCEPTED,
            ),
        ))
        .await
        .expect("set home ack should be delivered");

    assert!(task.await.expect("set home task should join").is_ok());

    let sent_home = sent
        .lock()
        .expect("sent messages lock should not poison")
        .iter()
        .find_map(|(_, msg)| match msg {
            dialect::MavMessage::COMMAND_INT(data)
                if data.command == dialect::MavCmd::MAV_CMD_DO_SET_HOME =>
            {
                Some(data.clone())
            }
            _ => None,
        })
        .expect("set home command should be sent as COMMAND_INT");

    assert_eq!(sent_home.param1, 0.0);
    assert_eq!(sent_home.x, quantize_degrees_e7(point.latitude_deg));
    assert_eq!(sent_home.y, quantize_degrees_e7(point.longitude_deg));
    assert_eq!(sent_home.z, point.altitude_msl_m as f32);

    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
}

// MAVLink crate deprecated this type/variant, but the wire protocol still requires it.
#[allow(deprecated)]
#[tokio::test]
async fn set_home_current_ack_flow() {
    let (vehicle, msg_tx, sent) = connect_mock_vehicle_with_sent().await;

    let task = {
        let vehicle = vehicle.clone();
        tokio::spawn(async move { vehicle.set_home_current().await })
    };

    tokio::time::sleep(Duration::from_millis(10)).await;
    msg_tx
        .send((
            crate::test_support::default_header(),
            ack_msg(
                dialect::MavCmd::MAV_CMD_DO_SET_HOME,
                dialect::MavResult::MAV_RESULT_ACCEPTED,
            ),
        ))
        .await
        .expect("set home current ack should be delivered");

    assert!(
        task.await
            .expect("set home current task should join")
            .is_ok()
    );

    let sent_home = sent
        .lock()
        .expect("sent messages lock should not poison")
        .iter()
        .find_map(|(_, msg)| match msg {
            dialect::MavMessage::COMMAND_LONG(data)
                if data.command == dialect::MavCmd::MAV_CMD_DO_SET_HOME =>
            {
                Some(data.clone())
            }
            _ => None,
        })
        .expect("set home current command should be sent");

    assert_eq!(sent_home.param1, 1.0);
    assert_eq!(sent_home.param5, 0.0);
    assert_eq!(sent_home.param6, 0.0);
    assert_eq!(sent_home.param7, 0.0);

    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
}

// MAVLink crate deprecated this type/variant, but the wire protocol still requires it.
#[allow(deprecated)]
#[tokio::test]
async fn set_origin_observation() {
    let (vehicle, msg_tx, sent) = connect_mock_vehicle_with_sent().await;
    let requested = GeoPoint3dMsl {
        latitude_deg: 47.39812304,
        longitude_deg: 8.54632104,
        altitude_msl_m: 505.5004,
    };

    msg_tx
        .send((
            crate::test_support::default_header(),
            gps_global_origin_msg(473_981_230, 85_463_210, 505_500),
        ))
        .await
        .expect("stale origin should be delivered");
    tokio::time::sleep(Duration::from_millis(20)).await;

    let task = {
        let vehicle = vehicle.clone();
        let requested = requested.clone();
        tokio::spawn(async move { vehicle.set_origin(requested).await })
    };

    tokio::time::sleep(Duration::from_millis(10)).await;

    let sent_origin = sent
        .lock()
        .expect("sent messages lock should not poison")
        .iter()
        .find_map(|(_, msg)| match msg {
            dialect::MavMessage::SET_GPS_GLOBAL_ORIGIN(data) => Some(data.clone()),
            _ => None,
        })
        .expect("set origin message should be sent");

    assert_eq!(sent_origin.target_system, 1);
    assert_eq!(sent_origin.latitude, 473_981_230);
    assert_eq!(sent_origin.longitude, 85_463_210);
    assert_eq!(sent_origin.altitude, 505_500);

    tokio::time::sleep(Duration::from_millis(40)).await;
    assert!(
        !task.is_finished(),
        "pre-existing matching origin should not confirm success"
    );

    msg_tx
        .send((
            crate::test_support::default_header(),
            gps_global_origin_msg(470_000_000, 80_000_000, 400_000),
        ))
        .await
        .expect("non-matching origin should be delivered");
    tokio::time::sleep(Duration::from_millis(20)).await;
    assert!(
        !task.is_finished(),
        "non-matching origin should be ignored until timeout or match"
    );

    msg_tx
        .send((
            crate::test_support::default_header(),
            gps_global_origin_msg(473_981_230, 85_463_210, 505_500),
        ))
        .await
        .expect("matching origin should be delivered");

    assert!(task.await.expect("set origin task should join").is_ok());

    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
}

#[test]
fn quantize_meters_mm_normal() {
    assert_eq!(quantize_meters_mm(1.5).unwrap(), 1500);
    assert_eq!(quantize_meters_mm(0.0).unwrap(), 0);
    assert_eq!(quantize_meters_mm(-100.001).unwrap(), -100_001);
}

#[test]
fn quantize_meters_mm_rejects_non_finite() {
    assert!(quantize_meters_mm(f64::NAN).is_err());
    assert!(quantize_meters_mm(f64::INFINITY).is_err());
    assert!(quantize_meters_mm(f64::NEG_INFINITY).is_err());
}

#[test]
fn quantize_meters_mm_rejects_overflow() {
    assert!(quantize_meters_mm(2_147_484.0).is_err());
    assert!(quantize_meters_mm(-2_147_484.0).is_err());
}
