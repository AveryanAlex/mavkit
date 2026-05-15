use super::calibration::{
    MAV_CMD_DO_ACCEPT_MAG_CAL_ID, MAV_CMD_DO_CANCEL_MAG_CAL_ID, MAV_CMD_DO_START_MAG_CAL_ID,
};
use super::handle::{SET_SERVO_INSTANCE_RANGE, SET_SERVO_PWM_RANGE_US};
use super::*;
use crate::dialect::{self, MavModeFlag};
use crate::error::VehicleError;
use crate::state::{self, create_channels};
use crate::test_support::{
    ConnectedVehicleHarness, ConnectedVehicleOptions, SentMessages, command_ack, default_header,
    fast_vehicle_test_config, heartbeat_for_type,
};
use crate::vehicle::Vehicle;
use mavlink::MavHeader;
use std::time::Duration;
use tokio::sync::mpsc;
use tokio::time::timeout;

fn heartbeat_msg_with_mode(mavtype: dialect::MavType, custom_mode: u32) -> dialect::MavMessage {
    heartbeat_for_type(
        mavtype,
        custom_mode,
        MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    )
}

fn ack_msg(command: dialect::MavCmd, result: dialect::MavResult) -> dialect::MavMessage {
    command_ack(command, result)
}

fn command_id_to_mav_cmd(command_id: u16) -> dialect::MavCmd {
    num_traits::FromPrimitive::from_u16(command_id)
        .expect("command id should be representable by mavkit::dialect::MavCmd")
}

fn ack_msg_by_id(command_id: u16, result: dialect::MavResult) -> dialect::MavMessage {
    ack_msg(command_id_to_mav_cmd(command_id), result)
}

fn fast_config() -> crate::config::VehicleConfig {
    fast_vehicle_test_config()
}

async fn connect_mock_vehicle_with_sent() -> (
    Vehicle,
    mpsc::Sender<(MavHeader, dialect::MavMessage)>,
    SentMessages,
) {
    connect_mock_vehicle_with_sent_for_type_and_mode(dialect::MavType::MAV_TYPE_QUADROTOR, 7).await
}

async fn connect_mock_vehicle_with_sent_for_type(
    mavtype: dialect::MavType,
) -> (
    Vehicle,
    mpsc::Sender<(MavHeader, dialect::MavMessage)>,
    SentMessages,
) {
    connect_mock_vehicle_with_sent_for_type_and_mode(mavtype, 7).await
}

async fn connect_mock_vehicle_with_sent_for_type_and_mode(
    mavtype: dialect::MavType,
    custom_mode: u32,
) -> (
    Vehicle,
    mpsc::Sender<(MavHeader, dialect::MavMessage)>,
    SentMessages,
) {
    let harness = ConnectedVehicleHarness::connect(ConnectedVehicleOptions {
        config: fast_config(),
        heartbeat_message: heartbeat_msg_with_mode(mavtype, custom_mode),
        join_timeout: Duration::from_millis(250),
        ..ConnectedVehicleOptions::default()
    })
    .await;

    (harness.vehicle, harness.msg_tx, harness.sent)
}

fn sent_mode_change_params(sent: &SentMessages) -> Vec<u32> {
    sent.lock()
        .expect("sent messages lock should not poison")
        .iter()
        .filter_map(|(_, msg)| match msg {
            dialect::MavMessage::COMMAND_LONG(data)
                if data.command == dialect::MavCmd::MAV_CMD_DO_SET_MODE =>
            {
                Some(data.param2 as u32)
            }
            _ => None,
        })
        .collect()
}

#[tokio::test]
async fn class_handle_availability() {
    for mavtype in [
        dialect::MavType::MAV_TYPE_QUADROTOR,
        dialect::MavType::MAV_TYPE_TRICOPTER,
        dialect::MavType::MAV_TYPE_HELICOPTER,
    ] {
        let (vehicle, _msg_tx, _sent) = connect_mock_vehicle_with_sent_for_type(mavtype).await;
        let ardupilot = vehicle.ardupilot();

        assert!(
            ardupilot.copter().is_some(),
            "{mavtype:?} should expose copter()"
        );
        assert!(
            ardupilot.plane().is_none(),
            "{mavtype:?} should not expose plane()"
        );
        assert!(
            ardupilot.rover().is_none(),
            "{mavtype:?} should not expose rover()"
        );
        assert!(
            ardupilot.sub().is_none(),
            "{mavtype:?} should not expose sub()"
        );

        vehicle
            .disconnect()
            .await
            .expect("disconnect should succeed");
    }

    let (plane, _msg_tx, _sent) =
        connect_mock_vehicle_with_sent_for_type(dialect::MavType::MAV_TYPE_FIXED_WING).await;
    {
        let plane = plane
            .ardupilot()
            .plane()
            .expect("fixed wing heartbeat should expose plane()");
        assert!(
            plane.vtol().is_none(),
            "plain fixed wing should not expose plane().vtol()"
        );
    }
    plane.disconnect().await.expect("disconnect should succeed");

    let (quadplane, _msg_tx, _sent) =
        connect_mock_vehicle_with_sent_for_type(dialect::MavType::MAV_TYPE_VTOL_FIXEDROTOR).await;
    {
        let quadplane = quadplane
            .ardupilot()
            .plane()
            .expect("vtol heartbeat should still expose plane()");
        assert!(
            quadplane.vtol().is_some(),
            "vtol heartbeat should expose plane().vtol() as a plane extension"
        );
    }
    quadplane
        .disconnect()
        .await
        .expect("disconnect should succeed");

    let (rover, _msg_tx, _sent) =
        connect_mock_vehicle_with_sent_for_type(dialect::MavType::MAV_TYPE_GROUND_ROVER).await;
    let ardupilot = rover.ardupilot();
    assert!(
        ardupilot.rover().is_some(),
        "ground rover heartbeat should expose rover()"
    );
    assert!(
        ardupilot.copter().is_none(),
        "ground rover heartbeat should not expose copter()"
    );
    assert!(
        ardupilot.plane().is_none(),
        "ground rover heartbeat should not expose plane()"
    );
    assert!(
        ardupilot.sub().is_none(),
        "ground rover heartbeat should not expose sub()"
    );
    rover.disconnect().await.expect("disconnect should succeed");

    let (sub, _msg_tx, _sent) =
        connect_mock_vehicle_with_sent_for_type(dialect::MavType::MAV_TYPE_SUBMARINE).await;
    let ardupilot = sub.ardupilot();
    assert!(
        ardupilot.sub().is_some(),
        "submarine heartbeat should expose sub()"
    );
    assert!(
        ardupilot.copter().is_none(),
        "submarine heartbeat should not expose copter()"
    );
    assert!(
        ardupilot.plane().is_none(),
        "submarine heartbeat should not expose plane()"
    );
    assert!(
        ardupilot.rover().is_none(),
        "submarine heartbeat should not expose rover()"
    );
    sub.disconnect().await.expect("disconnect should succeed");
}

#[tokio::test]
async fn guided_exclusivity() {
    let (vehicle, _msg_tx, _sent) =
        connect_mock_vehicle_with_sent_for_type_and_mode(dialect::MavType::MAV_TYPE_QUADROTOR, 4)
            .await;

    let session = vehicle
        .ardupilot()
        .guided()
        .await
        .expect("first guided session should acquire the local lease");
    assert_eq!(session.kind(), ArduGuidedKind::Copter);
    assert!(matches!(session.specific(), GuidedSpecific::Copter(_)));

    let conflict = vehicle.ardupilot().guided().await;
    assert!(matches!(
        conflict,
        Err(VehicleError::OperationConflict {
            conflicting_domain,
            conflicting_op,
        }) if conflicting_domain == "ardupilot_guided" && conflicting_op == "session_active"
    ));

    session
        .close()
        .await
        .expect("explicit close should release the local lease");

    let reacquired = vehicle
        .ardupilot()
        .guided()
        .await
        .expect("guided lease should be acquirable again after close");
    reacquired
        .close()
        .await
        .expect("reacquired guided session should close cleanly");

    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
}

#[tokio::test]
async fn guided_mode_asserting_acquisition() {
    let (vehicle, msg_tx, sent) = connect_mock_vehicle_with_sent().await;

    let guided_task = {
        let vehicle = vehicle.clone();
        tokio::spawn(async move { vehicle.ardupilot().guided().await })
    };

    tokio::time::sleep(Duration::from_millis(10)).await;
    assert_eq!(
        sent_mode_change_params(&sent),
        vec![4],
        "guided acquisition should request GUIDED mode when current mode is not guided"
    );

    msg_tx
        .send((
            default_header(),
            ack_msg(
                dialect::MavCmd::MAV_CMD_DO_SET_MODE,
                dialect::MavResult::MAV_RESULT_ACCEPTED,
            ),
        ))
        .await
        .expect("guided mode change ack should be delivered");

    tokio::time::sleep(Duration::from_millis(40)).await;
    assert!(
        !guided_task.is_finished(),
        "guided acquisition should wait for observed current mode after ACK"
    );

    msg_tx
        .send((
            default_header(),
            heartbeat_msg_with_mode(dialect::MavType::MAV_TYPE_QUADROTOR, 4),
        ))
        .await
        .expect("guiding heartbeat should be delivered");

    let session = guided_task
        .await
        .expect("guided task should join")
        .expect("guided acquisition should complete after mode observation");
    assert!(matches!(session.specific(), GuidedSpecific::Copter(_)));

    session
        .close()
        .await
        .expect("guided session should close cleanly");
    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
}

#[tokio::test]
async fn guided_session_terminal_on_mode_loss() {
    let (vehicle, msg_tx, _sent) =
        connect_mock_vehicle_with_sent_for_type_and_mode(dialect::MavType::MAV_TYPE_QUADROTOR, 4)
            .await;

    let session = vehicle
        .ardupilot()
        .guided()
        .await
        .expect("guided session should acquire while already in guided mode");

    msg_tx
        .send((
            default_header(),
            heartbeat_msg_with_mode(dialect::MavType::MAV_TYPE_QUADROTOR, 6),
        ))
        .await
        .expect("mode-loss heartbeat should be delivered");

    timeout(Duration::from_millis(250), async {
        loop {
            if session.terminal_reason_for_test().is_some() {
                break;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    })
    .await
    .expect("guided session should become terminal after leaving guided mode");

    assert_eq!(session.terminal_reason_for_test(), Some("guided_mode_lost"));

    session
        .close()
        .await
        .expect("terminal guided session close should still release local lease");
    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
}

#[tokio::test]
async fn guided_close_releases_lease_without_mode_restore() {
    let (vehicle, _msg_tx, sent) =
        connect_mock_vehicle_with_sent_for_type_and_mode(dialect::MavType::MAV_TYPE_QUADROTOR, 4)
            .await;

    let session = vehicle
        .ardupilot()
        .guided()
        .await
        .expect("guided session should acquire");

    assert!(matches!(
        vehicle.ardupilot().guided().await,
        Err(VehicleError::OperationConflict {
            conflicting_domain,
            conflicting_op,
        }) if conflicting_domain == "ardupilot_guided" && conflicting_op == "session_active"
    ));

    session
        .close()
        .await
        .expect("close should release only local lease state");

    let reacquired = vehicle
        .ardupilot()
        .guided()
        .await
        .expect("lease should be free after close");
    reacquired
        .close()
        .await
        .expect("reacquired guided session should close cleanly");

    assert!(
        sent_mode_change_params(&sent).is_empty(),
        "close must not restore prior mode or issue extra mode commands"
    );

    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
}

#[tokio::test]
async fn copter_goto_command() {
    let (vehicle, _msg_tx, sent) =
        connect_mock_vehicle_with_sent_for_type_and_mode(dialect::MavType::MAV_TYPE_QUADROTOR, 4)
            .await;

    let session = vehicle
        .ardupilot()
        .guided()
        .await
        .expect("guided session should acquire while already in guided mode");
    let GuidedSpecific::Copter(copter) = session.specific() else {
        panic!("quadrotor guided session should expose copter handle");
    };

    copter
        .goto(crate::GeoPoint3dRelHome {
            latitude_deg: 47.397742,
            longitude_deg: 8.545594,
            relative_alt_m: 32.5,
        })
        .await
        .expect("guided goto should send set-position-target message");

    let goto_msg = sent
        .lock()
        .expect("sent messages lock should not poison")
        .iter()
        .find_map(|(_, msg)| match msg {
            dialect::MavMessage::SET_POSITION_TARGET_GLOBAL_INT(data) => Some(data.clone()),
            _ => None,
        })
        .expect("guided goto should send set_position_target_global_int");

    assert_eq!(
        goto_msg.coordinate_frame,
        dialect::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT
    );
    assert_eq!(goto_msg.lat_int, 473_977_420);
    assert_eq!(goto_msg.lon_int, 85_455_940);
    assert_eq!(goto_msg.alt, 32.5);

    session
        .close()
        .await
        .expect("guided session should close cleanly");
    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
}

#[tokio::test]
async fn plane_guided_vtol_extension_availability() {
    let (plane, _msg_tx, _sent) =
        connect_mock_vehicle_with_sent_for_type_and_mode(dialect::MavType::MAV_TYPE_FIXED_WING, 15)
            .await;
    let plane_session = plane
        .ardupilot()
        .guided()
        .await
        .expect("fixed-wing guided session should acquire");
    let GuidedSpecific::Plane(fixed_wing) = plane_session.specific() else {
        panic!("fixed-wing guided session should expose plane handle");
    };
    assert_eq!(fixed_wing.kind(), ArduPlaneKind::FixedWing);
    assert!(fixed_wing.vtol().is_none());
    plane_session
        .close()
        .await
        .expect("fixed-wing guided session should close cleanly");
    plane.disconnect().await.expect("disconnect should succeed");

    let (quadplane, _msg_tx, _sent) = connect_mock_vehicle_with_sent_for_type_and_mode(
        dialect::MavType::MAV_TYPE_VTOL_FIXEDROTOR,
        15,
    )
    .await;
    let quadplane_session = quadplane
        .ardupilot()
        .guided()
        .await
        .expect("vtol plane guided session should acquire");
    {
        let GuidedSpecific::Plane(plane) = quadplane_session.specific() else {
            panic!("vtol plane guided session should expose plane handle");
        };
        assert_eq!(plane.kind(), ArduPlaneKind::Vtol);
        assert!(plane.vtol().is_some());
    }
    quadplane_session
        .close()
        .await
        .expect("vtol plane guided session should close cleanly");
    quadplane
        .disconnect()
        .await
        .expect("disconnect should succeed");
}

#[tokio::test]
async fn plane_reposition_rel_home_command_encoding() {
    let (vehicle, msg_tx, sent) =
        connect_mock_vehicle_with_sent_for_type_and_mode(dialect::MavType::MAV_TYPE_FIXED_WING, 15)
            .await;

    let session = vehicle
        .ardupilot()
        .guided()
        .await
        .expect("plane guided session should acquire");
    let task_session = session.clone();
    let reposition_task = tokio::spawn(async move {
        let GuidedSpecific::Plane(plane) = task_session.specific() else {
            panic!("plane guided session should expose plane handle");
        };
        plane
            .reposition_rel_home(crate::GeoPoint3dRelHome {
                latitude_deg: 47.397742,
                longitude_deg: 8.545594,
                relative_alt_m: 42.5,
            })
            .await
    });

    tokio::time::sleep(Duration::from_millis(10)).await;
    msg_tx
        .send((
            default_header(),
            ack_msg(
                dialect::MavCmd::MAV_CMD_DO_REPOSITION,
                dialect::MavResult::MAV_RESULT_ACCEPTED,
            ),
        ))
        .await
        .expect("plane reposition ack should be delivered");

    assert!(
        reposition_task
            .await
            .expect("plane reposition task should join")
            .is_ok()
    );

    let reposition = sent
        .lock()
        .expect("sent messages lock should not poison")
        .iter()
        .find_map(|(_, msg)| match msg {
            dialect::MavMessage::COMMAND_INT(data)
                if data.command == dialect::MavCmd::MAV_CMD_DO_REPOSITION =>
            {
                Some(data.clone())
            }
            _ => None,
        })
        .expect("plane reposition should send command_int");

    assert_eq!(
        reposition.frame,
        dialect::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT
    );
    assert_eq!(reposition.x, 473_977_420);
    assert_eq!(reposition.y, 85_455_940);
    assert_eq!(reposition.z, 42.5);
    assert_eq!(reposition.param1, 0.0);
    assert_eq!(reposition.param2, 1.0);
    assert_eq!(reposition.param3, 0.0);
    assert_eq!(reposition.param4, 0.0);

    session
        .close()
        .await
        .expect("guided session should close cleanly");
    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
}

#[tokio::test]
async fn vtol_takeoff_command_encoding() {
    let (vehicle, msg_tx, sent) = connect_mock_vehicle_with_sent_for_type_and_mode(
        dialect::MavType::MAV_TYPE_VTOL_FIXEDROTOR,
        15,
    )
    .await;

    let session = vehicle
        .ardupilot()
        .guided()
        .await
        .expect("vtol plane guided session should acquire");
    let task_session = session.clone();
    let takeoff_task = tokio::spawn(async move {
        let GuidedSpecific::Plane(plane) = task_session.specific() else {
            panic!("vtol plane guided session should expose plane handle");
        };
        plane
            .vtol()
            .expect("vtol plane should expose vtol guided extension")
            .takeoff(RelativeClimbTarget {
                relative_climb_m: 12.5,
            })
            .await
    });

    tokio::time::sleep(Duration::from_millis(10)).await;
    msg_tx
        .send((
            default_header(),
            ack_msg(
                dialect::MavCmd::MAV_CMD_NAV_TAKEOFF,
                dialect::MavResult::MAV_RESULT_ACCEPTED,
            ),
        ))
        .await
        .expect("vtol takeoff ack should be delivered");

    assert!(
        takeoff_task
            .await
            .expect("vtol takeoff task should join")
            .is_ok()
    );

    let takeoff = sent
        .lock()
        .expect("sent messages lock should not poison")
        .iter()
        .find_map(|(_, msg)| match msg {
            dialect::MavMessage::COMMAND_INT(data)
                if data.command == dialect::MavCmd::MAV_CMD_NAV_TAKEOFF =>
            {
                Some(data.clone())
            }
            _ => None,
        })
        .expect("vtol takeoff should send command_int");

    assert_eq!(takeoff.frame, dialect::MavFrame::MAV_FRAME_LOCAL_OFFSET_NED);
    assert_eq!(takeoff.x, 0);
    assert_eq!(takeoff.y, 0);
    assert_eq!(takeoff.z, -12.5);

    session
        .close()
        .await
        .expect("guided session should close cleanly");
    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
}

#[tokio::test]
async fn rover_drive_to() {
    let (vehicle, msg_tx, sent) = connect_mock_vehicle_with_sent_for_type_and_mode(
        dialect::MavType::MAV_TYPE_GROUND_ROVER,
        15,
    )
    .await;

    let session = vehicle
        .ardupilot()
        .guided()
        .await
        .expect("rover guided session should acquire");
    let task_session = session.clone();
    let drive_task = tokio::spawn(async move {
        let GuidedSpecific::Rover(rover) = task_session.specific() else {
            panic!("rover guided session should expose rover handle");
        };
        rover
            .drive_to(crate::GeoPoint2d {
                latitude_deg: 47.397742,
                longitude_deg: 8.545594,
            })
            .await
    });

    tokio::time::sleep(Duration::from_millis(10)).await;
    msg_tx
        .send((
            default_header(),
            ack_msg(
                dialect::MavCmd::MAV_CMD_DO_REPOSITION,
                dialect::MavResult::MAV_RESULT_ACCEPTED,
            ),
        ))
        .await
        .expect("rover reposition ack should be delivered");

    assert!(
        drive_task
            .await
            .expect("rover drive_to task should join")
            .is_ok()
    );

    let reposition = sent
        .lock()
        .expect("sent messages lock should not poison")
        .iter()
        .find_map(|(_, msg)| match msg {
            dialect::MavMessage::COMMAND_INT(data)
                if data.command == dialect::MavCmd::MAV_CMD_DO_REPOSITION =>
            {
                Some(data.clone())
            }
            _ => None,
        })
        .expect("rover drive_to should send command_int");

    assert_eq!(reposition.frame, dialect::MavFrame::MAV_FRAME_GLOBAL);
    assert_eq!(reposition.x, 473_977_420);
    assert_eq!(reposition.y, 85_455_940);
    assert_eq!(reposition.z, 0.0);
    assert_eq!(reposition.param2, 0.0);

    session
        .close()
        .await
        .expect("guided session should close cleanly");
    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
}

#[tokio::test]
async fn sub_goto_depth_command_encoding() {
    let (vehicle, msg_tx, sent) =
        connect_mock_vehicle_with_sent_for_type_and_mode(dialect::MavType::MAV_TYPE_SUBMARINE, 4)
            .await;

    let session = vehicle
        .ardupilot()
        .guided()
        .await
        .expect("sub guided session should acquire");
    let task_session = session.clone();
    let goto_task = tokio::spawn(async move {
        let GuidedSpecific::Sub(sub) = task_session.specific() else {
            panic!("sub guided session should expose sub handle");
        };
        sub.goto_depth(SubGotoDepthTarget {
            point: crate::GeoPoint2d {
                latitude_deg: 47.397742,
                longitude_deg: 8.545594,
            },
            depth_m: 12.5,
        })
        .await
    });

    tokio::time::sleep(Duration::from_millis(10)).await;
    msg_tx
        .send((
            default_header(),
            ack_msg(
                dialect::MavCmd::MAV_CMD_DO_REPOSITION,
                dialect::MavResult::MAV_RESULT_ACCEPTED,
            ),
        ))
        .await
        .expect("sub reposition ack should be delivered");

    assert!(
        goto_task
            .await
            .expect("sub goto_depth task should join")
            .is_ok()
    );

    let reposition = sent
        .lock()
        .expect("sent messages lock should not poison")
        .iter()
        .find_map(|(_, msg)| match msg {
            dialect::MavMessage::COMMAND_INT(data)
                if data.command == dialect::MavCmd::MAV_CMD_DO_REPOSITION =>
            {
                Some(data.clone())
            }
            _ => None,
        })
        .expect("sub goto_depth should send command_int");

    assert_eq!(
        reposition.frame,
        dialect::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT
    );
    assert_eq!(reposition.x, 473_977_420);
    assert_eq!(reposition.y, 85_455_940);
    assert_eq!(reposition.z, -12.5);
    assert_eq!(reposition.param2, 0.0);

    session
        .close()
        .await
        .expect("guided session should close cleanly");
    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
}

#[tokio::test]
async fn rover_guided_actions_fail_after_close() {
    let (vehicle, _msg_tx, _sent) = connect_mock_vehicle_with_sent_for_type_and_mode(
        dialect::MavType::MAV_TYPE_GROUND_ROVER,
        15,
    )
    .await;

    let session = vehicle
        .ardupilot()
        .guided()
        .await
        .expect("rover guided session should acquire");
    let closer = session.clone();
    let GuidedSpecific::Rover(rover) = session.specific() else {
        panic!("rover guided session should expose rover handle");
    };

    closer
        .close()
        .await
        .expect("closing a cloned guided session should release the shared lease");

    assert!(matches!(
        rover
            .drive_to(crate::GeoPoint2d {
                latitude_deg: 47.397742,
                longitude_deg: 8.545594,
            })
            .await,
        Err(VehicleError::OperationConflict {
            conflicting_domain,
            conflicting_op,
        }) if conflicting_domain == "ardupilot_guided" && conflicting_op == "session_closed"
    ));

    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
}

#[tokio::test]
async fn sub_guided_actions_fail_after_terminal_mode_loss() {
    let (vehicle, msg_tx, _sent) =
        connect_mock_vehicle_with_sent_for_type_and_mode(dialect::MavType::MAV_TYPE_SUBMARINE, 4)
            .await;

    let session = vehicle
        .ardupilot()
        .guided()
        .await
        .expect("sub guided session should acquire");
    let GuidedSpecific::Sub(sub) = session.specific() else {
        panic!("sub guided session should expose sub handle");
    };

    msg_tx
        .send((
            default_header(),
            heartbeat_msg_with_mode(dialect::MavType::MAV_TYPE_SUBMARINE, 2),
        ))
        .await
        .expect("mode-loss heartbeat should be delivered");

    timeout(Duration::from_millis(250), async {
        loop {
            if session.terminal_reason_for_test().is_some() {
                break;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    })
    .await
    .expect("guided session should become terminal after mode loss");

    assert!(matches!(
        sub.goto_depth(SubGotoDepthTarget {
            point: crate::GeoPoint2d {
                latitude_deg: 47.397742,
                longitude_deg: 8.545594,
            },
            depth_m: 8.0,
        })
        .await,
        Err(VehicleError::OperationConflict {
            conflicting_domain,
            conflicting_op,
        }) if conflicting_domain == "ardupilot_guided" && conflicting_op == "session_terminal"
    ));

    session
        .close()
        .await
        .expect("terminal guided session should still close cleanly");
    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
}

#[tokio::test]
async fn guided_actions_fail_after_close() {
    let (vehicle, _msg_tx, _sent) =
        connect_mock_vehicle_with_sent_for_type_and_mode(dialect::MavType::MAV_TYPE_QUADROTOR, 4)
            .await;

    let session = vehicle
        .ardupilot()
        .guided()
        .await
        .expect("guided session should acquire");
    let closer = session.clone();
    let GuidedSpecific::Copter(copter) = session.specific() else {
        panic!("quadrotor guided session should expose copter handle");
    };

    closer
        .close()
        .await
        .expect("closing a cloned guided session should release the shared lease");

    assert!(matches!(
        copter
            .goto(crate::GeoPoint3dRelHome {
                latitude_deg: 47.397742,
                longitude_deg: 8.545594,
                relative_alt_m: 20.0,
            })
            .await,
        Err(VehicleError::OperationConflict {
            conflicting_domain,
            conflicting_op,
        }) if conflicting_domain == "ardupilot_guided" && conflicting_op == "session_closed"
    ));

    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
}

#[tokio::test]
async fn guided_actions_fail_after_terminal_mode_loss() {
    let (vehicle, msg_tx, _sent) =
        connect_mock_vehicle_with_sent_for_type_and_mode(dialect::MavType::MAV_TYPE_QUADROTOR, 4)
            .await;

    let session = vehicle
        .ardupilot()
        .guided()
        .await
        .expect("guided session should acquire");
    let GuidedSpecific::Copter(copter) = session.specific() else {
        panic!("quadrotor guided session should expose copter handle");
    };

    msg_tx
        .send((
            default_header(),
            heartbeat_msg_with_mode(dialect::MavType::MAV_TYPE_QUADROTOR, 6),
        ))
        .await
        .expect("mode-loss heartbeat should be delivered");

    timeout(Duration::from_millis(250), async {
        loop {
            if session.terminal_reason_for_test().is_some() {
                break;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    })
    .await
    .expect("guided session should become terminal after mode loss");

    assert!(matches!(
        copter
            .goto(crate::GeoPoint3dRelHome {
                latitude_deg: 47.397742,
                longitude_deg: 8.545594,
                relative_alt_m: 20.0,
            })
            .await,
        Err(VehicleError::OperationConflict {
            conflicting_domain,
            conflicting_op,
        }) if conflicting_domain == "ardupilot_guided" && conflicting_op == "session_terminal"
    ));

    session
        .close()
        .await
        .expect("terminal guided session should still close cleanly");
    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
}

#[tokio::test]
async fn ardupilot_motor_test_rejects_invalid_ranges() {
    let (vehicle, _msg_tx, _sent) = connect_mock_vehicle_with_sent().await;

    assert!(matches!(
        vehicle.ardupilot().motor_test(0, 10.0, 5).await,
        Err(VehicleError::InvalidParameter(_))
    ));
    assert!(matches!(
        vehicle.ardupilot().motor_test(13, 10.0, 5).await,
        Err(VehicleError::InvalidParameter(_))
    ));
    assert!(matches!(
        vehicle.ardupilot().motor_test(1, -0.1, 5).await,
        Err(VehicleError::InvalidParameter(_))
    ));
    assert!(matches!(
        vehicle.ardupilot().motor_test(1, 100.1, 5).await,
        Err(VehicleError::InvalidParameter(_))
    ));
    assert!(matches!(
        vehicle.ardupilot().motor_test(1, 10.0, 0).await,
        Err(VehicleError::InvalidParameter(_))
    ));
    assert!(matches!(
        vehicle.ardupilot().motor_test(1, 10.0, 31).await,
        Err(VehicleError::InvalidParameter(_))
    ));

    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
}

#[tokio::test]
async fn ardupilot_motor_test_sends_expected_command_long_shape() {
    let (vehicle, msg_tx, sent) = connect_mock_vehicle_with_sent().await;

    let motor_test_task = {
        let vehicle = vehicle.clone();
        tokio::spawn(async move { vehicle.ardupilot().motor_test(3, 42.5, 9).await })
    };

    tokio::time::sleep(Duration::from_millis(10)).await;
    msg_tx
        .send((
            default_header(),
            ack_msg(
                dialect::MavCmd::MAV_CMD_DO_MOTOR_TEST,
                dialect::MavResult::MAV_RESULT_ACCEPTED,
            ),
        ))
        .await
        .expect("motor test ack should be delivered");

    assert!(
        motor_test_task
            .await
            .expect("motor test task should join")
            .is_ok()
    );

    let motor_test = sent
        .lock()
        .expect("sent messages lock should not poison")
        .iter()
        .find_map(|(_, msg)| match msg {
            dialect::MavMessage::COMMAND_LONG(data)
                if data.command == dialect::MavCmd::MAV_CMD_DO_MOTOR_TEST =>
            {
                Some(data.clone())
            }
            _ => None,
        })
        .expect("motor test command should be sent");

    assert_eq!(motor_test.param1, 3.0);
    assert_eq!(motor_test.param2, 0.0);
    assert!((motor_test.param3 - 42.5).abs() < f32::EPSILON);
    assert_eq!(motor_test.param4, 9.0);
    assert_eq!(motor_test.param5, 1.0);
    assert_eq!(motor_test.param6, 0.0);
    assert_eq!(motor_test.param7, 0.0);

    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
}

#[tokio::test]
async fn ardupilot_set_servo_rejects_invalid_ranges() {
    let (vehicle, _msg_tx, sent) = connect_mock_vehicle_with_sent().await;

    let sent_before = sent
        .lock()
        .expect("sent messages lock should not poison")
        .len();

    assert!(matches!(
        vehicle
            .ardupilot()
            .set_servo(
                *SET_SERVO_INSTANCE_RANGE.start() - 1,
                *SET_SERVO_PWM_RANGE_US.start()
            )
            .await,
        Err(VehicleError::InvalidParameter(_))
    ));
    assert!(matches!(
        vehicle
            .ardupilot()
            .set_servo(
                *SET_SERVO_INSTANCE_RANGE.end() + 1,
                *SET_SERVO_PWM_RANGE_US.start()
            )
            .await,
        Err(VehicleError::InvalidParameter(_))
    ));
    assert!(matches!(
        vehicle
            .ardupilot()
            .set_servo(
                *SET_SERVO_INSTANCE_RANGE.start(),
                *SET_SERVO_PWM_RANGE_US.start() - 1
            )
            .await,
        Err(VehicleError::InvalidParameter(_))
    ));
    assert!(matches!(
        vehicle
            .ardupilot()
            .set_servo(
                *SET_SERVO_INSTANCE_RANGE.start(),
                *SET_SERVO_PWM_RANGE_US.end() + 1
            )
            .await,
        Err(VehicleError::InvalidParameter(_))
    ));
    assert_eq!(
        sent.lock()
            .expect("sent messages lock should not poison")
            .len(),
        sent_before,
        "invalid set_servo calls must reject before any new COMMAND_LONG is sent"
    );

    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
}

#[tokio::test]
async fn ardupilot_set_servo_sends_expected_command_long_shape() {
    let (vehicle, msg_tx, sent) = connect_mock_vehicle_with_sent().await;

    let first_boundary = {
        let vehicle = vehicle.clone();
        tokio::spawn(async move {
            vehicle
                .ardupilot()
                .set_servo(
                    *SET_SERVO_INSTANCE_RANGE.start(),
                    *SET_SERVO_PWM_RANGE_US.start(),
                )
                .await
        })
    };

    tokio::time::sleep(Duration::from_millis(10)).await;
    msg_tx
        .send((
            default_header(),
            ack_msg(
                dialect::MavCmd::MAV_CMD_DO_SET_SERVO,
                dialect::MavResult::MAV_RESULT_ACCEPTED,
            ),
        ))
        .await
        .expect("first set_servo ack should be delivered");

    assert!(
        first_boundary
            .await
            .expect("first set_servo task should join")
            .is_ok()
    );

    tokio::time::sleep(Duration::from_millis(20)).await;

    let last_boundary = {
        let vehicle = vehicle.clone();
        tokio::spawn(async move {
            vehicle
                .ardupilot()
                .set_servo(
                    *SET_SERVO_INSTANCE_RANGE.end(),
                    *SET_SERVO_PWM_RANGE_US.end(),
                )
                .await
        })
    };

    tokio::time::sleep(Duration::from_millis(10)).await;
    msg_tx
        .send((
            default_header(),
            ack_msg(
                dialect::MavCmd::MAV_CMD_DO_SET_SERVO,
                dialect::MavResult::MAV_RESULT_ACCEPTED,
            ),
        ))
        .await
        .expect("second set_servo ack should be delivered");

    assert!(
        last_boundary
            .await
            .expect("second set_servo task should join")
            .is_ok()
    );

    let commands: Vec<_> = {
        let sent = sent.lock().expect("sent messages lock should not poison");
        sent.iter()
            .filter_map(|(_, msg)| match msg {
                dialect::MavMessage::COMMAND_LONG(data)
                    if data.command == dialect::MavCmd::MAV_CMD_DO_SET_SERVO =>
                {
                    Some(data.clone())
                }
                _ => None,
            })
            .collect()
    };
    assert_eq!(
        commands.len(),
        2,
        "set_servo should send one command per request"
    );

    let first = &commands[0];
    assert_eq!(first.param1, f32::from(*SET_SERVO_INSTANCE_RANGE.start()));
    assert_eq!(first.param2, f32::from(*SET_SERVO_PWM_RANGE_US.start()));
    assert_eq!(first.param3, 0.0);
    assert_eq!(first.param4, 0.0);
    assert_eq!(first.param5, 0.0);
    assert_eq!(first.param6, 0.0);
    assert_eq!(first.param7, 0.0);

    let last = &commands[1];
    assert_eq!(last.param1, f32::from(*SET_SERVO_INSTANCE_RANGE.end()));
    assert_eq!(last.param2, f32::from(*SET_SERVO_PWM_RANGE_US.end()));
    assert_eq!(last.param3, 0.0);
    assert_eq!(last.param4, 0.0);
    assert_eq!(last.param5, 0.0);
    assert_eq!(last.param6, 0.0);
    assert_eq!(last.param7, 0.0);
    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
}

#[tokio::test]
async fn ardupilot_set_servo_surfaces_ack_rejection() {
    let (vehicle, msg_tx, _sent) = connect_mock_vehicle_with_sent().await;

    let set_servo_task = {
        let vehicle = vehicle.clone();
        tokio::spawn(async move { vehicle.ardupilot().set_servo(3, 1500).await })
    };

    tokio::time::sleep(Duration::from_millis(10)).await;
    msg_tx
        .send((
            default_header(),
            ack_msg(
                dialect::MavCmd::MAV_CMD_DO_SET_SERVO,
                dialect::MavResult::MAV_RESULT_DENIED,
            ),
        ))
        .await
        .expect("set_servo rejection ack should be delivered");

    assert!(matches!(
        set_servo_task
            .await
            .expect("set_servo rejection task should join"),
        Err(VehicleError::CommandRejected {
            command,
            result: crate::error::CommandResult::Denied,
        }) if command == dialect::MavCmd::MAV_CMD_DO_SET_SERVO as u16
    ));

    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
}

#[tokio::test]
async fn start_mag_cal_sends_and_accepts_ack() {
    let (vehicle, msg_tx, sent) = connect_mock_vehicle_with_sent().await;

    let start_task = {
        let vehicle = vehicle.clone();
        tokio::spawn(async move { vehicle.ardupilot().start_mag_cal(0b0000_0111).await })
    };

    tokio::time::sleep(Duration::from_millis(10)).await;
    msg_tx
        .send((
            default_header(),
            ack_msg_by_id(
                MAV_CMD_DO_START_MAG_CAL_ID,
                dialect::MavResult::MAV_RESULT_ACCEPTED,
            ),
        ))
        .await
        .expect("start_mag_cal ack should be delivered");

    assert!(
        start_task
            .await
            .expect("start_mag_cal task should join")
            .is_ok()
    );

    let command = sent
        .lock()
        .expect("sent messages lock should not poison")
        .iter()
        .find_map(|(_, msg)| match msg {
            dialect::MavMessage::COMMAND_LONG(data)
                if data.command as u16 == MAV_CMD_DO_START_MAG_CAL_ID =>
            {
                Some(data.clone())
            }
            _ => None,
        })
        .expect("start_mag_cal command should be sent");

    assert_eq!(command.param1, 7.0);
    assert_eq!(command.param2, 0.0);
    assert_eq!(command.param3, 0.0);
    assert_eq!(command.param4, 0.0);
    assert_eq!(command.param5, 0.0);
    assert_eq!(command.param6, 0.0);
    assert_eq!(command.param7, 0.0);

    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
}

#[tokio::test]
async fn accept_mag_cal_sends_and_accepts_ack() {
    let (vehicle, msg_tx, sent) = connect_mock_vehicle_with_sent().await;

    let accept_task = {
        let vehicle = vehicle.clone();
        tokio::spawn(async move { vehicle.ardupilot().accept_mag_cal().await })
    };

    tokio::time::sleep(Duration::from_millis(10)).await;
    msg_tx
        .send((
            default_header(),
            ack_msg_by_id(
                MAV_CMD_DO_ACCEPT_MAG_CAL_ID,
                dialect::MavResult::MAV_RESULT_ACCEPTED,
            ),
        ))
        .await
        .expect("accept_mag_cal ack should be delivered");

    assert!(
        accept_task
            .await
            .expect("accept_mag_cal task should join")
            .is_ok()
    );

    let command = sent
        .lock()
        .expect("sent messages lock should not poison")
        .iter()
        .find_map(|(_, msg)| match msg {
            dialect::MavMessage::COMMAND_LONG(data)
                if data.command as u16 == MAV_CMD_DO_ACCEPT_MAG_CAL_ID =>
            {
                Some(data.clone())
            }
            _ => None,
        })
        .expect("accept_mag_cal command should be sent");

    assert_eq!(command.param1, 0.0);
    assert_eq!(command.param2, 0.0);
    assert_eq!(command.param3, 0.0);
    assert_eq!(command.param4, 0.0);
    assert_eq!(command.param5, 0.0);
    assert_eq!(command.param6, 0.0);
    assert_eq!(command.param7, 0.0);

    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
}

#[tokio::test]
async fn cancel_mag_cal_sends_and_accepts_ack() {
    let (vehicle, msg_tx, sent) = connect_mock_vehicle_with_sent().await;

    let cancel_task = {
        let vehicle = vehicle.clone();
        tokio::spawn(async move { vehicle.ardupilot().cancel_mag_cal().await })
    };

    tokio::time::sleep(Duration::from_millis(10)).await;
    msg_tx
        .send((
            default_header(),
            ack_msg_by_id(
                MAV_CMD_DO_CANCEL_MAG_CAL_ID,
                dialect::MavResult::MAV_RESULT_ACCEPTED,
            ),
        ))
        .await
        .expect("cancel_mag_cal ack should be delivered");

    assert!(
        cancel_task
            .await
            .expect("cancel_mag_cal task should join")
            .is_ok()
    );

    let command = sent
        .lock()
        .expect("sent messages lock should not poison")
        .iter()
        .find_map(|(_, msg)| match msg {
            dialect::MavMessage::COMMAND_LONG(data)
                if data.command as u16 == MAV_CMD_DO_CANCEL_MAG_CAL_ID =>
            {
                Some(data.clone())
            }
            _ => None,
        })
        .expect("cancel_mag_cal command should be sent");

    assert_eq!(command.param1, 0.0);
    assert_eq!(command.param2, 0.0);
    assert_eq!(command.param3, 0.0);
    assert_eq!(command.param4, 0.0);
    assert_eq!(command.param5, 0.0);
    assert_eq!(command.param6, 0.0);
    assert_eq!(command.param7, 0.0);

    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
}

async fn preflight_calibration_command(
    vehicle: Vehicle,
    msg_tx: &mpsc::Sender<(MavHeader, dialect::MavMessage)>,
    sent: &SentMessages,
    gyro: bool,
    accel: bool,
    baro: bool,
    accel_trim: bool,
) -> dialect::COMMAND_LONG_DATA {
    let preflight_task = tokio::spawn(async move {
        vehicle
            .ardupilot()
            .preflight_calibration(gyro, accel, baro, accel_trim)
            .await
    });

    tokio::time::sleep(Duration::from_millis(10)).await;
    msg_tx
        .send((
            default_header(),
            ack_msg(
                dialect::MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION,
                dialect::MavResult::MAV_RESULT_ACCEPTED,
            ),
        ))
        .await
        .expect("preflight calibration ack should be delivered");

    assert!(
        preflight_task
            .await
            .expect("preflight calibration task should join")
            .is_ok()
    );

    sent.lock()
        .expect("sent messages lock should not poison")
        .iter()
        .rev()
        .find_map(|(_, msg)| match msg {
            dialect::MavMessage::COMMAND_LONG(data)
                if data.command == dialect::MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION =>
            {
                Some(data.clone())
            }
            _ => None,
        })
        .expect("preflight calibration command should be sent")
}

#[tokio::test]
async fn preflight_cal_gyro() {
    let (vehicle, msg_tx, sent) = connect_mock_vehicle_with_sent().await;

    let command =
        preflight_calibration_command(vehicle.clone(), &msg_tx, &sent, true, false, false, false)
            .await;

    assert_eq!(command.param1, 1.0);
    assert_eq!(command.param2, 0.0);
    assert_eq!(command.param3, 0.0);
    assert_eq!(command.param4, 0.0);
    assert_eq!(command.param5, 0.0);
    assert_eq!(command.param6, 0.0);
    assert_eq!(command.param7, 0.0);

    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
}

#[tokio::test]
async fn preflight_cal_level_horizon_accel() {
    let (vehicle, msg_tx, sent) = connect_mock_vehicle_with_sent().await;

    let command =
        preflight_calibration_command(vehicle.clone(), &msg_tx, &sent, false, true, false, false)
            .await;

    assert_eq!(command.param1, 0.0);
    assert_eq!(command.param2, 0.0);
    assert_eq!(command.param3, 0.0);
    assert_eq!(command.param4, 0.0);
    assert_eq!(command.param5, 1.0);
    assert_eq!(command.param6, 0.0);
    assert_eq!(command.param7, 0.0);

    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
}

#[tokio::test]
async fn preflight_cal_accel_trim() {
    let (vehicle, msg_tx, sent) = connect_mock_vehicle_with_sent().await;

    let command =
        preflight_calibration_command(vehicle.clone(), &msg_tx, &sent, false, false, false, true)
            .await;

    assert_eq!(command.param1, 0.0);
    assert_eq!(command.param2, 0.0);
    assert_eq!(command.param3, 0.0);
    assert_eq!(command.param4, 0.0);
    assert_eq!(command.param5, 2.0);
    assert_eq!(command.param6, 0.0);
    assert_eq!(command.param7, 0.0);

    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
}

#[tokio::test]
async fn ardupilot_mag_cal_observations_aggregate_by_compass_id() {
    let (writers, channels) = create_channels();
    let cancel = tokio_util::sync::CancellationToken::new();
    let domain = ArduPilotDomain::new();
    let _tasks = domain.start(&channels, cancel.clone());

    assert_eq!(domain.mag_cal_progress().latest(), Some(Vec::new()));
    assert_eq!(domain.mag_cal_report().latest(), Some(Vec::new()));

    let mut progress_sub = domain.mag_cal_progress().subscribe();
    let mut report_sub = domain.mag_cal_report().subscribe();

    writers
        .mag_cal_progress
        .send_replace(Some(state::MagCalProgress {
            compass_id: 2,
            completion_pct: 45,
            status: state::MagCalStatus::RunningStepOne,
            attempt: 1,
        }));

    let first_progress = timeout(Duration::from_millis(250), async {
        loop {
            let observed = progress_sub.recv().await.expect("progress update expected");
            if observed.len() == 1 {
                break observed;
            }
        }
    })
    .await
    .expect("progress aggregation should update");

    assert_eq!(
        first_progress
            .iter()
            .map(|item| item.compass_id)
            .collect::<Vec<_>>(),
        vec![2]
    );

    writers
        .mag_cal_progress
        .send_replace(Some(state::MagCalProgress {
            compass_id: 1,
            completion_pct: 80,
            status: state::MagCalStatus::RunningStepTwo,
            attempt: 2,
        }));

    let progress = timeout(Duration::from_millis(250), async {
        loop {
            let observed = progress_sub.recv().await.expect("progress update expected");
            if observed.len() == 2 {
                break observed;
            }
        }
    })
    .await
    .expect("progress aggregation should update");

    assert_eq!(
        progress
            .iter()
            .map(|item| item.compass_id)
            .collect::<Vec<_>>(),
        vec![1, 2]
    );
    assert_eq!(progress[0].completion_pct, 80);
    assert_eq!(progress[1].completion_pct, 45);

    writers
        .mag_cal_report
        .send_replace(Some(state::MagCalReport {
            compass_id: 2,
            status: state::MagCalStatus::Success,
            fitness: 0.12,
            ofs_x: 1.0,
            ofs_y: 2.0,
            ofs_z: 3.0,
            autosaved: true,
        }));

    let reports = timeout(Duration::from_millis(250), async {
        loop {
            let observed = report_sub.recv().await.expect("report update expected");
            if observed.len() == 1 {
                break observed;
            }
        }
    })
    .await
    .expect("report aggregation should update");

    assert_eq!(reports[0].compass_id, 2);
    assert_eq!(reports[0].status, state::MagCalStatus::Success);
    assert!(reports[0].autosaved);

    cancel.cancel();
}
