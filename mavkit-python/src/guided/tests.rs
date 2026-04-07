use super::PyArduGuidedSession;
use super::test_support::connect_mock_vehicle;
use mavkit::dialect;

#[tokio::test(flavor = "current_thread")]
async fn close_is_idempotent_and_blocks_future_narrowing() {
    let vehicle = connect_mock_vehicle(dialect::MavType::MAV_TYPE_QUADROTOR, 4).await;
    let session = vehicle
        .ardupilot()
        .guided()
        .await
        .expect("guided session should acquire");
    let py_session = PyArduGuidedSession::new(session, "sys=1, comp=1".to_string());

    py_session
        .close_impl()
        .await
        .expect("first close should succeed");
    py_session
        .close_impl()
        .await
        .expect("second close should also succeed");

    assert!(matches!(
        py_session.copter_impl(),
        Err(mavkit::VehicleError::OperationConflict {
            conflicting_domain,
            conflicting_op,
        }) if conflicting_domain == "ardupilot_guided" && conflicting_op == "session_closed"
    ));
}

#[tokio::test(flavor = "current_thread")]
async fn retained_guided_handle_operations_fail_after_python_close() {
    let vehicle = connect_mock_vehicle(dialect::MavType::MAV_TYPE_GROUND_ROVER, 15).await;
    let session = vehicle
        .ardupilot()
        .guided()
        .await
        .expect("guided session should acquire");
    let py_session = PyArduGuidedSession::new(session, "sys=1, comp=1".to_string());
    let rover = py_session
        .rover_impl()
        .expect("rover narrowing should succeed before close")
        .expect("rover session should expose rover handle");

    py_session.close_impl().await.expect("close should succeed");

    assert!(matches!(
        rover.drive_impl(1.0, 0.0).await,
        Err(mavkit::VehicleError::OperationConflict {
            conflicting_domain,
            conflicting_op,
        }) if conflicting_domain == "ardupilot_guided" && conflicting_op == "session_closed"
    ));
}

#[tokio::test(flavor = "current_thread")]
async fn wrong_family_narrowing_returns_none_for_other_families() {
    let vehicle = connect_mock_vehicle(dialect::MavType::MAV_TYPE_QUADROTOR, 4).await;
    let session = vehicle
        .ardupilot()
        .guided()
        .await
        .expect("guided session should acquire");
    let py_session = PyArduGuidedSession::new(session, "sys=1, comp=1".to_string());

    assert!(
        py_session
            .plane_impl()
            .expect("plane narrowing should not error while open")
            .is_none()
    );
    assert!(
        py_session
            .rover_impl()
            .expect("rover narrowing should not error while open")
            .is_none()
    );
    assert!(
        py_session
            .sub_impl()
            .expect("sub narrowing should not error while open")
            .is_none()
    );
}

#[tokio::test(flavor = "current_thread")]
async fn plane_vtol_narrowing_tracks_plane_kind() {
    let fixed_wing_vehicle = connect_mock_vehicle(dialect::MavType::MAV_TYPE_FIXED_WING, 15).await;
    let fixed_wing_session = fixed_wing_vehicle
        .ardupilot()
        .guided()
        .await
        .expect("fixed-wing guided session should acquire");
    let fixed_wing_py_session =
        PyArduGuidedSession::new(fixed_wing_session, "sys=1, comp=1".to_string());
    let fixed_wing_plane = fixed_wing_py_session
        .plane_impl()
        .expect("plane narrowing should succeed")
        .expect("fixed-wing session should expose plane handle");
    assert!(
        fixed_wing_plane
            .vtol_impl()
            .expect("fixed-wing VTOL narrowing should not error")
            .is_none()
    );

    let vtol_vehicle = connect_mock_vehicle(dialect::MavType::MAV_TYPE_VTOL_FIXEDROTOR, 15).await;
    let vtol_session = vtol_vehicle
        .ardupilot()
        .guided()
        .await
        .expect("vtol plane guided session should acquire");
    let vtol_py_session = PyArduGuidedSession::new(vtol_session, "sys=1, comp=1".to_string());
    let vtol_plane = vtol_py_session
        .plane_impl()
        .expect("plane narrowing should succeed")
        .expect("vtol session should expose plane handle");
    assert!(
        vtol_plane
            .vtol_impl()
            .expect("vtol narrowing should not error")
            .is_some()
    );
}
