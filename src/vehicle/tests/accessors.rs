use super::support::*;
use crate::{AutopilotType, VehicleError, VehicleIdentity, VehicleType};
use std::sync::Arc;
use std::time::Duration;
use tokio::time::timeout;

#[test]
fn identity_is_sync_value() {
    let vehicle = dummy_vehicle();
    let identity: VehicleIdentity = vehicle.identity();

    assert_eq!(identity.system_id, 0);
    assert_eq!(identity.component_id, 0);
    assert_eq!(identity.autopilot, AutopilotType::Unknown);
    assert_eq!(identity.vehicle_type, VehicleType::Unknown);
}

#[test]
fn root_domain_accessors_compile() {
    let vehicle = dummy_vehicle();

    let _ = vehicle.info();
    let _ = vehicle.support();
    let _ = vehicle.link();
    let _ = vehicle.available_modes();
    let _ = vehicle.telemetry();
    let _ = vehicle.mission();
    let _ = vehicle.fence();
    let _ = vehicle.rally();
    let _ = vehicle.params();
    let _ = vehicle.raw();
    let _ = vehicle.ardupilot();
}

#[test]
fn link_state_starts_as_connecting() {
    let vehicle = dummy_vehicle();

    assert_eq!(
        vehicle.link().state().latest(),
        Some(crate::LinkState::Connecting)
    );
}

#[test]
fn params_get_returns_none_when_no_download() {
    let vehicle = dummy_vehicle();
    assert!(vehicle.params().get("BATT_CAPACITY").is_none());
}

#[tokio::test]
async fn info_handle_defaults_to_sys_locator_until_metadata_arrives() {
    let (vehicle, msg_tx) = connect_mock_vehicle().await;

    assert_eq!(vehicle.info().best_unique_id(), None);
    assert_eq!(vehicle.info().best_display_id(), "sys:1/1");
    assert_eq!(
        timeout(
            Duration::from_millis(100),
            vehicle.info().persistent_identity().wait()
        )
        .await
        .expect("pending identity should be published")
        .expect("pending identity should be readable"),
        crate::PersistentIdentity::Pending {
            system_id: 1,
            component_id: 1,
        }
    );

    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
    drop(msg_tx);
}

#[tokio::test]
async fn info_handle_uses_uid_precedence_and_upgrades_to_uid2() {
    let (vehicle, msg_tx) = connect_mock_vehicle().await;
    let uid = 0x0123_4567_89ab_cdef_u64;
    let uid2 = [
        0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00,
    ];

    msg_tx
        .send((
            crate::test_support::default_header(),
            crate::dialect::MavMessage::AUTOPILOT_VERSION(crate::dialect::AUTOPILOT_VERSION_DATA {
                uid,
                flight_sw_version: 0x01020304,
                os_sw_version: 0x05060708,
                board_version: 77,
                vendor_id: 42,
                product_id: 99,
                flight_custom_version: [0xde, 0xad, 0xbe, 0xef, 0xca, 0xfe, 0xba, 0xbe],
                ..crate::dialect::AUTOPILOT_VERSION_DATA::default()
            }),
        ))
        .await
        .expect("autopilot version should be delivered");

    let firmware = timeout(Duration::from_millis(250), vehicle.info().firmware().wait())
        .await
        .expect("firmware info should arrive")
        .expect("firmware info should be readable");
    assert_eq!(firmware.version.as_deref(), Some("1.2.3"));
    assert_eq!(firmware.os_version.as_deref(), Some("5.6.7"));
    assert_eq!(firmware.git_hash.as_deref(), Some("deadbeefcafebabe"));

    let hardware = timeout(Duration::from_millis(250), vehicle.info().hardware().wait())
        .await
        .expect("hardware info should arrive")
        .expect("hardware info should be readable");
    assert_eq!(hardware.board_vendor_id, Some(42));
    assert_eq!(hardware.board_product_id, Some(99));
    assert_eq!(hardware.board_version, Some(77));

    let unique_ids = timeout(
        Duration::from_millis(250),
        vehicle.info().unique_ids().wait(),
    )
    .await
    .expect("unique ids should arrive")
    .expect("unique ids should be readable");
    assert_eq!(unique_ids.uid, Some(uid));
    assert_eq!(
        vehicle.info().best_unique_id().as_deref(),
        Some("uid64:0123456789abcdef")
    );
    assert_eq!(vehicle.info().best_display_id(), "uid64:0123456789abcdef");
    assert_eq!(
        vehicle.info().persistent_identity().latest(),
        Some(crate::PersistentIdentity::Ready {
            canonical_id: "uid64:0123456789abcdef".into(),
            aliases: vec!["uid64:0123456789abcdef".into()],
        })
    );

    msg_tx
        .send((
            crate::test_support::default_header(),
            crate::dialect::MavMessage::AUTOPILOT_VERSION(crate::dialect::AUTOPILOT_VERSION_DATA {
                uid,
                uid2,
                flight_sw_version: 0x01020304,
                os_sw_version: 0x05060708,
                board_version: 77,
                vendor_id: 42,
                product_id: 99,
                flight_custom_version: [0xde, 0xad, 0xbe, 0xef, 0xca, 0xfe, 0xba, 0xbe],
                ..crate::dialect::AUTOPILOT_VERSION_DATA::default()
            }),
        ))
        .await
        .expect("upgraded autopilot version should be delivered");

    timeout(Duration::from_millis(250), async {
        loop {
            if vehicle.info().best_display_id() == "mcu:00112233445566778899aabb000000000000" {
                break;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    })
    .await
    .expect("uid2 upgrade should be observed");

    assert_eq!(
        vehicle.info().best_unique_id().as_deref(),
        Some("mcu:00112233445566778899aabb000000000000")
    );
    assert_eq!(
        vehicle.info().persistent_identity().latest(),
        Some(crate::PersistentIdentity::Ready {
            canonical_id: "mcu:00112233445566778899aabb000000000000".into(),
            aliases: vec![
                "mcu:00112233445566778899aabb000000000000".into(),
                "uid64:0123456789abcdef".into(),
            ],
        })
    );

    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
}

#[tokio::test]
async fn params_ops_fail_with_disconnected_without_event_loop() {
    let vehicle = dummy_vehicle();
    let download = vehicle
        .params()
        .download_all()
        .expect("download operation should start");

    assert!(matches!(
        download.wait().await,
        Err(VehicleError::Disconnected)
    ));
}

#[tokio::test]
async fn fence_wait_timeout_returns_default_state() {
    let vehicle = dummy_vehicle();
    let state = vehicle
        .fence()
        .wait_timeout(Duration::from_millis(50))
        .await
        .expect("should return default state");
    assert!(state.plan.is_none());
}

#[tokio::test]
async fn rally_wait_timeout_returns_default_state() {
    let vehicle = dummy_vehicle();
    let state = vehicle
        .rally()
        .wait_timeout(Duration::from_millis(50))
        .await
        .expect("should return default state");
    assert!(state.plan.is_none());
}

#[tokio::test]
async fn mission_wait_timeout_returns_default_state() {
    let vehicle = dummy_vehicle();
    let state = vehicle
        .mission()
        .wait_timeout(Duration::from_millis(50))
        .await
        .expect("should return default state");
    assert!(state.plan.is_none());
}

#[tokio::test]
async fn params_wait_timeout_returns_default_state() {
    let vehicle = dummy_vehicle();
    let state = vehicle
        .params()
        .wait_timeout(Duration::from_millis(50))
        .await
        .expect("should return default state");
    assert!(state.store.is_none());
}

#[tokio::test]
async fn param_download_op_wait_timeout_returns_error() {
    let vehicle = dummy_vehicle();
    let op = vehicle.params().download_all().expect("should start");
    let result = op.wait_timeout(Duration::from_millis(10)).await;
    assert!(
        matches!(
            result,
            Err(VehicleError::Timeout(_)) | Err(VehicleError::Disconnected)
        ),
        "expected Timeout or Disconnected, got {result:?}"
    );
}

#[tokio::test]
async fn mission_op_wait_timeout_returns_error() {
    let (vehicle, _msg_tx) = connect_mock_vehicle().await;
    let op = vehicle.mission().download().expect("should start download");
    let result = op.wait_timeout(Duration::from_millis(50)).await;
    assert!(
        result.is_err(),
        "expected error without protocol responses, got {result:?}"
    );
    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
}

#[test]
fn clone_shares_inner() {
    let vehicle = dummy_vehicle();
    let cloned = vehicle.clone();

    assert!(Arc::ptr_eq(&vehicle.inner, &cloned.inner));
}
