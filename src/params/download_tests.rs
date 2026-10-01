use std::time::Duration;

use crate::VehicleConfig;
use crate::dialect::{self, MavMessage};
use crate::error::VehicleError;
use crate::params::ParamDownloadOp;
use crate::test_support::{ConnectedVehicleHarness, ConnectedVehicleOptions, default_header};
use crate::types::ParamOperationProgress;

async fn start_download(timeout: Duration) -> (ConnectedVehicleHarness, ParamDownloadOp) {
    let mut config = VehicleConfig {
        transfer_timeout: timeout,
        auto_request_home: false,
        ..VehicleConfig::default()
    };
    config.init_policy.autopilot_version.enabled = false;
    config.init_policy.available_modes.enabled = false;
    config.init_policy.home.enabled = false;
    config.init_policy.origin.enabled = false;

    let harness = ConnectedVehicleHarness::connect(ConnectedVehicleOptions {
        config,
        ..ConnectedVehicleOptions::default()
    })
    .await;
    let operation = harness.vehicle.params().download_all().unwrap();
    for _ in 0..100 {
        if harness
            .sent
            .lock()
            .unwrap()
            .iter()
            .any(|(_, message)| matches!(message, MavMessage::PARAM_REQUEST_LIST(_)))
        {
            return (harness, operation);
        }
        tokio::time::sleep(Duration::from_millis(1)).await;
    }
    panic!("parameter download did not send PARAM_REQUEST_LIST");
}

async fn send_parameter(harness: &ConnectedVehicleHarness, index: u16) {
    harness
        .msg_tx
        .send((
            default_header(),
            MavMessage::PARAM_VALUE(dialect::PARAM_VALUE_DATA {
                param_id: format!("TEST_{index}").as_str().into(),
                param_value: f32::from(index),
                param_type: dialect::MavParamType::MAV_PARAM_TYPE_REAL32,
                param_count: 3,
                param_index: index,
            }),
        ))
        .await
        .unwrap();
}

#[tokio::test(start_paused = true)]
async fn complete_list_is_published_without_waiting_for_silence() {
    let (harness, operation) = start_download(Duration::from_secs(20)).await;
    for index in 0..3 {
        tokio::time::sleep(Duration::from_millis(6_500)).await;
        send_parameter(&harness, index).await;
    }

    let store = operation
        .wait_timeout(Duration::from_millis(100))
        .await
        .unwrap();
    assert_eq!(store.len(), 3);
    assert_eq!(
        harness.vehicle.params().latest().unwrap().store,
        Some(store)
    );
    assert_eq!(operation.latest(), Some(ParamOperationProgress::Completed));
    harness.vehicle.disconnect().await.unwrap();
}

#[tokio::test(start_paused = true)]
async fn continuous_duplicate_replies_do_not_delay_a_complete_list() {
    let (harness, operation) = start_download(Duration::from_secs(30)).await;
    for index in 0..3 {
        send_parameter(&harness, index).await;
    }
    for _ in 0..5 {
        tokio::time::sleep(Duration::from_millis(500)).await;
        send_parameter(&harness, 2).await;
    }

    assert_eq!(
        operation
            .wait_timeout(Duration::from_millis(100))
            .await
            .unwrap()
            .len(),
        3
    );
    harness.vehicle.disconnect().await.unwrap();
}

#[tokio::test(start_paused = true)]
async fn new_parameters_extend_the_download_deadline() {
    let (harness, operation) = start_download(Duration::from_secs(5)).await;
    for index in 0..3 {
        tokio::time::sleep(Duration::from_millis(1_800)).await;
        send_parameter(&harness, index).await;
    }

    assert_eq!(
        operation
            .wait_timeout(Duration::from_millis(100))
            .await
            .unwrap()
            .len(),
        3
    );
    harness.vehicle.disconnect().await.unwrap();
}

#[tokio::test(start_paused = true)]
async fn duplicate_replies_do_not_delay_requests_for_missing_indices() {
    let (harness, operation) = start_download(Duration::from_secs(20)).await;
    send_parameter(&harness, 2).await;
    send_parameter(&harness, 0).await;
    for _ in 0..5 {
        tokio::time::sleep(Duration::from_millis(500)).await;
        send_parameter(&harness, 2).await;
    }

    assert!(harness.vehicle.params().latest().unwrap().store.is_none());
    assert!(harness.sent.lock().unwrap().iter().any(|(_, message)| {
        matches!(message, MavMessage::PARAM_REQUEST_READ(data) if data.param_index == 1)
    }));

    send_parameter(&harness, 1).await;
    let store = operation
        .wait_timeout(Duration::from_millis(100))
        .await
        .unwrap();
    assert!((0..3).all(|index| store.get(&format!("TEST_{index}")).unwrap().index == index));
    harness.vehicle.disconnect().await.unwrap();
}

#[tokio::test(start_paused = true)]
async fn incomplete_list_times_out_while_duplicate_replies_continue() {
    let (harness, operation) = start_download(Duration::from_secs(5)).await;
    send_parameter(&harness, 0).await;
    send_parameter(&harness, 2).await;
    for _ in 0..12 {
        tokio::time::sleep(Duration::from_millis(500)).await;
        send_parameter(&harness, 2).await;
    }

    assert!(matches!(
        operation.wait().await,
        Err(VehicleError::Timeout(_))
    ));
    assert!(harness.vehicle.params().latest().unwrap().store.is_none());
    harness.vehicle.disconnect().await.unwrap();
}

#[tokio::test(start_paused = true)]
async fn incomplete_download_can_still_be_cancelled() {
    let (harness, operation) = start_download(Duration::from_secs(20)).await;
    send_parameter(&harness, 0).await;
    operation.cancel();

    assert!(matches!(
        operation.wait().await,
        Err(VehicleError::Cancelled)
    ));
    assert!(harness.vehicle.params().latest().unwrap().store.is_none());
    harness.vehicle.disconnect().await.unwrap();
}
