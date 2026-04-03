use crate::command::Command;
use crate::config::VehicleConfig;
use crate::dialect;
use crate::error::VehicleError;
use crate::test_support::{
    ConnectedVehicleHarness, ConnectedVehicleOptions, SentMessages, command_ack,
    fast_vehicle_test_config, heartbeat,
};
use crate::vehicle::{Vehicle, VehicleInner};
use mavlink::MavHeader;
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::mpsc;
use tokio_util::sync::CancellationToken;

pub(super) fn fast_config() -> VehicleConfig {
    fast_vehicle_test_config()
}

pub(super) fn heartbeat_msg(armed: bool, custom_mode: u32) -> dialect::MavMessage {
    heartbeat(armed, custom_mode)
}

pub(super) fn ack_msg(command: dialect::MavCmd, result: dialect::MavResult) -> dialect::MavMessage {
    command_ack(command, result)
}

pub(super) fn gps_global_origin_msg(
    latitude: i32,
    longitude: i32,
    altitude: i32,
) -> dialect::MavMessage {
    dialect::MavMessage::GPS_GLOBAL_ORIGIN(dialect::GPS_GLOBAL_ORIGIN_DATA {
        latitude,
        longitude,
        altitude,
        time_usec: 0,
    })
}

pub(super) fn available_modes_msg(
    number_modes: u8,
    mode_index: u8,
    custom_mode: u32,
    mode_name: &str,
) -> dialect::MavMessage {
    dialect::MavMessage::AVAILABLE_MODES(dialect::AVAILABLE_MODES_DATA {
        custom_mode,
        properties: dialect::MavModeProperty::empty(),
        number_modes,
        mode_index,
        standard_mode: dialect::MavStandardMode::MAV_STANDARD_MODE_NON_STANDARD,
        mode_name: mode_name.into(),
    })
}

pub(super) async fn connect_mock_vehicle()
-> (Vehicle, mpsc::Sender<(MavHeader, dialect::MavMessage)>) {
    let (vehicle, msg_tx, _sent) = connect_mock_vehicle_with_sent().await;
    (vehicle, msg_tx)
}

pub(super) async fn connect_mock_vehicle_with_sent() -> (
    Vehicle,
    mpsc::Sender<(MavHeader, dialect::MavMessage)>,
    SentMessages,
) {
    let harness = ConnectedVehicleHarness::connect(ConnectedVehicleOptions {
        config: fast_config(),
        join_timeout: Duration::from_millis(250),
        ..ConnectedVehicleOptions::default()
    })
    .await;

    (harness.vehicle, harness.msg_tx, harness.sent)
}

pub(super) async fn connect_mock_vehicle_with_header_and_sent(
    header: MavHeader,
) -> (
    Vehicle,
    mpsc::Sender<(MavHeader, dialect::MavMessage)>,
    SentMessages,
) {
    let harness = ConnectedVehicleHarness::connect(ConnectedVehicleOptions {
        config: fast_config(),
        heartbeat_header: header,
        heartbeat_message: heartbeat(false, 7),
        join_timeout: Duration::from_millis(250),
        ..ConnectedVehicleOptions::default()
    })
    .await;

    (harness.vehicle, harness.msg_tx, harness.sent)
}

pub(super) async fn wait_for_metric_disconnect<T: Clone + Send + Sync + 'static>(
    metric: crate::observation::MetricHandle<T>,
) -> Result<(), VehicleError> {
    metric.wait().await.map(|_| ())
}

pub(super) fn test_vehicle_with_command_rx() -> (Vehicle, mpsc::Receiver<Command>) {
    let (_, stores) = crate::state::create_channels();
    let cancel = CancellationToken::new();
    let (command_tx, command_rx) = mpsc::channel(1);

    (
        Vehicle {
            inner: Arc::new(VehicleInner::new(
                command_tx,
                cancel,
                stores,
                VehicleConfig::default(),
            )),
        },
        command_rx,
    )
}

pub(super) fn dummy_vehicle() -> Vehicle {
    let (vehicle, _command_rx) = test_vehicle_with_command_rx();
    vehicle
}
