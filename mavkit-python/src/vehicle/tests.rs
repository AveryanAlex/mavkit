use mavkit::dialect;
use mavlink::{
    AsyncMavConnection, MAVLinkMessageRaw, MAVLinkV2MessageRaw, MavHeader, MavlinkVersion,
};
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::sync::mpsc;
use tokio::time::timeout;

use crate::modes::PyModesHandle;
use crate::telemetry::PyTelemetryHandle;
use crate::vehicle::api::PyVehicle;

type SentMessages = Arc<Mutex<Vec<(MavHeader, dialect::MavMessage)>>>;

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

impl AsyncMavConnection<dialect::MavMessage> for MockConnection {
    fn recv<'life0, 'async_trait>(
        &'life0 self,
    ) -> std::pin::Pin<
        Box<
            dyn std::future::Future<
                    Output = Result<
                        (MavHeader, dialect::MavMessage),
                        mavlink::error::MessageReadError,
                    >,
                > + Send
                + 'async_trait,
        >,
    >
    where
        'life0: 'async_trait,
        Self: 'async_trait,
    {
        Box::pin(async move {
            let mut rx = self.recv_rx.lock().await;
            match rx.recv().await {
                Some(message) => Ok(message),
                None => Err(mavlink::error::MessageReadError::Io(std::io::Error::new(
                    std::io::ErrorKind::ConnectionReset,
                    "mock connection closed",
                ))),
            }
        })
    }

    fn recv_raw<'life0, 'async_trait>(
        &'life0 self,
    ) -> std::pin::Pin<
        Box<
            dyn std::future::Future<
                    Output = Result<mavlink::MAVLinkMessageRaw, mavlink::error::MessageReadError>,
                > + Send
                + 'async_trait,
        >,
    >
    where
        'life0: 'async_trait,
        Self: 'async_trait,
    {
        Box::pin(async move {
            let (header, message) = self.recv().await?;
            let mut raw = MAVLinkV2MessageRaw::new();
            raw.serialize_message(header, &message);
            Ok(MAVLinkMessageRaw::V2(raw))
        })
    }

    fn send<'life0, 'life1, 'life2, 'async_trait>(
        &'life0 self,
        header: &'life1 MavHeader,
        data: &'life2 dialect::MavMessage,
    ) -> std::pin::Pin<
        Box<
            dyn std::future::Future<Output = Result<usize, mavlink::error::MessageWriteError>>
                + Send
                + 'async_trait,
        >,
    >
    where
        'life0: 'async_trait,
        'life1: 'async_trait,
        'life2: 'async_trait,
        Self: 'async_trait,
    {
        let header = *header;
        let data = data.clone();
        Box::pin(async move {
            self.sent.lock().unwrap().push((header, data));
            Ok(0)
        })
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

fn default_header() -> MavHeader {
    MavHeader {
        system_id: 1,
        component_id: 1,
        sequence: 0,
    }
}

fn heartbeat_msg() -> dialect::MavMessage {
    dialect::MavMessage::HEARTBEAT(dialect::HEARTBEAT_DATA {
        custom_mode: 7,
        mavtype: dialect::MavType::MAV_TYPE_QUADROTOR,
        autopilot: dialect::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode: dialect::MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        system_status: dialect::MavState::MAV_STATE_STANDBY,
        mavlink_version: 3,
    })
}

fn fast_config() -> mavkit::VehicleConfig {
    mavkit::VehicleConfig {
        connect_timeout: Duration::from_millis(150),
        command_timeout: Duration::from_millis(50),
        command_completion_timeout: Duration::from_millis(150),
        auto_request_home: false,
        ..mavkit::VehicleConfig::default()
    }
}

async fn connect_mock_vehicle() -> mavkit::Vehicle {
    let (msg_tx, msg_rx) = mpsc::channel(16);
    let (conn, _sent) = MockConnection::new(msg_rx);
    let connect_task = tokio::spawn(async move {
        mavkit::Vehicle::from_connection(Box::new(conn), fast_config()).await
    });

    msg_tx
        .send((default_header(), heartbeat_msg()))
        .await
        .expect("heartbeat should be delivered");

    timeout(Duration::from_millis(250), connect_task)
        .await
        .expect("connect should complete")
        .expect("connect task should join")
        .expect("mock vehicle should connect")
}

#[tokio::test(flavor = "current_thread")]
async fn identity_uses_connected_vehicle_state() {
    let vehicle = connect_mock_vehicle().await;
    let py_vehicle = PyVehicle::from_inner(vehicle);

    let identity = crate::vehicle::api::PyVehicleIdentity {
        inner: py_vehicle.inner.identity(),
    };

    assert_eq!(identity.inner.system_id, 1);
    assert_eq!(identity.inner.component_id, 1);
    assert_eq!(
        identity.inner.autopilot,
        mavkit::AutopilotType::ArduPilotMega
    );
    assert_eq!(identity.inner.vehicle_type, mavkit::VehicleType::Quadrotor);
}

#[tokio::test(flavor = "current_thread")]
async fn repeated_handle_access_wraps_shared_vehicle_clone_semantics() {
    let vehicle = connect_mock_vehicle().await;
    let py_vehicle = PyVehicle::from_inner(vehicle);

    let telemetry_a = PyTelemetryHandle::new(py_vehicle.inner.clone());
    let telemetry_b = PyTelemetryHandle::new(py_vehicle.inner.clone());
    let modes_a = PyModesHandle::new(py_vehicle.inner.clone());
    let modes_b = PyModesHandle::new(py_vehicle.inner.clone());

    assert_eq!(telemetry_a.inner.identity(), telemetry_b.inner.identity());
    assert_eq!(
        modes_a.inner.available_modes().len(),
        modes_b.inner.available_modes().len()
    );
    assert_eq!(modes_a.inner.identity(), py_vehicle.inner.identity());
}
