use std::sync::{Arc, Mutex};
use std::time::Duration;

use async_trait::async_trait;
use futures::FutureExt;
use mavlink::{MAVLinkMessageRaw, MAVLinkV2MessageRaw, MavHeader, MavlinkVersion};
use tokio::sync::mpsc;
use tokio::time::timeout;

use crate::Vehicle;
use crate::config::VehicleConfig;
use crate::dialect;

pub type SentMessages = Arc<Mutex<Vec<(MavHeader, dialect::MavMessage)>>>;

pub struct MockConnection {
    recv_rx: tokio::sync::Mutex<mpsc::Receiver<(MavHeader, dialect::MavMessage)>>,
    sent: SentMessages,
}

impl MockConnection {
    pub fn new(rx: mpsc::Receiver<(MavHeader, dialect::MavMessage)>) -> (Self, SentMessages) {
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

pub fn default_header() -> MavHeader {
    MavHeader {
        system_id: 1,
        component_id: 1,
        sequence: 0,
    }
}

pub fn heartbeat(armed: bool, custom_mode: u32) -> dialect::MavMessage {
    let mut base_mode = dialect::MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    if armed {
        base_mode |= dialect::MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED;
    }

    heartbeat_for_type(dialect::MavType::MAV_TYPE_QUADROTOR, custom_mode, base_mode)
}

pub fn heartbeat_for_type(
    mavtype: dialect::MavType,
    custom_mode: u32,
    base_mode: dialect::MavModeFlag,
) -> dialect::MavMessage {
    dialect::MavMessage::HEARTBEAT(dialect::HEARTBEAT_DATA {
        custom_mode,
        mavtype,
        autopilot: dialect::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode,
        system_status: dialect::MavState::MAV_STATE_STANDBY,
        mavlink_version: 3,
    })
}

pub fn command_ack(command: dialect::MavCmd, result: dialect::MavResult) -> dialect::MavMessage {
    command_ack_with(command, result, 0, 0)
}

pub fn command_ack_with(
    command: dialect::MavCmd,
    result: dialect::MavResult,
    progress: u8,
    result_param2: i32,
) -> dialect::MavMessage {
    dialect::MavMessage::COMMAND_ACK(dialect::COMMAND_ACK_DATA {
        command,
        result,
        progress,
        result_param2,
        target_system: 0,
        target_component: 0,
    })
}

pub fn fast_vehicle_test_config() -> VehicleConfig {
    VehicleConfig {
        connect_timeout: Duration::from_millis(150),
        command_timeout: Duration::from_millis(50),
        command_completion_timeout: Duration::from_millis(150),
        auto_request_home: false,
        ..VehicleConfig::default()
    }
}

pub fn assert_approx(actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() < 1e-4,
        "expected {expected}, got {actual}"
    );
}

#[derive(Clone)]
pub struct ConnectedVehicleOptions {
    pub config: VehicleConfig,
    pub heartbeat_header: MavHeader,
    pub heartbeat_message: dialect::MavMessage,
    pub join_timeout: Duration,
    pub message_capacity: usize,
}

impl Default for ConnectedVehicleOptions {
    fn default() -> Self {
        Self {
            config: fast_vehicle_test_config(),
            heartbeat_header: default_header(),
            heartbeat_message: heartbeat(false, 7),
            join_timeout: Duration::from_millis(250),
            message_capacity: 16,
        }
    }
}

pub struct ConnectedVehicleHarness {
    pub vehicle: Vehicle,
    pub msg_tx: mpsc::Sender<(MavHeader, dialect::MavMessage)>,
    pub sent: SentMessages,
}

impl ConnectedVehicleHarness {
    pub async fn connect(options: ConnectedVehicleOptions) -> Self {
        let ConnectedVehicleOptions {
            config,
            heartbeat_header,
            heartbeat_message,
            join_timeout,
            message_capacity,
        } = options;

        let (msg_tx, msg_rx) = mpsc::channel(message_capacity);
        let (conn, sent) = MockConnection::new(msg_rx);
        let connect_task =
            tokio::spawn(async move { Vehicle::from_connection(Box::new(conn), config).await });

        msg_tx
            .send((heartbeat_header, heartbeat_message))
            .await
            .expect("heartbeat should be delivered to the mock connection");

        let vehicle = timeout(join_timeout, connect_task)
            .await
            .expect("connect should complete after first heartbeat")
            .expect("connect task should join")
            .expect("mock vehicle should connect");

        Self {
            vehicle,
            msg_tx,
            sent,
        }
    }
}

#[async_trait]
impl mavlink::AsyncMavConnection<dialect::MavMessage> for MockConnection {
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

    async fn try_recv(
        &self,
    ) -> Result<(MavHeader, dialect::MavMessage), mavlink::error::MessageReadError> {
        match self.recv().now_or_never() {
            Some(result) => result,
            None => Err(mavlink::error::MessageReadError::Io(
                std::io::ErrorKind::WouldBlock.into(),
            )),
        }
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
