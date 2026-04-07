use mavkit::dialect;
use mavlink::{
    AsyncMavConnection, MAVLinkMessageRaw, MAVLinkV2MessageRaw, MavHeader, MavlinkVersion,
};
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::sync::mpsc;
use tokio::time::timeout;

pub(super) async fn connect_mock_vehicle(
    mavtype: dialect::MavType,
    custom_mode: u32,
) -> mavkit::Vehicle {
    let (msg_tx, msg_rx) = mpsc::channel(16);
    let (conn, _sent) = MockConnection::new(msg_rx);
    let connect_task = tokio::spawn(async move {
        mavkit::Vehicle::from_connection(Box::new(conn), fast_config()).await
    });

    msg_tx
        .send((
            default_header(),
            heartbeat_msg_with_mode(mavtype, custom_mode),
        ))
        .await
        .expect("heartbeat should be delivered");

    timeout(Duration::from_millis(250), connect_task)
        .await
        .expect("connect should complete")
        .expect("connect task should join")
        .expect("mock vehicle should connect")
}

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
                    Output = Result<MAVLinkMessageRaw, mavlink::error::MessageReadError>,
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

fn heartbeat_msg_with_mode(mavtype: dialect::MavType, custom_mode: u32) -> dialect::MavMessage {
    dialect::MavMessage::HEARTBEAT(dialect::HEARTBEAT_DATA {
        custom_mode,
        mavtype,
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
