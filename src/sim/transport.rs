use async_trait::async_trait;
use mavlink::{MAVLinkMessageRaw, MAVLinkV2MessageRaw, MavHeader, MavlinkVersion};
use tokio::sync::mpsc;

use crate::dialect;

pub(crate) struct SimulatorConnection {
    recv_rx: tokio::sync::Mutex<mpsc::Receiver<(MavHeader, dialect::MavMessage)>>,
    send_tx: mpsc::Sender<(MavHeader, dialect::MavMessage)>,
}

pub(crate) struct SimulatorEndpoints {
    pub(crate) to_sdk_tx: mpsc::Sender<(MavHeader, dialect::MavMessage)>,
    pub(crate) from_sdk_rx: mpsc::Receiver<(MavHeader, dialect::MavMessage)>,
}

impl SimulatorConnection {
    pub(crate) fn new(capacity: usize) -> (Self, SimulatorEndpoints) {
        let (to_sdk_tx, to_sdk_rx) = mpsc::channel(capacity);
        let (from_sdk_tx, from_sdk_rx) = mpsc::channel(capacity);
        (
            Self {
                recv_rx: tokio::sync::Mutex::new(to_sdk_rx),
                send_tx: from_sdk_tx,
            },
            SimulatorEndpoints {
                to_sdk_tx,
                from_sdk_rx,
            },
        )
    }
}

#[async_trait]
impl mavlink::AsyncMavConnection<dialect::MavMessage> for SimulatorConnection {
    async fn recv(
        &self,
    ) -> Result<(MavHeader, dialect::MavMessage), mavlink::error::MessageReadError> {
        let mut rx = self.recv_rx.lock().await;
        match rx.recv().await {
            Some(message) => Ok(message),
            None => Err(mavlink::error::MessageReadError::Io(std::io::Error::new(
                std::io::ErrorKind::ConnectionReset,
                "demo simulator connection closed",
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
        let mut rx = self.recv_rx.lock().await;
        rx.try_recv().map_err(|err| match err {
            mpsc::error::TryRecvError::Empty => {
                mavlink::error::MessageReadError::Io(std::io::ErrorKind::WouldBlock.into())
            }
            mpsc::error::TryRecvError::Disconnected => {
                mavlink::error::MessageReadError::Io(std::io::Error::new(
                    std::io::ErrorKind::ConnectionReset,
                    "demo simulator connection closed",
                ))
            }
        })
    }

    async fn send(
        &self,
        header: &MavHeader,
        data: &dialect::MavMessage,
    ) -> Result<usize, mavlink::error::MessageWriteError> {
        self.send_tx
            .send((*header, data.clone()))
            .await
            .map_err(|_| {
                mavlink::error::MessageWriteError::Io(std::io::Error::new(
                    std::io::ErrorKind::BrokenPipe,
                    "demo simulator connection closed",
                ))
            })?;
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
