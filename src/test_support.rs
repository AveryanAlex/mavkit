use std::sync::{Arc, Mutex};

use async_trait::async_trait;
use mavlink::{MAVLinkMessageRaw, MAVLinkV2MessageRaw, MavHeader, MavlinkVersion};
use tokio::sync::mpsc;

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
