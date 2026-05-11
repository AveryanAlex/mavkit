use crate::dialect;
use mavlink::{MAVLinkMessageRaw, MavHeader, MavlinkVersion, Message, ReadVersion};
use std::io;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use tokio::sync::{Mutex, mpsc};
use tokio_util::sync::CancellationToken;

#[derive(Debug, Clone)]
/// Channel capacities for [`ByteConnection`] and [`ByteBridge`].
///
/// Both capacities are clamped to at least `1`. Inbound capacity controls how many ordered byte
/// chunks can be queued for MAVLink parsing. Outbound capacity controls how many serialized MAVLink
/// frames can wait to be drained by the bridge owner.
pub struct ByteConnectionConfig {
    pub inbound_capacity: usize,
    pub outbound_capacity: usize,
}

impl Default for ByteConnectionConfig {
    fn default() -> Self {
        Self {
            inbound_capacity: 64,
            outbound_capacity: 64,
        }
    }
}

/// An in-process MAVLink connection over ordered byte chunks.
///
/// This is intended for browser or callback-driven transports that do not expose `AsyncRead` /
/// `AsyncWrite`. Pair it with the returned [`ByteBridge`] from [`ByteConnection::new`] so transport
/// code can push inbound bytes and drain outbound frames.
pub struct ByteConnection {
    reader: Mutex<ReaderState>,
    writer: Mutex<WriterState>,
    outbound_tx: mpsc::Sender<Vec<u8>>,
    close: CancellationToken,
    protocol_version: AtomicU8,
    allow_recv_any_version: AtomicBool,
}

struct ReaderState {
    inbound_rx: mpsc::Receiver<Vec<u8>>,
    buffer: Vec<u8>,
}

struct WriterState {
    sequence: u8,
}

#[derive(Clone)]
/// Browser/app-facing bridge for [`ByteConnection`].
///
/// Inbound bytes are delivered in arrival order. Outbound frames are emitted one MAVLink frame per
/// `Vec<u8>`. Calling [`ByteBridge::close`] is immediate: future inbound writes are rejected,
/// `next_outbound` returns `None`, and the paired connection reports EOF for reads.
pub struct ByteBridge {
    inbound_tx: mpsc::Sender<Vec<u8>>,
    outbound_rx: Arc<Mutex<mpsc::Receiver<Vec<u8>>>>,
    close: CancellationToken,
}

#[derive(Debug, thiserror::Error)]
pub enum ByteBridgeError {
    #[error("byte connection closed")]
    Closed(Vec<u8>),
    #[error("byte connection inbound buffer full")]
    Full(Vec<u8>),
}

impl ByteConnection {
    /// Create a byte-oriented MAVLink connection plus its bridge handle.
    pub fn new(config: ByteConnectionConfig) -> (Self, ByteBridge) {
        let inbound_capacity = config.inbound_capacity.max(1);
        let outbound_capacity = config.outbound_capacity.max(1);
        let (inbound_tx, inbound_rx) = mpsc::channel(inbound_capacity);
        let (outbound_tx, outbound_rx) = mpsc::channel(outbound_capacity);
        let close = CancellationToken::new();

        (
            Self {
                reader: Mutex::new(ReaderState {
                    inbound_rx,
                    buffer: Vec::new(),
                }),
                writer: Mutex::new(WriterState { sequence: 0 }),
                outbound_tx,
                close: close.clone(),
                protocol_version: AtomicU8::new(Self::encode_version(MavlinkVersion::V2)),
                allow_recv_any_version: AtomicBool::new(true),
            },
            ByteBridge {
                inbound_tx,
                outbound_rx: Arc::new(Mutex::new(outbound_rx)),
                close,
            },
        )
    }

    fn encode_version(version: MavlinkVersion) -> u8 {
        match version {
            MavlinkVersion::V1 => 1,
            MavlinkVersion::V2 => 2,
        }
    }

    fn decode_version(value: u8) -> MavlinkVersion {
        match value {
            1 => MavlinkVersion::V1,
            _ => MavlinkVersion::V2,
        }
    }

    fn read_version(&self) -> ReadVersion {
        if self.allow_recv_any_version.load(Ordering::SeqCst) {
            ReadVersion::Any
        } else {
            ReadVersion::Single(Self::decode_version(
                self.protocol_version.load(Ordering::SeqCst),
            ))
        }
    }

    fn parse_raw_message(
        raw: MAVLinkMessageRaw,
    ) -> Result<(MavHeader, dialect::MavMessage), mavlink::error::MessageReadError> {
        let header = MavHeader {
            sequence: raw.sequence(),
            system_id: raw.system_id(),
            component_id: raw.component_id(),
        };
        let message = dialect::MavMessage::parse(raw.version(), raw.message_id(), raw.payload())?;
        Ok((header, message))
    }

    fn try_extract_frame(
        state: &mut ReaderState,
        read_version: ReadVersion,
    ) -> Result<Option<MAVLinkMessageRaw>, mavlink::error::MessageReadError> {
        loop {
            let Some(magic_index) = state
                .buffer
                .iter()
                .position(|byte| matches!(*byte, 0xFE | 0xFD))
            else {
                state.buffer.clear();
                return Ok(None);
            };

            if magic_index > 0 {
                state.buffer.drain(..magic_index);
            }

            let Some(frame_len) = candidate_frame_len(&state.buffer) else {
                return Ok(None);
            };

            if state.buffer.len() < frame_len {
                return Ok(None);
            }

            let frame = state.buffer[..frame_len].to_vec();
            let cursor = std::io::Cursor::new(frame);
            let mut reader = mavlink::peek_reader::PeekReader::new(cursor);

            match mavlink::read_versioned_raw_message::<dialect::MavMessage, _>(
                &mut reader,
                read_version,
            ) {
                Ok(raw) => {
                    state.buffer.drain(..frame_len);
                    return Ok(Some(raw));
                }
                Err(_) => {
                    state.buffer.drain(..1);
                }
            }
        }
    }
}

impl ByteBridge {
    /// Queue inbound transport bytes, waiting for bounded capacity when necessary.
    pub async fn push_inbound(&self, bytes: Vec<u8>) -> Result<(), ByteBridgeError> {
        let reserve = self.inbound_tx.reserve();
        tokio::pin!(reserve);

        tokio::select! {
            biased;
            _ = self.close.cancelled() => Err(ByteBridgeError::Closed(bytes)),
            permit = reserve => match permit {
                Ok(permit) => {
                    permit.send(bytes);
                    Ok(())
                }
                Err(_) => Err(ByteBridgeError::Closed(bytes)),
            },
        }
    }

    /// Try to queue inbound transport bytes without waiting.
    pub fn try_push_inbound(&self, bytes: Vec<u8>) -> Result<(), ByteBridgeError> {
        if self.close.is_cancelled() {
            return Err(ByteBridgeError::Closed(bytes));
        }

        match self.inbound_tx.try_send(bytes) {
            Ok(()) => Ok(()),
            Err(tokio::sync::mpsc::error::TrySendError::Full(bytes)) => {
                Err(ByteBridgeError::Full(bytes))
            }
            Err(tokio::sync::mpsc::error::TrySendError::Closed(bytes)) => {
                Err(ByteBridgeError::Closed(bytes))
            }
        }
    }

    /// Receive the next outbound MAVLink frame, or `None` immediately after close.
    pub async fn next_outbound(&self) -> Option<Vec<u8>> {
        let mut outbound_rx = self.outbound_rx.lock().await;

        tokio::select! {
            biased;
            _ = self.close.cancelled() => None,
            bytes = outbound_rx.recv() => bytes,
        }
    }

    /// Close the bridge and paired connection immediately.
    pub fn close(&self) {
        self.close.cancel();
    }

    /// Return whether the bridge has been closed.
    pub fn is_closed(&self) -> bool {
        self.close.is_cancelled()
    }
}

#[async_trait::async_trait]
impl mavlink::AsyncMavConnection<dialect::MavMessage> for ByteConnection {
    async fn recv(
        &self,
    ) -> Result<(MavHeader, dialect::MavMessage), mavlink::error::MessageReadError> {
        Self::parse_raw_message(self.recv_raw().await?)
    }

    async fn recv_raw(&self) -> Result<MAVLinkMessageRaw, mavlink::error::MessageReadError> {
        let read_version = self.read_version();
        let mut reader = self.reader.lock().await;

        loop {
            if let Some(raw) = Self::try_extract_frame(&mut reader, read_version)? {
                return Ok(raw);
            }

            tokio::select! {
                biased;
                _ = self.close.cancelled() => return Err(mavlink::error::MessageReadError::eof()),
                bytes = reader.inbound_rx.recv() => match bytes {
                    Some(bytes) => reader.buffer.extend(bytes),
                    None => return Err(mavlink::error::MessageReadError::eof()),
                },
            }
        }
    }

    async fn try_recv(
        &self,
    ) -> Result<(MavHeader, dialect::MavMessage), mavlink::error::MessageReadError> {
        let read_version = self.read_version();
        let mut reader = self.reader.lock().await;

        loop {
            if let Some(raw) = Self::try_extract_frame(&mut reader, read_version)? {
                return Self::parse_raw_message(raw);
            }

            if self.close.is_cancelled() {
                return Err(mavlink::error::MessageReadError::eof());
            }

            match reader.inbound_rx.try_recv() {
                Ok(bytes) => reader.buffer.extend(bytes),
                Err(tokio::sync::mpsc::error::TryRecvError::Empty) => {
                    return Err(mavlink::error::MessageReadError::Io(
                        io::ErrorKind::WouldBlock.into(),
                    ));
                }
                Err(tokio::sync::mpsc::error::TryRecvError::Disconnected) => {
                    return Err(mavlink::error::MessageReadError::eof());
                }
            }
        }
    }

    async fn send(
        &self,
        header: &MavHeader,
        data: &dialect::MavMessage,
    ) -> Result<usize, mavlink::error::MessageWriteError> {
        let mut writer = self.writer.lock().await;
        let header = MavHeader {
            sequence: writer.sequence,
            system_id: header.system_id,
            component_id: header.component_id,
        };
        writer.sequence = writer.sequence.wrapping_add(1);
        let protocol_version = Self::decode_version(self.protocol_version.load(Ordering::SeqCst));
        drop(writer);

        let mut bytes = Vec::new();
        let written = mavlink::write_versioned_msg(&mut bytes, protocol_version, header, data)?;

        let reserve = self.outbound_tx.reserve();
        tokio::pin!(reserve);

        tokio::select! {
            biased;
            _ = self.close.cancelled() => Err(closed_write_error()),
            permit = reserve => match permit {
                Ok(permit) => {
                    permit.send(bytes);
                    Ok(written)
                }
                Err(_) => Err(closed_write_error()),
            },
        }
    }

    fn set_protocol_version(&mut self, version: MavlinkVersion) {
        self.protocol_version
            .store(Self::encode_version(version), Ordering::SeqCst);
    }

    fn protocol_version(&self) -> MavlinkVersion {
        Self::decode_version(self.protocol_version.load(Ordering::SeqCst))
    }

    fn set_allow_recv_any_version(&mut self, allow: bool) {
        self.allow_recv_any_version.store(allow, Ordering::SeqCst);
    }

    fn allow_recv_any_version(&self) -> bool {
        self.allow_recv_any_version.load(Ordering::SeqCst)
    }
}

fn candidate_frame_len(buffer: &[u8]) -> Option<usize> {
    let magic = *buffer.first()?;

    match magic {
        0xFE => {
            if buffer.len() < 2 {
                return None;
            }
            Some(6 + usize::from(buffer[1]) + 2)
        }
        0xFD => {
            if buffer.len() < 10 {
                return None;
            }

            let payload_len = usize::from(buffer[1]);
            let signed_len = if buffer[2] & 0x01 != 0 { 13 } else { 0 };
            Some(10 + payload_len + 2 + signed_len)
        }
        _ => None,
    }
}

fn closed_write_error() -> mavlink::error::MessageWriteError {
    mavlink::error::MessageWriteError::Io(io::ErrorKind::BrokenPipe.into())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dialect;
    use mavlink::{AsyncMavConnection, MavHeader, MavlinkVersion};
    use std::sync::Arc;
    use std::time::Duration;

    fn heartbeat() -> dialect::MavMessage {
        dialect::MavMessage::HEARTBEAT(dialect::HEARTBEAT_DATA::default())
    }

    fn encode_heartbeat_frame() -> Vec<u8> {
        let mut bytes = Vec::new();
        mavlink::write_versioned_msg(
            &mut bytes,
            MavlinkVersion::V2,
            MavHeader {
                sequence: 7,
                system_id: 1,
                component_id: 1,
            },
            &heartbeat(),
        )
        .expect("heartbeat frame should encode");
        bytes
    }

    #[tokio::test]
    async fn recv_raw_parses_frame_split_across_chunks() {
        let (connection, bridge) = ByteConnection::new(ByteConnectionConfig::default());
        let frame = encode_heartbeat_frame();
        bridge.push_inbound(frame[..3].to_vec()).await.unwrap();
        bridge.push_inbound(frame[3..].to_vec()).await.unwrap();

        let raw = connection.recv_raw().await.unwrap();

        assert_eq!(raw.system_id(), 1);
        assert_eq!(raw.component_id(), 1);
        assert_eq!(raw.version(), MavlinkVersion::V2);
    }

    #[tokio::test]
    async fn recv_raw_discards_garbage_before_magic() {
        let (connection, bridge) = ByteConnection::new(ByteConnectionConfig::default());
        let frame = encode_heartbeat_frame();
        bridge.push_inbound(vec![0x00, 0x01, 0x02]).await.unwrap();
        bridge.push_inbound(frame).await.unwrap();

        let raw = connection.recv_raw().await.unwrap();

        assert_eq!(raw.system_id(), 1);
        assert_eq!(raw.component_id(), 1);
    }

    #[tokio::test]
    async fn recv_parses_typed_message_from_raw_frame() {
        let (connection, bridge) = ByteConnection::new(ByteConnectionConfig::default());
        bridge.push_inbound(encode_heartbeat_frame()).await.unwrap();

        let (header, message) = connection.recv().await.unwrap();

        assert_eq!(header.system_id, 1);
        assert!(matches!(message, dialect::MavMessage::HEARTBEAT(_)));
    }

    #[tokio::test]
    async fn try_recv_returns_would_block_when_no_complete_frame_is_available() {
        let (connection, bridge) = ByteConnection::new(ByteConnectionConfig::default());
        bridge.try_push_inbound(vec![0xFD, 0x09]).unwrap();

        let err = connection.try_recv().await.unwrap_err();

        assert!(
            matches!(err, mavlink::error::MessageReadError::Io(io) if io.kind() == std::io::ErrorKind::WouldBlock)
        );
    }

    #[tokio::test]
    async fn send_emits_one_outbound_frame() {
        let (connection, bridge) = ByteConnection::new(ByteConnectionConfig::default());

        let written = connection
            .send(
                &MavHeader {
                    sequence: 0,
                    system_id: 255,
                    component_id: 190,
                },
                &heartbeat(),
            )
            .await
            .unwrap();
        let outbound = bridge.next_outbound().await.unwrap();

        assert_eq!(outbound[0], 0xFD);
        assert_eq!(written, outbound.len());
    }

    #[tokio::test]
    async fn send_uses_connection_owned_wrapping_sequence() {
        let (connection, bridge) = ByteConnection::new(ByteConnectionConfig::default());

        connection
            .send(
                &MavHeader {
                    sequence: 99,
                    system_id: 255,
                    component_id: 190,
                },
                &heartbeat(),
            )
            .await
            .unwrap();
        connection
            .send(
                &MavHeader {
                    sequence: 99,
                    system_id: 255,
                    component_id: 190,
                },
                &heartbeat(),
            )
            .await
            .unwrap();

        let first = bridge.next_outbound().await.unwrap();
        let second = bridge.next_outbound().await.unwrap();

        assert_eq!(first[4], 0);
        assert_eq!(second[4], 1);
    }

    #[tokio::test]
    async fn try_push_inbound_reports_full_and_closed_without_dropping_bytes() {
        let (_connection, bridge) = ByteConnection::new(ByteConnectionConfig {
            inbound_capacity: 1,
            outbound_capacity: 1,
        });

        bridge.try_push_inbound(vec![1]).unwrap();
        match bridge.try_push_inbound(vec![2, 3]).unwrap_err() {
            ByteBridgeError::Full(bytes) => assert_eq!(bytes, vec![2, 3]),
            err => panic!("expected Full, got {err:?}"),
        }

        bridge.close();
        match bridge.try_push_inbound(vec![4, 5]).unwrap_err() {
            ByteBridgeError::Closed(bytes) => assert_eq!(bytes, vec![4, 5]),
            err => panic!("expected Closed, got {err:?}"),
        }
    }

    #[tokio::test]
    async fn close_makes_recv_raw_eof_and_next_outbound_none() {
        let (connection, bridge) = ByteConnection::new(ByteConnectionConfig::default());

        bridge.close();

        let err = match connection.recv_raw().await {
            Ok(_) => panic!("recv_raw should return EOF after close"),
            Err(err) => err,
        };
        assert!(matches!(err, mavlink::error::MessageReadError::Io(_)));
        assert!(bridge.next_outbound().await.is_none());
    }

    #[tokio::test]
    async fn send_waits_for_outbound_capacity_then_completes() {
        let (connection, bridge) = ByteConnection::new(ByteConnectionConfig {
            inbound_capacity: 1,
            outbound_capacity: 1,
        });
        let connection = Arc::new(connection);

        connection
            .send(
                &MavHeader {
                    sequence: 0,
                    system_id: 255,
                    component_id: 190,
                },
                &heartbeat(),
            )
            .await
            .unwrap();

        let pending_connection = connection.clone();
        let send_task = tokio::spawn(async move {
            pending_connection
                .send(
                    &MavHeader {
                        sequence: 0,
                        system_id: 255,
                        component_id: 190,
                    },
                    &heartbeat(),
                )
                .await
        });

        tokio::time::sleep(Duration::from_millis(10)).await;
        let first = bridge.next_outbound().await.unwrap();
        assert_eq!(first[0], 0xFD);

        let second = tokio::time::timeout(Duration::from_millis(250), send_task)
            .await
            .expect("send task should finish after draining capacity")
            .expect("send task should not panic")
            .expect("send should succeed after capacity drains");

        assert!(second > 0);
    }

    #[tokio::test]
    async fn send_returns_error_when_closed_while_waiting_for_capacity() {
        let (connection, bridge) = ByteConnection::new(ByteConnectionConfig {
            inbound_capacity: 1,
            outbound_capacity: 1,
        });
        let connection = Arc::new(connection);

        connection
            .send(
                &MavHeader {
                    sequence: 0,
                    system_id: 255,
                    component_id: 190,
                },
                &heartbeat(),
            )
            .await
            .unwrap();

        let pending_connection = connection.clone();
        let send_task = tokio::spawn(async move {
            pending_connection
                .send(
                    &MavHeader {
                        sequence: 0,
                        system_id: 255,
                        component_id: 190,
                    },
                    &heartbeat(),
                )
                .await
        });

        tokio::time::sleep(Duration::from_millis(10)).await;
        bridge.close();

        let result = tokio::time::timeout(Duration::from_millis(250), send_task)
            .await
            .expect("send task should finish after close")
            .expect("send task should not panic");

        assert!(result.is_err());
    }
}
