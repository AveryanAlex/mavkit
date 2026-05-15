use crate::dialect;
use mavlink::{MAVLinkMessageRaw, MavHeader, MavlinkVersion, Message, ReadVersion};
use std::io;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use tokio::sync::{Mutex, OwnedSemaphorePermit, Semaphore, mpsc};
use tokio_util::sync::CancellationToken;

const DEFAULT_INBOUND_BYTE_CAPACITY: usize = 64 * 1024;
const READER_BUFFER_COMPACT_THRESHOLD: usize = 4096;

#[derive(Debug, Clone)]
/// Channel capacities for [`ByteConnection`] and [`ByteBridge`].
///
/// Both capacities are clamped to at least `1`. Inbound capacity controls how many ordered byte
/// chunks can be queued for MAVLink parsing. Outbound capacity controls how many serialized MAVLink
/// frames can wait to be drained by the bridge owner. Inbound bytes are also capped internally to
/// bound parser memory independently of chunk count.
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
    inbound_rx: mpsc::Receiver<InboundChunk>,
    buffer: Vec<u8>,
    start: usize,
    buffer_permit: Option<OwnedSemaphorePermit>,
}

struct WriterState {
    sequence: u8,
}

struct InboundChunk {
    bytes: Vec<u8>,
    permit: OwnedSemaphorePermit,
}

#[derive(Clone)]
/// Browser/app-facing bridge for [`ByteConnection`].
///
/// Inbound bytes are delivered in arrival order. Outbound frames are emitted one MAVLink frame per
/// `Vec<u8>`. Calling [`ByteBridge::close`] is immediate: future inbound writes are rejected,
/// `next_outbound` returns `None`, and the paired connection reports EOF for reads.
pub struct ByteBridge {
    inbound_tx: mpsc::Sender<InboundChunk>,
    inbound_byte_permits: Arc<Semaphore>,
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
        let inbound_byte_permits = Arc::new(Semaphore::new(DEFAULT_INBOUND_BYTE_CAPACITY));
        let close = CancellationToken::new();

        (
            Self {
                reader: Mutex::new(ReaderState {
                    inbound_rx,
                    buffer: Vec::new(),
                    start: 0,
                    buffer_permit: None,
                }),
                writer: Mutex::new(WriterState { sequence: 0 }),
                outbound_tx,
                close: close.clone(),
                protocol_version: AtomicU8::new(Self::encode_version(MavlinkVersion::V2)),
                allow_recv_any_version: AtomicBool::new(true),
            },
            ByteBridge {
                inbound_tx,
                inbound_byte_permits,
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
                .readable()
                .iter()
                .position(|byte| matches!(*byte, 0xFE | 0xFD))
            else {
                state.consume(state.readable().len());
                return Ok(None);
            };

            if magic_index > 0 {
                state.consume(magic_index);
                continue;
            }

            let readable = state.readable();
            let Some(frame_len) = candidate_frame_len(readable) else {
                return Ok(None);
            };

            if readable.len() < frame_len {
                return Ok(None);
            }

            let frame = &readable[..frame_len];
            let cursor = std::io::Cursor::new(frame);
            let mut reader = mavlink::peek_reader::PeekReader::new(cursor);

            match mavlink::read_versioned_raw_message::<dialect::MavMessage, _>(
                &mut reader,
                read_version,
            ) {
                Ok(raw) => {
                    if raw_message_bytes(&raw) == frame {
                        state.consume(frame_len);
                        return Ok(Some(raw));
                    }

                    state.consume(1);
                }
                Err(_) => {
                    state.consume(1);
                }
            }
        }
    }
}

impl ReaderState {
    fn readable(&self) -> &[u8] {
        &self.buffer[self.start..]
    }

    fn push_chunk(&mut self, chunk: InboundChunk) {
        let InboundChunk { bytes, permit } = chunk;

        if !bytes.is_empty() {
            match self.buffer_permit.as_mut() {
                Some(buffer_permit) => buffer_permit.merge(permit),
                None => self.buffer_permit = Some(permit),
            }
        }

        self.buffer.extend(bytes);
    }

    fn consume(&mut self, byte_count: usize) {
        if byte_count == 0 {
            return;
        }

        debug_assert!(byte_count <= self.readable().len());
        self.release_buffered_bytes(byte_count);
        self.start += byte_count;

        if self.start == self.buffer.len() {
            self.buffer.clear();
            self.start = 0;
            self.buffer_permit = None;
        } else if self.start >= READER_BUFFER_COMPACT_THRESHOLD {
            self.buffer.drain(..self.start);
            self.start = 0;
        }
    }

    fn release_buffered_bytes(&mut self, byte_count: usize) {
        let Some(permit) = self.buffer_permit.as_mut() else {
            return;
        };

        let released = permit
            .split(byte_count)
            .expect("reader byte permits should match buffered bytes");
        drop(released);

        if permit.num_permits() == 0 {
            self.buffer_permit = None;
        }
    }
}

impl ByteBridge {
    /// Queue inbound transport bytes, waiting for bounded capacity when necessary.
    pub async fn push_inbound(&self, bytes: Vec<u8>) -> Result<(), ByteBridgeError> {
        if bytes.len() > DEFAULT_INBOUND_BYTE_CAPACITY {
            return Err(ByteBridgeError::Full(bytes));
        }

        let reserve = self.inbound_tx.reserve();
        tokio::pin!(reserve);

        let chunk_permit = tokio::select! {
            biased;
            _ = self.close.cancelled() => return Err(ByteBridgeError::Closed(bytes)),
            permit = &mut reserve => match permit {
                Ok(permit) => permit,
                Err(_) => return Err(ByteBridgeError::Closed(bytes)),
            },
        };

        let byte_permits = self
            .inbound_byte_permits
            .clone()
            .acquire_many_owned(bytes.len() as u32);
        tokio::pin!(byte_permits);

        let byte_permit = tokio::select! {
            biased;
            _ = self.close.cancelled() => return Err(ByteBridgeError::Closed(bytes)),
            permit = &mut byte_permits => match permit {
                Ok(permit) => permit,
                Err(_) => return Err(ByteBridgeError::Closed(bytes)),
            },
        };

        if self.close.is_cancelled() {
            return Err(ByteBridgeError::Closed(bytes));
        }

        chunk_permit.send(InboundChunk {
            bytes,
            permit: byte_permit,
        });
        Ok(())
    }

    /// Try to queue inbound transport bytes without waiting.
    pub fn try_push_inbound(&self, bytes: Vec<u8>) -> Result<(), ByteBridgeError> {
        if self.close.is_cancelled() {
            return Err(ByteBridgeError::Closed(bytes));
        }

        if bytes.len() > DEFAULT_INBOUND_BYTE_CAPACITY {
            return Err(ByteBridgeError::Full(bytes));
        }

        let permit = match self
            .inbound_byte_permits
            .clone()
            .try_acquire_many_owned(bytes.len() as u32)
        {
            Ok(permit) => permit,
            Err(_) => return Err(ByteBridgeError::Full(bytes)),
        };

        match self.inbound_tx.try_send(InboundChunk { bytes, permit }) {
            Ok(()) => Ok(()),
            Err(tokio::sync::mpsc::error::TrySendError::Full(chunk)) => {
                Err(ByteBridgeError::Full(chunk.bytes))
            }
            Err(tokio::sync::mpsc::error::TrySendError::Closed(chunk)) => {
                Err(ByteBridgeError::Closed(chunk.bytes))
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
                chunk = reader.inbound_rx.recv() => match chunk {
                    Some(chunk) => reader.push_chunk(chunk),
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
                Ok(chunk) => reader.push_chunk(chunk),
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
        let protocol_version = Self::decode_version(self.protocol_version.load(Ordering::SeqCst));

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
                    writer.sequence = writer.sequence.wrapping_add(1);
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

fn raw_message_bytes(raw: &MAVLinkMessageRaw) -> &[u8] {
    match raw {
        MAVLinkMessageRaw::V1(message) => message.raw_bytes(),
        MAVLinkMessageRaw::V2(message) => message.raw_bytes(),
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
        encode_heartbeat_frame_with_sequence(7)
    }

    fn encode_heartbeat_frame_with_sequence(sequence: u8) -> Vec<u8> {
        let mut bytes = Vec::new();
        mavlink::write_versioned_msg(
            &mut bytes,
            MavlinkVersion::V2,
            MavHeader {
                sequence,
                system_id: 1,
                component_id: 1,
            },
            &heartbeat(),
        )
        .expect("heartbeat frame should encode");
        bytes
    }

    fn outbound_sequence(frame: &[u8]) -> u8 {
        frame[4]
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
    async fn recv_raw_keeps_later_frames_after_corrupt_false_magic_candidate() {
        let (connection, bridge) = ByteConnection::new(ByteConnectionConfig::default());
        let first = encode_heartbeat_frame_with_sequence(7);
        let second = encode_heartbeat_frame_with_sequence(8);
        let false_payload_len = first.len() + second.len();
        let false_candidate_len = 10 + false_payload_len + 2;
        let mut bytes = vec![0xFD, false_payload_len as u8, 0, 0, 0, 0, 0, 0, 0, 0];
        bytes.extend(first);
        bytes.extend(second);
        bytes.resize(false_candidate_len, 0);

        bridge.push_inbound(bytes).await.unwrap();

        let first = connection.recv_raw().await.unwrap();
        let second = connection.recv_raw().await.unwrap();

        assert_eq!(first.sequence(), 7);
        assert_eq!(second.sequence(), 8);
    }

    #[test]
    fn parser_advances_offset_instead_of_draining_front_for_garbage() {
        let (_tx, inbound_rx) = mpsc::channel(1);
        let mut buffer = vec![0xFD, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        buffer.extend([0x00; 32]);
        buffer.extend([0xFD, 250]);
        let initial_len = buffer.len();
        let mut state = ReaderState {
            inbound_rx,
            buffer,
            start: 0,
            buffer_permit: None,
        };

        let extracted = ByteConnection::try_extract_frame(&mut state, ReadVersion::Any).unwrap();

        assert!(extracted.is_none());
        assert!(state.start > 0);
        assert_eq!(state.buffer.len(), initial_len);
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

        assert_eq!(outbound_sequence(&first), 0);
        assert_eq!(outbound_sequence(&second), 1);
    }

    #[tokio::test]
    async fn concurrent_sends_emit_frames_in_sequence_order() {
        let (connection, bridge) = ByteConnection::new(ByteConnectionConfig {
            inbound_capacity: 1,
            outbound_capacity: 64,
        });
        let connection = Arc::new(connection);
        let mut tasks = Vec::new();

        for _ in 0..32 {
            let connection = connection.clone();
            tasks.push(tokio::spawn(async move {
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
            }));
        }

        for task in tasks {
            task.await
                .expect("send task should not panic")
                .expect("send should succeed");
        }

        for expected in 0..32 {
            let outbound = bridge.next_outbound().await.unwrap();
            assert_eq!(outbound_sequence(&outbound), expected);
        }
    }

    #[tokio::test]
    async fn try_push_inbound_applies_byte_capacity_independently_of_chunk_capacity() {
        let (_connection, bridge) = ByteConnection::new(ByteConnectionConfig {
            inbound_capacity: 2,
            outbound_capacity: 1,
        });

        bridge
            .try_push_inbound(vec![0; DEFAULT_INBOUND_BYTE_CAPACITY])
            .unwrap();
        match bridge.try_push_inbound(vec![1]).unwrap_err() {
            ByteBridgeError::Full(bytes) => assert_eq!(bytes, vec![1]),
            err => panic!("expected Full, got {err:?}"),
        }

        match bridge
            .try_push_inbound(vec![2; DEFAULT_INBOUND_BYTE_CAPACITY + 1])
            .unwrap_err()
        {
            ByteBridgeError::Full(bytes) => {
                assert_eq!(bytes.len(), DEFAULT_INBOUND_BYTE_CAPACITY + 1)
            }
            err => panic!("expected Full, got {err:?}"),
        }
    }

    #[tokio::test]
    async fn push_inbound_waits_for_byte_capacity_until_parser_releases_bytes() {
        let (connection, bridge) = ByteConnection::new(ByteConnectionConfig {
            inbound_capacity: 2,
            outbound_capacity: 1,
        });

        bridge
            .push_inbound(vec![0; DEFAULT_INBOUND_BYTE_CAPACITY])
            .await
            .unwrap();

        let pending_bridge = bridge.clone();
        let push_task = tokio::spawn(async move { pending_bridge.push_inbound(vec![1]).await });

        tokio::time::sleep(Duration::from_millis(10)).await;
        assert!(!push_task.is_finished());

        let err = connection.try_recv().await.unwrap_err();
        assert!(
            matches!(err, mavlink::error::MessageReadError::Io(io) if io.kind() == std::io::ErrorKind::WouldBlock)
        );

        tokio::time::timeout(Duration::from_millis(250), push_task)
            .await
            .expect("pending push should finish after parser releases bytes")
            .expect("push task should not panic")
            .expect("push should succeed after byte capacity is released");
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

    #[tokio::test]
    async fn aborted_send_waiting_for_capacity_does_not_consume_sequence() {
        let (connection, bridge) = ByteConnection::new(ByteConnectionConfig {
            inbound_capacity: 1,
            outbound_capacity: 1,
        });
        let connection = Arc::new(connection);

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

        let pending_connection = connection.clone();
        let send_task = tokio::spawn(async move {
            pending_connection
                .send(
                    &MavHeader {
                        sequence: 99,
                        system_id: 255,
                        component_id: 190,
                    },
                    &heartbeat(),
                )
                .await
        });

        tokio::time::sleep(Duration::from_millis(10)).await;
        send_task.abort();
        assert!(send_task.await.unwrap_err().is_cancelled());

        let first = bridge.next_outbound().await.unwrap();
        assert_eq!(outbound_sequence(&first), 0);

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
        let second = bridge.next_outbound().await.unwrap();

        assert_eq!(outbound_sequence(&second), 1);
    }
}
