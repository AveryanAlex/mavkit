//! Async stream utilities for transports that do not have a native MAVLink connection.
//!
//! [`StreamConnection`] adapts any `AsyncRead + AsyncWrite` pair into
//! [`mavlink::AsyncMavConnection`]. [`ChannelBridge`] helps callback-driven
//! transports feed and drain those stream halves through channels.

use crate::dialect;
use core::ops::DerefMut;
use futures::FutureExt;
use futures::lock::Mutex;
use mavlink::async_peek_reader::AsyncPeekReader;
use mavlink::{
    MAVLinkMessageRaw, MavHeader, MavlinkVersion, ReadVersion, read_versioned_msg_async,
    read_versioned_raw_message_async, write_versioned_msg_async,
};
use std::io;
use std::pin::Pin;
use std::task::{Context, Poll, ready};
use tokio::io::{AsyncRead, AsyncWrite, ReadBuf};
use tokio::sync::mpsc;
use tokio_util::sync::PollSender;

/// Implements [`mavlink::AsyncMavConnection`] over any `AsyncRead + AsyncWrite` pair.
///
/// This lets mavkit communicate over transports that only expose raw byte streams,
/// such as BLE UART, Android Classic SPP, or custom framing layers.
pub struct StreamConnection<R: AsyncRead + Unpin + Send, W: AsyncWrite + Unpin + Send> {
    reader: Mutex<AsyncPeekReader<R>>,
    writer: Mutex<StreamWriter<W>>,
    protocol_version: MavlinkVersion,
    recv_any_version: bool,
}

struct StreamWriter<W> {
    inner: W,
    sequence: u8,
}

impl<R: AsyncRead + Unpin + Send, W: AsyncWrite + Unpin + Send> StreamConnection<R, W> {
    /// Create a new `StreamConnection` from an async reader and writer.
    pub fn new(reader: R, writer: W) -> Self {
        Self {
            reader: Mutex::new(AsyncPeekReader::new(reader)),
            writer: Mutex::new(StreamWriter {
                inner: writer,
                sequence: 0,
            }),
            protocol_version: MavlinkVersion::V2,
            recv_any_version: true,
        }
    }

    fn read_version(&self) -> ReadVersion {
        if self.recv_any_version {
            ReadVersion::Any
        } else {
            ReadVersion::Single(self.protocol_version)
        }
    }
}

#[async_trait::async_trait]
impl<R: AsyncRead + Unpin + Send, W: AsyncWrite + Unpin + Send>
    mavlink::AsyncMavConnection<dialect::MavMessage> for StreamConnection<R, W>
{
    async fn recv(
        &self,
    ) -> Result<(MavHeader, dialect::MavMessage), mavlink::error::MessageReadError> {
        let version = self.read_version();
        let mut reader = self.reader.lock().await;
        read_versioned_msg_async(reader.deref_mut(), version).await
    }

    async fn recv_raw(&self) -> Result<MAVLinkMessageRaw, mavlink::error::MessageReadError> {
        let version = self.read_version();
        let mut reader = self.reader.lock().await;
        read_versioned_raw_message_async::<dialect::MavMessage, _>(reader.deref_mut(), version)
            .await
    }

    async fn try_recv(
        &self,
    ) -> Result<(MavHeader, dialect::MavMessage), mavlink::error::MessageReadError> {
        match self.recv().now_or_never() {
            Some(result) => result,
            None => Err(mavlink::error::MessageReadError::Io(
                io::ErrorKind::WouldBlock.into(),
            )),
        }
    }

    async fn send(
        &self,
        header: &MavHeader,
        data: &dialect::MavMessage,
    ) -> Result<usize, mavlink::error::MessageWriteError> {
        let mut lock = self.writer.lock().await;
        let header = MavHeader {
            sequence: lock.sequence,
            system_id: header.system_id,
            component_id: header.component_id,
        };
        lock.sequence = lock.sequence.wrapping_add(1);
        write_versioned_msg_async(&mut lock.inner, self.protocol_version, header, data).await
    }

    fn set_protocol_version(&mut self, version: MavlinkVersion) {
        self.protocol_version = version;
    }

    fn protocol_version(&self) -> MavlinkVersion {
        self.protocol_version
    }

    fn set_allow_recv_any_version(&mut self, allow: bool) {
        self.recv_any_version = allow;
    }

    fn allow_recv_any_version(&self) -> bool {
        self.recv_any_version
    }
}

/// Bridges callback-style byte transports into async stream halves.
///
/// - Push transport input into [`Self::incoming_tx`]
/// - Read transport output from [`Self::outgoing_rx`]
/// - Pass [`Self::reader`] and [`Self::writer`] into [`StreamConnection`] or
///   [`crate::Vehicle::from_stream_parts`]
///
/// ```ignore
/// let mavkit::stream::ChannelBridge {
///     reader,
///     writer,
///     incoming_tx,
///     mut outgoing_rx,
/// } = mavkit::stream::ChannelBridge::new(32);
///
/// let vehicle = mavkit::Vehicle::from_stream_parts(reader, writer, config).await?;
/// # let _ = (&incoming_tx, &mut outgoing_rx, vehicle);
/// ```
pub struct ChannelBridge {
    /// Feed this into mavkit so transport callbacks can surface inbound bytes as `AsyncRead`.
    pub reader: ChannelReader,
    /// Feed this into mavkit so queued outbound bytes can be written as `AsyncWrite`.
    pub writer: ChannelWriter,
    /// Send received transport chunks here in the order they arrive.
    pub incoming_tx: mpsc::Sender<Vec<u8>>,
    /// Drain queued outbound chunks here and forward them to the transport.
    pub outgoing_rx: mpsc::Receiver<Vec<u8>>,
}

impl ChannelBridge {
    /// Create a new channel-backed byte-stream bridge.
    pub fn new(buffer_size: usize) -> Self {
        let (incoming_tx, incoming_rx) = mpsc::channel(buffer_size);
        let (outgoing_tx, outgoing_rx) = mpsc::channel(buffer_size);

        Self {
            reader: ChannelReader::new(incoming_rx),
            writer: ChannelWriter::new(outgoing_tx),
            incoming_tx,
            outgoing_rx,
        }
    }
}

/// An [`AsyncRead`] adapter backed by queued byte chunks.
pub struct ChannelReader {
    rx: mpsc::Receiver<Vec<u8>>,
    buffer: Vec<u8>,
    cursor: usize,
}

impl ChannelReader {
    /// Wrap a receiver of ordered byte chunks as [`AsyncRead`].
    pub fn new(rx: mpsc::Receiver<Vec<u8>>) -> Self {
        Self {
            rx,
            buffer: Vec::new(),
            cursor: 0,
        }
    }

    fn read_buffered(&mut self, dst: &mut ReadBuf<'_>) -> bool {
        if self.cursor >= self.buffer.len() {
            return false;
        }

        self.cursor += copy_into_read_buf(&self.buffer[self.cursor..], dst);
        if self.cursor == self.buffer.len() {
            self.buffer.clear();
            self.cursor = 0;
        }

        true
    }
}

impl AsyncRead for ChannelReader {
    fn poll_read(
        mut self: Pin<&mut Self>,
        cx: &mut Context<'_>,
        buf: &mut ReadBuf<'_>,
    ) -> Poll<io::Result<()>> {
        if buf.remaining() == 0 || self.read_buffered(buf) {
            return Poll::Ready(Ok(()));
        }

        let Some(chunk) = ready!(self.rx.poll_recv(cx)) else {
            return Poll::Ready(Ok(()));
        };

        let copied = copy_into_read_buf(&chunk, buf);
        if copied < chunk.len() {
            self.buffer = chunk;
            self.cursor = copied;
        }

        Poll::Ready(Ok(()))
    }
}

/// An [`AsyncWrite`] adapter backed by an mpsc channel.
///
/// Each `write` call becomes one queued byte chunk. `flush` and `shutdown` are
/// no-ops because queueing into the channel is the only completion boundary this
/// adapter can observe.
pub struct ChannelWriter {
    tx: PollSender<Vec<u8>>,
}

impl ChannelWriter {
    /// Wrap a sender as [`AsyncWrite`] while preserving channel backpressure.
    pub fn new(tx: mpsc::Sender<Vec<u8>>) -> Self {
        Self {
            tx: PollSender::new(tx),
        }
    }
}

impl AsyncWrite for ChannelWriter {
    fn poll_write(
        self: Pin<&mut Self>,
        cx: &mut Context<'_>,
        buf: &[u8],
    ) -> Poll<io::Result<usize>> {
        if buf.is_empty() {
            return Poll::Ready(Ok(0));
        }

        let this = self.get_mut();
        match ready!(this.tx.poll_reserve(cx)) {
            Ok(()) => {}
            Err(_) => return Poll::Ready(Err(channel_closed())),
        }

        let len = buf.len();
        Poll::Ready(
            this.tx
                .send_item(buf.to_vec())
                .map(|()| len)
                .map_err(|_| channel_closed()),
        )
    }

    fn poll_flush(self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<io::Result<()>> {
        Poll::Ready(Ok(()))
    }

    fn poll_shutdown(self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<io::Result<()>> {
        Poll::Ready(Ok(()))
    }
}

fn copy_into_read_buf(src: &[u8], dst: &mut ReadBuf<'_>) -> usize {
    let copied = src.len().min(dst.remaining());
    dst.put_slice(&src[..copied]);
    copied
}

fn channel_closed() -> io::Error {
    io::Error::new(io::ErrorKind::BrokenPipe, "channel closed")
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavlink::AsyncMavConnection;
    use tokio::io::{AsyncReadExt, AsyncWriteExt, duplex};

    #[tokio::test]
    async fn stream_connection_is_send_sync() {
        let (client, _server) = duplex(1024);
        let (read, write) = tokio::io::split(client);
        let conn = StreamConnection::new(read, write);

        fn assert_send_sync<T: Send + Sync>(_: &T) {}

        assert_send_sync(&conn);
        let _boxed: Box<dyn AsyncMavConnection<dialect::MavMessage> + Send + Sync> = Box::new(conn);
    }

    #[tokio::test]
    async fn channel_reader_yields_chunks_in_order() {
        let (tx, rx) = mpsc::channel(8);
        let mut reader = ChannelReader::new(rx);

        tx.send(b"hello".to_vec()).await.unwrap();
        tx.send(b" world".to_vec()).await.unwrap();
        drop(tx);

        let mut buf = vec![0u8; 32];
        let n = reader.read(&mut buf).await.unwrap();
        assert_eq!(&buf[..n], b"hello");

        let n = reader.read(&mut buf).await.unwrap();
        assert_eq!(&buf[..n], b" world");

        let n = reader.read(&mut buf).await.unwrap();
        assert_eq!(n, 0);
    }

    #[tokio::test]
    async fn channel_reader_buffers_partial_reads() {
        let (tx, rx) = mpsc::channel(8);
        let mut reader = ChannelReader::new(rx);

        tx.send(b"abcdef".to_vec()).await.unwrap();
        drop(tx);

        let mut buf = [0u8; 3];
        let n = reader.read(&mut buf).await.unwrap();
        assert_eq!(&buf[..n], b"abc");

        let n = reader.read(&mut buf).await.unwrap();
        assert_eq!(&buf[..n], b"def");
    }

    #[tokio::test]
    async fn channel_writer_sends_data() {
        let (tx, mut rx) = mpsc::channel(8);
        let mut writer = ChannelWriter::new(tx);

        writer.write_all(b"test data").await.unwrap();
        let chunk = rx.recv().await.unwrap();
        assert_eq!(chunk, b"test data");
    }

    #[tokio::test]
    async fn channel_bridge_roundtrip() {
        let ChannelBridge {
            reader,
            writer,
            incoming_tx,
            mut outgoing_rx,
        } = ChannelBridge::new(8);
        let mut reader = reader;
        let mut writer = writer;

        incoming_tx.send(b"from device".to_vec()).await.unwrap();

        let mut buf = vec![0u8; 32];
        let n = reader.read(&mut buf).await.unwrap();
        assert_eq!(&buf[..n], b"from device");

        writer.write_all(b"to device").await.unwrap();
        let chunk = outgoing_rx.recv().await.unwrap();
        assert_eq!(chunk, b"to device");
    }
}
