use core::ops::DerefMut;
use futures::lock::Mutex;
use mavlink::async_peek_reader::AsyncPeekReader;
use mavlink::common;
use mavlink::{
    read_versioned_msg_async, read_versioned_raw_message_async, write_versioned_msg_async,
    MAVLinkMessageRaw, MavHeader, MavlinkVersion, ReadVersion,
};
use tokio::io::{AsyncRead, AsyncWrite};

/// Implements [`AsyncMavConnection`] over any `AsyncRead + AsyncWrite` pair.
///
/// This allows mavkit to communicate via transports that aren't built into the
/// mavlink crate (e.g. BLE UART, Android Classic SPP) by wrapping raw byte
/// streams into the connection interface the event loop expects.
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
}

#[async_trait::async_trait]
impl<R: AsyncRead + Unpin + Send, W: AsyncWrite + Unpin + Send>
    mavlink::AsyncMavConnection<common::MavMessage> for StreamConnection<R, W>
{
    async fn recv(
        &self,
    ) -> Result<(MavHeader, common::MavMessage), mavlink::error::MessageReadError> {
        let mut reader = self.reader.lock().await;
        let version = if self.recv_any_version {
            ReadVersion::Any
        } else {
            ReadVersion::Single(self.protocol_version)
        };
        read_versioned_msg_async(reader.deref_mut(), version).await
    }

    async fn recv_raw(&self) -> Result<MAVLinkMessageRaw, mavlink::error::MessageReadError> {
        let mut reader = self.reader.lock().await;
        let version = if self.recv_any_version {
            ReadVersion::Any
        } else {
            ReadVersion::Single(self.protocol_version)
        };
        read_versioned_raw_message_async::<common::MavMessage, _>(reader.deref_mut(), version)
            .await
    }

    async fn send(
        &self,
        header: &MavHeader,
        data: &common::MavMessage,
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

#[cfg(test)]
mod tests {
    use super::*;
    use mavlink::AsyncMavConnection;
    use tokio::io::duplex;

    #[tokio::test]
    async fn stream_connection_is_send_sync() {
        let (client, _server) = duplex(1024);
        let (read, write) = tokio::io::split(client);
        let conn = StreamConnection::new(read, write);
        // Verify the connection satisfies Send + Sync (required by event loop)
        fn assert_send_sync<T: Send + Sync>(_: &T) {}
        assert_send_sync(&conn);
        // Verify it satisfies AsyncMavConnection
        let _boxed: Box<dyn AsyncMavConnection<common::MavMessage> + Send + Sync> =
            Box::new(conn);
    }
}
