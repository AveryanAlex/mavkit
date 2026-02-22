use std::io;
use std::pin::Pin;
use std::task::{Context, Poll};
use tokio::io::{AsyncRead, AsyncWrite, ReadBuf};
use tokio::sync::mpsc;

/// An [`AsyncRead`] adapter backed by an mpsc channel of byte buffers.
///
/// BLE notification callbacks push `Vec<u8>` chunks into the sender side;
/// the reader yields them in order, buffering partial reads.
pub struct ChannelReader {
    rx: mpsc::Receiver<Vec<u8>>,
    buf: Vec<u8>,
    pos: usize,
}

impl ChannelReader {
    pub fn new(rx: mpsc::Receiver<Vec<u8>>) -> Self {
        Self {
            rx,
            buf: Vec::new(),
            pos: 0,
        }
    }
}

impl AsyncRead for ChannelReader {
    fn poll_read(
        mut self: Pin<&mut Self>,
        cx: &mut Context<'_>,
        buf: &mut ReadBuf<'_>,
    ) -> Poll<io::Result<()>> {
        // If we have leftover bytes from a previous chunk, yield those first
        if self.pos < self.buf.len() {
            let remaining = &self.buf[self.pos..];
            let to_copy = remaining.len().min(buf.remaining());
            buf.put_slice(&remaining[..to_copy]);
            self.pos += to_copy;
            if self.pos >= self.buf.len() {
                self.buf.clear();
                self.pos = 0;
            }
            return Poll::Ready(Ok(()));
        }

        // Wait for the next chunk from the channel
        match self.rx.poll_recv(cx) {
            Poll::Ready(Some(chunk)) => {
                let to_copy = chunk.len().min(buf.remaining());
                buf.put_slice(&chunk[..to_copy]);
                if to_copy < chunk.len() {
                    // Buffer the remainder
                    self.buf = chunk;
                    self.pos = to_copy;
                }
                Poll::Ready(Ok(()))
            }
            Poll::Ready(None) => {
                // Channel closed → EOF
                Poll::Ready(Ok(()))
            }
            Poll::Pending => Poll::Pending,
        }
    }
}

/// An [`AsyncWrite`] adapter that forwards bytes via an mpsc channel.
///
/// Each `write` call sends the data as a `Vec<u8>` to the receiver side,
/// which is responsible for chunking to MTU and transmitting over BLE/SPP.
pub struct ChannelWriter {
    tx: mpsc::Sender<Vec<u8>>,
}

impl ChannelWriter {
    pub fn new(tx: mpsc::Sender<Vec<u8>>) -> Self {
        Self { tx }
    }
}

impl AsyncWrite for ChannelWriter {
    fn poll_write(
        self: Pin<&mut Self>,
        _cx: &mut Context<'_>,
        buf: &[u8],
    ) -> Poll<io::Result<usize>> {
        if buf.is_empty() {
            return Poll::Ready(Ok(0));
        }
        let data = buf.to_vec();
        let len = data.len();
        match self.tx.try_send(data) {
            Ok(()) => Poll::Ready(Ok(len)),
            Err(mpsc::error::TrySendError::Full(_)) => {
                // Channel is full — signal the caller to retry.
                // Wake immediately since capacity may free up soon.
                _cx.waker().wake_by_ref();
                Poll::Pending
            }
            Err(mpsc::error::TrySendError::Closed(_)) => Poll::Ready(Err(io::Error::new(
                io::ErrorKind::BrokenPipe,
                "channel closed",
            ))),
        }
    }

    fn poll_flush(self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<io::Result<()>> {
        Poll::Ready(Ok(()))
    }

    fn poll_shutdown(self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<io::Result<()>> {
        Poll::Ready(Ok(()))
    }
}

/// Create a pair of (reader, writer) channel adapters for bridging byte-oriented
/// transports (BLE, SPP) into `AsyncRead + AsyncWrite`.
///
/// Returns:
/// - `ChannelReader` - reads bytes pushed by the transport's incoming callback
/// - `ChannelWriter` - writes bytes that should be sent to the transport
/// - `mpsc::Sender<Vec<u8>>` - push incoming bytes from BLE notifications / SPP reads
/// - `mpsc::Receiver<Vec<u8>>` - consume outgoing bytes to send over BLE / SPP
pub fn channel_pair(
    buffer_size: usize,
) -> (
    ChannelReader,
    ChannelWriter,
    mpsc::Sender<Vec<u8>>,
    mpsc::Receiver<Vec<u8>>,
) {
    let (incoming_tx, incoming_rx) = mpsc::channel(buffer_size);
    let (outgoing_tx, outgoing_rx) = mpsc::channel(buffer_size);
    (
        ChannelReader::new(incoming_rx),
        ChannelWriter::new(outgoing_tx),
        incoming_tx,
        outgoing_rx,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use tokio::io::{AsyncReadExt, AsyncWriteExt};

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
        // EOF
        let n = reader.read(&mut buf).await.unwrap();
        assert_eq!(n, 0);
    }

    #[tokio::test]
    async fn channel_reader_buffers_partial_reads() {
        let (tx, rx) = mpsc::channel(8);
        let mut reader = ChannelReader::new(rx);

        tx.send(b"abcdef".to_vec()).await.unwrap();
        drop(tx);

        // Read only 3 bytes at a time
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
    async fn channel_pair_roundtrip() {
        let (reader, writer, incoming_tx, mut outgoing_rx) = channel_pair(8);
        let mut reader = reader;
        let mut writer = writer;

        // Simulate incoming BLE data
        incoming_tx.send(b"from device".to_vec()).await.unwrap();

        let mut buf = vec![0u8; 32];
        let n = reader.read(&mut buf).await.unwrap();
        assert_eq!(&buf[..n], b"from device");

        // Simulate outgoing data
        writer.write_all(b"to device").await.unwrap();
        let chunk = outgoing_rx.recv().await.unwrap();
        assert_eq!(chunk, b"to device");
    }
}
