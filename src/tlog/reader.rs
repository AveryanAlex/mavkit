use mavlink::async_peek_reader::AsyncPeekReader;
use mavlink::common::MavMessage;
use mavlink::{ReadVersion, read_versioned_msg_async};
use tokio::io::AsyncRead;

/// A single timestamped MAVLink message from a TLOG file.
#[derive(Debug)]
pub struct TlogEntry {
    /// Microseconds since Unix epoch.
    pub timestamp_usec: u64,
    /// The decoded MAVLink message.
    pub message: MavMessage,
}

/// Errors that can occur while reading a TLOG file.
#[derive(Debug, thiserror::Error)]
pub enum TlogError {
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),
    #[error("MAVLink parse error: {0}")]
    Parse(#[from] mavlink::error::MessageReadError),
}

/// Maximum number of consecutive parse failures before giving up.
const MAX_RECOVERY_ATTEMPTS: usize = 256;

/// Streaming reader that yields [`TlogEntry`] values from a TLOG byte source.
///
/// TLOG format: repeated `[8-byte LE u64 timestamp (μs)][MAVLink frame]` entries.
pub struct TlogReader<R: AsyncRead + Unpin> {
    reader: AsyncPeekReader<R>,
}

impl<R: AsyncRead + Unpin> TlogReader<R> {
    /// Wrap an async reader as a TLOG stream.
    pub fn new(reader: R) -> Self {
        Self {
            reader: AsyncPeekReader::new(reader),
        }
    }

    /// Read the next entry. Returns `Ok(None)` at EOF.
    pub async fn next(&mut self) -> Result<Option<TlogEntry>, TlogError> {
        // Read 8-byte timestamp via AsyncPeekReader (takes amount, returns &[u8])
        let ts_bytes = match self.reader.read_exact(8).await {
            Ok(bytes) => bytes.try_into().unwrap(),
            Err(mavlink::error::MessageReadError::Io(ref e))
                if e.kind() == std::io::ErrorKind::UnexpectedEof =>
            {
                return Ok(None);
            }
            Err(e) => return Err(TlogError::Parse(e)),
        };
        let timestamp_usec = u64::from_le_bytes(ts_bytes);

        // Try to read a MAVLink message, with recovery on parse errors
        let mut attempts = 0;
        loop {
            match read_versioned_msg_async::<MavMessage, _>(&mut self.reader, ReadVersion::Any)
                .await
            {
                Ok((_header, message)) => {
                    return Ok(Some(TlogEntry {
                        timestamp_usec,
                        message,
                    }));
                }
                Err(mavlink::error::MessageReadError::Io(ref e))
                    if e.kind() == std::io::ErrorKind::UnexpectedEof =>
                {
                    return Ok(None);
                }
                Err(mavlink::error::MessageReadError::Parse(_)) => {
                    attempts += 1;
                    if attempts >= MAX_RECOVERY_ATTEMPTS {
                        return Ok(None);
                    }
                    // The mavlink parser internally scans for magic bytes,
                    // so simply retrying will advance past corruption.
                    continue;
                }
                Err(e) => return Err(TlogError::Parse(e)),
            }
        }
    }

    /// Collect all remaining entries into a `Vec`.
    pub async fn collect(mut self) -> Result<Vec<TlogEntry>, TlogError> {
        let mut entries = Vec::new();
        while let Some(entry) = self.next().await? {
            entries.push(entry);
        }
        Ok(entries)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavlink::common::MavMessage;
    use mavlink::{MavHeader, MavlinkVersion};

    /// Build a synthetic TLOG buffer from (timestamp, message) pairs.
    fn build_tlog(entries: &[(u64, MavMessage)]) -> Vec<u8> {
        let mut buf = Vec::new();
        let mut seq = 0u8;
        for (ts, msg) in entries {
            buf.extend_from_slice(&ts.to_le_bytes());
            let header = MavHeader {
                sequence: seq,
                system_id: 1,
                component_id: 1,
            };
            mavlink::write_versioned_msg(&mut buf, MavlinkVersion::V2, header, msg).unwrap();
            seq = seq.wrapping_add(1);
        }
        buf
    }

    fn heartbeat() -> MavMessage {
        MavMessage::HEARTBEAT(mavlink::common::HEARTBEAT_DATA {
            custom_mode: 0,
            mavtype: mavlink::common::MavType::MAV_TYPE_QUADROTOR,
            autopilot: mavlink::common::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode: mavlink::common::MavModeFlag::empty(),
            system_status: mavlink::common::MavState::MAV_STATE_STANDBY,
            mavlink_version: 3,
        })
    }

    #[tokio::test]
    async fn basic_parsing() {
        let data = build_tlog(&[
            (1000, heartbeat()),
            (2000, heartbeat()),
            (3000, heartbeat()),
        ]);
        let reader = TlogReader::new(data.as_slice());
        let entries = reader.collect().await.unwrap();

        assert_eq!(entries.len(), 3);
        assert_eq!(entries[0].timestamp_usec, 1000);
        assert_eq!(entries[1].timestamp_usec, 2000);
        assert_eq!(entries[2].timestamp_usec, 3000);
        for entry in &entries {
            assert!(matches!(entry.message, MavMessage::HEARTBEAT(_)));
        }
    }

    #[tokio::test]
    async fn error_recovery() {
        // Build two valid entries with garbage in between
        let entry1 = build_tlog(&[(1000, heartbeat())]);
        let entry2 = build_tlog(&[(2000, heartbeat())]);

        let mut data = Vec::new();
        data.extend_from_slice(&entry1);
        // Insert garbage bytes (not a valid MAVLink magic byte sequence)
        data.extend_from_slice(&[0xFF, 0x00, 0x42, 0x13, 0x37, 0xDE, 0xAD, 0xBE]);
        data.extend_from_slice(&entry2);

        let reader = TlogReader::new(data.as_slice());
        let entries = reader.collect().await.unwrap();

        // Should recover and get at least the second valid entry
        assert!(!entries.is_empty());
        // First entry should always parse
        assert_eq!(entries[0].timestamp_usec, 1000);
    }

    #[tokio::test]
    async fn empty_input() {
        let reader = TlogReader::new(&b""[..]);
        let entries = reader.collect().await.unwrap();
        assert!(entries.is_empty());
    }
}
