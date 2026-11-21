use std::io::Write;
use std::time::{SystemTime, UNIX_EPOCH};

use crate::dialect::MavMessage;
use mavlink::{MavHeader, MavlinkVersion};

use super::reader::TlogError;

/// Synchronous writer that produces TLOG-formatted output.
///
/// TLOG format: repeated `[8-byte big-endian u64 timestamp (μs)][MAVLink frame]` entries.
///
/// # Example
///
/// ```no_run
/// use mavkit::tlog::TlogWriter;
/// use mavlink::MavlinkVersion;
/// use std::io::BufWriter;
///
/// let file = std::fs::File::create("recording.tlog").unwrap();
/// let mut writer = TlogWriter::new(BufWriter::new(file), MavlinkVersion::V2);
/// // ... write entries with writer.write_now(&header, &msg) ...
/// writer.flush().unwrap();
/// ```
pub struct TlogWriter<W: Write> {
    writer: mavlink_tlog::TlogWriter<W>,
    version: MavlinkVersion,
}

impl<W: Write> TlogWriter<W> {
    /// Create a new TLOG writer with the given MAVLink version.
    pub fn new(writer: W, version: MavlinkVersion) -> Self {
        Self {
            writer: mavlink_tlog::TlogWriter::new(writer),
            version,
        }
    }

    /// Write a TLOG entry with an explicit timestamp.
    ///
    /// Returns the number of bytes written (8-byte timestamp + MAVLink frame).
    pub fn write_entry(
        &mut self,
        timestamp_usec: u64,
        header: &MavHeader,
        msg: &MavMessage,
    ) -> Result<usize, TlogError> {
        let mut frame = Vec::with_capacity(280);
        let msg_len = mavlink::write_versioned_msg(&mut frame, self.version, *header, msg)
            .map_err(|e| match e {
                mavlink::error::MessageWriteError::Io(io) => TlogError::Io(io),
                other => TlogError::Io(std::io::Error::other(other.to_string())),
            })?;
        self.writer.write_frame(timestamp_usec, &frame)?;
        Ok(mavlink_tlog::TIMESTAMP_LEN + msg_len)
    }

    /// Write a TLOG entry timestamped with the current system time.
    pub fn write_now(&mut self, header: &MavHeader, msg: &MavMessage) -> Result<usize, TlogError> {
        let timestamp_usec = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .expect("system time before Unix epoch")
            .as_micros() as u64;
        self.write_entry(timestamp_usec, header, msg)
    }

    /// Flush the underlying writer.
    pub fn flush(&mut self) -> Result<(), TlogError> {
        self.writer.flush()?;
        Ok(())
    }

    /// Consume the writer and return the inner writer.
    pub fn into_inner(self) -> W {
        self.writer.into_inner()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dialect;
    use crate::tlog::{TlogFile, TlogReader};

    fn heartbeat() -> MavMessage {
        MavMessage::HEARTBEAT(dialect::HEARTBEAT_DATA {
            custom_mode: 0,
            mavtype: dialect::MavType::MAV_TYPE_QUADROTOR,
            autopilot: dialect::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode: dialect::MavModeFlag::empty(),
            system_status: dialect::MavState::MAV_STATE_STANDBY,
            mavlink_version: 3,
        })
    }

    fn heartbeat_active() -> MavMessage {
        MavMessage::HEARTBEAT(dialect::HEARTBEAT_DATA {
            custom_mode: 4,
            mavtype: dialect::MavType::MAV_TYPE_FIXED_WING,
            autopilot: dialect::MavAutopilot::MAV_AUTOPILOT_PX4,
            base_mode: dialect::MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED,
            system_status: dialect::MavState::MAV_STATE_ACTIVE,
            mavlink_version: 3,
        })
    }

    fn test_header(seq: u8) -> MavHeader {
        MavHeader {
            sequence: seq,
            system_id: 1,
            component_id: 1,
        }
    }

    #[tokio::test]
    async fn roundtrip_write_read() {
        let mut buf = Vec::new();
        let mut writer = TlogWriter::new(&mut buf, MavlinkVersion::V2);

        writer
            .write_entry(1000, &test_header(0), &heartbeat())
            .unwrap();
        writer
            .write_entry(2000, &test_header(1), &heartbeat_active())
            .unwrap();
        writer
            .write_entry(3000, &test_header(2), &heartbeat())
            .unwrap();

        let reader = TlogReader::new(buf.as_slice());
        let entries = reader.collect().await.unwrap();

        assert_eq!(entries.len(), 3);
        assert_eq!(entries[0].timestamp_usec, 1000);
        assert_eq!(entries[1].timestamp_usec, 2000);
        assert_eq!(entries[2].timestamp_usec, 3000);
        assert!(matches!(entries[0].message, MavMessage::HEARTBEAT(_)));
        assert!(matches!(entries[1].message, MavMessage::HEARTBEAT(_)));
        assert!(matches!(entries[2].message, MavMessage::HEARTBEAT(_)));
    }

    #[tokio::test]
    async fn write_now_timestamp_is_recent() {
        let before = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_micros() as u64;

        let mut buf = Vec::new();
        let mut writer = TlogWriter::new(&mut buf, MavlinkVersion::V2);
        writer.write_now(&test_header(0), &heartbeat()).unwrap();

        let after = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_micros() as u64;

        let reader = TlogReader::new(buf.as_slice());
        let entries = reader.collect().await.unwrap();
        assert_eq!(entries.len(), 1);

        let ts = entries[0].timestamp_usec;
        assert!(ts >= before, "timestamp {ts} should be >= {before}");
        assert!(ts <= after, "timestamp {ts} should be <= {after}");
    }

    #[tokio::test]
    async fn roundtrip_with_tlog_file() {
        let mut tmp = tempfile::NamedTempFile::new().unwrap();
        {
            let mut writer = TlogWriter::new(&mut tmp, MavlinkVersion::V2);
            writer
                .write_entry(5000, &test_header(0), &heartbeat())
                .unwrap();
            writer
                .write_entry(10000, &test_header(1), &heartbeat_active())
                .unwrap();
            writer
                .write_entry(15000, &test_header(2), &heartbeat())
                .unwrap();
            writer.flush().unwrap();
        }

        let tlog = TlogFile::open(tmp.path()).await.unwrap();

        // Verify time range
        let range = tlog.time_range().await.unwrap();
        assert_eq!(range, Some((5000, 15000)));

        // Verify seek works
        let mut reader = tlog.seek_to_timestamp(10000).await.unwrap();
        let entry = reader.next().await.unwrap().unwrap();
        assert_eq!(entry.timestamp_usec, 10000);
    }

    #[test]
    fn stamp_is_big_endian_like_qgc_and_pymavlink() {
        let mut writer = TlogWriter::new(Vec::new(), MavlinkVersion::V2);
        writer
            .write_entry(1_700_000_000_000_000, &test_header(0), &heartbeat())
            .unwrap();
        let buf = writer.into_inner();
        assert_eq!(buf[..8], 1_700_000_000_000_000u64.to_be_bytes());

        let records: Vec<_> = mavlink_tlog::TlogSlice::new(&buf).collect();
        assert_eq!(records.len(), 1);
        assert_eq!(records[0].timestamp_us, 1_700_000_000_000_000);
    }

    #[tokio::test]
    async fn flush_and_into_inner() {
        let mut buf = Vec::new();
        let mut writer = TlogWriter::new(&mut buf, MavlinkVersion::V2);
        writer.write_now(&test_header(0), &heartbeat()).unwrap();
        writer.flush().unwrap();
        let inner = writer.into_inner();
        assert!(!inner.is_empty());
    }
}
