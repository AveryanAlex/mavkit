use std::path::{Path, PathBuf};

use tokio::fs::File;
use tokio::io::{AsyncReadExt, AsyncSeekExt, BufReader, SeekFrom};

use super::reader::{TlogError, TlogReader};

/// File-level wrapper for reading TLOG files.
pub struct TlogFile {
    path: PathBuf,
}

impl TlogFile {
    /// Open a TLOG file, validating that the path exists.
    pub async fn open(path: impl AsRef<Path>) -> Result<Self, TlogError> {
        let path = path.as_ref().to_path_buf();
        // Verify the file exists and is readable
        let _ = File::open(&path).await?;
        Ok(Self { path })
    }

    /// Return a fresh reader over the file's entries.
    pub async fn entries(&self) -> Result<TlogReader<BufReader<File>>, TlogError> {
        let file = File::open(&self.path).await?;
        Ok(TlogReader::new(BufReader::new(file)))
    }

    /// Return a reader positioned at the first entry with `timestamp_usec >= target_usec`.
    ///
    /// Performs a linear scan from the beginning. Returns a reader at EOF if no
    /// entry meets the threshold.
    pub async fn seek_to_timestamp(
        &self,
        target_usec: u64,
    ) -> Result<TlogReader<BufReader<File>>, TlogError> {
        let file = File::open(&self.path).await?;
        let mut buf_reader = BufReader::new(file);

        loop {
            let pos = buf_reader.stream_position().await?;

            // Try to read the 8-byte timestamp
            let mut ts_buf = [0u8; 8];
            match buf_reader.read_exact(&mut ts_buf).await {
                Ok(_) => {}
                Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => {
                    // At EOF — return a reader that will immediately yield None
                    return Ok(TlogReader::new(buf_reader));
                }
                Err(e) => return Err(TlogError::Io(e)),
            }
            let ts = u64::from_le_bytes(ts_buf);

            if ts >= target_usec {
                // Seek back to the start of this entry so the reader sees it
                buf_reader.seek(SeekFrom::Start(pos)).await?;
                return Ok(TlogReader::new(buf_reader));
            }

            // Read the MAVLink frame to advance past it
            use mavlink::async_peek_reader::AsyncPeekReader;
            use mavlink::{ReadVersion, read_versioned_msg_async};
            let mut peek = AsyncPeekReader::new(&mut buf_reader);
            match read_versioned_msg_async::<mavlink::common::MavMessage, _>(
                &mut peek,
                ReadVersion::Any,
            )
            .await
            {
                Ok(_) => continue,
                Err(mavlink::error::MessageReadError::Io(ref e))
                    if e.kind() == std::io::ErrorKind::UnexpectedEof =>
                {
                    // Truncated entry at end of file
                    buf_reader.seek(SeekFrom::Start(pos)).await?;
                    return Ok(TlogReader::new(buf_reader));
                }
                Err(e) => return Err(TlogError::Parse(e)),
            }
        }
    }

    /// Return the time range `(first_timestamp, last_timestamp)` in the file.
    ///
    /// Returns `None` for empty files.
    pub async fn time_range(&self) -> Result<Option<(u64, u64)>, TlogError> {
        let file = File::open(&self.path).await?;
        let mut buf_reader = BufReader::new(file);

        // Read first timestamp
        let mut ts_buf = [0u8; 8];
        match buf_reader.read_exact(&mut ts_buf).await {
            Ok(_) => {}
            Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => return Ok(None),
            Err(e) => return Err(TlogError::Io(e)),
        }
        let first_ts = u64::from_le_bytes(ts_buf);

        // Scan the entire file for the last valid timestamp
        buf_reader.seek(SeekFrom::Start(0)).await?;
        let mut reader = TlogReader::new(buf_reader);
        let mut last_ts = first_ts;
        while let Some(entry) = reader.next().await? {
            last_ts = entry.timestamp_usec;
        }

        Ok(Some((first_ts, last_ts)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavlink::common::MavMessage;
    use mavlink::{MavHeader, MavlinkVersion};

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

    async fn write_temp_tlog(entries: &[(u64, MavMessage)]) -> tempfile::NamedTempFile {
        let data = build_tlog(entries);
        let mut tmp = tempfile::NamedTempFile::new().unwrap();
        std::io::Write::write_all(&mut tmp, &data).unwrap();
        tmp
    }

    #[tokio::test]
    async fn seek_to_timestamp() {
        let tmp = write_temp_tlog(&[
            (1000, heartbeat()),
            (2000, heartbeat()),
            (3000, heartbeat()),
        ])
        .await;

        let tlog = TlogFile::open(tmp.path()).await.unwrap();
        let mut reader = tlog.seek_to_timestamp(2000).await.unwrap();

        let entry = reader.next().await.unwrap().unwrap();
        assert_eq!(entry.timestamp_usec, 2000);

        let entry = reader.next().await.unwrap().unwrap();
        assert_eq!(entry.timestamp_usec, 3000);

        assert!(reader.next().await.unwrap().is_none());
    }

    #[tokio::test]
    async fn time_range() {
        let tmp = write_temp_tlog(&[
            (1000, heartbeat()),
            (2000, heartbeat()),
            (3000, heartbeat()),
        ])
        .await;

        let tlog = TlogFile::open(tmp.path()).await.unwrap();
        let range = tlog.time_range().await.unwrap();
        assert_eq!(range, Some((1000, 3000)));
    }

    #[tokio::test]
    async fn time_range_empty() {
        let tmp = tempfile::NamedTempFile::new().unwrap();
        let tlog = TlogFile::open(tmp.path()).await.unwrap();
        let range = tlog.time_range().await.unwrap();
        assert_eq!(range, None);
    }
}
