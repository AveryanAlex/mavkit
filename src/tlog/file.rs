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
    /// Uses binary search on file byte positions for O(log n) seeking on large files,
    /// with a linear scan fallback for the final small region. Returns a reader at EOF
    /// if no entry meets the threshold.
    pub async fn seek_to_timestamp(
        &self,
        target_usec: u64,
    ) -> Result<TlogReader<BufReader<File>>, TlogError> {
        let file = File::open(&self.path).await?;
        let mut buf_reader = BufReader::new(file);

        let file_len = buf_reader.seek(SeekFrom::End(0)).await?;

        // Minimum TLOG entry: 8-byte timestamp + smallest MAVLink v2 frame (12 bytes)
        const MIN_ENTRY_SIZE: u64 = 20;
        // When the search range is this small, switch to linear scan
        const LINEAR_THRESHOLD: u64 = 4096;

        if file_len < MIN_ENTRY_SIZE {
            buf_reader.seek(SeekFrom::Start(0)).await?;
            return Ok(TlogReader::new(buf_reader));
        }

        let mut lo: u64 = 0;
        let mut hi: u64 = file_len;

        // Binary search phase: narrow down to a small byte range
        while hi - lo > LINEAR_THRESHOLD {
            let mid = lo + (hi - lo) / 2;

            match Self::find_entry_at_or_after(&mut buf_reader, mid, hi).await? {
                Some((entry_pos, ts)) => {
                    if ts < target_usec {
                        // This entry is before our target; advance past it
                        lo = entry_pos + MIN_ENTRY_SIZE;
                    } else {
                        hi = entry_pos;
                    }
                }
                None => {
                    // No valid entry found in [mid, hi) — target must be in [lo, mid)
                    hi = mid;
                }
            }
        }

        // Linear scan from `lo` to find exact position
        buf_reader.seek(SeekFrom::Start(lo)).await?;
        Self::linear_seek(buf_reader, target_usec).await
    }

    /// Scan forward from `start` (up to `limit`) to find the next valid TLOG entry.
    /// Returns `Some((entry_byte_offset, timestamp))` or `None` if no entry found.
    async fn find_entry_at_or_after(
        reader: &mut BufReader<File>,
        start: u64,
        limit: u64,
    ) -> Result<Option<(u64, u64)>, TlogError> {
        // MAVLink v1 magic = 0xFE, v2 magic = 0xFD
        // TLOG entry = [8-byte LE timestamp][MAVLink frame starting with magic]
        // Strategy: scan for magic bytes, check if 8 bytes prior is a plausible timestamp.
        const CHUNK_SIZE: usize = 4096;
        let mut pos = start;

        while pos < limit {
            reader.seek(SeekFrom::Start(pos)).await?;
            let to_read = CHUNK_SIZE.min((limit - pos) as usize);
            let mut buf = vec![0u8; to_read];
            let n = reader.read(&mut buf).await?;
            if n == 0 {
                return Ok(None);
            }
            buf.truncate(n);

            for (i, &byte) in buf.iter().enumerate() {
                if byte == 0xFD || byte == 0xFE {
                    let magic_file_pos = pos + i as u64;
                    // The timestamp would start 8 bytes before the magic byte
                    if magic_file_pos < 8 {
                        continue;
                    }
                    let ts_pos = magic_file_pos - 8;

                    // Read the 8-byte timestamp
                    reader.seek(SeekFrom::Start(ts_pos)).await?;
                    let mut ts_buf = [0u8; 8];
                    match reader.read_exact(&mut ts_buf).await {
                        Ok(_) => {}
                        Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => continue,
                        Err(e) => return Err(TlogError::Io(e)),
                    }
                    let ts = u64::from_le_bytes(ts_buf);

                    // Validate: try to parse the MAVLink frame at this position
                    reader.seek(SeekFrom::Start(magic_file_pos)).await?;
                    let mut peek = mavlink::async_peek_reader::AsyncPeekReader::new(&mut *reader);
                    match mavlink::read_versioned_msg_async::<mavlink::common::MavMessage, _>(
                        &mut peek,
                        mavlink::ReadVersion::Any,
                    )
                    .await
                    {
                        Ok(_) => return Ok(Some((ts_pos, ts))),
                        Err(mavlink::error::MessageReadError::Io(ref e))
                            if e.kind() == std::io::ErrorKind::UnexpectedEof =>
                        {
                            return Ok(None);
                        }
                        Err(_) => continue, // Not a valid frame, keep scanning
                    }
                }
            }
            pos += n as u64;
        }
        Ok(None)
    }

    /// Linear scan from current position to find first entry with ts >= target.
    /// Takes ownership of the BufReader so it can be handed to TlogReader.
    async fn linear_seek(
        mut buf_reader: BufReader<File>,
        target_usec: u64,
    ) -> Result<TlogReader<BufReader<File>>, TlogError> {
        loop {
            let pos = buf_reader.stream_position().await?;

            let mut ts_buf = [0u8; 8];
            match buf_reader.read_exact(&mut ts_buf).await {
                Ok(_) => {}
                Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => {
                    return Ok(TlogReader::new(buf_reader));
                }
                Err(e) => return Err(TlogError::Io(e)),
            }
            let ts = u64::from_le_bytes(ts_buf);

            if ts >= target_usec {
                buf_reader.seek(SeekFrom::Start(pos)).await?;
                return Ok(TlogReader::new(buf_reader));
            }

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
                    buf_reader.seek(SeekFrom::Start(pos)).await?;
                    return Ok(TlogReader::new(buf_reader));
                }
                Err(e) => return Err(TlogError::Parse(e)),
            }
        }
    }

    /// Return the time range `(first_timestamp, last_timestamp)` in the file.
    ///
    /// Returns `None` for empty files.  For the last timestamp, seeks near EOF
    /// and scans only the tail of the file instead of reading every entry.
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

        let file_len = buf_reader.seek(SeekFrom::End(0)).await?;

        // Find the last timestamp by seeking near EOF and scanning forward.
        // Expand the search window backwards if no valid entry is found.
        const SCAN_SIZE: u64 = 8192;
        let mut scan_start = file_len.saturating_sub(SCAN_SIZE);

        let entry_pos = loop {
            match Self::find_entry_at_or_after(&mut buf_reader, scan_start, file_len).await? {
                Some((pos, _ts)) => break pos,
                None => {
                    if scan_start == 0 {
                        return Ok(Some((first_ts, first_ts)));
                    }
                    scan_start = scan_start.saturating_sub(SCAN_SIZE);
                }
            }
        };

        // Scan forward from the found entry to collect the last timestamp.
        // Since we started near EOF, this reads only a small tail of the file.
        buf_reader.seek(SeekFrom::Start(entry_pos)).await?;
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

    #[tokio::test]
    async fn seek_to_first_entry() {
        let tmp = write_temp_tlog(&[
            (1000, heartbeat()),
            (2000, heartbeat()),
            (3000, heartbeat()),
        ])
        .await;
        let tlog = TlogFile::open(tmp.path()).await.unwrap();
        let mut reader = tlog.seek_to_timestamp(1000).await.unwrap();
        let entry = reader.next().await.unwrap().unwrap();
        assert_eq!(entry.timestamp_usec, 1000);
    }

    #[tokio::test]
    async fn seek_before_all_entries() {
        let tmp = write_temp_tlog(&[(1000, heartbeat()), (2000, heartbeat())]).await;
        let tlog = TlogFile::open(tmp.path()).await.unwrap();
        let mut reader = tlog.seek_to_timestamp(500).await.unwrap();
        let entry = reader.next().await.unwrap().unwrap();
        assert_eq!(entry.timestamp_usec, 1000);
    }

    #[tokio::test]
    async fn seek_past_all_entries() {
        let tmp = write_temp_tlog(&[(1000, heartbeat()), (2000, heartbeat())]).await;
        let tlog = TlogFile::open(tmp.path()).await.unwrap();
        let mut reader = tlog.seek_to_timestamp(9999).await.unwrap();
        assert!(reader.next().await.unwrap().is_none());
    }

    #[tokio::test]
    async fn seek_single_entry() {
        let tmp = write_temp_tlog(&[(5000, heartbeat())]).await;
        let tlog = TlogFile::open(tmp.path()).await.unwrap();

        let mut reader = tlog.seek_to_timestamp(5000).await.unwrap();
        let entry = reader.next().await.unwrap().unwrap();
        assert_eq!(entry.timestamp_usec, 5000);
        assert!(reader.next().await.unwrap().is_none());
    }

    #[tokio::test]
    async fn seek_many_entries() {
        // Generate enough entries to exercise the binary search path (> LINEAR_THRESHOLD bytes)
        let entries: Vec<(u64, MavMessage)> =
            (0..200).map(|i| (1000 + i * 100, heartbeat())).collect();
        let tmp = write_temp_tlog(&entries).await;
        let tlog = TlogFile::open(tmp.path()).await.unwrap();

        // Seek to a mid-range timestamp
        let mut reader = tlog.seek_to_timestamp(10_000).await.unwrap();
        let entry = reader.next().await.unwrap().unwrap();
        assert_eq!(entry.timestamp_usec, 10_000);

        // Seek between entries — should land on the next one
        let mut reader = tlog.seek_to_timestamp(10_050).await.unwrap();
        let entry = reader.next().await.unwrap().unwrap();
        assert_eq!(entry.timestamp_usec, 10_100);
    }

    #[tokio::test]
    async fn seek_empty_file() {
        let tmp = tempfile::NamedTempFile::new().unwrap();
        let tlog = TlogFile::open(tmp.path()).await.unwrap();
        let mut reader = tlog.seek_to_timestamp(1000).await.unwrap();
        assert!(reader.next().await.unwrap().is_none());
    }

    #[tokio::test]
    async fn time_range_many_entries() {
        // Enough entries to exceed the 8KB near-EOF scan window,
        // exercising the optimized tail-seek path.
        let entries: Vec<(u64, MavMessage)> =
            (0..500).map(|i| (1000 + i * 100, heartbeat())).collect();
        let tmp = write_temp_tlog(&entries).await;
        let tlog = TlogFile::open(tmp.path()).await.unwrap();
        let range = tlog.time_range().await.unwrap();
        assert_eq!(range, Some((1000, 1000 + 499 * 100)));
    }

    #[tokio::test]
    async fn time_range_single_entry() {
        let tmp = write_temp_tlog(&[(42_000, heartbeat())]).await;
        let tlog = TlogFile::open(tmp.path()).await.unwrap();
        let range = tlog.time_range().await.unwrap();
        assert_eq!(range, Some((42_000, 42_000)));
    }
}
