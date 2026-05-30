//! Typed MAVFTP file and directory operations.
//!
//! MAVFTP uses MAVLink's `FILE_TRANSFER_PROTOCOL` message as a packet envelope. Public callers
//! work with paths, byte buffers, and operation handles while this module owns sessions,
//! sequence numbers, retries, and response correlation.

use std::fmt;
use std::future::Future;
use std::pin::Pin;
use std::sync::atomic::Ordering;
use std::time::Duration;

use serde::{Deserialize, Serialize};
use tokio::sync::oneshot;
use tokio_stream::{Stream, StreamExt};
use tokio_util::sync::CancellationToken;

use crate::command::Command;
use crate::dialect;
use crate::error::VehicleError;
use crate::mission::RetryPolicy;
use crate::observation::{ObservationHandle, ObservationSubscription, ObservationWriter};
use crate::operation::OperationHandle;
use crate::raw::RawMessage;
use crate::runtime;
use crate::vehicle::Vehicle;

const FTP_MESSAGE_ID: u32 = 110;
const FTP_PAYLOAD_LEN: usize = 251;
const FTP_HEADER_LEN: usize = 12;
const FTP_DATA_CAPACITY: usize = FTP_PAYLOAD_LEN - FTP_HEADER_LEN;

const OPCODE_TERMINATE_SESSION: u8 = 1;
const OPCODE_LIST_DIRECTORY: u8 = 3;
const OPCODE_OPEN_FILE_RO: u8 = 4;
const OPCODE_READ_FILE: u8 = 5;
const OPCODE_CREATE_FILE: u8 = 6;
const OPCODE_WRITE_FILE: u8 = 7;
const OPCODE_REMOVE_FILE: u8 = 8;
const OPCODE_CREATE_DIRECTORY: u8 = 9;
const OPCODE_REMOVE_DIRECTORY: u8 = 10;
const OPCODE_OPEN_FILE_WO: u8 = 11;
const OPCODE_TRUNCATE_FILE: u8 = 12;
const OPCODE_RENAME: u8 = 13;
const OPCODE_CALC_FILE_CRC32: u8 = 14;
const OPCODE_ACK: u8 = 128;
const OPCODE_NAK: u8 = 129;

/// MAVFTP destination carried inside each `FILE_TRANSFER_PROTOCOL` message.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct FtpTarget {
    /// MAVLink network ID. Use `0` for the normal local network.
    pub network_id: u8,
    /// MAVLink system ID of the FTP server.
    pub system_id: u8,
    /// MAVLink component ID of the FTP server.
    pub component_id: u8,
}

impl FtpTarget {
    /// Creates a target on the local MAVLink network.
    pub const fn new(system_id: u8, component_id: u8) -> Self {
        Self {
            network_id: 0,
            system_id,
            component_id,
        }
    }
}

/// Kind of one directory-listing entry.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FtpEntryKind {
    File,
    Directory,
    Skip,
    Unknown,
}

/// One entry returned by [`FtpHandle::list_directory`].
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct FtpEntry {
    pub name: String,
    pub kind: FtpEntryKind,
    /// File size in bytes when the server supplied it.
    pub size: Option<u64>,
}

/// Named MAVFTP NAK reason codes.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FtpNakCode {
    Fail,
    FailErrno,
    InvalidDataSize,
    InvalidSession,
    NoSessionsAvailable,
    Eof,
    UnknownCommand,
    FileExists,
    FileProtected,
    FileNotFound,
    Other(u8),
}

impl FtpNakCode {
    fn from_wire(value: u8) -> Self {
        match value {
            1 => Self::Fail,
            2 => Self::FailErrno,
            3 => Self::InvalidDataSize,
            4 => Self::InvalidSession,
            5 => Self::NoSessionsAvailable,
            6 => Self::Eof,
            7 => Self::UnknownCommand,
            8 => Self::FileExists,
            9 => Self::FileProtected,
            10 => Self::FileNotFound,
            other => Self::Other(other),
        }
    }
}

impl fmt::Display for FtpNakCode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Fail => write!(f, "fail"),
            Self::FailErrno => write!(f, "fail_errno"),
            Self::InvalidDataSize => write!(f, "invalid_data_size"),
            Self::InvalidSession => write!(f, "invalid_session"),
            Self::NoSessionsAvailable => write!(f, "no_sessions_available"),
            Self::Eof => write!(f, "eof"),
            Self::UnknownCommand => write!(f, "unknown_command"),
            Self::FileExists => write!(f, "file_exists"),
            Self::FileProtected => write!(f, "file_protected"),
            Self::FileNotFound => write!(f, "file_not_found"),
            Self::Other(value) => write!(f, "other({value})"),
        }
    }
}

/// MAVFTP-specific protocol failure.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize, thiserror::Error)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum FtpError {
    /// The FTP server returned a NAK packet.
    #[error("request rejected: code={code}, errno={errno:?}")]
    Nak { code: FtpNakCode, errno: Option<u8> },
    /// The FTP server returned a packet that violates the MAVFTP wire format.
    #[error("invalid response: {0}")]
    InvalidResponse(String),
}

impl FtpError {
    fn from_nak_data(data: &[u8]) -> Self {
        let code = data
            .first()
            .copied()
            .map(FtpNakCode::from_wire)
            .unwrap_or(FtpNakCode::Fail);
        let errno = matches!(code, FtpNakCode::FailErrno)
            .then(|| data.get(1).copied())
            .flatten();
        Self::Nak { code, errno }
    }

    fn is_nak(&self, expected: FtpNakCode) -> bool {
        matches!(self, Self::Nak { code, .. } if *code == expected)
    }
}

/// Lifecycle phases for an FTP download or upload operation.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FtpOperationProgress {
    Opening,
    Downloading {
        bytes_transferred: u64,
        total_bytes: Option<u64>,
    },
    Uploading {
        bytes_transferred: u64,
        total_bytes: u64,
    },
    Finalizing,
    Completed,
    Failed,
    Cancelled,
}

/// Handle for an in-flight FTP operation.
pub struct FtpOperationHandle<T: Send + 'static> {
    core: OperationHandle<T, FtpOperationProgress>,
}

impl<T: Send + 'static> FtpOperationHandle<T> {
    fn new(
        progress: ObservationHandle<FtpOperationProgress>,
        result_rx: oneshot::Receiver<Result<T, VehicleError>>,
        cancel: CancellationToken,
    ) -> Self {
        Self {
            core: OperationHandle::new(progress, result_rx, cancel),
        }
    }

    pub fn latest(&self) -> Option<FtpOperationProgress> {
        self.core.latest()
    }

    pub fn subscribe(&self) -> ObservationSubscription<FtpOperationProgress> {
        self.core.subscribe()
    }

    pub async fn wait(&self) -> Result<T, VehicleError> {
        self.core.wait().await
    }

    /// Like [`wait`](Self::wait), but fails if the wait exceeds `timeout`.
    pub async fn wait_timeout(&self, timeout: Duration) -> Result<T, VehicleError> {
        self.core.wait_timeout(timeout, "FTP operation wait").await
    }

    pub fn cancel(&self) {
        self.core.cancel();
    }

    pub fn cancel_token(&self) -> CancellationToken {
        self.core.cancel_token()
    }
}

/// Handle for an FTP download operation.
pub type FtpDownloadOp = FtpOperationHandle<Vec<u8>>;
/// Handle for an FTP upload operation.
pub type FtpUploadOp = FtpOperationHandle<()>;

/// Accessor for MAVFTP file and directory operations.
///
/// Obtained from [`Vehicle::ftp`](crate::Vehicle::ftp). FTP operations are serialized with each
/// other because the server-side session protocol is stateful. They do not block mission or
/// parameter transfers.
pub struct FtpHandle<'a> {
    vehicle: &'a Vehicle,
    target: FtpTarget,
}

impl<'a> FtpHandle<'a> {
    pub(crate) fn new(vehicle: &'a Vehicle) -> Self {
        let identity = vehicle.identity();
        Self {
            vehicle,
            target: FtpTarget::new(identity.system_id, identity.component_id),
        }
    }

    /// Overrides the default FTP destination inferred from the connected vehicle heartbeat.
    pub fn with_target(&self, target: FtpTarget) -> Self {
        Self {
            vehicle: self.vehicle,
            target,
        }
    }

    /// Lists entries below `path` until the server reports end-of-directory.
    pub async fn list_directory(&self, path: &str) -> Result<Vec<FtpEntry>, VehicleError> {
        let path = path_data(path)?;
        self.with_client("list_directory", |mut client| async move {
            client.list_directory(path).await
        })
        .await
    }

    /// Starts downloading `remote_path` into memory.
    pub fn download(&self, remote_path: &str) -> Result<FtpDownloadOp, VehicleError> {
        let path = path_data(remote_path)?;
        let reservation = self
            .vehicle
            .inner
            .ftp_protocol
            .begin_operation("ftp", "download")?;
        let (progress_writer, progress) = ObservationHandle::watch();
        let _ = progress_writer.publish(FtpOperationProgress::Opening);
        let (result_tx, result_rx) = oneshot::channel();
        let op = FtpOperationHandle::new(progress, result_rx, reservation.cancel.clone());

        let vehicle = self.vehicle.clone();
        let target = self.target;
        let timeout = vehicle.inner._config.transfer_timeout;
        let protocol = vehicle.inner.ftp_protocol.clone();
        let op_id = reservation.id;
        let cancel = reservation.cancel;
        runtime::spawn(async move {
            let mut client = FtpClient::new(vehicle, target, cancel.clone());
            let result = run_with_timeout(timeout, "download", cancel, async {
                client.download(path, &progress_writer).await
            })
            .await;
            publish_final_progress(&progress_writer, &result);
            protocol.finish_operation(op_id);
            let _ = result_tx.send(result);
        });

        Ok(op)
    }

    /// Starts uploading `data` to `remote_path`, replacing an existing file if necessary.
    pub fn upload(&self, remote_path: &str, data: Vec<u8>) -> Result<FtpUploadOp, VehicleError> {
        let path = path_data(remote_path)?;
        if u32::try_from(data.len()).is_err() {
            return Err(VehicleError::InvalidParameter(
                "MAVFTP uploads cannot exceed u32::MAX bytes".to_string(),
            ));
        }
        let reservation = self
            .vehicle
            .inner
            .ftp_protocol
            .begin_operation("ftp", "upload")?;
        let (progress_writer, progress) = ObservationHandle::watch();
        let _ = progress_writer.publish(FtpOperationProgress::Opening);
        let (result_tx, result_rx) = oneshot::channel();
        let op = FtpOperationHandle::new(progress, result_rx, reservation.cancel.clone());

        let vehicle = self.vehicle.clone();
        let target = self.target;
        let timeout = vehicle.inner._config.transfer_timeout;
        let protocol = vehicle.inner.ftp_protocol.clone();
        let op_id = reservation.id;
        let cancel = reservation.cancel;
        runtime::spawn(async move {
            let mut client = FtpClient::new(vehicle, target, cancel.clone());
            let result = run_with_timeout(timeout, "upload", cancel, async {
                client.upload(path, data, &progress_writer).await
            })
            .await;
            publish_final_progress(&progress_writer, &result);
            protocol.finish_operation(op_id);
            let _ = result_tx.send(result);
        });

        Ok(op)
    }

    /// Removes a remote file.
    pub async fn remove_file(&self, path: &str) -> Result<(), VehicleError> {
        let path = path_data(path)?;
        self.with_client("remove_file", |mut client| async move {
            client.path_command(OPCODE_REMOVE_FILE, path, 0).await
        })
        .await
    }

    /// Creates a remote directory.
    pub async fn create_directory(&self, path: &str) -> Result<(), VehicleError> {
        let path = path_data(path)?;
        self.with_client("create_directory", |mut client| async move {
            client.path_command(OPCODE_CREATE_DIRECTORY, path, 0).await
        })
        .await
    }

    /// Removes an empty remote directory.
    pub async fn remove_directory(&self, path: &str) -> Result<(), VehicleError> {
        let path = path_data(path)?;
        self.with_client("remove_directory", |mut client| async move {
            client.path_command(OPCODE_REMOVE_DIRECTORY, path, 0).await
        })
        .await
    }

    /// Renames a remote file or directory.
    pub async fn rename(&self, from: &str, to: &str) -> Result<(), VehicleError> {
        let paths = rename_data(from, to)?;
        self.with_client("rename", |mut client| async move {
            client.request_ack(0, OPCODE_RENAME, 0, paths).await?;
            Ok(())
        })
        .await
    }

    /// Truncates a remote file to `length` bytes.
    pub async fn truncate(&self, path: &str, length: u32) -> Result<(), VehicleError> {
        let path = path_data(path)?;
        self.with_client("truncate", |mut client| async move {
            client
                .path_command(OPCODE_TRUNCATE_FILE, path, length)
                .await
        })
        .await
    }

    /// Calculates the server-side CRC-32 of a remote file.
    pub async fn crc32(&self, path: &str) -> Result<u32, VehicleError> {
        let path = path_data(path)?;
        self.with_client("crc32", |mut client| async move {
            let response = client
                .request_ack(0, OPCODE_CALC_FILE_CRC32, 0, path)
                .await?;
            response.data_u32("CRC-32 response")
        })
        .await
    }

    async fn with_client<T, F, Fut>(
        &self,
        op_name: &'static str,
        action: F,
    ) -> Result<T, VehicleError>
    where
        F: FnOnce(FtpClient) -> Fut,
        Fut: Future<Output = Result<T, VehicleError>>,
    {
        let reservation = self
            .vehicle
            .inner
            .ftp_protocol
            .begin_operation("ftp", op_name)?;
        let protocol = self.vehicle.inner.ftp_protocol.clone();
        let timeout = self.vehicle.inner._config.transfer_timeout;
        let cancel = reservation.cancel;
        let client = FtpClient::new(self.vehicle.clone(), self.target, cancel.clone());
        let result = run_with_timeout(timeout, op_name, cancel, action(client)).await;
        protocol.finish_operation(reservation.id);
        result
    }
}

fn publish_final_progress<T>(
    writer: &ObservationWriter<FtpOperationProgress>,
    result: &Result<T, VehicleError>,
) {
    let phase = match result {
        Ok(_) => FtpOperationProgress::Completed,
        Err(VehicleError::Cancelled) => FtpOperationProgress::Cancelled,
        Err(_) => FtpOperationProgress::Failed,
    };
    let _ = writer.publish(phase);
}

async fn run_with_timeout<T>(
    timeout: Duration,
    op_name: &'static str,
    cancel: CancellationToken,
    operation: impl Future<Output = Result<T, VehicleError>>,
) -> Result<T, VehicleError> {
    tokio::select! {
        _ = cancel.cancelled() => Err(VehicleError::Cancelled),
        result = runtime::timeout(timeout, operation) => {
            result.map_err(|_| VehicleError::Timeout(format!("MAVFTP {op_name}")))?
        }
    }
}

type FtpMessageStream = Pin<Box<dyn Stream<Item = RawMessage> + Send>>;

struct FtpClient {
    vehicle: Vehicle,
    target: FtpTarget,
    incoming: FtpMessageStream,
    retry_policy: RetryPolicy,
    cancel: CancellationToken,
}

impl FtpClient {
    fn new(vehicle: Vehicle, target: FtpTarget, cancel: CancellationToken) -> Self {
        let incoming = Box::pin(vehicle.raw().subscribe_filtered(FTP_MESSAGE_ID));
        let retry_policy = vehicle.inner._config.retry_policy;
        Self {
            vehicle,
            target,
            incoming,
            retry_policy,
            cancel,
        }
    }

    async fn list_directory(&mut self, path: Vec<u8>) -> Result<Vec<FtpEntry>, VehicleError> {
        let mut offset = 0_u32;
        let mut entries = Vec::new();
        loop {
            match self
                .request(0, OPCODE_LIST_DIRECTORY, offset, path.clone())
                .await?
            {
                FtpResponse::Ack(response) => {
                    let page = parse_directory_entries(&response.data)?;
                    if page.is_empty() {
                        break;
                    }
                    offset = offset.checked_add(page.len() as u32).ok_or_else(|| {
                        FtpError::InvalidResponse("directory offset overflowed u32".to_string())
                    })?;
                    entries.extend(page);
                }
                FtpResponse::Nak(err) if err.is_nak(FtpNakCode::Eof) => break,
                FtpResponse::Nak(err) => return Err(err.into()),
            }
        }
        Ok(entries)
    }

    async fn download(
        &mut self,
        path: Vec<u8>,
        progress: &ObservationWriter<FtpOperationProgress>,
    ) -> Result<Vec<u8>, VehicleError> {
        let open = self.request_ack(0, OPCODE_OPEN_FILE_RO, 0, path).await?;
        let session = open.session;
        let expected_size = open.data_u32("open-file response")?;
        let _ = progress.publish(FtpOperationProgress::Downloading {
            bytes_transferred: 0,
            total_bytes: Some(u64::from(expected_size)),
        });

        let mut output = Vec::new();
        let result = async {
            loop {
                let offset = u32::try_from(output.len()).map_err(|_| {
                    FtpError::InvalidResponse("download offset overflowed u32".to_string())
                })?;
                let remaining = expected_size as usize - output.len();
                let requested_size = if remaining == 0 {
                    FTP_DATA_CAPACITY
                } else {
                    remaining.min(FTP_DATA_CAPACITY)
                } as u8;
                match self
                    .request_with_size(
                        session,
                        OPCODE_READ_FILE,
                        offset,
                        Vec::new(),
                        requested_size,
                    )
                    .await?
                {
                    FtpResponse::Ack(response) => {
                        if response.data.is_empty() {
                            break;
                        }
                        if response.data.len() > usize::from(requested_size) {
                            return Err(FtpError::InvalidResponse(format!(
                                "server returned {} bytes for a {requested_size}-byte read",
                                response.data.len()
                            ))
                            .into());
                        }
                        output.extend_from_slice(&response.data);
                        if output.len() > expected_size as usize {
                            return Err(FtpError::InvalidResponse(format!(
                                "server returned more than the expected {expected_size} bytes"
                            ))
                            .into());
                        }
                        let _ = progress.publish(FtpOperationProgress::Downloading {
                            bytes_transferred: output.len() as u64,
                            total_bytes: Some(u64::from(expected_size)),
                        });
                    }
                    FtpResponse::Nak(err) if err.is_nak(FtpNakCode::Eof) => break,
                    FtpResponse::Nak(err) => return Err(err.into()),
                }
            }
            Ok(output)
        }
        .await;

        let _ = progress.publish(FtpOperationProgress::Finalizing);
        let _ = self.terminate_session(session).await;
        result
    }

    async fn upload(
        &mut self,
        path: Vec<u8>,
        data: Vec<u8>,
        progress: &ObservationWriter<FtpOperationProgress>,
    ) -> Result<(), VehicleError> {
        let session = match self.request(0, OPCODE_CREATE_FILE, 0, path.clone()).await? {
            FtpResponse::Ack(response) => response.session,
            FtpResponse::Nak(err) if err.is_nak(FtpNakCode::FileExists) => {
                self.path_command(OPCODE_TRUNCATE_FILE, path.clone(), 0)
                    .await?;
                self.request_ack(0, OPCODE_OPEN_FILE_WO, 0, path)
                    .await?
                    .session
            }
            FtpResponse::Nak(err) => return Err(err.into()),
        };

        let total_bytes = data.len() as u64;
        let _ = progress.publish(FtpOperationProgress::Uploading {
            bytes_transferred: 0,
            total_bytes,
        });
        let result: Result<(), VehicleError> = async {
            let mut offset = 0_u32;
            for chunk in data.chunks(FTP_DATA_CAPACITY) {
                let response = self
                    .request_ack(session, OPCODE_WRITE_FILE, offset, chunk.to_vec())
                    .await?;
                if !response.data.is_empty() {
                    let written = response.data_u32("write-file response")?;
                    if written != chunk.len() as u32 {
                        return Err(FtpError::InvalidResponse(format!(
                            "server reported writing {written} bytes for a {}-byte chunk",
                            chunk.len()
                        ))
                        .into());
                    }
                }
                offset = offset.checked_add(chunk.len() as u32).ok_or_else(|| {
                    FtpError::InvalidResponse("upload offset overflowed u32".to_string())
                })?;
                let _ = progress.publish(FtpOperationProgress::Uploading {
                    bytes_transferred: u64::from(offset),
                    total_bytes,
                });
            }
            Ok(())
        }
        .await;

        let _ = progress.publish(FtpOperationProgress::Finalizing);
        let _ = self.terminate_session(session).await;
        result?;
        Ok(())
    }

    async fn terminate_session(&mut self, session: u8) -> Result<(), VehicleError> {
        let packet = FtpPacket {
            sequence: self
                .vehicle
                .inner
                .ftp_sequence
                .fetch_add(1, Ordering::Relaxed),
            session,
            opcode: OPCODE_TERMINATE_SESSION,
            size: 0,
            requested_opcode: 0,
            burst_complete: 0,
            offset: 0,
            data: Vec::new(),
        };
        self.send(&packet).await
    }

    async fn path_command(
        &mut self,
        opcode: u8,
        path: Vec<u8>,
        offset: u32,
    ) -> Result<(), VehicleError> {
        self.request_ack(0, opcode, offset, path).await?;
        Ok(())
    }

    async fn request_ack(
        &mut self,
        session: u8,
        opcode: u8,
        offset: u32,
        data: Vec<u8>,
    ) -> Result<FtpPacket, VehicleError> {
        match self.request(session, opcode, offset, data).await? {
            FtpResponse::Ack(response) => Ok(response),
            FtpResponse::Nak(err) => Err(err.into()),
        }
    }

    async fn request(
        &mut self,
        session: u8,
        opcode: u8,
        offset: u32,
        data: Vec<u8>,
    ) -> Result<FtpResponse, VehicleError> {
        let size = u8::try_from(data.len()).map_err(|_| {
            VehicleError::InvalidParameter(format!(
                "MAVFTP packet contains {} bytes, exceeding the {FTP_DATA_CAPACITY}-byte limit",
                data.len()
            ))
        })?;
        self.request_with_size(session, opcode, offset, data, size)
            .await
    }

    async fn request_with_size(
        &mut self,
        session: u8,
        opcode: u8,
        offset: u32,
        data: Vec<u8>,
        size: u8,
    ) -> Result<FtpResponse, VehicleError> {
        if usize::from(size) > FTP_DATA_CAPACITY || data.len() > FTP_DATA_CAPACITY {
            return Err(VehicleError::InvalidParameter(format!(
                "MAVFTP packet exceeds the {FTP_DATA_CAPACITY}-byte limit"
            )));
        }
        let sequence = self
            .vehicle
            .inner
            .ftp_sequence
            .fetch_add(1, Ordering::Relaxed);
        let packet = FtpPacket {
            sequence,
            session,
            opcode,
            size,
            requested_opcode: 0,
            burst_complete: 0,
            offset,
            data,
        };
        let request_timeout = Duration::from_millis(self.retry_policy.request_timeout_ms);

        for _ in 0..=self.retry_policy.max_retries {
            self.send(&packet).await?;
            let cancel = self.cancel.clone();
            match tokio::select! {
                _ = cancel.cancelled() => return Err(VehicleError::Cancelled),
                result = runtime::timeout(request_timeout, self.wait_for_response(sequence, opcode)) => result,
            } {
                Ok(response) => return response,
                Err(_) => continue,
            }
        }

        Err(VehicleError::Timeout(format!(
            "MAVFTP opcode {opcode} response"
        )))
    }

    async fn send(&mut self, packet: &FtpPacket) -> Result<(), VehicleError> {
        let message = packet.to_message(self.target);
        self.vehicle
            .send_command(|reply| Command::RawSend {
                message: Box::new(message),
                reply,
            })
            .await
    }

    async fn wait_for_response(
        &mut self,
        sequence: u16,
        requested_opcode: u8,
    ) -> Result<FtpResponse, VehicleError> {
        while let Some(raw) = self.incoming.next().await {
            if (self.target.system_id != 0 && raw.system_id != self.target.system_id)
                || (self.target.component_id != 0 && raw.component_id != self.target.component_id)
            {
                continue;
            }
            let dialect::MavMessage::FILE_TRANSFER_PROTOCOL(message) = raw.to_mavlink()? else {
                continue;
            };
            let config = &self.vehicle.inner._config;
            if (message.target_system != 0 && message.target_system != config.gcs_system_id)
                || (message.target_component != 0
                    && message.target_component != config.gcs_component_id)
            {
                continue;
            }
            let response = FtpPacket::from_payload(&message.payload)?;
            if response.sequence != sequence.wrapping_add(1)
                || response.requested_opcode != requested_opcode
            {
                continue;
            }
            return match response.opcode {
                OPCODE_ACK => Ok(FtpResponse::Ack(response)),
                OPCODE_NAK => Ok(FtpResponse::Nak(FtpError::from_nak_data(&response.data))),
                opcode => Err(FtpError::InvalidResponse(format!(
                    "expected ACK or NAK, received opcode {opcode}"
                ))
                .into()),
            };
        }
        Err(VehicleError::Disconnected)
    }
}

enum FtpResponse {
    Ack(FtpPacket),
    Nak(FtpError),
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct FtpPacket {
    sequence: u16,
    session: u8,
    opcode: u8,
    size: u8,
    requested_opcode: u8,
    burst_complete: u8,
    offset: u32,
    data: Vec<u8>,
}

impl FtpPacket {
    fn from_payload(payload: &[u8; FTP_PAYLOAD_LEN]) -> Result<Self, FtpError> {
        let size = payload[4];
        let data_len = usize::from(size);
        if data_len > FTP_DATA_CAPACITY {
            return Err(FtpError::InvalidResponse(format!(
                "payload size {data_len} exceeds the {FTP_DATA_CAPACITY}-byte limit"
            )));
        }
        Ok(Self {
            sequence: u16::from_le_bytes([payload[0], payload[1]]),
            session: payload[2],
            opcode: payload[3],
            size,
            requested_opcode: payload[5],
            burst_complete: payload[6],
            offset: u32::from_le_bytes([payload[8], payload[9], payload[10], payload[11]]),
            data: payload[FTP_HEADER_LEN..FTP_HEADER_LEN + data_len].to_vec(),
        })
    }

    fn to_message(&self, target: FtpTarget) -> dialect::MavMessage {
        let mut payload = [0_u8; FTP_PAYLOAD_LEN];
        payload[0..2].copy_from_slice(&self.sequence.to_le_bytes());
        payload[2] = self.session;
        payload[3] = self.opcode;
        payload[4] = self.size;
        payload[5] = self.requested_opcode;
        payload[6] = self.burst_complete;
        payload[8..12].copy_from_slice(&self.offset.to_le_bytes());
        payload[FTP_HEADER_LEN..FTP_HEADER_LEN + self.data.len()].copy_from_slice(&self.data);
        dialect::MavMessage::FILE_TRANSFER_PROTOCOL(dialect::FILE_TRANSFER_PROTOCOL_DATA {
            target_network: target.network_id,
            target_system: target.system_id,
            target_component: target.component_id,
            payload,
        })
    }

    fn data_u32(&self, context: &str) -> Result<u32, VehicleError> {
        let bytes: [u8; 4] = self
            .data
            .get(0..4)
            .ok_or_else(|| {
                FtpError::InvalidResponse(format!("{context} did not include a u32 payload"))
            })?
            .try_into()
            .map_err(|_| {
                FtpError::InvalidResponse(format!("{context} did not include a u32 payload"))
            })?;
        Ok(u32::from_le_bytes(bytes))
    }
}

fn path_data(path: &str) -> Result<Vec<u8>, VehicleError> {
    let data = path.as_bytes();
    if data.contains(&0) {
        return Err(VehicleError::InvalidParameter(
            "MAVFTP paths cannot contain NUL bytes".to_string(),
        ));
    }
    if data.len() > FTP_DATA_CAPACITY {
        return Err(VehicleError::InvalidParameter(format!(
            "MAVFTP path contains {} bytes, exceeding the {FTP_DATA_CAPACITY}-byte limit",
            data.len()
        )));
    }
    Ok(data.to_vec())
}

fn rename_data(from: &str, to: &str) -> Result<Vec<u8>, VehicleError> {
    let from = path_data(from)?;
    let to = path_data(to)?;
    let data_len = from.len() + 1 + to.len();
    if data_len > FTP_DATA_CAPACITY {
        return Err(VehicleError::InvalidParameter(format!(
            "MAVFTP rename paths contain {data_len} bytes, exceeding the {FTP_DATA_CAPACITY}-byte packet limit"
        )));
    }
    let mut data = Vec::with_capacity(data_len);
    data.extend(from);
    data.push(0);
    data.extend(to);
    Ok(data)
}

fn parse_directory_entries(data: &[u8]) -> Result<Vec<FtpEntry>, VehicleError> {
    data.split(|byte| *byte == 0)
        .filter(|entry| !entry.is_empty())
        .map(parse_directory_entry)
        .collect()
}

fn parse_directory_entry(entry: &[u8]) -> Result<FtpEntry, VehicleError> {
    let (&marker, raw_name) = entry.split_first().ok_or_else(|| {
        FtpError::InvalidResponse("directory listing contained an empty entry".to_string())
    })?;
    let (kind, include_marker) = match marker {
        b'F' => (FtpEntryKind::File, false),
        b'D' => (FtpEntryKind::Directory, false),
        b'S' => (FtpEntryKind::Skip, false),
        _ => (FtpEntryKind::Unknown, true),
    };
    let raw_name = if include_marker { entry } else { raw_name };
    let name = std::str::from_utf8(raw_name).map_err(|err| {
        FtpError::InvalidResponse(format!("directory entry is not valid UTF-8: {err}"))
    })?;
    let (name, size) = if kind == FtpEntryKind::File {
        match name.rsplit_once('\t') {
            Some((name, size)) => {
                let size = size.parse::<u64>().map_err(|err| {
                    FtpError::InvalidResponse(format!("invalid file size '{size}': {err}"))
                })?;
                (name.to_string(), Some(size))
            }
            None => (name.to_string(), None),
        }
    } else {
        (name.to_string(), None)
    };
    Ok(FtpEntry { name, kind, size })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn packet_round_trip_preserves_header_and_data() {
        let packet = FtpPacket {
            sequence: 42,
            session: 7,
            opcode: OPCODE_WRITE_FILE,
            size: 3,
            requested_opcode: 0,
            burst_complete: 0,
            offset: 1234,
            data: vec![1, 2, 3],
        };
        let dialect::MavMessage::FILE_TRANSFER_PROTOCOL(message) =
            packet.to_message(FtpTarget::new(1, 1))
        else {
            panic!("packet should create FILE_TRANSFER_PROTOCOL");
        };

        assert_eq!(FtpPacket::from_payload(&message.payload), Ok(packet));
    }

    #[test]
    fn directory_entries_parse_files_directories_and_skips() {
        let entries = parse_directory_entries(b"Flog.bin\t123\0Dlogs\0S1\0")
            .expect("directory page should parse");

        assert_eq!(
            entries,
            vec![
                FtpEntry {
                    name: "log.bin".to_string(),
                    kind: FtpEntryKind::File,
                    size: Some(123),
                },
                FtpEntry {
                    name: "logs".to_string(),
                    kind: FtpEntryKind::Directory,
                    size: None,
                },
                FtpEntry {
                    name: "1".to_string(),
                    kind: FtpEntryKind::Skip,
                    size: None,
                },
            ]
        );
    }

    #[test]
    fn rename_rejects_combined_paths_larger_than_packet() {
        let path = "x".repeat(FTP_DATA_CAPACITY / 2 + 1);
        let error = rename_data(&path, &path).expect_err("combined rename paths should not fit");

        assert!(matches!(error, VehicleError::InvalidParameter(_)));
    }
}
