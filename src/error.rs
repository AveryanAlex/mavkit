#[derive(Debug, thiserror::Error)]
pub enum VehicleError {
    #[error("connection failed: {0}")]
    ConnectionFailed(String),
    #[error("vehicle disconnected")]
    Disconnected,
    #[error("operation timed out")]
    Timeout,
    #[error("operation cancelled")]
    Cancelled,
    #[error("command {command} rejected: {result}")]
    CommandRejected { command: String, result: String },
    #[error("no heartbeat received yet")]
    IdentityUnknown,
    #[error("mode '{0}' not available for this vehicle")]
    ModeNotAvailable(String),
    #[error("mission transfer failed: [{code}] {message}")]
    MissionTransfer { code: String, message: String },
    #[error("mission validation failed: {0}")]
    MissionValidation(String),
    #[error("MAVLink I/O: {0}")]
    Io(#[from] std::io::Error),
}
