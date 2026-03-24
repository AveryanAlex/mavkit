//! Error types returned by MAVKit operations.

use std::fmt;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// Normalized command-ack outcome used in high-level errors.
pub enum CommandResult {
    Denied,
    Failed,
    Unsupported,
    TemporarilyRejected,
    Other(u8),
}

impl fmt::Display for CommandResult {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Denied => write!(f, "denied"),
            Self::Failed => write!(f, "failed"),
            Self::Unsupported => write!(f, "unsupported"),
            Self::TemporarilyRejected => write!(f, "temporarily_rejected"),
            Self::Other(value) => write!(f, "other({value})"),
        }
    }
}

/// Errors that can occur during vehicle communication and operations.
#[derive(Debug, thiserror::Error)]
pub enum VehicleError {
    #[error("connection failed: {0}")]
    ConnectionFailed(String),
    #[error("vehicle disconnected")]
    Disconnected,
    #[error("command rejected: command={command}, result={result}")]
    CommandRejected { command: u16, result: CommandResult },
    #[error("command outcome unknown: command={command}")]
    OutcomeUnknown { command: u16 },
    #[error("operation timed out: {0}")]
    Timeout(String),
    #[error("unsupported: {0}")]
    Unsupported(String),
    #[error("invalid parameter: {0}")]
    InvalidParameter(String),
    #[error("mode '{0}' not available for this vehicle")]
    ModeNotAvailable(String),
    #[error("transfer failed [{domain}:{phase}] {detail}")]
    TransferFailed {
        domain: String,
        phase: String,
        detail: String,
    },
    #[error("operation conflict: domain={conflicting_domain}, op={conflicting_op}")]
    OperationConflict {
        conflicting_domain: String,
        conflicting_op: String,
    },
    #[error("operation cancelled")]
    Cancelled,
    #[error("mission validation failed: {0}")]
    MissionValidation(String),
    #[error("no heartbeat received yet")]
    IdentityUnknown,
    #[error("MAVLink I/O: {0}")]
    Io(#[from] std::io::Error),
}

#[cfg(test)]
mod tests {
    use super::{CommandResult, VehicleError};

    #[test]
    fn command_result_display() {
        assert_eq!(CommandResult::Denied.to_string(), "denied");
        assert_eq!(CommandResult::Failed.to_string(), "failed");
        assert_eq!(CommandResult::Unsupported.to_string(), "unsupported");
        assert_eq!(CommandResult::TemporarilyRejected.to_string(), "temporarily_rejected");
        assert_eq!(CommandResult::Other(42).to_string(), "other(42)");
    }

    #[test]
    fn command_rejected_uses_typed_payload() {
        let err = VehicleError::CommandRejected {
            command: 400,
            result: CommandResult::Denied,
        };

        assert!(matches!(
            err,
            VehicleError::CommandRejected {
                command: 400,
                result: CommandResult::Denied,
            }
        ));
    }

    #[test]
    fn display_includes_transfer_context() {
        let err = VehicleError::TransferFailed {
            domain: "mission".to_string(),
            phase: "await_ack".to_string(),
            detail: "no mission ack".to_string(),
        };

        let text = err.to_string();
        assert!(text.contains("mission"));
        assert!(text.contains("await_ack"));
        assert!(text.contains("no mission ack"));
    }

    #[test]
    fn io_error_has_source() {
        let io = std::io::Error::other("socket closed");
        let err = VehicleError::Io(io);

        let source = std::error::Error::source(&err);
        assert!(source.is_some());
    }

    #[test]
    fn operation_conflict_is_displayable() {
        let err = VehicleError::OperationConflict {
            conflicting_domain: "mission".to_string(),
            conflicting_op: "upload".to_string(),
        };
        let text = err.to_string();
        assert!(text.contains("mission"));
        assert!(text.contains("upload"));
    }

    #[test]
    fn timeout_preserves_context_string() {
        let err = VehicleError::Timeout("waiting for mode change".into());
        let text = err.to_string();
        assert!(
            text.contains("waiting for mode change"),
            "Timeout display should include context: {text}"
        );

        let err2 = VehicleError::Timeout("sending command".into());
        assert!(err2.to_string().contains("sending command"));
    }
}
