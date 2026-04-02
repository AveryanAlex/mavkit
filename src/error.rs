//! Error types returned by MAVKit operations.

use std::fmt;

/// Normalized command-ack outcome reported by the vehicle via `COMMAND_ACK`.
///
/// Carried inside [`VehicleError::CommandRejected`] to let callers distinguish
/// retriable failures (`TemporarilyRejected`) from permanent ones.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommandResult {
    /// The vehicle understood the command but refused it (e.g. pre-arm checks failed).
    Denied,
    /// The vehicle accepted the command but execution failed internally.
    Failed,
    /// The command is not implemented by this vehicle firmware.
    Unsupported,
    /// The vehicle cannot run the command right now; retry after conditions change.
    TemporarilyRejected,
    /// Any raw result code not mapped to a named variant.
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

/// Structured reason why a single mission item failed pre-upload validation.
///
/// Carried inside [`VehicleError::InvalidMissionItem`].
#[derive(Debug, Clone)]
pub enum MissionValidationReason {
    /// A coordinate value is outside its legal range.
    InvalidCoordinate { detail: String },
    /// A required field is absent or unset.
    MissingField { field: &'static str },
    /// A numeric parameter is outside the allowed range.
    ParameterOutOfRange {
        field: &'static str,
        value: f64,
        min: f64,
        max: f64,
    },
    /// The command type is not recognised or supported.
    UnsupportedCommand { command: u16 },
    /// Any other validation failure not covered by the above.
    Other(String),
}

impl fmt::Display for MissionValidationReason {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidCoordinate { detail } => write!(f, "invalid coordinate: {detail}"),
            Self::MissingField { field } => write!(f, "missing field: {field}"),
            Self::ParameterOutOfRange {
                field,
                value,
                min,
                max,
            } => write!(
                f,
                "parameter out of range: {field}={value} not in [{min}, {max}]"
            ),
            Self::UnsupportedCommand { command } => {
                write!(f, "unsupported command: {command}")
            }
            Self::Other(msg) => write!(f, "{msg}"),
        }
    }
}

/// Errors that can occur during vehicle communication and operations.
#[derive(Debug, thiserror::Error)]
pub enum VehicleError {
    /// The underlying transport could not be established or the initial handshake failed.
    ///
    /// Occurs during [`Vehicle::connect`](crate::Vehicle::connect) and related constructors.
    /// Check the address string, firewall rules, and that the vehicle is reachable.
    #[error("connection failed: {0}")]
    ConnectionFailed(String),

    /// The MAVLink link dropped after a successful connection, or the event loop exited.
    ///
    /// Any in-flight operation returns this when the channel it is waiting on closes.
    /// Re-connect via a new [`Vehicle::connect`](crate::Vehicle::connect) call.
    #[error("vehicle disconnected")]
    Disconnected,

    /// The vehicle replied with a non-accepted `COMMAND_ACK`.
    ///
    /// Inspect `result` to distinguish permanent failures (`Denied`, `Unsupported`) from
    /// transient ones (`TemporarilyRejected`). The `command` field carries the raw MAVLink
    /// command ID for cross-referencing with the MAVLink specification.
    #[error("command rejected: command={command}, result={result}")]
    CommandRejected { command: u16, result: CommandResult },

    /// A command was sent but the outcome is unknown — the operation may or may not have taken effect.
    ///
    /// Typically happens when the caller cancels the operation or the connection drops after the
    /// command was transmitted but before an ACK was received. The `context` field describes
    /// which phase of the operation was interrupted. Treat as a possible partial-completion and
    /// re-verify vehicle state before retrying.
    #[error("command outcome unknown: command={command} ({context})")]
    OutcomeUnknown { command: u16, context: String },

    /// A deadline elapsed before the expected response arrived.
    ///
    /// The context string identifies which phase timed out (e.g. `"connecting to vehicle"`,
    /// `"waiting for mode change"`). Increase the relevant timeout in
    /// [`VehicleConfig`](crate::config::VehicleConfig) or check link latency.
    #[error("operation timed out: {0}")]
    Timeout(String),

    /// The requested operation is not supported by this vehicle or firmware version.
    #[error("unsupported: {0}")]
    Unsupported(String),

    /// A caller-supplied argument value is out of range or semantically invalid.
    ///
    /// Returned before any network traffic is sent, so the vehicle state is unchanged.
    #[error("invalid parameter: {0}")]
    InvalidParameter(String),

    /// The requested mode name does not appear in the vehicle's available-modes catalog.
    ///
    /// Only returned by [`Vehicle::set_mode_by_name`](crate::Vehicle::set_mode_by_name) and its
    /// `_no_wait` variant. Try reading `vehicle.available_modes()` to see what modes the vehicle
    /// reports, or use `vehicle.set_mode(custom_mode)` with a numeric ID.
    #[error("mode '{0}' not available for this vehicle")]
    ModeNotAvailable(String),

    /// A mission, fence, rally, or parameter transfer failed mid-flight.
    ///
    /// `domain` names the operation type (`"mission"`, `"params"`, etc.); `phase` names the
    /// protocol step where failure occurred. Retrying is generally safe; if the problem persists,
    /// increase [`VehicleConfig::transfer_timeout`](crate::config::VehicleConfig::transfer_timeout)
    /// or inspect link quality.
    #[error("transfer failed [{domain}:{phase}] {detail}")]
    TransferFailed {
        domain: String,
        phase: String,
        detail: String,
    },

    /// A second concurrent operation was attempted on a domain that only allows one at a time.
    ///
    /// `conflicting_domain` names the affected domain and `conflicting_op` names the operation
    /// already in progress. Wait for the first operation to finish (or cancel it) before starting
    /// another.
    #[error("operation conflict: domain={conflicting_domain}, op={conflicting_op}")]
    OperationConflict {
        conflicting_domain: String,
        conflicting_op: String,
    },

    /// The caller cancelled the operation via its [`CancellationToken`](tokio_util::sync::CancellationToken).
    ///
    /// The vehicle state after cancellation is undefined — re-query telemetry before acting.
    #[error("operation cancelled")]
    Cancelled,

    /// A single mission item failed pre-upload validation.
    ///
    /// `index` is the 0-based position in the caller-supplied item list. The upload was rejected
    /// entirely; no items were sent to the vehicle. Fix the item at `index` and retry.
    #[error("invalid mission item at index {index}: {reason}")]
    InvalidMissionItem {
        index: usize,
        reason: MissionValidationReason,
    },

    /// The mission plan as a whole is invalid (e.g. too many items, incompatible item types).
    ///
    /// No items were sent to the vehicle. The error message describes the constraint that was
    /// violated.
    #[error("invalid mission plan: {0}")]
    InvalidMissionPlan(String),

    /// A parameter fetch or typed read was attempted for a name the vehicle does not know.
    ///
    /// Verify the parameter name spelling and that the vehicle firmware exposes it. Only returned
    /// by typed parameter accessors on [`ParamsHandle`](crate::ParamsHandle).
    #[error("parameter not found: {name}")]
    ParameterNotFound { name: String },

    /// A typed parameter accessor was called but the vehicle reported a different type.
    ///
    /// `expected` is the Rust-side type requested and `actual` is the raw MAVLink type the vehicle
    /// returned. Use the correct typed accessor or read the parameter as an untyped value first.
    #[error("parameter type mismatch for {name}: expected {expected}, got {actual}")]
    ParameterTypeMismatch {
        name: String,
        expected: String,
        actual: String,
    },

    /// [`Vehicle::identity`](crate::Vehicle::identity) was called before any heartbeat arrived.
    ///
    /// This should not normally happen because `Vehicle::connect` waits for the first heartbeat
    /// before returning. It can occur on a vehicle handle that was constructed directly without
    /// going through the connect path.
    #[error("no heartbeat received yet")]
    IdentityUnknown,

    /// An underlying I/O error from the MAVLink transport layer.
    #[error("MAVLink I/O: {0}")]
    Io(#[from] std::io::Error),
}

#[cfg(test)]
mod tests {
    use super::{CommandResult, MissionValidationReason, VehicleError};

    #[test]
    fn command_result_display() {
        assert_eq!(CommandResult::Denied.to_string(), "denied");
        assert_eq!(CommandResult::Failed.to_string(), "failed");
        assert_eq!(CommandResult::Unsupported.to_string(), "unsupported");
        assert_eq!(
            CommandResult::TemporarilyRejected.to_string(),
            "temporarily_rejected"
        );
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

    #[test]
    fn invalid_mission_item_display() {
        let err = VehicleError::InvalidMissionItem {
            index: 3,
            reason: MissionValidationReason::InvalidCoordinate {
                detail: "latitude 120 outside [-90, 90]".to_string(),
            },
        };
        let text = err.to_string();
        assert!(text.contains("index 3"));
        assert!(text.contains("latitude 120"));
    }

    #[test]
    fn invalid_mission_plan_display() {
        let err = VehicleError::InvalidMissionPlan("too many items".to_string());
        assert!(err.to_string().contains("too many items"));
    }

    #[test]
    fn parameter_not_found_display() {
        let err = VehicleError::ParameterNotFound {
            name: "BATT_CAPACITY".to_string(),
        };
        assert!(err.to_string().contains("BATT_CAPACITY"));
    }

    #[test]
    fn parameter_type_mismatch_display() {
        let err = VehicleError::ParameterTypeMismatch {
            name: "ARMING_CHECK".to_string(),
            expected: "Int32".to_string(),
            actual: "Real32".to_string(),
        };
        let text = err.to_string();
        assert!(text.contains("ARMING_CHECK"));
        assert!(text.contains("Int32"));
        assert!(text.contains("Real32"));
    }

    #[test]
    fn outcome_unknown_includes_context() {
        let err = VehicleError::OutcomeUnknown {
            command: 400,
            context: "cancelled after send".to_string(),
        };
        let text = err.to_string();
        assert!(text.contains("400"));
        assert!(text.contains("cancelled after send"));
    }

    #[test]
    fn mission_validation_reason_display() {
        assert!(
            MissionValidationReason::MissingField { field: "altitude" }
                .to_string()
                .contains("altitude")
        );
        assert!(
            MissionValidationReason::ParameterOutOfRange {
                field: "speed",
                value: 200.0,
                min: 0.0,
                max: 100.0,
            }
            .to_string()
            .contains("speed")
        );
        assert!(
            MissionValidationReason::UnsupportedCommand { command: 999 }
                .to_string()
                .contains("999")
        );
        assert!(
            MissionValidationReason::Other("bad".to_string())
                .to_string()
                .contains("bad")
        );
    }
}
