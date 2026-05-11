//! Raw MAVLink value types, subscriptions, and low-level handle seams.
//!
//! Public access remains rooted at [`RawHandle`], [`RawMessage`], and [`CommandAck`] while
//! wire/value helpers, subscription machinery, and exact-path regressions live in smaller files.

mod subscription;
#[cfg(test)]
mod tests;
mod types;

use crate::VehicleError;
use crate::command::{Command, CommandIntPayload, send_command_int_ack};
use crate::dialect::{self, MavCmd};
use crate::runtime;
use crate::vehicle::Vehicle;
use num_traits::FromPrimitive;
use std::time::Duration;
use tokio_stream::{Stream, StreamExt};

pub use types::{CommandAck, RawMessage};

use subscription::RawSubscription;

/// Low-level MAVLink escape hatch for commands and raw message streaming.
///
/// Obtained from [`Vehicle::raw`](crate::Vehicle::raw). All high-level vehicle actions use the
/// typed `Vehicle` API; this handle is intended for one-off protocol exploration, debugging, and
/// capabilities not yet exposed through the typed API.
///
/// # Conflict model
///
/// `command_long` and `command_int` share the same command-ack scope as the typed vehicle
/// commands (e.g., `Vehicle::arm`). Sending a raw command while any typed command is awaiting
/// its ack returns [`VehicleError::OperationConflict`] immediately.
pub struct RawHandle<'a> {
    vehicle: &'a Vehicle,
}

impl<'a> RawHandle<'a> {
    pub(crate) fn new(vehicle: &'a Vehicle) -> Self {
        Self { vehicle }
    }

    /// Sends a `COMMAND_LONG` and waits for a `COMMAND_ACK`.
    ///
    /// `command` must be a valid `MAV_CMD` numeric value; an unknown value returns
    /// [`VehicleError::InvalidParameter`] immediately without sending anything.
    ///
    /// Returns [`VehicleError::OperationConflict`] if a typed command is already awaiting an ack.
    /// Returns [`VehicleError::Timeout`] if no ack arrives within the configured command timeout.
    pub async fn command_long(
        &self,
        command: u16,
        params: [f32; 7],
    ) -> Result<CommandAck, VehicleError> {
        let command = parse_command(command)?;
        self.vehicle
            .send_command(|reply| Command::RawCommandLongAck {
                command,
                params,
                reply,
            })
            .await
    }

    /// Sends a `COMMAND_INT` and waits for a `COMMAND_ACK`.
    ///
    /// `command` must be a valid `MAV_CMD` numeric value; `frame` must be a valid `MAV_FRAME`
    /// numeric value. Unknown values return [`VehicleError::InvalidParameter`] immediately.
    ///
    /// Returns [`VehicleError::OperationConflict`] if a typed command is already awaiting an ack.
    /// Returns [`VehicleError::Timeout`] if no ack arrives within the configured command timeout.
    #[allow(
        clippy::too_many_arguments,
        reason = "public RawHandle::command_int signature is required by the Task 21 design contract"
    )]
    pub async fn command_int(
        &self,
        command: u16,
        frame: u8,
        current: u8,
        autocontinue: u8,
        params: [f32; 4],
        x: i32,
        y: i32,
        z: f32,
    ) -> Result<CommandAck, VehicleError> {
        let command = parse_command(command)?;
        let frame = parse_frame(frame)?;

        send_command_int_ack(
            self.vehicle.inner.command_tx.clone(),
            CommandIntPayload {
                command,
                frame,
                current,
                autocontinue,
                params,
                x,
                y,
                z,
            },
        )
        .await
    }

    /// Sends `MAV_CMD_REQUEST_MESSAGE` for `message_id` and returns the first matching message.
    ///
    /// Subscribes to the raw message stream before issuing the request to avoid missing the
    /// response. Returns [`VehicleError::Timeout`] if no matching message arrives within
    /// `timeout`.
    pub async fn request_message(
        &self,
        message_id: u32,
        timeout: Duration,
    ) -> Result<RawMessage, VehicleError> {
        let stream = self.subscribe_filtered(message_id);
        tokio::pin!(stream);

        self.vehicle
            .send_command(|reply| Command::RawCommandLong {
                command: MavCmd::MAV_CMD_REQUEST_MESSAGE,
                params: [message_id as f32, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                reply,
            })
            .await?;

        runtime::timeout(timeout, async {
            stream.next().await.ok_or(VehicleError::Disconnected)
        })
        .await
        .map_err(|_| VehicleError::Timeout("raw command".into()))?
    }

    /// Requests the vehicle to stream `message_id` at the specified interval.
    ///
    /// `interval_us` is microseconds between messages; 0 disables the stream, -1 requests the
    /// default rate. Sends `MAV_CMD_SET_MESSAGE_INTERVAL` without waiting for an ack.
    pub async fn set_message_interval(
        &self,
        message_id: u32,
        interval_us: i32,
    ) -> Result<(), VehicleError> {
        self.vehicle
            .send_command(|reply| Command::RawCommandLong {
                command: MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
                params: [
                    message_id as f32,
                    interval_us as f32,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
                reply,
            })
            .await
    }

    /// Sends a raw MAVLink message to the vehicle without waiting for any ack.
    ///
    /// The `RawMessage` payload is re-parsed before sending to ensure it is a valid MAVLink
    /// message for the active dialect. Returns [`VehicleError::InvalidParameter`] if the payload
    /// cannot be re-parsed.
    pub async fn send(&self, message: RawMessage) -> Result<(), VehicleError> {
        let message = message.to_mavlink()?;
        self.vehicle
            .send_command(|reply| Command::RawSend {
                message: Box::new(message),
                reply,
            })
            .await
    }

    /// Returns a stream of all incoming MAVLink messages as raw frames.
    ///
    /// Messages may be lagged if the consumer is slow — lagged items are silently dropped by the
    /// underlying broadcast channel. The stream closes when the link transitions to a terminal
    /// state or when the underlying raw broadcast channel closes.
    pub fn subscribe(&self) -> impl Stream<Item = RawMessage> + Send + 'static {
        self.subscription_stream(None)
    }

    /// Returns a stream of incoming MAVLink messages filtered to the given `message_id`.
    ///
    /// Equivalent to [`subscribe`](Self::subscribe) with a client-side filter. Useful when
    /// polling for a specific response without allocating a full message fanout. Like
    /// [`subscribe`](Self::subscribe), lagged items are dropped and the stream closes on
    /// disconnect or broadcast shutdown.
    pub fn subscribe_filtered(
        &self,
        message_id: u32,
    ) -> impl Stream<Item = RawMessage> + Send + 'static {
        self.subscription_stream(Some(message_id))
    }

    fn subscription_stream(
        &self,
        message_id: Option<u32>,
    ) -> impl Stream<Item = RawMessage> + Send + 'static {
        RawSubscription::new(
            self.vehicle.inner.stores.raw_message_tx.subscribe(),
            self.vehicle.inner.stores.link_state_observation.subscribe(),
            message_id,
        )
    }
}

fn parse_command(command: u16) -> Result<MavCmd, VehicleError> {
    MavCmd::from_u16(command)
        .ok_or_else(|| VehicleError::InvalidParameter(format!("unknown MAV_CMD value {command}")))
}

fn parse_frame(frame: u8) -> Result<dialect::MavFrame, VehicleError> {
    dialect::MavFrame::from_u8(frame)
        .ok_or_else(|| VehicleError::InvalidParameter(format!("unknown MAV_FRAME value {frame}")))
}
