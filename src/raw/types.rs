use crate::VehicleError;
use crate::dialect;
use crate::time::Instant;
use mavlink::{MavHeader, MavlinkVersion, Message};

/// Decoded `COMMAND_ACK` payload returned from raw command helpers.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CommandAck {
    pub command: u16,
    pub result: u8,
    pub progress: Option<u8>,
    pub result_param2: Option<i32>,
}

impl CommandAck {
    #[cfg(test)]
    pub(crate) fn from_wire(ack: dialect::COMMAND_ACK_DATA) -> Self {
        Self::from_wire_values(
            ack.command as u16,
            ack.result as u8,
            ack.progress,
            ack.result_param2,
        )
    }

    pub(crate) fn from_wire_values(
        command: u16,
        result: u8,
        progress: u8,
        result_param2: i32,
    ) -> Self {
        Self {
            command,
            result,
            progress: Some(progress),
            result_param2: Some(result_param2),
        }
    }
}

/// Raw MAVLink frame payload with sender identity and receive timestamp.
#[derive(Clone, Debug)]
pub struct RawMessage {
    pub message_id: u32,
    pub system_id: u8,
    pub component_id: u8,
    pub payload: Vec<u8>,
    pub received_at: Instant,
}

impl PartialEq for RawMessage {
    fn eq(&self, other: &Self) -> bool {
        self.message_id == other.message_id
            && self.system_id == other.system_id
            && self.component_id == other.component_id
            && self.payload == other.payload
    }
}

impl RawMessage {
    pub(crate) fn from_mavlink(header: MavHeader, message: dialect::MavMessage) -> Self {
        let mut payload = [0_u8; 255];
        let payload_len = message.ser(MavlinkVersion::V2, &mut payload);

        Self {
            message_id: message.message_id(),
            system_id: header.system_id,
            component_id: header.component_id,
            payload: payload[..payload_len].to_vec(),
            received_at: Instant::now(),
        }
    }

    pub(crate) fn to_mavlink(&self) -> Result<dialect::MavMessage, VehicleError> {
        dialect::MavMessage::parse(MavlinkVersion::V2, self.message_id, &self.payload).map_err(
            |err| {
                VehicleError::InvalidParameter(format!(
                    "invalid raw MAVLink payload for message_id {}: {err}",
                    self.message_id
                ))
            },
        )
    }
}
