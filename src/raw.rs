use crate::VehicleError;
use crate::command::{Command, RawCommandIntPayload};
use crate::dialect::{self, MavCmd};
use crate::vehicle::Vehicle;
use mavlink::{MavHeader, MavlinkVersion, Message};
use num_traits::FromPrimitive;
use std::time::{Duration, Instant};
use tokio_stream::Stream;
use tokio_stream::StreamExt;
use tokio_stream::wrappers::BroadcastStream;

#[derive(Clone, Debug, PartialEq, Eq)]
/// Decoded `COMMAND_ACK` payload returned from raw command helpers.
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

#[derive(Clone, Debug)]
/// Raw MAVLink frame payload with sender identity and receive timestamp.
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

    fn to_mavlink(&self) -> Result<dialect::MavMessage, VehicleError> {
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

/// Low-level MAVLink escape hatch for commands and raw message streaming.
pub struct RawHandle<'a> {
    vehicle: &'a Vehicle,
}

impl<'a> RawHandle<'a> {
    pub(crate) fn new(vehicle: &'a Vehicle) -> Self {
        Self { vehicle }
    }

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

        self.vehicle
            .send_command(|reply| Command::RawCommandInt {
                payload: RawCommandIntPayload {
                    command,
                    frame,
                    current,
                    autocontinue,
                    params,
                    x,
                    y,
                    z,
                },
                reply,
            })
            .await
    }

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

        tokio::time::timeout(timeout, async {
            stream.next().await.ok_or(VehicleError::Disconnected)
        })
        .await
        .map_err(|_| VehicleError::Timeout)?
    }

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

    pub async fn send(&self, message: RawMessage) -> Result<(), VehicleError> {
        let message = message.to_mavlink()?;
        self.vehicle
            .send_command(|reply| Command::RawSend {
                message: Box::new(message),
                reply,
            })
            .await
    }

    pub fn subscribe(&self) -> impl Stream<Item = RawMessage> + Send + 'static {
        self.subscription_stream(None)
    }

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
        BroadcastStream::new(self.vehicle.inner.stores.raw_message_tx.subscribe()).filter_map(
            move |result| match result {
                Ok((header, message)) => {
                    let raw = RawMessage::from_mavlink(header, message);
                    message_id
                        .is_none_or(|expected| expected == raw.message_id)
                        .then_some(raw)
                }
                Err(_) => None,
            },
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Vehicle;
    use crate::config::VehicleConfig;
    use crate::dialect::{self, MavModeFlag, MavState};
    use crate::test_support::{MockConnection, SentMessages};
    use std::sync::{Arc, Mutex};
    use tokio::sync::mpsc;
    use tokio::time::timeout;

    fn default_header() -> MavHeader {
        MavHeader {
            system_id: 1,
            component_id: 1,
            sequence: 0,
        }
    }

    fn heartbeat_msg(armed: bool, custom_mode: u32) -> dialect::MavMessage {
        let mut base_mode = MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        if armed {
            base_mode |= MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED;
        }

        dialect::MavMessage::HEARTBEAT(dialect::HEARTBEAT_DATA {
            custom_mode,
            mavtype: dialect::MavType::MAV_TYPE_QUADROTOR,
            autopilot: dialect::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode,
            system_status: MavState::MAV_STATE_STANDBY,
            mavlink_version: 3,
        })
    }

    fn fast_config() -> VehicleConfig {
        VehicleConfig {
            connect_timeout: Duration::from_millis(150),
            command_timeout: Duration::from_millis(50),
            command_completion_timeout: Duration::from_millis(150),
            auto_request_home: false,
            ..VehicleConfig::default()
        }
    }

    fn ack_msg(
        command: dialect::MavCmd,
        result: dialect::MavResult,
        progress: u8,
        result_param2: i32,
    ) -> dialect::MavMessage {
        dialect::MavMessage::COMMAND_ACK(dialect::COMMAND_ACK_DATA {
            command,
            result,
            progress,
            result_param2,
            target_system: 0,
            target_component: 0,
        })
    }

    fn global_position_int_msg() -> dialect::MavMessage {
        dialect::MavMessage::GLOBAL_POSITION_INT(dialect::GLOBAL_POSITION_INT_DATA {
            time_boot_ms: 42,
            lat: 473_977_420,
            lon: 85_455_940,
            alt: 500_000,
            relative_alt: 15_000,
            vx: 0,
            vy: 0,
            vz: 0,
            hdg: 9_000,
        })
    }

    async fn connect_mock_vehicle_with_sent() -> (
        Vehicle,
        mpsc::Sender<(MavHeader, dialect::MavMessage)>,
        SentMessages,
    ) {
        let (msg_tx, msg_rx) = mpsc::channel(16);
        let (conn, sent) = MockConnection::new(msg_rx);
        let connect_task =
            tokio::spawn(
                async move { Vehicle::from_connection(Box::new(conn), fast_config()).await },
            );

        msg_tx
            .send((default_header(), heartbeat_msg(false, 7)))
            .await
            .expect("heartbeat should be delivered to mock connection");

        let vehicle = timeout(Duration::from_millis(250), connect_task)
            .await
            .expect("connect should complete after first heartbeat")
            .expect("connect task should join")
            .expect("mock vehicle should connect");

        (vehicle, msg_tx, sent)
    }

    #[tokio::test]
    async fn command_long_ack() {
        let (vehicle, msg_tx, sent) = connect_mock_vehicle_with_sent().await;

        let command_task = {
            let vehicle = vehicle.clone();
            tokio::spawn(async move {
                vehicle
                    .raw()
                    .command_long(
                        dialect::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM as u16,
                        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    )
                    .await
            })
        };

        tokio::time::sleep(Duration::from_millis(10)).await;
        msg_tx
            .send((
                default_header(),
                ack_msg(
                    dialect::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
                    dialect::MavResult::MAV_RESULT_ACCEPTED,
                    77,
                    13,
                ),
            ))
            .await
            .expect("command ack should be delivered");

        let ack = command_task
            .await
            .expect("command task should join")
            .expect("raw command_long should succeed");
        assert_eq!(
            ack.command,
            dialect::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM as u16
        );
        assert_eq!(ack.result, dialect::MavResult::MAV_RESULT_ACCEPTED as u8);
        assert_eq!(ack.progress, Some(77));
        assert_eq!(ack.result_param2, Some(13));

        assert!(sent.lock().unwrap().iter().any(|(_, msg)| matches!(
            msg,
        dialect::MavMessage::COMMAND_LONG(data)
            if data.command == dialect::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM
        )));

        vehicle
            .disconnect()
            .await
            .expect("disconnect should succeed");
    }

    #[test]
    fn command_ack_preserves_zero_values() {
        let ack = CommandAck::from_wire(dialect::COMMAND_ACK_DATA {
            command: dialect::MavCmd::MAV_CMD_REQUEST_MESSAGE,
            result: dialect::MavResult::MAV_RESULT_ACCEPTED,
            progress: 0,
            result_param2: 0,
            target_system: 0,
            target_component: 0,
        });

        assert_eq!(ack.progress, Some(0));
        assert_eq!(ack.result_param2, Some(0));
    }

    #[tokio::test]
    async fn send_bypasses_ack() {
        let (vehicle, _msg_tx, sent) = connect_mock_vehicle_with_sent().await;
        let raw_message = RawMessage::from_mavlink(
            default_header(),
            dialect::MavMessage::COMMAND_LONG(dialect::COMMAND_LONG_DATA {
                target_system: 1,
                target_component: 1,
                command: dialect::MavCmd::MAV_CMD_REQUEST_MESSAGE,
                confirmation: 0,
                param1: 33.0,
                param2: 0.0,
                param3: 0.0,
                param4: 0.0,
                param5: 0.0,
                param6: 0.0,
                param7: 0.0,
            }),
        );

        timeout(Duration::from_millis(100), vehicle.raw().send(raw_message))
            .await
            .expect("raw send should not wait for any COMMAND_ACK")
            .expect("raw send should succeed");

        assert!(sent.lock().unwrap().iter().any(|(_, msg)| matches!(
            msg,
        dialect::MavMessage::COMMAND_LONG(data)
            if data.command == dialect::MavCmd::MAV_CMD_REQUEST_MESSAGE && data.param1 == 33.0
        )));

        vehicle
            .disconnect()
            .await
            .expect("disconnect should succeed");
    }

    #[tokio::test]
    async fn subscribe_filtered() {
        let (vehicle, msg_tx, _sent) = connect_mock_vehicle_with_sent().await;
        let stream = vehicle.raw().subscribe_filtered(33);
        tokio::pin!(stream);

        msg_tx
            .send((
                default_header(),
                dialect::MavMessage::HEARTBEAT(dialect::HEARTBEAT_DATA::default()),
            ))
            .await
            .expect("heartbeat should be delivered");
        msg_tx
            .send((default_header(), global_position_int_msg()))
            .await
            .expect("global position int should be delivered");

        let message = timeout(Duration::from_millis(250), stream.next())
            .await
            .expect("filtered subscription should yield a matching message")
            .expect("filtered subscription should stay open");
        assert_eq!(message.message_id, 33);

        let decoded =
            dialect::MavMessage::parse(MavlinkVersion::V2, message.message_id, &message.payload)
                .expect("filtered raw payload should parse back into a typed MAVLink message");
        assert!(matches!(
            decoded,
            dialect::MavMessage::GLOBAL_POSITION_INT(_)
        ));

        vehicle
            .disconnect()
            .await
            .expect("disconnect should succeed");
    }

    #[tokio::test]
    async fn command_int_shares_ack_scope_with_typed_commands() {
        let (vehicle, msg_tx, _sent) = connect_mock_vehicle_with_sent().await;

        let arm_task = {
            let vehicle = vehicle.clone();
            tokio::spawn(async move { vehicle.arm(false).await })
        };

        tokio::time::sleep(Duration::from_millis(10)).await;
        let raw_conflict = vehicle
            .raw()
            .command_int(
                dialect::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM as u16,
                dialect::MavFrame::MAV_FRAME_GLOBAL as u8,
                0,
                0,
                [1.0, 0.0, 0.0, 0.0],
                0,
                0,
                0.0,
            )
            .await;

        assert!(matches!(
            raw_conflict,
            Err(VehicleError::OperationConflict {
                conflicting_domain,
                conflicting_op,
            }) if conflicting_domain == "command" && conflicting_op == "ack_key_in_flight"
        ));

        msg_tx
            .send((
                default_header(),
                ack_msg(
                    dialect::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
                    dialect::MavResult::MAV_RESULT_ACCEPTED,
                    0,
                    0,
                ),
            ))
            .await
            .expect("arm ack should be delivered");

        arm_task
            .await
            .expect("arm task should join")
            .expect("arm should succeed");

        vehicle
            .disconnect()
            .await
            .expect("disconnect should succeed");
    }
}
