use crate::config::VehicleConfig;
use crate::dialect::{self, MavCmd};
use crate::error::{CommandResult, VehicleError};
use crate::raw::CommandAck;
use mavlink::{AsyncMavConnection, MavHeader};
use num_traits::FromPrimitive;
use std::collections::HashMap;
use std::sync::Mutex;
use tokio::sync::mpsc;
use tokio_util::sync::CancellationToken;

type MavConnection = dyn AsyncMavConnection<dialect::MavMessage> + Sync + Send;

pub(super) struct CommandLongRequest<'a> {
    pub(super) connection: &'a MavConnection,
    pub(super) config: &'a VehicleConfig,
    pub(super) cancel: &'a CancellationToken,
    pub(super) target_system: u8,
    pub(super) target_component: u8,
    pub(super) command_id: u16,
    pub(super) params: [f32; 7],
}

pub(super) struct CommandIntRequest<'a> {
    pub(super) connection: &'a MavConnection,
    pub(super) config: &'a VehicleConfig,
    pub(super) cancel: &'a CancellationToken,
    pub(super) target_system: u8,
    pub(super) target_component: u8,
    pub(super) command: MavCmd,
    pub(super) frame: dialect::MavFrame,
    pub(super) current: u8,
    pub(super) autocontinue: u8,
    pub(super) params: [f32; 4],
    pub(super) x: i32,
    pub(super) y: i32,
    pub(super) z: f32,
}

#[derive(Debug, Clone, Copy)]
pub(super) struct WireCommandAck {
    pub(super) command_id: u16,
    pub(super) result: u8,
    pub(super) progress: u8,
    pub(super) result_param2: i32,
    pub(super) target_system: u8,
    pub(super) target_component: u8,
}

impl From<dialect::COMMAND_ACK_DATA> for WireCommandAck {
    fn from(value: dialect::COMMAND_ACK_DATA) -> Self {
        Self {
            command_id: value.command as u16,
            result: value.result as u8,
            progress: value.progress,
            result_param2: value.result_param2,
            target_system: value.target_system,
            target_component: value.target_component,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct CommandKey {
    target_system: u8,
    target_component: u8,
    command_id: u16,
}

impl CommandKey {
    fn new(target_system: u8, target_component: u8, command_id: u16) -> Self {
        Self {
            target_system,
            target_component,
            command_id,
        }
    }
}

pub(super) struct AckCommandDispatcher {
    local_system_id: u8,
    local_component_id: u8,
    active: Mutex<HashMap<CommandKey, mpsc::UnboundedSender<WireCommandAck>>>,
}

impl AckCommandDispatcher {
    pub(super) fn new(local_system_id: u8, local_component_id: u8) -> Self {
        Self {
            local_system_id,
            local_component_id,
            active: Mutex::new(HashMap::new()),
        }
    }

    pub(super) fn route_command_ack(&self, header: MavHeader, ack: &dialect::COMMAND_ACK_DATA) {
        self.route_command_ack_raw(header, (*ack).clone().into());
    }

    pub(super) fn route_command_ack_raw(&self, header: MavHeader, ack: WireCommandAck) {
        if ack.target_system != 0 && ack.target_system != self.local_system_id {
            return;
        }
        if ack.target_component != 0 && ack.target_component != self.local_component_id {
            return;
        }

        let key = CommandKey::new(header.system_id, header.component_id, ack.command_id);
        let tx = {
            let active = self.active.lock().unwrap();
            active.get(&key).cloned()
        };

        if let Some(tx) = tx {
            let _ = tx.send(ack);
        }
    }

    pub(super) async fn dispatch_command_long(
        &self,
        request: CommandLongRequest<'_>,
    ) -> Result<CommandAck, VehicleError> {
        let key = CommandKey::new(
            request.target_system,
            request.target_component,
            request.command_id,
        );
        let mut ack_rx = self.register_active_key(key)?;

        let result = self.run_command_long(&request, &mut ack_rx).await;

        self.unregister_active_key(key);
        result
    }

    pub(super) async fn dispatch_command_int(
        &self,
        request: CommandIntRequest<'_>,
    ) -> Result<CommandAck, VehicleError> {
        let key = CommandKey::new(
            request.target_system,
            request.target_component,
            request.command as u16,
        );
        let mut ack_rx = self.register_active_key(key)?;

        let result = self.run_command_int(&request, &mut ack_rx).await;

        self.unregister_active_key(key);
        result
    }

    fn register_active_key(
        &self,
        key: CommandKey,
    ) -> Result<mpsc::UnboundedReceiver<WireCommandAck>, VehicleError> {
        let mut active = self.active.lock().unwrap();
        if active.contains_key(&key) {
            return Err(VehicleError::OperationConflict {
                conflicting_domain: "command".to_string(),
                conflicting_op: "ack_key_in_flight".to_string(),
            });
        }

        let (tx, rx) = mpsc::unbounded_channel();
        active.insert(key, tx);
        Ok(rx)
    }

    fn unregister_active_key(&self, key: CommandKey) {
        let mut active = self.active.lock().unwrap();
        active.remove(&key);
    }

    async fn run_command_long(
        &self,
        request: &CommandLongRequest<'_>,
        ack_rx: &mut mpsc::UnboundedReceiver<WireCommandAck>,
    ) -> Result<CommandAck, VehicleError> {
        let mut sent_once = false;

        for attempt in 0..=u16::from(request.config.retry_policy.max_retries) {
            if request.cancel.is_cancelled() {
                return Self::cancelled_error(sent_once, request.command_id);
            }

            super::send_message(
                request.connection,
                request.config,
                dialect::MavMessage::COMMAND_LONG(dialect::COMMAND_LONG_DATA {
                    target_system: request.target_system,
                    target_component: request.target_component,
                    command: Self::wire_command_from_id(request.command_id)?,
                    confirmation: attempt.min(u16::from(u8::MAX)) as u8,
                    param1: request.params[0],
                    param2: request.params[1],
                    param3: request.params[2],
                    param4: request.params[3],
                    param5: request.params[4],
                    param6: request.params[5],
                    param7: request.params[6],
                }),
            )
            .await?;
            sent_once = true;

            match Self::recv_ack_or_timeout(
                ack_rx,
                request.config.command_timeout,
                request.cancel,
                sent_once,
                request.command_id,
            )
            .await?
            {
                None => continue,
                Some(ack) if Self::is_in_progress(ack.result) => {
                    return Self::wait_for_terminal_ack(
                        ack_rx,
                        request.config.command_completion_timeout,
                        request.cancel,
                        request.command_id,
                    )
                    .await;
                }
                Some(ack) => return Self::map_terminal_ack(request.command_id, ack),
            }
        }

        Err(VehicleError::Timeout("command ack".into()))
    }

    async fn run_command_int(
        &self,
        request: &CommandIntRequest<'_>,
        ack_rx: &mut mpsc::UnboundedReceiver<WireCommandAck>,
    ) -> Result<CommandAck, VehicleError> {
        let mut sent_once = false;

        for _ in 0..=u16::from(request.config.retry_policy.max_retries) {
            if request.cancel.is_cancelled() {
                return Self::cancelled_error(sent_once, request.command as u16);
            }

            super::send_message(
                request.connection,
                request.config,
                dialect::MavMessage::COMMAND_INT(dialect::COMMAND_INT_DATA {
                    target_system: request.target_system,
                    target_component: request.target_component,
                    frame: request.frame,
                    command: request.command,
                    current: request.current,
                    autocontinue: request.autocontinue,
                    param1: request.params[0],
                    param2: request.params[1],
                    param3: request.params[2],
                    param4: request.params[3],
                    x: request.x,
                    y: request.y,
                    z: request.z,
                }),
            )
            .await?;
            sent_once = true;

            let command_id = request.command as u16;
            match Self::recv_ack_or_timeout(
                ack_rx,
                request.config.command_timeout,
                request.cancel,
                sent_once,
                command_id,
            )
            .await?
            {
                None => continue,
                Some(ack) if Self::is_in_progress(ack.result) => {
                    return Self::wait_for_terminal_ack(
                        ack_rx,
                        request.config.command_completion_timeout,
                        request.cancel,
                        command_id,
                    )
                    .await;
                }
                Some(ack) => return Self::map_terminal_ack(command_id, ack),
            }
        }

        Err(VehicleError::Timeout("command ack".into()))
    }

    async fn wait_for_terminal_ack(
        ack_rx: &mut mpsc::UnboundedReceiver<WireCommandAck>,
        timeout: std::time::Duration,
        cancel: &CancellationToken,
        command_id: u16,
    ) -> Result<CommandAck, VehicleError> {
        let deadline = tokio::time::Instant::now() + timeout;

        loop {
            let now = tokio::time::Instant::now();
            if now >= deadline {
                return Err(VehicleError::Timeout("command ack".into()));
            }
            let remaining = deadline - now;

            let maybe_ack =
                Self::recv_ack_or_timeout(ack_rx, remaining, cancel, true, command_id).await?;
            match maybe_ack {
                None => return Err(VehicleError::Timeout("command ack".into())),
                Some(ack) if Self::is_in_progress(ack.result) => continue,
                Some(ack) => return Self::map_terminal_ack(command_id, ack),
            }
        }
    }

    async fn recv_ack_or_timeout(
        ack_rx: &mut mpsc::UnboundedReceiver<WireCommandAck>,
        timeout: std::time::Duration,
        cancel: &CancellationToken,
        sent_once: bool,
        command_id: u16,
    ) -> Result<Option<WireCommandAck>, VehicleError> {
        tokio::select! {
            biased;
            _ = cancel.cancelled() => {
                if sent_once {
                    Err(VehicleError::OutcomeUnknown {
                        command: command_id,
                    })
                } else {
                    Err(VehicleError::Cancelled)
                }
            },
            ack = ack_rx.recv() => {
                match ack {
                    Some(ack) => Ok(Some(ack)),
                    None => Err(VehicleError::Disconnected),
                }
            }
            _ = tokio::time::sleep(timeout) => Ok(None),
        }
    }

    fn cancelled_error(sent_once: bool, command_id: u16) -> Result<CommandAck, VehicleError> {
        if sent_once {
            Err(VehicleError::OutcomeUnknown {
                command: command_id,
            })
        } else {
            Err(VehicleError::Cancelled)
        }
    }

    fn map_terminal_ack(command_id: u16, ack: WireCommandAck) -> Result<CommandAck, VehicleError> {
        if Self::is_accepted(ack.result) {
            return Ok(CommandAck::from_wire_values(
                ack.command_id,
                ack.result,
                ack.progress,
                ack.result_param2,
            ));
        }

        Err(VehicleError::CommandRejected {
            command: command_id,
            result: Self::map_command_result(ack.result),
        })
    }

    fn wire_command_from_id(command_id: u16) -> Result<MavCmd, VehicleError> {
        MavCmd::from_u16(command_id).ok_or_else(|| {
            VehicleError::Unsupported(format!(
                "COMMAND_LONG id {command_id} is not representable by mavkit::dialect::MavCmd"
            ))
        })
    }

    fn is_in_progress(result: u8) -> bool {
        result == dialect::MavResult::MAV_RESULT_IN_PROGRESS as u8
    }

    fn is_accepted(result: u8) -> bool {
        result == dialect::MavResult::MAV_RESULT_ACCEPTED as u8
    }

    fn map_command_result(result: u8) -> CommandResult {
        if result == dialect::MavResult::MAV_RESULT_DENIED as u8 {
            return CommandResult::Denied;
        }
        if result == dialect::MavResult::MAV_RESULT_FAILED as u8 {
            return CommandResult::Failed;
        }
        if result == dialect::MavResult::MAV_RESULT_UNSUPPORTED as u8 {
            return CommandResult::Unsupported;
        }
        if result == dialect::MavResult::MAV_RESULT_TEMPORARILY_REJECTED as u8 {
            return CommandResult::TemporarilyRejected;
        }

        CommandResult::Other(result)
    }
}
