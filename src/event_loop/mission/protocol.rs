use super::super::{CommandContext, recv_routed, send_message};
use crate::dialect;
use crate::error::VehicleError;
use crate::mission::{MissionTransferMachine, MissionType};
use std::time::Duration;
use tokio_util::sync::CancellationToken;

use super::convert::to_mav_mission_type;

fn transfer_domain(mission_type: MissionType) -> &'static str {
    match mission_type {
        MissionType::Mission => "mission",
        MissionType::Fence => "fence",
        MissionType::Rally => "rally",
    }
}

pub(super) fn transfer_failed(
    mission_type: MissionType,
    phase: impl Into<String>,
    detail: impl Into<String>,
) -> VehicleError {
    VehicleError::TransferFailed {
        domain: transfer_domain(mission_type).to_string(),
        phase: phase.into(),
        detail: detail.into(),
    }
}

pub(super) async fn wait_for_mission_ack<F>(
    machine: &mut MissionTransferMachine,
    mission_type: MissionType,
    ctx: &mut CommandContext,
    op_cancel: &CancellationToken,
    retry_msg: F,
) -> Result<(), VehicleError>
where
    F: Fn() -> dialect::MavMessage,
{
    let mav_mission_type = to_mav_mission_type(mission_type);
    loop {
        let timeout = Duration::from_millis(machine.timeout_ms());
        let deadline = tokio::time::sleep(timeout);
        tokio::pin!(deadline);
        let cancel = op_cancel.clone();

        tokio::select! {
            biased;
            _ = cancel.cancelled() => {
                machine.cancel();
                let _ = ctx.writers.mission_progress.send(Some(machine.progress()));
                return Err(VehicleError::Cancelled);
            }
            _ = &mut deadline => {
                if let Some(err) = machine.on_timeout() {
                    let _ = ctx.writers.mission_progress.send(Some(machine.progress()));
                    return Err(transfer_failed(mission_type, err.code, err.message));
                }
                let _ = ctx.writers.mission_progress.send(Some(machine.progress()));
                send_message(ctx.connection.as_ref(), &ctx.config, retry_msg()).await?;
            }
            result = recv_routed(&mut ctx.inbound_rx) => {
                let (_, msg) = result?;

                if let dialect::MavMessage::MISSION_ACK(data) = &msg {
                    if data.mission_type != mav_mission_type {
                        continue;
                    }
                    if data.mavtype == dialect::MavMissionResult::MAV_MISSION_ACCEPTED {
                        machine.on_ack_success();
                        let _ = ctx.writers.mission_progress.send(Some(machine.progress()));
                        return Ok(());
                    }
                    return Err(transfer_failed(
                        mission_type,
                        "transfer.ack_error",
                        format!("MISSION_ACK error: {:?}", data.mavtype),
                    ));
                }
            }
        }
    }
}
