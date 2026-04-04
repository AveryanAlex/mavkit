use super::super::{CommandContext, get_target, recv_routed, send_message};
use crate::dialect::{self, MavCmd};
use crate::error::VehicleError;
use crate::mission::MissionType;
use std::time::Duration;

use super::protocol::transfer_failed;

/// Handle a set-current command. `seq` is a **semantic** 0-indexed waypoint
/// index (home excluded). For the Mission wire protocol the first visible
/// waypoint is wire seq 1, so we add 1 before sending.
pub(in crate::event_loop) async fn handle_mission_set_current(
    seq: u16,
    ctx: &mut CommandContext,
) -> Result<(), VehicleError> {
    // Semantic → wire: home occupies wire seq 0 for Mission type.
    let wire_seq = seq.checked_add(1).ok_or_else(|| {
        VehicleError::InvalidParameter(format!("mission seq {seq} overflows wire sequence range"))
    })?;

    let target = get_target(&ctx.vehicle_target)?;
    let max_retries = ctx.config.retry_policy.max_retries;
    let request_timeout_ms = ctx.config.retry_policy.request_timeout_ms;

    for _attempt in 0..=max_retries {
        send_message(
            ctx.connection.as_ref(),
            &ctx.config,
            dialect::MavMessage::COMMAND_LONG(dialect::COMMAND_LONG_DATA {
                target_system: target.system_id,
                target_component: target.component_id,
                command: MavCmd::MAV_CMD_DO_SET_MISSION_CURRENT,
                confirmation: 0,
                param1: f32::from(wire_seq),
                param2: 0.0,
                param3: 0.0,
                param4: 0.0,
                param5: 0.0,
                param6: 0.0,
                param7: 0.0,
            }),
        )
        .await?;

        let timeout = Duration::from_millis(request_timeout_ms);
        let deadline = tokio::time::sleep(timeout);
        tokio::pin!(deadline);
        let cancel = ctx.cancel.clone();

        loop {
            tokio::select! {
                biased;
                _ = cancel.cancelled() => return Err(VehicleError::Cancelled),
                _ = &mut deadline => break, // retry outer loop
                result = recv_routed(&mut ctx.inbound_rx) => {
                    let (_, msg) = result?;

                    match &msg {
                        dialect::MavMessage::COMMAND_ACK(data)
                            if data.command == MavCmd::MAV_CMD_DO_SET_MISSION_CURRENT
                                && data.result == dialect::MavResult::MAV_RESULT_ACCEPTED =>
                        {
                            return Ok(());
                        }
                        dialect::MavMessage::MISSION_CURRENT(data) if data.seq == wire_seq => {
                            return Ok(());
                        }
                        _ => {}
                    }
                }
            }
        }
    }

    Err(transfer_failed(
        MissionType::Mission,
        "mission.set_current_timeout",
        "Did not receive confirmation for set-current command",
    ))
}
