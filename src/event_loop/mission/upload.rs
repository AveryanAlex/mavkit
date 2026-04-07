use super::super::{CommandContext, VehicleTarget, get_target, recv_routed, send_message};
use crate::dialect;
use crate::error::VehicleError;
use crate::mission::{
    MissionItem, MissionTransferMachine, MissionType, TransferPhase, WireMissionPlan, wire,
};
use std::collections::HashSet;
use std::time::Duration;
use tokio_util::sync::CancellationToken;

use super::convert::{to_mav_frame, to_mav_mission_type};
use super::protocol::{transfer_failed, wait_for_mission_ack};

fn send_requested_item_msg(
    wire_items: &[MissionItem],
    target: VehicleTarget,
    mission_type: MissionType,
    seq: u16,
) -> Result<dialect::MavMessage, VehicleError> {
    let item = wire_items.get(seq as usize).ok_or_else(|| {
        transfer_failed(
            mission_type,
            "item_out_of_range",
            format!("requested item {seq} out of range"),
        )
    })?;

    let (raw_command, frame, params, x, y, z) = item.command.clone().into_wire();
    let command = num_traits::FromPrimitive::from_u16(raw_command).ok_or_else(|| {
        transfer_failed(
            mission_type,
            "unsupported_command",
            format!("unsupported MAV_CMD value {}", raw_command),
        )
    })?;
    let frame = to_mav_frame(frame.into());

    Ok(dialect::MavMessage::MISSION_ITEM_INT(
        dialect::MISSION_ITEM_INT_DATA {
            param1: params[0],
            param2: params[1],
            param3: params[2],
            param4: params[3],
            x,
            y,
            z,
            seq,
            command,
            target_system: target.system_id,
            target_component: target.component_id,
            frame,
            current: 0,
            autocontinue: u8::from(item.autocontinue),
            mission_type: to_mav_mission_type(mission_type),
        },
    ))
}

#[allow(
    deprecated,
    reason = "the MAVLink crate deprecated these mission transfer variants, but the MAVLink mission upload wire protocol still requires them"
)]
pub(in crate::event_loop) async fn handle_mission_upload(
    plan: WireMissionPlan,
    ctx: &mut CommandContext,
    op_cancel: &CancellationToken,
) -> Result<(), VehicleError> {
    let wire_items = wire::items_for_wire_upload(&plan);
    let target = get_target(&ctx.vehicle_target)?;
    let mav_mission_type = to_mav_mission_type(plan.mission_type);

    let mut machine = MissionTransferMachine::new_upload(
        plan.mission_type,
        wire_items.len() as u16,
        ctx.config.retry_policy,
    );
    let _ = ctx.writers.mission_progress.send(Some(machine.progress()));

    let count_msg = dialect::MavMessage::MISSION_COUNT(dialect::MISSION_COUNT_DATA {
        count: wire_items.len() as u16,
        target_system: target.system_id,
        target_component: target.component_id,
        mission_type: mav_mission_type,
        opaque_id: 0,
    });

    send_message(ctx.connection.as_ref(), &ctx.config, count_msg.clone()).await?;

    if wire_items.is_empty() {
        return wait_for_mission_ack(&mut machine, plan.mission_type, ctx, op_cancel, || {
            count_msg.clone()
        })
        .await;
    }

    let mut acknowledged = HashSet::<u16>::new();

    while machine.progress().phase != TransferPhase::AwaitAck {
        let timeout = Duration::from_millis(machine.timeout_ms());
        let deadline = tokio::time::sleep(timeout);
        tokio::pin!(deadline);
        let cancel = op_cancel.clone();

        let msg = loop {
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
                        return Err(transfer_failed(plan.mission_type, err.code, err.message));
                    }
                    let _ = ctx.writers.mission_progress.send(Some(machine.progress()));
                    send_message(ctx.connection.as_ref(), &ctx.config, count_msg.clone()).await?;
                    break None;
                }
                result = recv_routed(&mut ctx.inbound_rx) => {
                    let (_, msg) = result?;

                    match &msg {
                        dialect::MavMessage::MISSION_REQUEST_INT(data) if data.mission_type == mav_mission_type => {
                            break Some(("int", data.seq));
                        }
                        dialect::MavMessage::MISSION_REQUEST(data) if data.mission_type == mav_mission_type => {
                            break Some(("req", data.seq));
                        }
                        dialect::MavMessage::MISSION_ACK(data) if data.mission_type == mav_mission_type => {
                            if data.mavtype == dialect::MavMissionResult::MAV_MISSION_ACCEPTED {
                                machine.on_ack_success();
                                let _ = ctx.writers.mission_progress.send(Some(machine.progress()));
                                return Ok(());
                            }
                            return Err(transfer_failed(
                                plan.mission_type,
                                "transfer.ack_error",
                                format!("MISSION_ACK error: {:?}", data.mavtype),
                            ));
                        }
                        _ => {}
                    }
                    continue;
                }
            }
        };

        if let Some((_kind, seq)) = msg {
            let item_msg = send_requested_item_msg(&wire_items, target, plan.mission_type, seq)?;
            send_message(ctx.connection.as_ref(), &ctx.config, item_msg).await?;
            if acknowledged.insert(seq) {
                machine.on_item_transferred();
                let _ = ctx.writers.mission_progress.send(Some(machine.progress()));
            }
        }
    }

    wait_for_mission_ack(&mut machine, plan.mission_type, ctx, op_cancel, || {
        count_msg.clone()
    })
    .await
}
