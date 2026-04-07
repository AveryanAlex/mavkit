use super::super::{CommandContext, get_target, recv_routed, send_message};
use crate::dialect;
use crate::error::VehicleError;
use crate::mission::{
    MissionItem, MissionTransferMachine, MissionType, WireMissionPlan, commands, wire,
};
use std::time::Duration;
use tokio_util::sync::CancellationToken;

use super::convert::{
    from_mav_frame, from_mission_item_int, mission_type_matches, to_mav_mission_type,
};
use super::protocol::transfer_failed;

#[allow(
    deprecated,
    reason = "the MAVLink crate deprecated MISSION_ITEM, but downloads still need to decode the float-form fallback from the wire"
)]
fn from_mission_item_float(data: &dialect::MISSION_ITEM_DATA) -> MissionItem {
    let is_global = matches!(
        data.frame,
        dialect::MavFrame::MAV_FRAME_GLOBAL
            | dialect::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT
            | dialect::MavFrame::MAV_FRAME_GLOBAL_TERRAIN_ALT
            | dialect::MavFrame::MAV_FRAME_GLOBAL_INT
            | dialect::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
            | dialect::MavFrame::MAV_FRAME_GLOBAL_TERRAIN_ALT_INT
    );

    MissionItem {
        command: commands::MissionCommand::from_wire(
            data.command as u16,
            commands::MissionFrame::from(from_mav_frame(data.frame)),
            [data.param1, data.param2, data.param3, data.param4],
            if is_global {
                (f64::from(data.x) * 1e7) as i32
            } else {
                data.x as i32
            },
            if is_global {
                (f64::from(data.y) * 1e7) as i32
            } else {
                data.y as i32
            },
            data.z,
        ),
        autocontinue: data.autocontinue > 0,
    }
}

#[allow(
    deprecated,
    reason = "the MAVLink crate deprecated these mission transfer variants, but the MAVLink mission download wire protocol still requires them"
)]
pub(in crate::event_loop) async fn handle_mission_download(
    mission_type: MissionType,
    ctx: &mut CommandContext,
    op_cancel: &CancellationToken,
) -> Result<WireMissionPlan, VehicleError> {
    let target = get_target(&ctx.vehicle_target)?;
    let mav_mission_type = to_mav_mission_type(mission_type);
    let mut machine = MissionTransferMachine::new_download(mission_type, ctx.config.retry_policy);
    let _ = ctx.writers.mission_progress.send(Some(machine.progress()));

    let request_list_msg =
        dialect::MavMessage::MISSION_REQUEST_LIST(dialect::MISSION_REQUEST_LIST_DATA {
            target_system: target.system_id,
            target_component: target.component_id,
            mission_type: mav_mission_type,
        });
    send_message(
        ctx.connection.as_ref(),
        &ctx.config,
        request_list_msg.clone(),
    )
    .await?;

    let count = loop {
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
                send_message(ctx.connection.as_ref(), &ctx.config, request_list_msg.clone()).await?;
            }
            result = recv_routed(&mut ctx.inbound_rx) => {
                let (_, msg) = result?;

                if let dialect::MavMessage::MISSION_COUNT(data) = &msg
                    && mission_type_matches(data.mission_type, mission_type)
                {
                    break data.count;
                }
            }
        }
    };

    machine.set_download_total(count);
    let _ = ctx.writers.mission_progress.send(Some(machine.progress()));

    // Request each item
    let mut items = Vec::with_capacity(count as usize);
    for seq in 0..count {
        let mut use_int_request = true;

        let request_int_msg =
            dialect::MavMessage::MISSION_REQUEST_INT(dialect::MISSION_REQUEST_INT_DATA {
                seq,
                target_system: target.system_id,
                target_component: target.component_id,
                mission_type: mav_mission_type,
            });
        let request_float_msg =
            dialect::MavMessage::MISSION_REQUEST(dialect::MISSION_REQUEST_DATA {
                seq,
                target_system: target.system_id,
                target_component: target.component_id,
                mission_type: mav_mission_type,
            });

        let make_request_msg = |use_int: bool| -> dialect::MavMessage {
            if use_int {
                request_int_msg.clone()
            } else {
                request_float_msg.clone()
            }
        };

        send_message(
            ctx.connection.as_ref(),
            &ctx.config,
            make_request_msg(use_int_request),
        )
        .await?;

        let item = loop {
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
                    if use_int_request {
                        use_int_request = false;
                    }
                    send_message(ctx.connection.as_ref(), &ctx.config, make_request_msg(use_int_request)).await?;
                }
                result = recv_routed(&mut ctx.inbound_rx) => {
                    let (_, msg) = result?;

                    match &msg {
                        dialect::MavMessage::MISSION_ITEM_INT(data)
                            if data.seq == seq && mission_type_matches(data.mission_type, mission_type) =>
                        {
                            break from_mission_item_int(data);
                        }
                        dialect::MavMessage::MISSION_ITEM(data)
                            if data.seq == seq && mission_type_matches(data.mission_type, mission_type) =>
                        {
                            break from_mission_item_float(data);
                        }
                        _ => {}
                    }
                }
            }
        };

        items.push(item);
        machine.on_item_transferred();
        let _ = ctx.writers.mission_progress.send(Some(machine.progress()));
    }

    // Send ACK
    let _ = send_message(
        ctx.connection.as_ref(),
        &ctx.config,
        dialect::MavMessage::MISSION_ACK(dialect::MISSION_ACK_DATA {
            target_system: target.system_id,
            target_component: target.component_id,
            mavtype: dialect::MavMissionResult::MAV_MISSION_ACCEPTED,
            mission_type: mav_mission_type,
            opaque_id: 0,
        }),
    )
    .await;

    machine.on_ack_success();
    let _ = ctx.writers.mission_progress.send(Some(machine.progress()));

    Ok(wire::plan_from_wire_download(mission_type, items))
}
