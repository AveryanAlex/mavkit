use super::{CommandContext, VehicleTarget, get_target, recv_routed, send_message};
use crate::dialect::{self, MavCmd};
use crate::error::VehicleError;
use crate::mission::{
    IssueSeverity, MissionFrame, MissionItem, MissionPlan, MissionTransferMachine, MissionType,
    TransferPhase, commands, items_for_wire_upload, plan_from_wire_download, validate_plan,
};
use std::collections::HashSet;
use std::time::Duration;

pub(super) fn to_mav_mission_type(mission_type: MissionType) -> dialect::MavMissionType {
    match mission_type {
        MissionType::Mission => dialect::MavMissionType::MAV_MISSION_TYPE_MISSION,
        MissionType::Fence => dialect::MavMissionType::MAV_MISSION_TYPE_FENCE,
        MissionType::Rally => dialect::MavMissionType::MAV_MISSION_TYPE_RALLY,
    }
}

pub(super) fn to_mav_frame(frame: MissionFrame) -> dialect::MavFrame {
    match frame {
        MissionFrame::Mission => dialect::MavFrame::MAV_FRAME_MISSION,
        MissionFrame::GlobalInt => dialect::MavFrame::MAV_FRAME_GLOBAL,
        MissionFrame::GlobalRelativeAltInt => dialect::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MissionFrame::GlobalTerrainAltInt => dialect::MavFrame::MAV_FRAME_GLOBAL_TERRAIN_ALT,
        MissionFrame::LocalNed => dialect::MavFrame::MAV_FRAME_LOCAL_NED,
        MissionFrame::Other => dialect::MavFrame::MAV_FRAME_MISSION,
    }
}

#[allow(deprecated)]
pub(super) fn from_mav_frame(frame: dialect::MavFrame) -> MissionFrame {
    match frame {
        dialect::MavFrame::MAV_FRAME_MISSION => MissionFrame::Mission,
        dialect::MavFrame::MAV_FRAME_GLOBAL | dialect::MavFrame::MAV_FRAME_GLOBAL_INT => {
            MissionFrame::GlobalInt
        }
        dialect::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT
        | dialect::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT_INT => {
            MissionFrame::GlobalRelativeAltInt
        }
        dialect::MavFrame::MAV_FRAME_GLOBAL_TERRAIN_ALT
        | dialect::MavFrame::MAV_FRAME_GLOBAL_TERRAIN_ALT_INT => MissionFrame::GlobalTerrainAltInt,
        dialect::MavFrame::MAV_FRAME_LOCAL_NED => MissionFrame::LocalNed,
        _ => MissionFrame::Other,
    }
}

pub(super) fn from_mission_item_int(data: &dialect::MISSION_ITEM_INT_DATA) -> MissionItem {
    let frame = from_mav_frame(data.frame);
    MissionItem {
        seq: data.seq,
        command: commands::MissionCommand::from_wire(
            data.command as u16,
            commands::MissionFrame::from(frame),
            [data.param1, data.param2, data.param3, data.param4],
            data.x,
            data.y,
            data.z,
        ),
        current: data.current > 0,
        autocontinue: data.autocontinue > 0,
    }
}

#[allow(deprecated)]
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
        seq: data.seq,
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
        current: data.current > 0,
        autocontinue: data.autocontinue > 0,
    }
}

pub(super) fn mission_type_matches(
    received: dialect::MavMissionType,
    expected: MissionType,
) -> bool {
    let expected_mav = to_mav_mission_type(expected);
    if expected == MissionType::Mission {
        received == expected_mav || received == dialect::MavMissionType::MAV_MISSION_TYPE_MISSION
    } else {
        received == expected_mav
    }
}

fn transfer_domain(mission_type: MissionType) -> &'static str {
    match mission_type {
        MissionType::Mission => "mission",
        MissionType::Fence => "fence",
        MissionType::Rally => "rally",
    }
}

fn transfer_failed(
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
            seq: item.seq,
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

// ---------------------------------------------------------------------------
// Mission Upload
// ---------------------------------------------------------------------------

#[allow(deprecated)]
pub(super) async fn handle_mission_upload(
    plan: MissionPlan,
    ctx: &mut CommandContext,
) -> Result<(), VehicleError> {
    // Validate
    let issues = validate_plan(&plan);
    if let Some(issue) = issues.iter().find(|i| i.severity == IssueSeverity::Error) {
        return Err(VehicleError::MissionValidation(format!(
            "{}: {}",
            issue.code, issue.message
        )));
    }

    let wire_items = items_for_wire_upload(&plan);
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

    // If empty plan, just wait for ACK
    if wire_items.is_empty() {
        return wait_for_mission_ack(&mut machine, plan.mission_type, ctx, || count_msg.clone())
            .await;
    }

    let mut acknowledged = HashSet::<u16>::new();

    // Wait for MISSION_REQUEST_INT / MISSION_REQUEST messages
    while machine.progress().phase != TransferPhase::AwaitAck {
        let timeout = Duration::from_millis(machine.timeout_ms());
        let deadline = tokio::time::sleep(timeout);
        tokio::pin!(deadline);
        let cancel = ctx.cancel.clone();

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

    // Await final ACK
    wait_for_mission_ack(&mut machine, plan.mission_type, ctx, || count_msg.clone()).await
}

async fn wait_for_mission_ack<F>(
    machine: &mut MissionTransferMachine,
    mission_type: MissionType,
    ctx: &mut CommandContext,
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
        let cancel = ctx.cancel.clone();

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

// ---------------------------------------------------------------------------
// Mission Download
// ---------------------------------------------------------------------------

#[allow(deprecated)]
pub(super) async fn handle_mission_download(
    mission_type: MissionType,
    ctx: &mut CommandContext,
) -> Result<MissionPlan, VehicleError> {
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

    // Wait for MISSION_COUNT
    let count = loop {
        let timeout = Duration::from_millis(machine.timeout_ms());
        let deadline = tokio::time::sleep(timeout);
        tokio::pin!(deadline);
        let cancel = ctx.cancel.clone();

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
            let cancel = ctx.cancel.clone();

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

    Ok(plan_from_wire_download(mission_type, items))
}

// ---------------------------------------------------------------------------
// Mission Clear
// ---------------------------------------------------------------------------

pub(super) async fn handle_mission_clear(
    mission_type: MissionType,
    ctx: &mut CommandContext,
) -> Result<(), VehicleError> {
    let target = get_target(&ctx.vehicle_target)?;
    let mav_mission_type = to_mav_mission_type(mission_type);

    let mut machine = MissionTransferMachine::new_upload(mission_type, 0, ctx.config.retry_policy);
    let _ = ctx.writers.mission_progress.send(Some(machine.progress()));

    let clear_msg = dialect::MavMessage::MISSION_CLEAR_ALL(dialect::MISSION_CLEAR_ALL_DATA {
        target_system: target.system_id,
        target_component: target.component_id,
        mission_type: mav_mission_type,
    });

    send_message(ctx.connection.as_ref(), &ctx.config, clear_msg.clone()).await?;

    wait_for_mission_ack(&mut machine, mission_type, ctx, || clear_msg.clone()).await
}

// ---------------------------------------------------------------------------
// Mission Set Current
// ---------------------------------------------------------------------------

/// Handle a set-current command. `seq` is a **semantic** 0-indexed waypoint
/// index (home excluded). For the Mission wire protocol the first visible
/// waypoint is wire seq 1, so we add 1 before sending.
pub(super) async fn handle_mission_set_current(
    seq: u16,
    ctx: &mut CommandContext,
) -> Result<(), VehicleError> {
    // Semantic → wire: home occupies wire seq 0 for Mission type.
    let wire_seq = seq + 1;

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
