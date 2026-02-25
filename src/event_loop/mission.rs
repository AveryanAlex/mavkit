use super::{
    CommandContext, VehicleTarget, get_target, send_message, update_state, update_vehicle_target,
};
use crate::error::VehicleError;
use crate::mission::{
    IssueSeverity, MissionFrame, MissionItem, MissionPlan, MissionTransferMachine, MissionType,
    TransferPhase, items_for_wire_upload, plan_from_wire_download, validate_plan,
};
use mavlink::common::{self, MavCmd};
use std::collections::HashSet;
use std::time::Duration;

pub(super) fn to_mav_mission_type(mission_type: MissionType) -> common::MavMissionType {
    match mission_type {
        MissionType::Mission => common::MavMissionType::MAV_MISSION_TYPE_MISSION,
        MissionType::Fence => common::MavMissionType::MAV_MISSION_TYPE_FENCE,
        MissionType::Rally => common::MavMissionType::MAV_MISSION_TYPE_RALLY,
    }
}

pub(super) fn to_mav_frame(frame: MissionFrame) -> common::MavFrame {
    match frame {
        MissionFrame::Mission => common::MavFrame::MAV_FRAME_MISSION,
        MissionFrame::GlobalInt => common::MavFrame::MAV_FRAME_GLOBAL,
        MissionFrame::GlobalRelativeAltInt => common::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MissionFrame::GlobalTerrainAltInt => common::MavFrame::MAV_FRAME_GLOBAL_TERRAIN_ALT,
        MissionFrame::LocalNed => common::MavFrame::MAV_FRAME_LOCAL_NED,
        MissionFrame::Other => common::MavFrame::MAV_FRAME_MISSION,
    }
}

#[allow(deprecated)]
pub(super) fn from_mav_frame(frame: common::MavFrame) -> MissionFrame {
    match frame {
        common::MavFrame::MAV_FRAME_MISSION => MissionFrame::Mission,
        common::MavFrame::MAV_FRAME_GLOBAL | common::MavFrame::MAV_FRAME_GLOBAL_INT => {
            MissionFrame::GlobalInt
        }
        common::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT
        | common::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT_INT => MissionFrame::GlobalRelativeAltInt,
        common::MavFrame::MAV_FRAME_GLOBAL_TERRAIN_ALT
        | common::MavFrame::MAV_FRAME_GLOBAL_TERRAIN_ALT_INT => MissionFrame::GlobalTerrainAltInt,
        common::MavFrame::MAV_FRAME_LOCAL_NED => MissionFrame::LocalNed,
        _ => MissionFrame::Other,
    }
}

pub(super) fn from_mission_item_int(data: &common::MISSION_ITEM_INT_DATA) -> MissionItem {
    MissionItem {
        seq: data.seq,
        command: data.command as u16,
        frame: from_mav_frame(data.frame),
        current: data.current > 0,
        autocontinue: data.autocontinue > 0,
        param1: data.param1,
        param2: data.param2,
        param3: data.param3,
        param4: data.param4,
        x: data.x,
        y: data.y,
        z: data.z,
    }
}

#[allow(deprecated)]
fn from_mission_item_float(data: &common::MISSION_ITEM_DATA) -> MissionItem {
    let is_global = matches!(
        data.frame,
        common::MavFrame::MAV_FRAME_GLOBAL
            | common::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT
            | common::MavFrame::MAV_FRAME_GLOBAL_TERRAIN_ALT
            | common::MavFrame::MAV_FRAME_GLOBAL_INT
            | common::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
            | common::MavFrame::MAV_FRAME_GLOBAL_TERRAIN_ALT_INT
    );

    MissionItem {
        seq: data.seq,
        command: data.command as u16,
        frame: from_mav_frame(data.frame),
        current: data.current > 0,
        autocontinue: data.autocontinue > 0,
        param1: data.param1,
        param2: data.param2,
        param3: data.param3,
        param4: data.param4,
        x: if is_global {
            (data.x as f64 * 1e7) as i32
        } else {
            data.x as i32
        },
        y: if is_global {
            (data.y as f64 * 1e7) as i32
        } else {
            data.y as i32
        },
        z: data.z,
    }
}

pub(super) fn mission_type_matches(
    received: common::MavMissionType,
    expected: MissionType,
) -> bool {
    let expected_mav = to_mav_mission_type(expected);
    if expected == MissionType::Mission {
        received == expected_mav || received == common::MavMissionType::MAV_MISSION_TYPE_MISSION
    } else {
        received == expected_mav
    }
}

fn send_requested_item_msg(
    wire_items: &[MissionItem],
    target: VehicleTarget,
    mission_type: MissionType,
    seq: u16,
) -> Result<common::MavMessage, VehicleError> {
    let item = wire_items
        .get(seq as usize)
        .ok_or_else(|| VehicleError::MissionTransfer {
            code: "item_out_of_range".to_string(),
            message: format!("requested item {seq} out of range"),
        })?;

    let command = num_traits::FromPrimitive::from_u16(item.command).ok_or_else(|| {
        VehicleError::MissionTransfer {
            code: "unsupported_command".to_string(),
            message: format!("unsupported MAV_CMD value {}", item.command),
        }
    })?;
    let frame = to_mav_frame(item.frame);

    Ok(common::MavMessage::MISSION_ITEM_INT(
        common::MISSION_ITEM_INT_DATA {
            param1: item.param1,
            param2: item.param2,
            param3: item.param3,
            param4: item.param4,
            x: item.x,
            y: item.y,
            z: item.z,
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
    ctx: &mut CommandContext<'_>,
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
    let target = get_target(ctx.vehicle_target)?;
    let mav_mission_type = to_mav_mission_type(plan.mission_type);

    let mut machine = MissionTransferMachine::new_upload(
        plan.mission_type,
        wire_items.len() as u16,
        ctx.config.retry_policy,
    );
    let _ = ctx.writers.mission_progress.send(Some(machine.progress()));

    let count_msg = common::MavMessage::MISSION_COUNT(common::MISSION_COUNT_DATA {
        count: wire_items.len() as u16,
        target_system: target.system_id,
        target_component: target.component_id,
        mission_type: mav_mission_type,
        opaque_id: 0,
    });

    send_message(ctx.connection, ctx.config, count_msg.clone()).await?;

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

        let msg = loop {
            tokio::select! {
                biased;
                _ = ctx.cancel.cancelled() => {
                    machine.cancel();
                    let _ = ctx.writers.mission_progress.send(Some(machine.progress()));
                    return Err(VehicleError::Cancelled);
                }
                _ = &mut deadline => {
                    if let Some(err) = machine.on_timeout() {
                        let _ = ctx.writers.mission_progress.send(Some(machine.progress()));
                        return Err(VehicleError::MissionTransfer {
                            code: err.code,
                            message: err.message,
                        });
                    }
                    let _ = ctx.writers.mission_progress.send(Some(machine.progress()));
                    send_message(ctx.connection, ctx.config, count_msg.clone()).await?;
                    break None;
                }
                result = ctx.connection.recv() => {
                    let (header, msg) = result
                        .map_err(|err| VehicleError::Io(std::io::Error::other(err.to_string())))?;
                    update_vehicle_target(ctx.vehicle_target, &header, &msg);
                    update_state(&header, &msg, ctx.writers, ctx.vehicle_target);

                    match &msg {
                        common::MavMessage::MISSION_REQUEST_INT(data) if data.mission_type == mav_mission_type => {
                            break Some(("int", data.seq));
                        }
                        common::MavMessage::MISSION_REQUEST(data) if data.mission_type == mav_mission_type => {
                            break Some(("req", data.seq));
                        }
                        common::MavMessage::MISSION_ACK(data) if data.mission_type == mav_mission_type => {
                            if data.mavtype == common::MavMissionResult::MAV_MISSION_ACCEPTED {
                                machine.on_ack_success();
                                let _ = ctx.writers.mission_progress.send(Some(machine.progress()));
                                return Ok(());
                            }
                            return Err(VehicleError::MissionTransfer {
                                code: "transfer.ack_error".to_string(),
                                message: format!("MISSION_ACK error: {:?}", data.mavtype),
                            });
                        }
                        _ => {}
                    }
                    continue;
                }
            }
        };

        if let Some((_kind, seq)) = msg {
            let item_msg = send_requested_item_msg(&wire_items, target, plan.mission_type, seq)?;
            send_message(ctx.connection, ctx.config, item_msg).await?;
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
    ctx: &mut CommandContext<'_>,
    retry_msg: F,
) -> Result<(), VehicleError>
where
    F: Fn() -> common::MavMessage,
{
    let mav_mission_type = to_mav_mission_type(mission_type);
    loop {
        let timeout = Duration::from_millis(machine.timeout_ms());
        let deadline = tokio::time::sleep(timeout);
        tokio::pin!(deadline);

        tokio::select! {
            biased;
            _ = ctx.cancel.cancelled() => {
                machine.cancel();
                let _ = ctx.writers.mission_progress.send(Some(machine.progress()));
                return Err(VehicleError::Cancelled);
            }
            _ = &mut deadline => {
                if let Some(err) = machine.on_timeout() {
                    let _ = ctx.writers.mission_progress.send(Some(machine.progress()));
                    return Err(VehicleError::MissionTransfer {
                        code: err.code,
                        message: err.message,
                    });
                }
                let _ = ctx.writers.mission_progress.send(Some(machine.progress()));
                send_message(ctx.connection, ctx.config, retry_msg()).await?;
            }
            result = ctx.connection.recv() => {
                let (header, msg) =
                    result.map_err(|err| VehicleError::Io(std::io::Error::other(err.to_string())))?;
                update_vehicle_target(ctx.vehicle_target, &header, &msg);
                update_state(&header, &msg, ctx.writers, ctx.vehicle_target);

                if let common::MavMessage::MISSION_ACK(data) = &msg {
                    if data.mission_type != mav_mission_type {
                        continue;
                    }
                    if data.mavtype == common::MavMissionResult::MAV_MISSION_ACCEPTED {
                        machine.on_ack_success();
                        let _ = ctx.writers.mission_progress.send(Some(machine.progress()));
                        return Ok(());
                    }
                    return Err(VehicleError::MissionTransfer {
                        code: "transfer.ack_error".to_string(),
                        message: format!("MISSION_ACK error: {:?}", data.mavtype),
                    });
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
    ctx: &mut CommandContext<'_>,
) -> Result<MissionPlan, VehicleError> {
    let target = get_target(ctx.vehicle_target)?;
    let mav_mission_type = to_mav_mission_type(mission_type);
    let mut machine = MissionTransferMachine::new_download(mission_type, ctx.config.retry_policy);
    let _ = ctx.writers.mission_progress.send(Some(machine.progress()));

    let request_list_msg =
        common::MavMessage::MISSION_REQUEST_LIST(common::MISSION_REQUEST_LIST_DATA {
            target_system: target.system_id,
            target_component: target.component_id,
            mission_type: mav_mission_type,
        });
    send_message(ctx.connection, ctx.config, request_list_msg.clone()).await?;

    // Wait for MISSION_COUNT
    let count = loop {
        let timeout = Duration::from_millis(machine.timeout_ms());
        let deadline = tokio::time::sleep(timeout);
        tokio::pin!(deadline);

        tokio::select! {
            biased;
            _ = ctx.cancel.cancelled() => {
                machine.cancel();
                let _ = ctx.writers.mission_progress.send(Some(machine.progress()));
                return Err(VehicleError::Cancelled);
            }
            _ = &mut deadline => {
                if let Some(err) = machine.on_timeout() {
                    let _ = ctx.writers.mission_progress.send(Some(machine.progress()));
                    return Err(VehicleError::MissionTransfer {
                        code: err.code,
                        message: err.message,
                    });
                }
                let _ = ctx.writers.mission_progress.send(Some(machine.progress()));
                send_message(ctx.connection, ctx.config, request_list_msg.clone()).await?;
            }
            result = ctx.connection.recv() => {
                let (header, msg) =
                    result.map_err(|err| VehicleError::Io(std::io::Error::other(err.to_string())))?;
                update_vehicle_target(ctx.vehicle_target, &header, &msg);
                update_state(&header, &msg, ctx.writers, ctx.vehicle_target);

                if let common::MavMessage::MISSION_COUNT(data) = &msg
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
            common::MavMessage::MISSION_REQUEST_INT(common::MISSION_REQUEST_INT_DATA {
                seq,
                target_system: target.system_id,
                target_component: target.component_id,
                mission_type: mav_mission_type,
            });
        let request_float_msg = common::MavMessage::MISSION_REQUEST(common::MISSION_REQUEST_DATA {
            seq,
            target_system: target.system_id,
            target_component: target.component_id,
            mission_type: mav_mission_type,
        });

        let make_request_msg = |use_int: bool| -> common::MavMessage {
            if use_int {
                request_int_msg.clone()
            } else {
                request_float_msg.clone()
            }
        };

        send_message(
            ctx.connection,
            ctx.config,
            make_request_msg(use_int_request),
        )
        .await?;

        let item = loop {
            let timeout = Duration::from_millis(machine.timeout_ms());
            let deadline = tokio::time::sleep(timeout);
            tokio::pin!(deadline);

            tokio::select! {
                biased;
                _ = ctx.cancel.cancelled() => {
                    machine.cancel();
                    let _ = ctx.writers.mission_progress.send(Some(machine.progress()));
                    return Err(VehicleError::Cancelled);
                }
                _ = &mut deadline => {
                    if let Some(err) = machine.on_timeout() {
                        let _ = ctx.writers.mission_progress.send(Some(machine.progress()));
                        return Err(VehicleError::MissionTransfer {
                            code: err.code,
                            message: err.message,
                        });
                    }
                    let _ = ctx.writers.mission_progress.send(Some(machine.progress()));
                    if use_int_request {
                        use_int_request = false;
                    }
                    send_message(ctx.connection, ctx.config, make_request_msg(use_int_request)).await?;
                }
                result = ctx.connection.recv() => {
                    let (header, msg) = result
                        .map_err(|err| VehicleError::Io(std::io::Error::other(err.to_string())))?;
                    update_vehicle_target(ctx.vehicle_target, &header, &msg);
                    update_state(&header, &msg, ctx.writers, ctx.vehicle_target);

                    match &msg {
                        common::MavMessage::MISSION_ITEM_INT(data)
                            if data.seq == seq && mission_type_matches(data.mission_type, mission_type) =>
                        {
                            break from_mission_item_int(data);
                        }
                        common::MavMessage::MISSION_ITEM(data)
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
        ctx.connection,
        ctx.config,
        common::MavMessage::MISSION_ACK(common::MISSION_ACK_DATA {
            target_system: target.system_id,
            target_component: target.component_id,
            mavtype: common::MavMissionResult::MAV_MISSION_ACCEPTED,
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
    ctx: &mut CommandContext<'_>,
) -> Result<(), VehicleError> {
    let target = get_target(ctx.vehicle_target)?;
    let mav_mission_type = to_mav_mission_type(mission_type);

    let mut machine = MissionTransferMachine::new_upload(mission_type, 0, ctx.config.retry_policy);
    let _ = ctx.writers.mission_progress.send(Some(machine.progress()));

    let clear_msg = common::MavMessage::MISSION_CLEAR_ALL(common::MISSION_CLEAR_ALL_DATA {
        target_system: target.system_id,
        target_component: target.component_id,
        mission_type: mav_mission_type,
    });

    send_message(ctx.connection, ctx.config, clear_msg.clone()).await?;

    wait_for_mission_ack(&mut machine, mission_type, ctx, || clear_msg.clone()).await
}

// ---------------------------------------------------------------------------
// Mission Set Current
// ---------------------------------------------------------------------------

pub(super) async fn handle_mission_set_current(
    seq: u16,
    ctx: &mut CommandContext<'_>,
) -> Result<(), VehicleError> {
    let target = get_target(ctx.vehicle_target)?;
    let retry_policy = &ctx.config.retry_policy;

    for _attempt in 0..=retry_policy.max_retries {
        send_message(
            ctx.connection,
            ctx.config,
            common::MavMessage::COMMAND_LONG(common::COMMAND_LONG_DATA {
                target_system: target.system_id,
                target_component: target.component_id,
                command: MavCmd::MAV_CMD_DO_SET_MISSION_CURRENT,
                confirmation: 0,
                param1: seq as f32,
                param2: 0.0,
                param3: 0.0,
                param4: 0.0,
                param5: 0.0,
                param6: 0.0,
                param7: 0.0,
            }),
        )
        .await?;

        let timeout = Duration::from_millis(retry_policy.request_timeout_ms);
        let deadline = tokio::time::sleep(timeout);
        tokio::pin!(deadline);

        loop {
            tokio::select! {
                biased;
                _ = ctx.cancel.cancelled() => return Err(VehicleError::Cancelled),
                _ = &mut deadline => break, // retry outer loop
                result = ctx.connection.recv() => {
                    let (header, msg) = result
                        .map_err(|err| VehicleError::Io(std::io::Error::other(err.to_string())))?;
                    update_vehicle_target(ctx.vehicle_target, &header, &msg);
                    update_state(&header, &msg, ctx.writers, ctx.vehicle_target);

                    match &msg {
                        common::MavMessage::COMMAND_ACK(data) => {
                            if data.command == MavCmd::MAV_CMD_DO_SET_MISSION_CURRENT
                                && data.result == common::MavResult::MAV_RESULT_ACCEPTED
                            {
                                return Ok(());
                            }
                        }
                        common::MavMessage::MISSION_CURRENT(data) => {
                            if data.seq == seq {
                                return Ok(());
                            }
                        }
                        _ => {}
                    }
                }
            }
        }
    }

    Err(VehicleError::MissionTransfer {
        code: "mission.set_current_timeout".to_string(),
        message: "Did not receive confirmation for set-current command".to_string(),
    })
}
