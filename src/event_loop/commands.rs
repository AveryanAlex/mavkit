use super::{
    CommandContext, MAGIC_FORCE_ARM_VALUE, MAGIC_FORCE_DISARM_VALUE, VehicleTarget, get_target,
    send_message, update_state, update_vehicle_target,
};
use crate::error::VehicleError;
use mavlink::common::{self, MavCmd};
use std::time::Duration;

/// Bitmask for SET_POSITION_TARGET_GLOBAL_INT: use only lat/lon/alt, ignore
/// velocity, acceleration, and yaw fields.
const GUIDED_GOTO_TYPE_MASK: u16 = 0x07F8;

// ---------------------------------------------------------------------------
// Arm / Disarm
// ---------------------------------------------------------------------------

pub(super) async fn handle_arm_disarm(
    arm: bool,
    force: bool,
    ctx: &mut CommandContext<'_>,
) -> Result<(), VehicleError> {
    let target = get_target(ctx.vehicle_target)?;
    let param1 = if arm { 1.0 } else { 0.0 };
    let param2 = if force {
        if arm {
            MAGIC_FORCE_ARM_VALUE
        } else {
            MAGIC_FORCE_DISARM_VALUE
        }
    } else {
        0.0
    };

    send_command_long_ack(
        MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
        [param1, param2, 0.0, 0.0, 0.0, 0.0, 0.0],
        target,
        ctx,
    )
    .await
}

async fn send_command_long_ack(
    command: MavCmd,
    params: [f32; 7],
    target: VehicleTarget,
    ctx: &mut CommandContext<'_>,
) -> Result<(), VehicleError> {
    let retry_policy = &ctx.config.retry_policy;
    for _attempt in 0..=retry_policy.max_retries {
        send_message(
            ctx.connection,
            ctx.config,
            common::MavMessage::COMMAND_LONG(common::COMMAND_LONG_DATA {
                target_system: target.system_id,
                target_component: target.component_id,
                command,
                confirmation: 0,
                param1: params[0],
                param2: params[1],
                param3: params[2],
                param4: params[3],
                param5: params[4],
                param6: params[5],
                param7: params[6],
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
                _ = &mut deadline => break, // retry
                result = ctx.connection.recv() => {
                    let (header, msg) = result
                        .map_err(|err| VehicleError::Io(std::io::Error::other(err.to_string())))?;
                    update_vehicle_target(ctx.vehicle_target, &header, &msg);
                    update_state(&header, &msg, ctx.writers, ctx.vehicle_target);
                    if let common::MavMessage::COMMAND_ACK(ack) = &msg
                        && ack.command == command
                    {
                        if ack.result == common::MavResult::MAV_RESULT_ACCEPTED {
                            return Ok(());
                        }
                        return Err(VehicleError::CommandRejected {
                            command: format!("{command:?}"),
                            result: format!("{:?}", ack.result),
                        });
                    }
                }
            }
        }
    }

    Err(VehicleError::Timeout)
}

// ---------------------------------------------------------------------------
// Set mode
// ---------------------------------------------------------------------------

pub(super) async fn handle_set_mode(
    custom_mode: u32,
    ctx: &mut CommandContext<'_>,
) -> Result<(), VehicleError> {
    let target = get_target(ctx.vehicle_target)?;

    // Try COMMAND_LONG(DO_SET_MODE) first
    let do_set_mode_result = send_command_long_ack(
        MavCmd::MAV_CMD_DO_SET_MODE,
        [1.0, custom_mode as f32, 0.0, 0.0, 0.0, 0.0, 0.0],
        target,
        ctx,
    )
    .await;

    if do_set_mode_result.is_ok() {
        return Ok(());
    }

    // Fallback: wait for confirming heartbeat
    let timeout = Duration::from_secs(2);
    let deadline = tokio::time::sleep(timeout);
    tokio::pin!(deadline);

    loop {
        tokio::select! {
            biased;
            _ = ctx.cancel.cancelled() => return Err(VehicleError::Cancelled),
            _ = &mut deadline => {
                return Err(VehicleError::CommandRejected {
                    command: format!("DO_SET_MODE({custom_mode})"),
                    result: "no confirming HEARTBEAT".to_string(),
                });
            }
            result = ctx.connection.recv() => {
                let (header, msg) =
                    result.map_err(|err| VehicleError::Io(std::io::Error::other(err.to_string())))?;
                update_vehicle_target(ctx.vehicle_target, &header, &msg);
                update_state(&header, &msg, ctx.writers, ctx.vehicle_target);
                if let common::MavMessage::HEARTBEAT(hb) = &msg
                    && hb.custom_mode == custom_mode
                {
                    return Ok(());
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Generic COMMAND_LONG (public API)
// ---------------------------------------------------------------------------

pub(super) async fn handle_command_long(
    command: MavCmd,
    params: [f32; 7],
    ctx: &mut CommandContext<'_>,
) -> Result<(), VehicleError> {
    let target = get_target(ctx.vehicle_target)?;
    send_command_long_ack(command, params, target, ctx).await
}

// ---------------------------------------------------------------------------
// Guided goto
// ---------------------------------------------------------------------------

pub(super) async fn handle_guided_goto(
    lat_e7: i32,
    lon_e7: i32,
    alt_m: f32,
    ctx: &mut CommandContext<'_>,
) -> Result<(), VehicleError> {
    let target = get_target(ctx.vehicle_target)?;
    let type_mask = common::PositionTargetTypemask::from_bits_truncate(GUIDED_GOTO_TYPE_MASK);

    send_message(
        ctx.connection,
        ctx.config,
        common::MavMessage::SET_POSITION_TARGET_GLOBAL_INT(
            common::SET_POSITION_TARGET_GLOBAL_INT_DATA {
                time_boot_ms: 0,
                target_system: target.system_id,
                target_component: target.component_id,
                coordinate_frame: common::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
                type_mask,
                lat_int: lat_e7,
                lon_int: lon_e7,
                alt: alt_m,
                vx: 0.0,
                vy: 0.0,
                vz: 0.0,
                afx: 0.0,
                afy: 0.0,
                afz: 0.0,
                yaw: 0.0,
                yaw_rate: 0.0,
            },
        ),
    )
    .await
}
