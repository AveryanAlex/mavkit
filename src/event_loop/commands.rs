use super::{
    CommandContext, MAGIC_FORCE_ARM_VALUE, MAGIC_FORCE_DISARM_VALUE, VehicleTarget,
    dispatcher::{CommandIntRequest, CommandLongRequest},
    get_target, send_message,
};
use crate::command::RawCommandIntPayload;
use crate::dialect::{self, MavCmd};
use crate::error::VehicleError;
use crate::raw::CommandAck;
use std::time::Instant;

/// Bitmask for SET_POSITION_TARGET_GLOBAL_INT: use only lat/lon/alt, ignore
/// velocity, acceleration, and yaw fields.
const GUIDED_GOTO_TYPE_MASK: u16 = 0x07F8;

// ---------------------------------------------------------------------------
// Arm / Disarm
// ---------------------------------------------------------------------------

pub(super) async fn handle_arm_disarm(
    arm: bool,
    force: bool,
    ctx: &mut CommandContext,
) -> Result<(), VehicleError> {
    let target = get_target(&ctx.vehicle_target)?;
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
        MavCmd::MAV_CMD_COMPONENT_ARM_DISARM as u16,
        [param1, param2, 0.0, 0.0, 0.0, 0.0, 0.0],
        target,
        ctx,
    )
    .await
    .map(|_| ())
}

async fn send_command_long_ack(
    command_id: u16,
    params: [f32; 7],
    target: VehicleTarget,
    ctx: &mut CommandContext,
) -> Result<CommandAck, VehicleError> {
    ctx.ack_dispatcher
        .dispatch_command_long(CommandLongRequest {
            connection: ctx.connection.as_ref(),
            config: &ctx.config,
            cancel: &ctx.cancel,
            target_system: target.system_id,
            target_component: target.component_id,
            command_id,
            params,
        })
        .await
}

// ---------------------------------------------------------------------------
// Set mode
// ---------------------------------------------------------------------------

pub(super) async fn handle_set_mode(
    custom_mode: u32,
    ctx: &mut CommandContext,
) -> Result<(), VehicleError> {
    let target = get_target(&ctx.vehicle_target)?;

    // COMMAND_LONG params are f32; mode IDs above 2^24 would lose precision.
    // ArduPilot custom_mode values are small integers, so this is safe.
    debug_assert!(
        custom_mode <= (1 << f32::MANTISSA_DIGITS),
        "custom_mode {custom_mode} exceeds f32 exact-integer range"
    );
    send_command_long_ack(
        MavCmd::MAV_CMD_DO_SET_MODE as u16,
        [1.0, custom_mode as f32, 0.0, 0.0, 0.0, 0.0, 0.0],
        target,
        ctx,
    )
    .await
    .map(|_| ())
}

// ---------------------------------------------------------------------------
// Generic COMMAND_LONG (public API)
// ---------------------------------------------------------------------------

pub(super) async fn handle_command_long(
    command: MavCmd,
    params: [f32; 7],
    ctx: &mut CommandContext,
) -> Result<CommandAck, VehicleError> {
    let target = get_target(&ctx.vehicle_target)?;
    send_command_long_ack(command as u16, params, target, ctx).await
}

pub(super) async fn handle_command_long_raw(
    command_id: u16,
    params: [f32; 7],
    ctx: &mut CommandContext,
) -> Result<CommandAck, VehicleError> {
    let target = get_target(&ctx.vehicle_target)?;
    send_command_long_ack(command_id, params, target, ctx).await
}

pub(super) async fn handle_command_int(
    payload: RawCommandIntPayload,
    ctx: &mut CommandContext,
) -> Result<CommandAck, VehicleError> {
    let target = get_target(&ctx.vehicle_target)?;
    ctx.ack_dispatcher
        .dispatch_command_int(CommandIntRequest {
            connection: ctx.connection.as_ref(),
            config: &ctx.config,
            cancel: &ctx.cancel,
            target_system: target.system_id,
            target_component: target.component_id,
            command: payload.command,
            frame: payload.frame,
            current: payload.current,
            autocontinue: payload.autocontinue,
            params: payload.params,
            x: payload.x,
            y: payload.y,
            z: payload.z,
        })
        .await
}

pub(super) async fn handle_raw_send(
    message: dialect::MavMessage,
    ctx: &mut CommandContext,
) -> Result<(), VehicleError> {
    send_message(ctx.connection.as_ref(), &ctx.config, message).await
}

#[allow(deprecated)]
pub(super) async fn handle_set_origin(
    latitude: i32,
    longitude: i32,
    altitude: i32,
    ctx: &mut CommandContext,
) -> Result<Instant, VehicleError> {
    let target = get_target(&ctx.vehicle_target)?;

    send_message(
        ctx.connection.as_ref(),
        &ctx.config,
        dialect::MavMessage::SET_GPS_GLOBAL_ORIGIN(dialect::SET_GPS_GLOBAL_ORIGIN_DATA {
            latitude,
            longitude,
            altitude,
            target_system: target.system_id,
            time_usec: 0,
        }),
    )
    .await?;

    Ok(Instant::now())
}

// ---------------------------------------------------------------------------
// Guided goto
// ---------------------------------------------------------------------------

pub(super) async fn handle_guided_goto(
    lat_e7: i32,
    lon_e7: i32,
    alt_m: f32,
    ctx: &mut CommandContext,
) -> Result<(), VehicleError> {
    let target = get_target(&ctx.vehicle_target)?;
    let type_mask = dialect::PositionTargetTypemask::from_bits_truncate(GUIDED_GOTO_TYPE_MASK);

    send_message(
        ctx.connection.as_ref(),
        &ctx.config,
        dialect::MavMessage::SET_POSITION_TARGET_GLOBAL_INT(
            dialect::SET_POSITION_TARGET_GLOBAL_INT_DATA {
                time_boot_ms: 0,
                target_system: target.system_id,
                target_component: target.component_id,
                coordinate_frame: dialect::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
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
