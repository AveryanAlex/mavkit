use super::super::{CommandContext, get_target, send_message};
use crate::dialect;
use crate::error::VehicleError;
use crate::mission::{MissionTransferMachine, MissionType};
use tokio_util::sync::CancellationToken;

use super::convert::to_mav_mission_type;
use super::protocol::wait_for_mission_ack;

pub(in crate::event_loop) async fn handle_mission_clear(
    mission_type: MissionType,
    ctx: &mut CommandContext,
    op_cancel: &CancellationToken,
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

    wait_for_mission_ack(&mut machine, mission_type, ctx, op_cancel, || {
        clear_msg.clone()
    })
    .await
}
