use crate::dialect;
use crate::mission::{commands, MissionFrame, MissionItem, MissionType};

pub(in crate::event_loop) fn to_mav_mission_type(
    mission_type: MissionType,
) -> dialect::MavMissionType {
    match mission_type {
        MissionType::Mission => dialect::MavMissionType::MAV_MISSION_TYPE_MISSION,
        MissionType::Fence => dialect::MavMissionType::MAV_MISSION_TYPE_FENCE,
        MissionType::Rally => dialect::MavMissionType::MAV_MISSION_TYPE_RALLY,
    }
}

pub(in crate::event_loop) fn to_mav_frame(frame: MissionFrame) -> dialect::MavFrame {
    match frame {
        MissionFrame::Mission => dialect::MavFrame::MAV_FRAME_MISSION,
        MissionFrame::GlobalInt => dialect::MavFrame::MAV_FRAME_GLOBAL,
        MissionFrame::GlobalRelativeAltInt => dialect::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MissionFrame::GlobalTerrainAltInt => dialect::MavFrame::MAV_FRAME_GLOBAL_TERRAIN_ALT,
        MissionFrame::LocalNed => dialect::MavFrame::MAV_FRAME_LOCAL_NED,
        MissionFrame::Other => dialect::MavFrame::MAV_FRAME_MISSION,
    }
}

// MAVLink crate deprecated this type/variant, but the wire protocol still requires it.
#[allow(deprecated)]
pub(in crate::event_loop) fn from_mav_frame(frame: dialect::MavFrame) -> MissionFrame {
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

pub(in crate::event_loop) fn from_mission_item_int(
    data: &dialect::MISSION_ITEM_INT_DATA,
) -> MissionItem {
    let frame = from_mav_frame(data.frame);
    MissionItem {
        command: commands::MissionCommand::from_wire(
            data.command as u16,
            commands::MissionFrame::from(frame),
            [data.param1, data.param2, data.param3, data.param4],
            data.x,
            data.y,
            data.z,
        ),
        autocontinue: data.autocontinue > 0,
    }
}

pub(in crate::event_loop) fn mission_type_matches(
    received: dialect::MavMissionType,
    expected: MissionType,
) -> bool {
    received == to_mav_mission_type(expected)
}
