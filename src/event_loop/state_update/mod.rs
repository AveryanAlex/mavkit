use super::VehicleTarget;
use crate::dialect;
use crate::state::StateWriters;
use mavlink::MavHeader;
use tracing::trace;

mod flight;
mod gps;
mod radio;
mod status_text;
mod system;
#[cfg(test)]
mod tests;
mod vehicle;

struct UpdateContext<'a> {
    header: &'a MavHeader,
    writers: &'a StateWriters,
    vehicle_target: &'a Option<VehicleTarget>,
}

pub(super) fn update_state(
    header: &MavHeader,
    message: &dialect::MavMessage,
    writers: &StateWriters,
    vehicle_target: &Option<VehicleTarget>,
) {
    let context = UpdateContext {
        header,
        writers,
        vehicle_target,
    };

    match message {
        dialect::MavMessage::HEARTBEAT(data) => vehicle::handle_heartbeat(&context, data),
        dialect::MavMessage::VFR_HUD(data) => flight::handle_vfr_hud(&context, data),
        dialect::MavMessage::GLOBAL_POSITION_INT(data) => {
            flight::handle_global_position_int(&context, data)
        }
        dialect::MavMessage::LOCAL_POSITION_NED(data) => {
            flight::handle_local_position_ned(&context, data)
        }
        dialect::MavMessage::SYS_STATUS(data) => system::handle_sys_status(&context, data),
        dialect::MavMessage::GPS_RAW_INT(data) => gps::handle_gps_raw_int(&context, data),
        dialect::MavMessage::MISSION_CURRENT(data) => {
            vehicle::handle_mission_current(&context, data)
        }
        dialect::MavMessage::HOME_POSITION(data) => gps::handle_home_position(&context, data),
        dialect::MavMessage::GPS_GLOBAL_ORIGIN(data) => {
            gps::handle_gps_global_origin(&context, data)
        }
        dialect::MavMessage::ATTITUDE(data) => flight::handle_attitude(&context, data),
        dialect::MavMessage::NAV_CONTROLLER_OUTPUT(data) => {
            flight::handle_nav_controller_output(&context, data)
        }
        dialect::MavMessage::TERRAIN_REPORT(data) => flight::handle_terrain_report(&context, data),
        dialect::MavMessage::BATTERY_STATUS(data) => system::handle_battery_status(&context, data),
        dialect::MavMessage::RC_CHANNELS(data) => radio::handle_rc_channels(&context, data),
        dialect::MavMessage::SERVO_OUTPUT_RAW(data) => {
            radio::handle_servo_output_raw(&context, data)
        }
        dialect::MavMessage::STATUSTEXT(data) => status_text::handle_status_text(&context, data),
        dialect::MavMessage::MAG_CAL_REPORT(data) => vehicle::handle_mag_cal_report(&context, data),
        dialect::MavMessage::MAG_CAL_PROGRESS(data) => {
            vehicle::handle_mag_cal_progress(&context, data)
        }
        _ => {
            trace!("unhandled message type");
        }
    }
}
