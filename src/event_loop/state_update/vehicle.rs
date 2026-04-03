use super::UpdateContext;
use crate::dialect::{self, MavModeFlag};
use crate::mission;
use crate::state::{
    AutopilotType, MagCalProgress, MagCalReport, MagCalStatus, SystemStatus, VehicleState,
    VehicleType, WireMissionState,
};
use crate::telemetry::TelemetryMessageKind;

pub(super) fn handle_heartbeat(context: &UpdateContext<'_>, data: &dialect::HEARTBEAT_DATA) {
    if let Some(target) = context.vehicle_target {
        let autopilot_type = AutopilotType::from_mav(target.autopilot);
        let vtype = VehicleType::from_mav(target.vehicle_type);
        let armed = data
            .base_mode
            .contains(MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED);
        let mode_name = crate::modes::mode_name(autopilot_type, vtype, data.custom_mode);

        let _ = context.writers.vehicle_state.send(VehicleState {
            armed,
            custom_mode: data.custom_mode,
            mode_name,
            system_status: SystemStatus::from_mav(data.system_status),
            vehicle_type: vtype,
            autopilot: autopilot_type,
            system_id: target.system_id,
            component_id: target.component_id,
            heartbeat_received: true,
        });

        context.writers.telemetry_metrics.armed.publish(
            armed,
            TelemetryMessageKind::Heartbeat,
            None,
        );
    }
}

pub(super) fn handle_mission_current(
    context: &UpdateContext<'_>,
    data: &dialect::MISSION_CURRENT_DATA,
) {
    // MISSION_CURRENT always reports Mission type execution state
    let _ = context
        .writers
        .mission_state
        .send(WireMissionState::from_wire(
            mission::MissionType::Mission,
            data.seq,
            data.total,
        ));
}

pub(super) fn handle_mag_cal_report(
    context: &UpdateContext<'_>,
    data: &dialect::MAG_CAL_REPORT_DATA,
) {
    let _ = context.writers.mag_cal_report.send(Some(MagCalReport {
        compass_id: data.compass_id,
        status: MagCalStatus::from_mav(data.cal_status),
        fitness: data.fitness,
        ofs_x: data.ofs_x,
        ofs_y: data.ofs_y,
        ofs_z: data.ofs_z,
        autosaved: data.autosaved != 0,
    }));
}

pub(super) fn handle_mag_cal_progress(
    context: &UpdateContext<'_>,
    data: &dialect::MAG_CAL_PROGRESS_DATA,
) {
    let _ = context.writers.mag_cal_progress.send(Some(MagCalProgress {
        compass_id: data.compass_id,
        completion_pct: data.completion_pct,
        status: MagCalStatus::from_mav(data.cal_status),
        attempt: data.attempt,
    }));
}
