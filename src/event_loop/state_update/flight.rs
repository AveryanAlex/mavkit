use super::UpdateContext;
use crate::dialect;
use crate::telemetry::{
    EulerAttitude, GlobalPosition, GuidanceState, TelemetryMessageKind, TerrainClearance,
    WaypointProgress, infer_timestamp_from_time_boot_ms,
};

pub(super) fn handle_vfr_hud(context: &UpdateContext<'_>, data: &dialect::VFR_HUD_DATA) {
    context
        .writers
        .telemetry_metrics
        .message_writers
        .vfr_hud
        .publish(data.clone(), None);

    context
        .writers
        .telemetry_metrics
        .position_groundspeed_mps
        .publish(
            f64::from(data.groundspeed),
            TelemetryMessageKind::VfrHud,
            None,
        );
    context
        .writers
        .telemetry_metrics
        .position_airspeed_mps
        .publish(f64::from(data.airspeed), TelemetryMessageKind::VfrHud, None);
    context
        .writers
        .telemetry_metrics
        .position_climb_rate_mps
        .publish(f64::from(data.climb), TelemetryMessageKind::VfrHud, None);
    context
        .writers
        .telemetry_metrics
        .position_heading_deg
        .publish(f64::from(data.heading), TelemetryMessageKind::VfrHud, None);
    context
        .writers
        .telemetry_metrics
        .position_throttle_pct
        .publish(f64::from(data.throttle), TelemetryMessageKind::VfrHud, None);
}

pub(super) fn handle_global_position_int(
    context: &UpdateContext<'_>,
    data: &dialect::GLOBAL_POSITION_INT_DATA,
) {
    let vx = f64::from(data.vx) / 100.0;
    let vy = f64::from(data.vy) / 100.0;

    let vehicle_time = infer_timestamp_from_time_boot_ms(
        TelemetryMessageKind::GlobalPositionInt,
        data.time_boot_ms,
    );
    context
        .writers
        .telemetry_metrics
        .message_writers
        .global_position_int
        .publish(data.clone(), vehicle_time.clone());
    context.writers.telemetry_metrics.position_global.publish(
        GlobalPosition {
            latitude_deg: f64::from(data.lat) / 1e7,
            longitude_deg: f64::from(data.lon) / 1e7,
            altitude_msl_m: f64::from(data.alt) / 1000.0,
            relative_alt_m: f64::from(data.relative_alt) / 1000.0,
        },
        TelemetryMessageKind::GlobalPositionInt,
        vehicle_time.clone(),
    );

    let ground_speed = (vx * vx + vy * vy).sqrt();
    context
        .writers
        .telemetry_metrics
        .position_groundspeed_mps
        .publish(
            ground_speed,
            TelemetryMessageKind::GlobalPositionInt,
            vehicle_time.clone(),
        );

    if data.hdg != u16::MAX {
        context
            .writers
            .telemetry_metrics
            .position_heading_deg
            .publish(
                f64::from(data.hdg) / 100.0,
                TelemetryMessageKind::GlobalPositionInt,
                vehicle_time.clone(),
            );
    }

    context
        .writers
        .telemetry_metrics
        .position_climb_rate_mps
        .publish(
            -(f64::from(data.vz) / 100.0),
            TelemetryMessageKind::GlobalPositionInt,
            vehicle_time,
        );
}

pub(super) fn handle_local_position_ned(
    context: &UpdateContext<'_>,
    data: &dialect::LOCAL_POSITION_NED_DATA,
) {
    context
        .writers
        .telemetry_metrics
        .message_writers
        .local_position_ned
        .publish(
            data.clone(),
            infer_timestamp_from_time_boot_ms(
                TelemetryMessageKind::LocalPositionNed,
                data.time_boot_ms,
            ),
        );
}

pub(super) fn handle_attitude(context: &UpdateContext<'_>, data: &dialect::ATTITUDE_DATA) {
    let vehicle_time =
        infer_timestamp_from_time_boot_ms(TelemetryMessageKind::Attitude, data.time_boot_ms);
    context
        .writers
        .telemetry_metrics
        .message_writers
        .attitude
        .publish(data.clone(), vehicle_time.clone());
    context.writers.telemetry_metrics.attitude_euler.publish(
        EulerAttitude {
            roll_deg: f64::from(data.roll.to_degrees()),
            pitch_deg: f64::from(data.pitch.to_degrees()),
            yaw_deg: f64::from(data.yaw.to_degrees()),
        },
        TelemetryMessageKind::Attitude,
        vehicle_time,
    );
}

pub(super) fn handle_nav_controller_output(
    context: &UpdateContext<'_>,
    data: &dialect::NAV_CONTROLLER_OUTPUT_DATA,
) {
    context
        .writers
        .telemetry_metrics
        .message_writers
        .nav_controller_output
        .publish(data.clone(), None);

    context
        .writers
        .telemetry_metrics
        .navigation_waypoint
        .publish(
            WaypointProgress {
                distance_m: f64::from(data.wp_dist),
                bearing_deg: f64::from(data.nav_bearing),
            },
            TelemetryMessageKind::NavControllerOutput,
            None,
        );
    context
        .writers
        .telemetry_metrics
        .navigation_guidance
        .publish(
            GuidanceState {
                bearing_deg: f64::from(data.target_bearing),
                cross_track_error_m: f64::from(data.xtrack_error),
            },
            TelemetryMessageKind::NavControllerOutput,
            None,
        );
}

pub(super) fn handle_terrain_report(
    context: &UpdateContext<'_>,
    data: &dialect::TERRAIN_REPORT_DATA,
) {
    context
        .writers
        .telemetry_metrics
        .message_writers
        .terrain_report
        .publish(data.clone(), None);

    context.writers.telemetry_metrics.terrain_clearance.publish(
        TerrainClearance {
            terrain_height_m: f64::from(data.terrain_height),
            height_above_terrain_m: f64::from(data.current_height),
        },
        TelemetryMessageKind::TerrainReport,
        None,
    );
}
