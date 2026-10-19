use super::UpdateContext;
use crate::dialect;
use crate::state::{SensorHealth, set_if_changed};
use crate::telemetry::{
    TelemetryMessageKind, sensor_health::sensor_health_summary_from_sys_status,
};

pub(super) fn handle_sys_status(context: &UpdateContext<'_>, data: &dialect::SYS_STATUS_DATA) {
    context
        .writers
        .telemetry_metrics
        .message_writers
        .sys_status
        .publish(data.clone(), None);
    let power = crate::telemetry::decode::system_power(data);
    context
        .writers
        .telemetry_metrics
        .publish_battery_from_sys_status(power.remaining_pct, power.voltage_v, power.current_a);

    let new_health = SensorHealth::from_bitmasks(
        data.onboard_control_sensors_present.bits(),
        data.onboard_control_sensors_enabled.bits(),
        data.onboard_control_sensors_health.bits(),
    );
    context
        .writers
        .sensor_health
        .send_if_modified(|h| set_if_changed(h, new_health.clone()));
    context.writers.telemetry_metrics.sensor_health.publish(
        sensor_health_summary_from_sys_status(
            data.onboard_control_sensors_present,
            data.onboard_control_sensors_enabled,
            data.onboard_control_sensors_health,
        ),
        TelemetryMessageKind::SysStatus,
        None,
    );
}

pub(super) fn handle_battery_status(
    context: &UpdateContext<'_>,
    data: &dialect::BATTERY_STATUS_DATA,
) {
    context
        .writers
        .telemetry_metrics
        .message_writers
        .battery_status
        .publish(data.id, data.clone(), None);
    if data.id == 0 {
        let battery = crate::telemetry::decode::battery_status(data);
        context
            .writers
            .telemetry_metrics
            .publish_battery_from_primary_status(
                battery.remaining_pct,
                battery.cell_voltages_v,
                battery.current_a,
                battery.energy_consumed_wh,
                battery.time_remaining_s,
            );
    }
}
