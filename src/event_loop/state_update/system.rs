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
    let pct = if data.battery_remaining >= 0 {
        Some(f64::from(data.battery_remaining))
    } else {
        None
    };
    let volt = if data.voltage_battery != u16::MAX {
        Some(f64::from(data.voltage_battery) / 1000.0)
    } else {
        None
    };
    let cur = if data.current_battery >= 0 {
        Some(f64::from(data.current_battery) / 100.0)
    } else {
        None
    };

    context
        .writers
        .telemetry_metrics
        .publish_battery_from_sys_status(pct, volt, cur);

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
    let cells: Vec<f64> = data
        .voltages
        .iter()
        .filter(|&&v| v != u16::MAX)
        .map(|&v| f64::from(v) / 1000.0)
        .collect();
    let cells_opt = if !cells.is_empty() { Some(cells) } else { None };
    let energy = if data.energy_consumed >= 0 {
        Some(f64::from(data.energy_consumed) / 36.0)
    } else {
        None
    };
    let remaining = if data.time_remaining > 0 {
        Some(data.time_remaining)
    } else {
        None
    };

    if data.id == 0 {
        let remaining_pct = if data.battery_remaining >= 0 {
            Some(f64::from(data.battery_remaining))
        } else {
            None
        };
        let current_a = if data.current_battery >= 0 {
            Some(f64::from(data.current_battery) / 100.0)
        } else {
            None
        };

        context
            .writers
            .telemetry_metrics
            .publish_battery_from_primary_status(
                remaining_pct,
                cells_opt,
                current_a,
                energy,
                remaining,
            );
    }
}
