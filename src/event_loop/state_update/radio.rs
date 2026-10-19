use super::UpdateContext;
use crate::dialect;
use crate::telemetry::{
    TelemetryMessageKind, infer_timestamp_from_time_boot_ms, infer_timestamp_from_time_usec,
};

pub(super) fn handle_rc_channels(context: &UpdateContext<'_>, data: &dialect::RC_CHANNELS_DATA) {
    let channels = crate::telemetry::decode::rc_channels(data);
    let count = channels.len();
    let rssi = if data.rssi != u8::MAX {
        Some(data.rssi)
    } else {
        None
    };

    let vehicle_time =
        infer_timestamp_from_time_boot_ms(TelemetryMessageKind::RcChannels, data.time_boot_ms);
    context
        .writers
        .telemetry_metrics
        .message_writers
        .rc_channels
        .publish(data.clone(), vehicle_time.clone());
    for (index, value) in channels.into_iter().enumerate() {
        let channel = &context.writers.telemetry_metrics.rc_channels_pwm_us[index];
        if let Some(value) = value {
            channel.publish(
                value,
                TelemetryMessageKind::RcChannels,
                vehicle_time.clone(),
            );
        } else {
            channel.clear();
        }
    }

    for channel in context
        .writers
        .telemetry_metrics
        .rc_channels_pwm_us
        .iter()
        .skip(count)
    {
        channel.clear();
    }

    if let Some(rssi_pct) = rssi {
        context.writers.telemetry_metrics.rc_rssi_pct.publish(
            rssi_pct,
            TelemetryMessageKind::RcChannels,
            vehicle_time,
        );
    }
}

pub(super) fn handle_servo_output_raw(
    context: &UpdateContext<'_>,
    data: &dialect::SERVO_OUTPUT_RAW_DATA,
) {
    let vehicle_time = infer_timestamp_from_time_usec(
        TelemetryMessageKind::ServoOutputRaw,
        u64::from(data.time_usec),
    );
    context
        .writers
        .telemetry_metrics
        .message_writers
        .servo_output_raw
        .publish(data.port, data.clone(), vehicle_time.clone());
    let values = crate::telemetry::decode::servo_outputs(data);
    for (index, value) in values.iter().enumerate() {
        context.writers.telemetry_metrics.actuator_servo_pwm_us[index].publish(
            *value,
            TelemetryMessageKind::ServoOutputRaw,
            vehicle_time.clone(),
        );
    }
}
