use super::UpdateContext;
use crate::dialect;
use crate::telemetry::{
    TelemetryMessageKind, infer_timestamp_from_time_boot_ms, infer_timestamp_from_time_usec,
};

/// Maximum number of RC channels in the RC_CHANNELS MAVLink message.
const RC_CHANNELS_MAX: usize = 18;

pub(super) fn handle_rc_channels(context: &UpdateContext<'_>, data: &dialect::RC_CHANNELS_DATA) {
    let count = (data.chancount as usize).min(RC_CHANNELS_MAX);
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
    let all = [
        data.chan1_raw,
        data.chan2_raw,
        data.chan3_raw,
        data.chan4_raw,
        data.chan5_raw,
        data.chan6_raw,
        data.chan7_raw,
        data.chan8_raw,
        data.chan9_raw,
        data.chan10_raw,
        data.chan11_raw,
        data.chan12_raw,
        data.chan13_raw,
        data.chan14_raw,
        data.chan15_raw,
        data.chan16_raw,
        data.chan17_raw,
        data.chan18_raw,
    ];

    for (index, value) in all.iter().enumerate().take(count) {
        context.writers.telemetry_metrics.rc_channels_pwm_us[index].publish(
            *value,
            TelemetryMessageKind::RcChannels,
            vehicle_time.clone(),
        );
    }

    for index in count..RC_CHANNELS_MAX {
        context.writers.telemetry_metrics.rc_channels_pwm_us[index].clear();
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
    let values = [
        data.servo1_raw,
        data.servo2_raw,
        data.servo3_raw,
        data.servo4_raw,
        data.servo5_raw,
        data.servo6_raw,
        data.servo7_raw,
        data.servo8_raw,
        data.servo9_raw,
        data.servo10_raw,
        data.servo11_raw,
        data.servo12_raw,
        data.servo13_raw,
        data.servo14_raw,
        data.servo15_raw,
        data.servo16_raw,
    ];
    for (index, value) in values.iter().enumerate() {
        context.writers.telemetry_metrics.actuator_servo_pwm_us[index].publish(
            *value,
            TelemetryMessageKind::ServoOutputRaw,
            vehicle_time.clone(),
        );
    }
}
