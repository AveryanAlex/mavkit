//! Pure MAVLink unit/sentinel decoding shared by live telemetry and log readers.
use crate::{EulerAttitude, GlobalPosition, GpsFixType, GpsQuality, dialect};

#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct SystemPower {
    pub remaining_pct: Option<f64>,
    pub voltage_v: Option<f64>,
    pub current_a: Option<f64>,
}

#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct BatteryStatus {
    pub remaining_pct: Option<f64>,
    pub current_a: Option<f64>,
    pub cell_voltages_v: Option<Vec<f64>>,
    pub energy_consumed_wh: Option<f64>,
    pub time_remaining_s: Option<i32>,
}

pub fn system_power(data: &dialect::SYS_STATUS_DATA) -> SystemPower {
    SystemPower {
        remaining_pct: (data.battery_remaining >= 0).then_some(f64::from(data.battery_remaining)),
        voltage_v: (data.voltage_battery != u16::MAX)
            .then_some(f64::from(data.voltage_battery) / 1000.0),
        current_a: (data.current_battery >= 0).then_some(f64::from(data.current_battery) / 100.0),
    }
}

pub fn battery_status(data: &dialect::BATTERY_STATUS_DATA) -> BatteryStatus {
    let cells: Vec<_> = data
        .voltages
        .iter()
        .filter(|&&v| v != u16::MAX)
        .map(|&v| f64::from(v) / 1000.0)
        .collect();
    BatteryStatus {
        remaining_pct: (data.battery_remaining >= 0).then_some(f64::from(data.battery_remaining)),
        current_a: (data.current_battery >= 0).then_some(f64::from(data.current_battery) / 100.0),
        cell_voltages_v: (!cells.is_empty()).then_some(cells),
        energy_consumed_wh: (data.energy_consumed >= 0)
            .then_some(f64::from(data.energy_consumed) / 36.0),
        time_remaining_s: (data.time_remaining > 0).then_some(data.time_remaining),
    }
}

pub fn gps_quality(data: &dialect::GPS_RAW_INT_DATA) -> GpsQuality {
    GpsQuality {
        fix_type: GpsFixType::from_raw(data.fix_type as u8),
        satellites: (data.satellites_visible != u8::MAX).then_some(data.satellites_visible),
        hdop: (data.eph != u16::MAX).then_some(f64::from(data.eph) / 100.0),
    }
}

pub fn global_position(data: &dialect::GLOBAL_POSITION_INT_DATA) -> GlobalPosition {
    GlobalPosition {
        latitude_deg: f64::from(data.lat) / 1e7,
        longitude_deg: f64::from(data.lon) / 1e7,
        altitude_msl_m: f64::from(data.alt) / 1000.0,
        relative_alt_m: f64::from(data.relative_alt) / 1000.0,
    }
}

pub fn attitude(data: &dialect::ATTITUDE_DATA) -> EulerAttitude {
    EulerAttitude {
        roll_deg: f64::from(data.roll).to_degrees(),
        pitch_deg: f64::from(data.pitch).to_degrees(),
        yaw_deg: f64::from(data.yaw).to_degrees(),
    }
}

pub fn rc_channels(data: &dialect::RC_CHANNELS_DATA) -> Vec<Option<u16>> {
    [
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
    ]
    .into_iter()
    .take(usize::from(data.chancount).min(18))
    .map(|value| (value != u16::MAX).then_some(value))
    .collect()
}

pub fn servo_outputs(data: &dialect::SERVO_OUTPUT_RAW_DATA) -> [u16; 16] {
    [
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
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn energy_uses_hectojoules_and_unknowns_stay_missing() {
        let mut data = dialect::BATTERY_STATUS_DATA {
            energy_consumed: 3600,
            ..Default::default()
        };
        assert_eq!(battery_status(&data).energy_consumed_wh, Some(100.0));
        data.energy_consumed = -1;
        assert_eq!(battery_status(&data).energy_consumed_wh, None);
        let gps = dialect::GPS_RAW_INT_DATA {
            satellites_visible: 255,
            eph: 65535,
            ..Default::default()
        };
        assert_eq!(gps_quality(&gps).satellites, None);
        assert_eq!(gps_quality(&gps).hdop, None);
    }
    #[test]
    fn rc_samples_preserve_channel_numbers_and_zero() {
        let data = dialect::RC_CHANNELS_DATA {
            chancount: 18,
            chan1_raw: 0,
            chan2_raw: 65535,
            chan18_raw: 1800,
            ..Default::default()
        };
        let channels = rc_channels(&data);
        assert_eq!(channels.len(), 18);
        assert_eq!(channels[0], Some(0));
        assert_eq!(channels[1], None);
        assert_eq!(channels[17], Some(1800));
    }
}
