use crate::error::VehicleError;

pub(super) const MAV_CMD_DO_START_MAG_CAL_ID: u16 = 42424;
pub(super) const MAV_CMD_DO_ACCEPT_MAG_CAL_ID: u16 = 42425;
pub(super) const MAV_CMD_DO_CANCEL_MAG_CAL_ID: u16 = 42426;

pub(super) fn preflight_calibration_params(
    gyro: bool,
    accel: bool,
    baro: bool,
    accel_trim: bool,
) -> Result<[f32; 7], VehicleError> {
    let selected = [gyro, accel, baro, accel_trim]
        .into_iter()
        .filter(|enabled| *enabled)
        .count();
    if selected != 1 {
        return Err(VehicleError::InvalidParameter(
            "preflight_calibration requires exactly one requested calibration".into(),
        ));
    }

    let params = if gyro {
        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    } else if accel {
        [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0]
    } else if baro {
        [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0]
    } else {
        [0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0]
    };

    Ok(params)
}
