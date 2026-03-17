use super::ArduRoverGuidedHandle;
use crate::command::{Command, RawCommandIntPayload};
use crate::dialect;
use crate::error::VehicleError;
use crate::geo::GeoPoint2d;
use crate::mission::send_domain_command;

const VELOCITY_YAW_RATE_TYPE_MASK: u16 = 0x05E7;

impl<'a> ArduRoverGuidedHandle<'a> {
    pub async fn drive_to(&self, target: GeoPoint2d) -> Result<(), VehicleError> {
        self._session.ensure_active()?;
        let lat_e7 = quantize_degrees_e7(target.latitude_deg, "latitude_deg")?;
        let lon_e7 = quantize_degrees_e7(target.longitude_deg, "longitude_deg")?;

        send_domain_command(self._session.command_tx(), |reply| Command::RawCommandInt {
            payload: RawCommandIntPayload {
                command: dialect::MavCmd::MAV_CMD_DO_REPOSITION,
                frame: dialect::MavFrame::MAV_FRAME_GLOBAL,
                current: 0,
                autocontinue: 0,
                params: [0.0, 0.0, 0.0, 0.0],
                x: lat_e7,
                y: lon_e7,
                z: 0.0,
            },
            reply,
        })
        .await
        .map(|_| ())
    }

    pub async fn drive(&self, forward_mps: f32, turn_rate_dps: f32) -> Result<(), VehicleError> {
        self._session.ensure_active()?;
        let forward_mps = finite_f32(forward_mps as f64, "forward_mps")?;
        let yaw_rate = degrees_to_radians_f32(turn_rate_dps as f64, "turn_rate_dps")?;
        let (target_system, target_component) = self._session.target();
        let type_mask =
            dialect::PositionTargetTypemask::from_bits_truncate(VELOCITY_YAW_RATE_TYPE_MASK);

        send_domain_command(self._session.command_tx(), |reply| Command::RawSend {
            message: Box::new(dialect::MavMessage::SET_POSITION_TARGET_LOCAL_NED(
                dialect::SET_POSITION_TARGET_LOCAL_NED_DATA {
                    time_boot_ms: 0,
                    target_system,
                    target_component,
                    coordinate_frame: dialect::MavFrame::MAV_FRAME_BODY_FRD,
                    type_mask,
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    vx: forward_mps,
                    vy: 0.0,
                    vz: 0.0,
                    afx: 0.0,
                    afy: 0.0,
                    afz: 0.0,
                    yaw: 0.0,
                    yaw_rate,
                },
            )),
            reply,
        })
        .await
    }

    pub async fn hold(&self) -> Result<(), VehicleError> {
        self.drive(0.0, 0.0).await
    }
}

fn quantize_degrees_e7(value: f64, field: &str) -> Result<i32, VehicleError> {
    if !value.is_finite() {
        return Err(VehicleError::InvalidParameter(format!(
            "{field} must be finite, got {value}"
        )));
    }

    Ok((value * 1e7).round() as i32)
}

fn finite_f32(value: f64, field: &str) -> Result<f32, VehicleError> {
    if !value.is_finite() {
        return Err(VehicleError::InvalidParameter(format!(
            "{field} must be finite, got {value}"
        )));
    }

    Ok(value as f32)
}

fn degrees_to_radians_f32(value: f64, field: &str) -> Result<f32, VehicleError> {
    finite_f32(value, field).map(f32::to_radians)
}
