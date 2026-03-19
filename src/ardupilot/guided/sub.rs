use super::{ArduSubGuidedHandle, SubGotoDepthTarget};
use crate::command::{Command, RawCommandIntPayload};
use crate::dialect;
use crate::error::VehicleError;
use crate::geo::{try_latitude_e7, try_longitude_e7};
use crate::mission::send_domain_command;

const VELOCITY_YAW_RATE_TYPE_MASK: u16 = 0x05C7;

impl<'a> ArduSubGuidedHandle<'a> {
    pub async fn goto_depth(&self, target: SubGotoDepthTarget) -> Result<(), VehicleError> {
        self._session.ensure_active()?;
        let lat_e7 = try_latitude_e7(target.point.latitude_deg)?;
        let lon_e7 = try_longitude_e7(target.point.longitude_deg)?;
        let depth_m = validate_non_negative_f32(target.depth_m, "depth_m")?;

        send_domain_command(self._session.command_tx(), |reply| Command::RawCommandInt {
            payload: RawCommandIntPayload {
                command: dialect::MavCmd::MAV_CMD_DO_REPOSITION,
                frame: dialect::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
                current: 0,
                autocontinue: 0,
                params: [0.0, 0.0, 0.0, 0.0],
                x: lat_e7,
                y: lon_e7,
                z: -depth_m,
            },
            reply,
        })
        .await
        .map(|_| ())
    }

    pub async fn set_velocity_body(
        &self,
        forward_mps: f32,
        lateral_mps: f32,
        vertical_mps: f32,
        yaw_rate_dps: f32,
    ) -> Result<(), VehicleError> {
        self._session.ensure_active()?;
        let forward_mps = finite_f32(f64::from(forward_mps), "forward_mps")?;
        let lateral_mps = finite_f32(f64::from(lateral_mps), "lateral_mps")?;
        let vertical_mps = finite_f32(f64::from(vertical_mps), "vertical_mps")?;
        let yaw_rate = degrees_to_radians_f32(f64::from(yaw_rate_dps), "yaw_rate_dps")?;
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
                    vy: lateral_mps,
                    vz: vertical_mps,
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
        self._session.ensure_active()?;

        send_domain_command(self._session.command_tx(), |reply| Command::RawCommandInt {
            payload: RawCommandIntPayload {
                command: dialect::MavCmd::MAV_CMD_NAV_LOITER_UNLIM,
                frame: dialect::MavFrame::MAV_FRAME_MISSION,
                current: 0,
                autocontinue: 0,
                params: [0.0, 0.0, 0.0, 0.0],
                x: 0,
                y: 0,
                z: 0.0,
            },
            reply,
        })
        .await
        .map(|_| ())
    }
}

fn finite_f32(value: f64, field: &str) -> Result<f32, VehicleError> {
    if !value.is_finite() {
        return Err(VehicleError::InvalidParameter(format!(
            "{field} must be finite, got {value}"
        )));
    }

    Ok(value as f32)
}

fn validate_non_negative_f32(value: f32, field: &str) -> Result<f32, VehicleError> {
    if !value.is_finite() || value < 0.0 {
        return Err(VehicleError::InvalidParameter(format!(
            "{field} must be finite and >= 0, got {value}"
        )));
    }

    Ok(value)
}

fn degrees_to_radians_f32(value: f64, field: &str) -> Result<f32, VehicleError> {
    finite_f32(value, field).map(f32::to_radians)
}
