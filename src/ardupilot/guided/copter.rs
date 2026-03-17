use super::{ArduCopterGuidedHandle, RelativeClimbTarget};
use crate::command::Command;
use crate::dialect;
use crate::error::VehicleError;
use crate::geo::{GeoPoint3dMsl, GeoPoint3dRelHome};
use crate::mission::send_domain_command;

const POSITION_ONLY_TYPE_MASK: u16 = 0x07F8;
const VELOCITY_ONLY_TYPE_MASK: u16 = 0x0DC7;

impl<'a> ArduCopterGuidedHandle<'a> {
    pub(crate) fn new(session: &'a super::ArduGuidedSession) -> Self {
        Self { _session: session }
    }

    pub async fn takeoff(&self, target: RelativeClimbTarget) -> Result<(), VehicleError> {
        self._session.ensure_active()?;
        validate_positive_f32(target.relative_climb_m, "relative_climb_m")?;

        send_domain_command(self._session.command_tx(), |reply| Command::RawCommandInt {
            payload: crate::command::RawCommandIntPayload {
                command: dialect::MavCmd::MAV_CMD_NAV_TAKEOFF,
                frame: dialect::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
                current: 0,
                autocontinue: 0,
                params: [0.0, 0.0, 0.0, 0.0],
                x: 0,
                y: 0,
                z: target.relative_climb_m,
            },
            reply,
        })
        .await
        .map(|_| ())
    }

    pub async fn goto(&self, target: GeoPoint3dRelHome) -> Result<(), VehicleError> {
        self._session.ensure_active()?;
        let lat_e7 = quantize_degrees_e7(target.latitude_deg, "latitude_deg")?;
        let lon_e7 = quantize_degrees_e7(target.longitude_deg, "longitude_deg")?;
        let alt_m = finite_f32(target.relative_alt_m, "relative_alt_m")?;

        send_domain_command(self._session.command_tx(), |reply| Command::GuidedGoto {
            lat_e7,
            lon_e7,
            alt_m,
            reply,
        })
        .await
    }

    pub async fn goto_msl(&self, target: GeoPoint3dMsl) -> Result<(), VehicleError> {
        self._session.ensure_active()?;
        self.send_position_target_global_int(
            dialect::MavFrame::MAV_FRAME_GLOBAL,
            quantize_degrees_e7(target.latitude_deg, "latitude_deg")?,
            quantize_degrees_e7(target.longitude_deg, "longitude_deg")?,
            finite_f32(target.altitude_msl_m, "altitude_msl_m")?,
        )
        .await
    }

    pub async fn set_velocity_ned(
        &self,
        north_mps: f32,
        east_mps: f32,
        down_mps: f32,
    ) -> Result<(), VehicleError> {
        self._session.ensure_active()?;
        let north_mps = finite_f32(north_mps as f64, "north_mps")?;
        let east_mps = finite_f32(east_mps as f64, "east_mps")?;
        let down_mps = finite_f32(down_mps as f64, "down_mps")?;
        let (target_system, target_component) = self._session.target();
        let type_mask =
            dialect::PositionTargetTypemask::from_bits_truncate(VELOCITY_ONLY_TYPE_MASK);

        send_domain_command(self._session.command_tx(), |reply| Command::RawSend {
            message: Box::new(dialect::MavMessage::SET_POSITION_TARGET_LOCAL_NED(
                dialect::SET_POSITION_TARGET_LOCAL_NED_DATA {
                    time_boot_ms: 0,
                    target_system,
                    target_component,
                    coordinate_frame: dialect::MavFrame::MAV_FRAME_LOCAL_NED,
                    type_mask,
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    vx: north_mps,
                    vy: east_mps,
                    vz: down_mps,
                    afx: 0.0,
                    afy: 0.0,
                    afz: 0.0,
                    yaw: 0.0,
                    yaw_rate: 0.0,
                },
            )),
            reply,
        })
        .await
    }

    pub async fn hold(&self) -> Result<(), VehicleError> {
        self._session.ensure_active()?;

        send_domain_command(self._session.command_tx(), |reply| Command::RawCommandInt {
            payload: crate::command::RawCommandIntPayload {
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

    async fn send_position_target_global_int(
        &self,
        frame: dialect::MavFrame,
        lat_e7: i32,
        lon_e7: i32,
        alt_m: f32,
    ) -> Result<(), VehicleError> {
        let (target_system, target_component) = self._session.target();
        let type_mask =
            dialect::PositionTargetTypemask::from_bits_truncate(POSITION_ONLY_TYPE_MASK);

        send_domain_command(self._session.command_tx(), |reply| Command::RawSend {
            message: Box::new(dialect::MavMessage::SET_POSITION_TARGET_GLOBAL_INT(
                dialect::SET_POSITION_TARGET_GLOBAL_INT_DATA {
                    time_boot_ms: 0,
                    target_system,
                    target_component,
                    coordinate_frame: frame,
                    type_mask,
                    lat_int: lat_e7,
                    lon_int: lon_e7,
                    alt: alt_m,
                    vx: 0.0,
                    vy: 0.0,
                    vz: 0.0,
                    afx: 0.0,
                    afy: 0.0,
                    afz: 0.0,
                    yaw: 0.0,
                    yaw_rate: 0.0,
                },
            )),
            reply,
        })
        .await
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

fn validate_positive_f32(value: f32, field: &str) -> Result<(), VehicleError> {
    if !value.is_finite() || value <= 0.0 {
        return Err(VehicleError::InvalidParameter(format!(
            "{field} must be finite and > 0, got {value}"
        )));
    }

    Ok(())
}
