use super::RelativeClimbTarget;
use crate::command::{Command, RawCommandIntPayload};
use crate::dialect;
use crate::error::VehicleError;
use crate::geo::{GeoPoint3dMsl, GeoPoint3dRelHome};
use crate::mission::send_domain_command;

const MAV_DO_REPOSITION_FLAGS_CHANGE_MODE: f32 = 1.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// Guided plane subtype, fixed wing or VTOL-capable.
pub enum ArduPlaneKind {
    FixedWing,
    Vtol,
}

#[derive(Debug)]
/// Guided control surface for ArduPlane sessions.
pub struct ArduPlaneGuidedHandle<'a> {
    pub(crate) _session: &'a super::ArduGuidedSession,
}

impl<'a> ArduPlaneGuidedHandle<'a> {
    pub(crate) fn new(session: &'a super::ArduGuidedSession) -> Self {
        Self { _session: session }
    }

    pub fn kind(&self) -> ArduPlaneKind {
        self._session
            .plane_kind()
            .expect("plane guided handles are only constructed for plane sessions")
    }

    pub fn vtol(&self) -> Option<ArduPlaneVtolGuidedHandle<'_>> {
        (self.kind() == ArduPlaneKind::Vtol)
            .then_some(ArduPlaneVtolGuidedHandle::new(self._session))
    }

    pub async fn reposition(&self, target: GeoPoint3dMsl) -> Result<(), VehicleError> {
        self._session.ensure_active()?;
        send_reposition(
            self._session,
            dialect::MavFrame::MAV_FRAME_GLOBAL,
            quantize_degrees_e7(target.latitude_deg, "latitude_deg")?,
            quantize_degrees_e7(target.longitude_deg, "longitude_deg")?,
            finite_f32(target.altitude_msl_m, "altitude_msl_m")?,
        )
        .await
    }

    pub async fn reposition_rel_home(&self, target: GeoPoint3dRelHome) -> Result<(), VehicleError> {
        self._session.ensure_active()?;
        send_reposition(
            self._session,
            dialect::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
            quantize_degrees_e7(target.latitude_deg, "latitude_deg")?,
            quantize_degrees_e7(target.longitude_deg, "longitude_deg")?,
            finite_f32(target.relative_alt_m, "relative_alt_m")?,
        )
        .await
    }
}

#[derive(Debug)]
/// VTOL-specific guided controls for ArduPlane VTOL sessions.
pub struct ArduPlaneVtolGuidedHandle<'a> {
    pub(crate) _session: &'a super::ArduGuidedSession,
}

impl<'a> ArduPlaneVtolGuidedHandle<'a> {
    pub(crate) fn new(session: &'a super::ArduGuidedSession) -> Self {
        Self { _session: session }
    }

    pub async fn takeoff(&self, target: RelativeClimbTarget) -> Result<(), VehicleError> {
        self._session.ensure_active()?;
        validate_positive_f32(target.relative_climb_m, "relative_climb_m")?;

        send_domain_command(self._session.command_tx(), |reply| Command::RawCommandInt {
            payload: RawCommandIntPayload {
                command: dialect::MavCmd::MAV_CMD_NAV_TAKEOFF,
                frame: dialect::MavFrame::MAV_FRAME_LOCAL_OFFSET_NED,
                current: 0,
                autocontinue: 0,
                params: [0.0, 0.0, 0.0, 0.0],
                x: 0,
                y: 0,
                z: -target.relative_climb_m,
            },
            reply,
        })
        .await
        .map(|_| ())
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

async fn send_reposition(
    session: &super::ArduGuidedSession,
    frame: dialect::MavFrame,
    lat_e7: i32,
    lon_e7: i32,
    alt_m: f32,
) -> Result<(), VehicleError> {
    send_domain_command(session.command_tx(), |reply| Command::RawCommandInt {
        payload: RawCommandIntPayload {
            command: dialect::MavCmd::MAV_CMD_DO_REPOSITION,
            frame,
            current: 0,
            autocontinue: 0,
            params: [0.0, MAV_DO_REPOSITION_FLAGS_CHANGE_MODE, 0.0, 0.0],
            x: lat_e7,
            y: lon_e7,
            z: alt_m,
        },
        reply,
    })
    .await
    .map(|_| ())
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
