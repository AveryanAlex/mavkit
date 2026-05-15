use super::{
    ArduCopterHandle, ArduGuidedKind, ArduGuidedSession, ArduPlaneHandle, ArduPlaneKind,
    ArduRoverHandle, ArduSubHandle, MagCalProgress, MagCalReport, calibration, guided,
    vehicle_family,
};
use crate::command::Command;
use crate::dialect;
use crate::error::VehicleError;
use crate::observation::ObservationHandle;
use crate::operation::send_cancellable_domain_command;
use crate::state::VehicleType;
use crate::vehicle::VehicleInner;
use calibration::{
    MAV_CMD_DO_ACCEPT_MAG_CAL_ID, MAV_CMD_DO_CANCEL_MAG_CAL_ID, MAV_CMD_DO_START_MAG_CAL_ID,
};

pub(super) const SET_SERVO_INSTANCE_RANGE: std::ops::RangeInclusive<u8> = 1..=16;
pub(super) const SET_SERVO_PWM_RANGE_US: std::ops::RangeInclusive<u16> = 1000..=2000;

/// ArduPilot-specific capability accessor.
pub struct ArduPilotHandle<'a> {
    inner: &'a VehicleInner,
}

impl<'a> ArduPilotHandle<'a> {
    pub(crate) fn new(inner: &'a VehicleInner) -> Self {
        Self { inner }
    }

    pub fn copter(&self) -> Option<ArduCopterHandle<'a>> {
        matches!(
            vehicle_family::classify(self.vehicle_type()),
            Some(vehicle_family::VehicleFamily::Copter)
        )
        .then_some(ArduCopterHandle::new(self.inner))
    }

    pub fn plane(&self) -> Option<ArduPlaneHandle<'a>> {
        matches!(
            vehicle_family::classify(self.vehicle_type()),
            Some(vehicle_family::VehicleFamily::Plane(_))
        )
        .then_some(ArduPlaneHandle::new(self.inner))
    }

    pub fn rover(&self) -> Option<ArduRoverHandle<'a>> {
        matches!(
            vehicle_family::classify(self.vehicle_type()),
            Some(vehicle_family::VehicleFamily::Rover)
        )
        .then_some(ArduRoverHandle::new(self.inner))
    }

    pub fn sub(&self) -> Option<ArduSubHandle<'a>> {
        matches!(
            vehicle_family::classify(self.vehicle_type()),
            Some(vehicle_family::VehicleFamily::Sub)
        )
        .then_some(ArduSubHandle::new(self.inner))
    }

    pub async fn guided(&self) -> Result<ArduGuidedSession, VehicleError> {
        let kind = self.guided_kind().ok_or_else(|| {
            VehicleError::Unsupported("guided session is unavailable for this vehicle type".into())
        })?;
        let guided_custom_mode = self
            .inner
            .resolve_mode_by_name("GUIDED")
            .ok_or_else(|| VehicleError::ModeNotAvailable("GUIDED".to_string()))?;
        let lease_id = self.inner.ardupilot.begin_guided_lease()?;

        let current_custom_mode = self
            .inner
            .modes
            .current()
            .latest()
            .map(|mode| mode.custom_mode);
        if current_custom_mode != Some(guided_custom_mode)
            && let Err(err) = self.inner.set_mode(guided_custom_mode, true).await
        {
            self.inner.ardupilot.release_guided_lease(lease_id);
            return Err(err);
        }

        Ok(ArduGuidedSession::new(
            self.inner,
            guided::GuidedSessionInit {
                lease_scope: self.inner.ardupilot.guided_lease_scope(),
                lease_id,
                kind,
                plane_kind: self.guided_plane_kind(),
                guided_mode_custom_mode: guided_custom_mode,
                command_tx: self.inner.command_tx.clone(),
                target_system: self.inner.stores.vehicle_state.borrow().system_id,
                target_component: self.inner.stores.vehicle_state.borrow().component_id,
            },
        ))
    }

    pub async fn request_prearm_checks(&self) -> Result<(), VehicleError> {
        self.send_long(dialect::MavCmd::MAV_CMD_RUN_PREARM_CHECKS, [0.0; 7])
            .await
    }

    pub async fn motor_test(
        &self,
        instance: u8,
        throttle_pct: f32,
        duration_s: u16,
    ) -> Result<(), VehicleError> {
        if !(1..=12).contains(&instance) {
            return Err(VehicleError::InvalidParameter(format!(
                "motor_test instance must be in 1..=12, got {instance}"
            )));
        }
        if !(0.0..=100.0).contains(&throttle_pct) {
            return Err(VehicleError::InvalidParameter(format!(
                "motor_test throttle_pct must be in 0..=100, got {throttle_pct}"
            )));
        }
        if !(1..=30).contains(&duration_s) {
            return Err(VehicleError::InvalidParameter(format!(
                "motor_test duration_s must be in 1..=30, got {duration_s}"
            )));
        }

        self.send_long(
            dialect::MavCmd::MAV_CMD_DO_MOTOR_TEST,
            [
                f32::from(instance),
                0.0,
                throttle_pct,
                f32::from(duration_s),
                1.0,
                0.0,
                0.0,
            ],
        )
        .await
    }

    /// Directly sets an ArduPilot servo output on a permissive channel.
    ///
    /// ArduPilot only applies `MAV_CMD_DO_SET_SERVO` on outputs and in modes that allow
    /// direct servo control, so callers should treat acceptance as transport-level success,
    /// not a guarantee that the output actually moved.
    pub async fn set_servo(&self, instance: u8, pwm_us: u16) -> Result<(), VehicleError> {
        if !SET_SERVO_INSTANCE_RANGE.contains(&instance) {
            return Err(VehicleError::InvalidParameter(format!(
                "set_servo instance must be in {}..={}, got {instance}",
                SET_SERVO_INSTANCE_RANGE.start(),
                SET_SERVO_INSTANCE_RANGE.end(),
            )));
        }
        if !SET_SERVO_PWM_RANGE_US.contains(&pwm_us) {
            return Err(VehicleError::InvalidParameter(format!(
                "set_servo pwm_us must be in {}..={} microseconds, got {pwm_us}",
                SET_SERVO_PWM_RANGE_US.start(),
                SET_SERVO_PWM_RANGE_US.end(),
            )));
        }

        self.send_long(
            dialect::MavCmd::MAV_CMD_DO_SET_SERVO,
            [
                f32::from(instance),
                f32::from(pwm_us),
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ],
        )
        .await
    }

    pub async fn preflight_calibration(
        &self,
        gyro: bool,
        accel: bool,
        baro: bool,
        accel_trim: bool,
    ) -> Result<(), VehicleError> {
        let params = calibration::preflight_calibration_params(gyro, accel, baro, accel_trim)?;

        self.send_long(dialect::MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION, params)
            .await
    }

    pub async fn reboot(&self) -> Result<(), VehicleError> {
        self.send_long(
            dialect::MavCmd::MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        .await
    }

    pub async fn reboot_to_bootloader(&self) -> Result<(), VehicleError> {
        self.send_long(
            dialect::MavCmd::MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            [3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        .await
    }

    pub fn mag_cal_progress(&self) -> ObservationHandle<Vec<MagCalProgress>> {
        self.inner.ardupilot.mag_cal_progress()
    }

    pub fn mag_cal_report(&self) -> ObservationHandle<Vec<MagCalReport>> {
        self.inner.ardupilot.mag_cal_report()
    }

    pub async fn start_mag_cal(&self, compass_mask: u8) -> Result<(), VehicleError> {
        self.send_long_raw(
            MAV_CMD_DO_START_MAG_CAL_ID,
            [f32::from(compass_mask), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        .await
    }

    pub async fn accept_mag_cal(&self) -> Result<(), VehicleError> {
        self.send_long_raw(
            MAV_CMD_DO_ACCEPT_MAG_CAL_ID,
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        .await
    }

    pub async fn cancel_mag_cal(&self) -> Result<(), VehicleError> {
        self.send_long_raw(
            MAV_CMD_DO_CANCEL_MAG_CAL_ID,
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        .await
    }

    async fn send_long(
        &self,
        command: dialect::MavCmd,
        params: [f32; 7],
    ) -> Result<(), VehicleError> {
        send_cancellable_domain_command(self.inner.command_tx.clone(), |reply, cancel| {
            Command::Long {
                command,
                params,
                reply,
                cancel,
            }
        })
        .await
    }

    async fn send_long_raw(&self, command_id: u16, params: [f32; 7]) -> Result<(), VehicleError> {
        send_cancellable_domain_command(self.inner.command_tx.clone(), |reply, cancel| {
            Command::LongRaw {
                command_id,
                params,
                reply,
                cancel,
            }
        })
        .await
    }

    fn vehicle_type(&self) -> VehicleType {
        self.inner.stores.vehicle_state.borrow().vehicle_type
    }

    fn guided_kind(&self) -> Option<ArduGuidedKind> {
        vehicle_family::guided_kind(self.vehicle_type())
    }

    fn guided_plane_kind(&self) -> Option<ArduPlaneKind> {
        vehicle_family::plane_kind(self.vehicle_type())
    }
}
