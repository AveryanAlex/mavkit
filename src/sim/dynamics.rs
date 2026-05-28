use crate::dialect;
use crate::mission::HomePosition;

use super::state::{
    ACCEPTANCE_RADIUS_M, AttitudeRates, DemoVehicleFamily, NavSource, NavTarget, SimulatorCore,
    TAKEOFF_ALT_MARGIN_M, VelocityNed, auto_mode, guided_mode, is_hover_hold_mode,
    is_quadplane_vtol_mode, profile_vehicle_family,
};

const STANDARD_GRAVITY_MPS2: f32 = 9.806_65;
const COPTER_HORIZONTAL_SPEED_MPS: f64 = 10.0;
const COPTER_CLIMB_SPEED_MPS: f64 = 2.0;
const COPTER_DESCENT_SPEED_MPS: f64 = 1.5;
const COPTER_POSITION_GAIN: f64 = 0.9;
const COPTER_VELOCITY_TIME_CONSTANT_S: f64 = 0.7;
const COPTER_MAX_HORIZONTAL_ACCEL_MPS2: f64 = 3.5;
const COPTER_MAX_HORIZONTAL_THRUST_ACCEL_MPS2: f64 = 4.5;
const COPTER_MAX_VERTICAL_ACCEL_MPS2: f64 = 2.5;
const COPTER_MAX_TILT_RAD: f32 = 25.0_f32.to_radians();
const COPTER_ATTITUDE_RATE_RAD_PER_SEC: f32 = 100.0_f32.to_radians();
const COPTER_MAX_YAW_RATE_RAD_PER_SEC: f32 = 70.0_f32.to_radians();
const COPTER_YAW_SPEED_THRESHOLD_MPS: f64 = 0.8;
const COPTER_DRAG_ACCEL_PER_SPEED_SQUARED: f64 = 0.035_694_773;
const PLANE_CRUISE_SPEED_MPS: f64 = 12.0;
const QUADPLANE_CRUISE_SPEED_MPS: f64 = 10.0;
const PLANE_CLIMB_SPEED_MPS: f64 = 2.5;
const PLANE_DESCENT_SPEED_MPS: f64 = 1.8;
const PLANE_TURN_RATE_RAD_PER_SEC: f32 = 25.0_f32.to_radians();
const QUADPLANE_TURN_RATE_RAD_PER_SEC: f32 = 35.0_f32.to_radians();
const IDLE_LOITER_TURN_RATE_RAD_PER_SEC: f32 = 18.0_f32.to_radians();
const PLANE_HEADING_GAIN: f32 = 1.4;
const PLANE_MAX_BANK_RAD: f32 = 35.0_f32.to_radians();
const PLANE_ROLL_RATE_RAD_PER_SEC: f32 = 70.0_f32.to_radians();
const PLANE_ALTITUDE_GAIN: f64 = 0.8;
const PLANE_MAX_PITCH_UP_RAD: f32 = 15.0_f32.to_radians();
const PLANE_MAX_PITCH_DOWN_RAD: f32 = 12.0_f32.to_radians();
const PLANE_PITCH_RATE_RAD_PER_SEC: f32 = 30.0_f32.to_radians();

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ActiveDynamics {
    Copter,
    FixedWing,
}

#[derive(Debug, Clone, Copy)]
struct AttitudeSample {
    roll_rad: f32,
    pitch_rad: f32,
    yaw_rad: f32,
}

impl SimulatorCore {
    pub(crate) fn advance_navigation(&mut self) {
        let dt_s = self.tick_dt_s();
        let previous_attitude = self.attitude_sample();

        if !self.snapshot.armed {
            self.park();
            self.update_attitude_rates(previous_attitude, dt_s);
            return;
        }

        let Some(target) = self.active_nav_target() else {
            self.advance_idle_behavior(dt_s);
            self.update_attitude_rates(previous_attitude, dt_s);
            return;
        };

        match self.active_dynamics() {
            ActiveDynamics::Copter => self.advance_copter_navigation(target, dt_s),
            ActiveDynamics::FixedWing => self.advance_plane_navigation(target, dt_s),
        }

        self.update_attitude_rates(previous_attitude, dt_s);
    }

    pub(crate) fn reset_motion_state(&mut self) {
        self.velocity = VelocityNed {
            north_mps: 0.0,
            east_mps: 0.0,
            down_mps: 0.0,
        };
        self.snapshot.roll_rad = 0.0;
        self.snapshot.pitch_rad = 0.0;
        self.attitude_rates = AttitudeRates::default();
        self.snapshot.relative_alt_m = self.snapshot.altitude_msl_m - self.snapshot.home.altitude_m;
    }

    fn active_dynamics(&self) -> ActiveDynamics {
        match profile_vehicle_family(self.config.profile) {
            DemoVehicleFamily::Copter => ActiveDynamics::Copter,
            DemoVehicleFamily::Plane => ActiveDynamics::FixedWing,
            DemoVehicleFamily::QuadPlane => {
                if is_quadplane_vtol_mode(self.config.profile, self.snapshot.custom_mode) {
                    ActiveDynamics::Copter
                } else {
                    ActiveDynamics::FixedWing
                }
            }
        }
    }

    fn park(&mut self) {
        self.reset_motion_state();
    }

    fn advance_idle_behavior(&mut self, dt_s: f64) {
        if self.active_dynamics() == ActiveDynamics::Copter
            || is_hover_hold_mode(self.config.profile, self.snapshot.custom_mode)
        {
            self.park();
            return;
        }

        let family = profile_vehicle_family(self.config.profile);
        let turn_rate = match family {
            DemoVehicleFamily::Plane => IDLE_LOITER_TURN_RATE_RAD_PER_SEC,
            DemoVehicleFamily::QuadPlane => IDLE_LOITER_TURN_RATE_RAD_PER_SEC * 1.1,
            DemoVehicleFamily::Copter => 0.0,
        };
        let speed_mps = self.cruise_speed_mps(None);
        let bank_cmd = bank_for_yaw_rate(speed_mps, turn_rate);
        self.snapshot.roll_rad = slew_value(
            self.snapshot.roll_rad,
            bank_cmd,
            PLANE_ROLL_RATE_RAD_PER_SEC,
            dt_s,
        );
        self.snapshot.pitch_rad = slew_value(
            self.snapshot.pitch_rad,
            0.0,
            PLANE_PITCH_RATE_RAD_PER_SEC,
            dt_s,
        );
        let yaw_rate = coordinated_turn_rate(speed_mps, self.snapshot.roll_rad);
        self.snapshot.yaw_rad = wrap_angle(self.snapshot.yaw_rad + yaw_rate * dt_s as f32);
        let step_north_m = f64::from(self.snapshot.yaw_rad.cos()) * speed_mps * dt_s;
        let step_east_m = f64::from(self.snapshot.yaw_rad.sin()) * speed_mps * dt_s;
        apply_horizontal_step(
            &mut self.snapshot.latitude_deg,
            &mut self.snapshot.longitude_deg,
            step_north_m,
            step_east_m,
        );
        self.velocity = VelocityNed {
            north_mps: step_north_m / dt_s,
            east_mps: step_east_m / dt_s,
            down_mps: 0.0,
        };
        self.snapshot.relative_alt_m = self.snapshot.altitude_msl_m - self.snapshot.home.altitude_m;
    }

    fn advance_copter_navigation(&mut self, target: NavTarget, dt_s: f64) {
        let (north_m, east_m) = lat_lon_delta_m(
            self.snapshot.latitude_deg,
            self.snapshot.longitude_deg,
            target.latitude_deg,
            target.longitude_deg,
        );
        let up_m = target.altitude_msl_m - self.snapshot.altitude_msl_m;
        let horizontal_distance_m = (north_m * north_m + east_m * east_m).sqrt();
        let distance_m = (horizontal_distance_m * horizontal_distance_m + up_m * up_m).sqrt();

        if distance_m <= target.acceptance_radius_m {
            self.finish_nav_target(target);
            return;
        }

        let desired_horizontal_speed_mps =
            (horizontal_distance_m * COPTER_POSITION_GAIN).min(COPTER_HORIZONTAL_SPEED_MPS);
        let (desired_north_mps, desired_east_mps) = if horizontal_distance_m > f64::EPSILON {
            let scale = desired_horizontal_speed_mps / horizontal_distance_m;
            (north_m * scale, east_m * scale)
        } else {
            (0.0, 0.0)
        };

        let feedback_accel_north_mps2 = ((desired_north_mps - self.velocity.north_mps)
            / COPTER_VELOCITY_TIME_CONSTANT_S)
            .clamp(
                -COPTER_MAX_HORIZONTAL_ACCEL_MPS2,
                COPTER_MAX_HORIZONTAL_ACCEL_MPS2,
            );
        let feedback_accel_east_mps2 =
            ((desired_east_mps - self.velocity.east_mps) / COPTER_VELOCITY_TIME_CONSTANT_S).clamp(
                -COPTER_MAX_HORIZONTAL_ACCEL_MPS2,
                COPTER_MAX_HORIZONTAL_ACCEL_MPS2,
            );

        let (drag_accel_north_mps2, drag_accel_east_mps2) =
            copter_horizontal_drag_accel(self.velocity.north_mps, self.velocity.east_mps);
        let mut thrust_accel_north_mps2 = feedback_accel_north_mps2 - drag_accel_north_mps2;
        let mut thrust_accel_east_mps2 = feedback_accel_east_mps2 - drag_accel_east_mps2;
        limit_horizontal_accel(
            &mut thrust_accel_north_mps2,
            &mut thrust_accel_east_mps2,
            COPTER_MAX_HORIZONTAL_THRUST_ACCEL_MPS2,
        );

        let accel_north_mps2 = thrust_accel_north_mps2 + drag_accel_north_mps2;
        let accel_east_mps2 = thrust_accel_east_mps2 + drag_accel_east_mps2;

        let desired_up_mps = if up_m >= 0.0 {
            (up_m * COPTER_POSITION_GAIN).min(COPTER_CLIMB_SPEED_MPS)
        } else {
            (up_m * COPTER_POSITION_GAIN).max(-COPTER_DESCENT_SPEED_MPS)
        };
        let current_up_mps = -self.velocity.down_mps;
        let accel_up_mps2 = ((desired_up_mps - current_up_mps) / COPTER_VELOCITY_TIME_CONSTANT_S)
            .clamp(
                -COPTER_MAX_VERTICAL_ACCEL_MPS2,
                COPTER_MAX_VERTICAL_ACCEL_MPS2,
            );

        let mut next_north_mps = self.velocity.north_mps + accel_north_mps2 * dt_s;
        let mut next_east_mps = self.velocity.east_mps + accel_east_mps2 * dt_s;
        let mut next_up_mps = current_up_mps + accel_up_mps2 * dt_s;

        limit_horizontal_velocity(
            &mut next_north_mps,
            &mut next_east_mps,
            COPTER_HORIZONTAL_SPEED_MPS,
        );
        next_up_mps = next_up_mps.clamp(-COPTER_DESCENT_SPEED_MPS, COPTER_CLIMB_SPEED_MPS);

        let mut step_north_m = next_north_mps * dt_s;
        let mut step_east_m = next_east_mps * dt_s;
        limit_horizontal_step(
            &mut step_north_m,
            &mut step_east_m,
            north_m,
            east_m,
            horizontal_distance_m,
        );
        let step_up_m = clamp_step_to_remaining(next_up_mps * dt_s, up_m);

        next_north_mps = step_north_m / dt_s;
        next_east_mps = step_east_m / dt_s;
        next_up_mps = step_up_m / dt_s;

        let actual_accel_north_mps2 = (next_north_mps - self.velocity.north_mps) / dt_s;
        let actual_accel_east_mps2 = (next_east_mps - self.velocity.east_mps) / dt_s;
        let actual_thrust_accel_north_mps2 = actual_accel_north_mps2 - drag_accel_north_mps2;
        let actual_thrust_accel_east_mps2 = actual_accel_east_mps2 - drag_accel_east_mps2;
        self.update_copter_attitude(
            actual_thrust_accel_north_mps2,
            actual_thrust_accel_east_mps2,
            desired_north_mps,
            desired_east_mps,
            east_m.atan2(north_m) as f32,
            dt_s,
        );

        apply_horizontal_step(
            &mut self.snapshot.latitude_deg,
            &mut self.snapshot.longitude_deg,
            step_north_m,
            step_east_m,
        );
        self.snapshot.altitude_msl_m += step_up_m;
        self.snapshot.relative_alt_m = self.snapshot.altitude_msl_m - self.snapshot.home.altitude_m;
        self.velocity = VelocityNed {
            north_mps: next_north_mps,
            east_mps: next_east_mps,
            down_mps: -next_up_mps,
        };

        let (remaining_north_m, remaining_east_m) = lat_lon_delta_m(
            self.snapshot.latitude_deg,
            self.snapshot.longitude_deg,
            target.latitude_deg,
            target.longitude_deg,
        );
        let remaining_horizontal_m =
            (remaining_north_m * remaining_north_m + remaining_east_m * remaining_east_m).sqrt();
        let remaining_up_m = target.altitude_msl_m - self.snapshot.altitude_msl_m;
        let remaining_distance_m = (remaining_horizontal_m * remaining_horizontal_m
            + remaining_up_m * remaining_up_m)
            .sqrt();
        if remaining_distance_m <= target.acceptance_radius_m {
            self.finish_nav_target(target);
        }
    }

    fn advance_plane_navigation(&mut self, target: NavTarget, dt_s: f64) {
        let (north_m, east_m) = lat_lon_delta_m(
            self.snapshot.latitude_deg,
            self.snapshot.longitude_deg,
            target.latitude_deg,
            target.longitude_deg,
        );
        let up_m = target.altitude_msl_m - self.snapshot.altitude_msl_m;
        let horizontal_distance_m = (north_m * north_m + east_m * east_m).sqrt();
        let altitude_acceptance_m = target.acceptance_radius_m.max(TAKEOFF_ALT_MARGIN_M);

        if horizontal_distance_m <= target.acceptance_radius_m
            && up_m.abs() <= altitude_acceptance_m
        {
            self.finish_nav_target(target);
            return;
        }

        if horizontal_distance_m <= target.acceptance_radius_m {
            let step_up_m = self.clamped_vertical_step(up_m, dt_s);
            let speed_mps = self.cruise_speed_mps(Some(target));
            let pitch_cmd = pitch_for_climb_rate(step_up_m / dt_s, speed_mps);
            self.snapshot.roll_rad = slew_value(
                self.snapshot.roll_rad,
                0.0,
                PLANE_ROLL_RATE_RAD_PER_SEC,
                dt_s,
            );
            self.snapshot.pitch_rad = slew_value(
                self.snapshot.pitch_rad,
                pitch_cmd,
                PLANE_PITCH_RATE_RAD_PER_SEC,
                dt_s,
            );
            self.snapshot.altitude_msl_m += step_up_m;
            self.snapshot.relative_alt_m =
                self.snapshot.altitude_msl_m - self.snapshot.home.altitude_m;
            self.velocity = VelocityNed {
                north_mps: 0.0,
                east_mps: 0.0,
                down_mps: -step_up_m / dt_s,
            };
            if (up_m - step_up_m).abs() <= altitude_acceptance_m {
                self.finish_nav_target(target);
            }
            return;
        }

        let speed_mps = self.cruise_speed_mps(Some(target));
        let desired_yaw = east_m.atan2(north_m) as f32;
        let yaw_error = wrap_angle(desired_yaw - self.snapshot.yaw_rad);
        let desired_yaw_rate = (yaw_error * PLANE_HEADING_GAIN)
            .clamp(-self.turn_rate_rad_per_sec(), self.turn_rate_rad_per_sec());
        let bank_cmd = bank_for_yaw_rate(speed_mps, desired_yaw_rate);
        self.snapshot.roll_rad = slew_value(
            self.snapshot.roll_rad,
            bank_cmd,
            PLANE_ROLL_RATE_RAD_PER_SEC,
            dt_s,
        );
        let yaw_rate = coordinated_turn_rate(speed_mps, self.snapshot.roll_rad)
            .clamp(-self.turn_rate_rad_per_sec(), self.turn_rate_rad_per_sec());
        self.snapshot.yaw_rad = wrap_angle(self.snapshot.yaw_rad + yaw_rate * dt_s as f32);

        let climb_rate_cmd = self.plane_climb_rate_cmd(up_m);
        let pitch_cmd = pitch_for_climb_rate(climb_rate_cmd, speed_mps);
        self.snapshot.pitch_rad = slew_value(
            self.snapshot.pitch_rad,
            pitch_cmd,
            PLANE_PITCH_RATE_RAD_PER_SEC,
            dt_s,
        );

        let horizontal_speed_mps = (speed_mps * f64::from(self.snapshot.pitch_rad.cos()))
            .abs()
            .max(1.0);
        let horizontal_step_limit_m = horizontal_speed_mps * dt_s;
        let (step_north_m, step_east_m) = if horizontal_distance_m <= horizontal_step_limit_m {
            (north_m, east_m)
        } else {
            (
                f64::from(self.snapshot.yaw_rad.cos()) * horizontal_step_limit_m,
                f64::from(self.snapshot.yaw_rad.sin()) * horizontal_step_limit_m,
            )
        };
        let step_up_m = clamp_step_to_remaining(
            speed_mps * f64::from(self.snapshot.pitch_rad.sin()) * dt_s,
            up_m,
        );

        apply_horizontal_step(
            &mut self.snapshot.latitude_deg,
            &mut self.snapshot.longitude_deg,
            step_north_m,
            step_east_m,
        );
        self.snapshot.altitude_msl_m += step_up_m;
        self.snapshot.relative_alt_m = self.snapshot.altitude_msl_m - self.snapshot.home.altitude_m;
        self.velocity = VelocityNed {
            north_mps: step_north_m / dt_s,
            east_mps: step_east_m / dt_s,
            down_mps: -step_up_m / dt_s,
        };

        let (remaining_north_m, remaining_east_m) = lat_lon_delta_m(
            self.snapshot.latitude_deg,
            self.snapshot.longitude_deg,
            target.latitude_deg,
            target.longitude_deg,
        );
        let remaining_horizontal_m =
            (remaining_north_m * remaining_north_m + remaining_east_m * remaining_east_m).sqrt();
        if remaining_horizontal_m <= target.acceptance_radius_m
            && (up_m - step_up_m).abs() <= altitude_acceptance_m
        {
            self.finish_nav_target(target);
        }
    }

    fn finish_nav_target(&mut self, target: NavTarget) {
        self.snapshot.latitude_deg = target.latitude_deg;
        self.snapshot.longitude_deg = target.longitude_deg;
        self.snapshot.altitude_msl_m = if target.disarm_on_reach {
            self.snapshot.home.altitude_m
        } else {
            target.altitude_msl_m
        };
        if target.disarm_on_reach {
            self.snapshot.armed = false;
        }
        self.park();
        if target.source == NavSource::Mission {
            self.pending_reached_wire_seq = target.wire_seq;
            self.nav_target = None;
        }
    }

    fn clamped_vertical_step(&self, up_m: f64, dt_s: f64) -> f64 {
        let limit_m = if up_m >= 0.0 {
            PLANE_CLIMB_SPEED_MPS * dt_s
        } else {
            PLANE_DESCENT_SPEED_MPS * dt_s
        };
        up_m.clamp(-limit_m, limit_m)
    }

    fn cruise_speed_mps(&self, target: Option<NavTarget>) -> f64 {
        if target.is_some_and(|nav| nav.source == NavSource::Mission) {
            if let Some(speed_mps) = self.mission_speed_override_mps {
                return speed_mps.max(1.0);
            }
        }

        match profile_vehicle_family(self.config.profile) {
            DemoVehicleFamily::Copter => COPTER_HORIZONTAL_SPEED_MPS,
            DemoVehicleFamily::Plane => PLANE_CRUISE_SPEED_MPS,
            DemoVehicleFamily::QuadPlane => QUADPLANE_CRUISE_SPEED_MPS,
        }
    }

    fn turn_rate_rad_per_sec(&self) -> f32 {
        match profile_vehicle_family(self.config.profile) {
            DemoVehicleFamily::Copter => 0.0,
            DemoVehicleFamily::Plane => PLANE_TURN_RATE_RAD_PER_SEC,
            DemoVehicleFamily::QuadPlane => QUADPLANE_TURN_RATE_RAD_PER_SEC,
        }
    }

    fn tick_dt_s(&self) -> f64 {
        1.0 / f64::from(self.config.tick_hz)
    }

    fn plane_climb_rate_cmd(&self, up_m: f64) -> f64 {
        if up_m >= 0.0 {
            (up_m * PLANE_ALTITUDE_GAIN).min(PLANE_CLIMB_SPEED_MPS)
        } else {
            (up_m * PLANE_ALTITUDE_GAIN).max(-PLANE_DESCENT_SPEED_MPS)
        }
    }

    fn update_copter_attitude(
        &mut self,
        thrust_accel_north_mps2: f64,
        thrust_accel_east_mps2: f64,
        desired_north_mps: f64,
        desired_east_mps: f64,
        target_bearing_rad: f32,
        dt_s: f64,
    ) {
        let cy = f64::from(self.snapshot.yaw_rad.cos());
        let sy = f64::from(self.snapshot.yaw_rad.sin());
        let accel_forward_mps2 = cy * thrust_accel_north_mps2 + sy * thrust_accel_east_mps2;
        let accel_right_mps2 = -sy * thrust_accel_north_mps2 + cy * thrust_accel_east_mps2;

        let gravity = f64::from(STANDARD_GRAVITY_MPS2);
        let pitch_cmd = (-(accel_forward_mps2 / gravity).atan() as f32)
            .clamp(-COPTER_MAX_TILT_RAD, COPTER_MAX_TILT_RAD);
        let roll_cmd = ((accel_right_mps2 / gravity).atan() as f32)
            .clamp(-COPTER_MAX_TILT_RAD, COPTER_MAX_TILT_RAD);

        self.snapshot.roll_rad = slew_value(
            self.snapshot.roll_rad,
            roll_cmd,
            COPTER_ATTITUDE_RATE_RAD_PER_SEC,
            dt_s,
        );
        self.snapshot.pitch_rad = slew_value(
            self.snapshot.pitch_rad,
            pitch_cmd,
            COPTER_ATTITUDE_RATE_RAD_PER_SEC,
            dt_s,
        );

        let desired_speed_mps =
            (desired_north_mps * desired_north_mps + desired_east_mps * desired_east_mps).sqrt();
        let desired_yaw_rad = if desired_speed_mps > COPTER_YAW_SPEED_THRESHOLD_MPS {
            desired_east_mps.atan2(desired_north_mps) as f32
        } else {
            target_bearing_rad
        };
        self.snapshot.yaw_rad = slew_angle(
            self.snapshot.yaw_rad,
            desired_yaw_rad,
            COPTER_MAX_YAW_RATE_RAD_PER_SEC,
            dt_s,
        );
    }

    fn attitude_sample(&self) -> AttitudeSample {
        AttitudeSample {
            roll_rad: self.snapshot.roll_rad,
            pitch_rad: self.snapshot.pitch_rad,
            yaw_rad: self.snapshot.yaw_rad,
        }
    }

    fn update_attitude_rates(&mut self, previous: AttitudeSample, dt_s: f64) {
        let inv_dt = (1.0 / dt_s) as f32;
        self.attitude_rates = AttitudeRates {
            roll_rad_per_sec: (self.snapshot.roll_rad - previous.roll_rad) * inv_dt,
            pitch_rad_per_sec: (self.snapshot.pitch_rad - previous.pitch_rad) * inv_dt,
            yaw_rad_per_sec: wrap_angle(self.snapshot.yaw_rad - previous.yaw_rad) * inv_dt,
        };
    }

    pub(crate) fn active_nav_target(&self) -> Option<NavTarget> {
        match self.nav_target {
            Some(target)
                if target.source == NavSource::Guided
                    && self.snapshot.custom_mode == guided_mode(self.config.profile) =>
            {
                Some(target)
            }
            Some(target)
                if target.source == NavSource::Mission
                    && self.snapshot.custom_mode == auto_mode(self.config.profile) =>
            {
                Some(target)
            }
            _ => None,
        }
    }

    pub(crate) fn nav_target_for_mission_item(
        &self,
        item: &dialect::MISSION_ITEM_INT_DATA,
    ) -> Option<NavTarget> {
        match item.command {
            dialect::MavCmd::MAV_CMD_NAV_WAYPOINT => Some(NavTarget {
                source: NavSource::Mission,
                latitude_deg: f64::from(item.x) / 1e7,
                longitude_deg: f64::from(item.y) / 1e7,
                altitude_msl_m: mission_item_altitude_msl(item, &self.snapshot.home),
                acceptance_radius_m: waypoint_acceptance_radius(item.param2),
                wire_seq: Some(item.seq),
                disarm_on_reach: false,
            }),
            dialect::MavCmd::MAV_CMD_NAV_TAKEOFF => Some(NavTarget {
                source: NavSource::Mission,
                latitude_deg: if item.x == 0 {
                    self.snapshot.latitude_deg
                } else {
                    f64::from(item.x) / 1e7
                },
                longitude_deg: if item.y == 0 {
                    self.snapshot.longitude_deg
                } else {
                    f64::from(item.y) / 1e7
                },
                altitude_msl_m: mission_item_altitude_msl(item, &self.snapshot.home)
                    .max(self.snapshot.home.altitude_m + TAKEOFF_ALT_MARGIN_M),
                acceptance_radius_m: TAKEOFF_ALT_MARGIN_M
                    .max(waypoint_acceptance_radius(item.param2)),
                wire_seq: Some(item.seq),
                disarm_on_reach: false,
            }),
            dialect::MavCmd::MAV_CMD_NAV_LAND => Some(NavTarget {
                source: NavSource::Mission,
                latitude_deg: if item.x == 0 {
                    self.snapshot.latitude_deg
                } else {
                    f64::from(item.x) / 1e7
                },
                longitude_deg: if item.y == 0 {
                    self.snapshot.longitude_deg
                } else {
                    f64::from(item.y) / 1e7
                },
                altitude_msl_m: mission_item_altitude_msl(item, &self.snapshot.home)
                    .min(self.snapshot.home.altitude_m + 0.5),
                acceptance_radius_m: ACCEPTANCE_RADIUS_M,
                wire_seq: Some(item.seq),
                disarm_on_reach: true,
            }),
            dialect::MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH => Some(NavTarget {
                source: NavSource::Mission,
                latitude_deg: self.snapshot.home.latitude_deg,
                longitude_deg: self.snapshot.home.longitude_deg,
                altitude_msl_m: self.snapshot.home.altitude_m + TAKEOFF_ALT_MARGIN_M,
                acceptance_radius_m: ACCEPTANCE_RADIUS_M,
                wire_seq: Some(item.seq),
                disarm_on_reach: false,
            }),
            _ => None,
        }
    }
}

pub(crate) fn degrees_to_e7(value: f64) -> i32 {
    (value * 1e7).round() as i32
}

pub(crate) fn meters_to_millimeters(value: f64) -> i32 {
    (value * 1000.0).round() as i32
}

pub(crate) fn meters_per_second_to_cms(value: f64) -> i16 {
    (value * 100.0).round() as i16
}

pub(crate) fn meters_per_second_to_cms_u16(value: f64) -> u16 {
    (value.max(0.0) * 100.0).round() as u16
}

pub(crate) fn horizontal_speed_mps(velocity: VelocityNed) -> f64 {
    (velocity.north_mps * velocity.north_mps + velocity.east_mps * velocity.east_mps).sqrt()
}

pub(crate) fn normalized_heading_deg(value: f32) -> i16 {
    radians_to_degrees(value).rem_euclid(360.0).round() as i16
}

pub(crate) fn lat_lon_delta_m(
    from_lat_deg: f64,
    from_lon_deg: f64,
    to_lat_deg: f64,
    to_lon_deg: f64,
) -> (f64, f64) {
    let mean_lat_rad = ((from_lat_deg + to_lat_deg) * 0.5).to_radians();
    let north_m = (to_lat_deg - from_lat_deg) * 111_320.0;
    let east_m = (to_lon_deg - from_lon_deg) * 111_320.0 * mean_lat_rad.cos();
    (north_m, east_m)
}

#[allow(
    deprecated,
    reason = "the demo simulator still normalizes legacy relative-alt mission frames used on current MAVLink links"
)]
pub(crate) fn mission_item_altitude_msl(
    item: &dialect::MISSION_ITEM_INT_DATA,
    home: &HomePosition,
) -> f64 {
    match item.frame {
        dialect::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT
        | dialect::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT_INT => {
            home.altitude_m + f64::from(item.z)
        }
        _ => f64::from(item.z),
    }
}

pub(crate) fn waypoint_acceptance_radius(param2: f32) -> f64 {
    if param2.is_finite() && param2 > 0.0 {
        f64::from(param2)
    } else {
        ACCEPTANCE_RADIUS_M
    }
}

pub(crate) fn radians_to_cdeg(value: f32) -> u16 {
    let degrees = radians_to_degrees(value).rem_euclid(360.0);
    (degrees * 100.0).round() as u16
}

pub(crate) fn radians_to_degrees(value: f32) -> f32 {
    value.to_degrees()
}

pub(crate) fn wrap_angle(value: f32) -> f32 {
    let tau = std::f32::consts::TAU;
    ((value + std::f32::consts::PI).rem_euclid(tau)) - std::f32::consts::PI
}

fn bank_for_yaw_rate(speed_mps: f64, yaw_rate_rad_per_sec: f32) -> f32 {
    ((speed_mps as f32 * yaw_rate_rad_per_sec) / STANDARD_GRAVITY_MPS2)
        .atan()
        .clamp(-PLANE_MAX_BANK_RAD, PLANE_MAX_BANK_RAD)
}

fn coordinated_turn_rate(speed_mps: f64, roll_rad: f32) -> f32 {
    if speed_mps <= f64::EPSILON {
        return 0.0;
    }
    STANDARD_GRAVITY_MPS2 * roll_rad.tan() / speed_mps as f32
}

fn pitch_for_climb_rate(climb_rate_mps: f64, speed_mps: f64) -> f32 {
    if speed_mps <= f64::EPSILON {
        return 0.0;
    }
    (climb_rate_mps / speed_mps).clamp(-1.0, 1.0).asin().clamp(
        -f64::from(PLANE_MAX_PITCH_DOWN_RAD),
        f64::from(PLANE_MAX_PITCH_UP_RAD),
    ) as f32
}

fn slew_value(current: f32, target: f32, max_rate_rad_per_sec: f32, dt_s: f64) -> f32 {
    let max_step = max_rate_rad_per_sec * dt_s as f32;
    current + (target - current).clamp(-max_step, max_step)
}

fn slew_angle(current: f32, target: f32, max_rate_rad_per_sec: f32, dt_s: f64) -> f32 {
    let max_step = max_rate_rad_per_sec * dt_s as f32;
    wrap_angle(current + wrap_angle(target - current).clamp(-max_step, max_step))
}

fn clamp_step_to_remaining(step_m: f64, remaining_m: f64) -> f64 {
    if remaining_m == 0.0 || step_m.signum() != remaining_m.signum() {
        return step_m;
    }

    if step_m.abs() > remaining_m.abs() {
        remaining_m
    } else {
        step_m
    }
}

fn limit_horizontal_velocity(north_mps: &mut f64, east_mps: &mut f64, max_speed_mps: f64) {
    let speed_mps = (*north_mps * *north_mps + *east_mps * *east_mps).sqrt();
    if speed_mps <= max_speed_mps || speed_mps <= f64::EPSILON {
        return;
    }

    let scale = max_speed_mps / speed_mps;
    *north_mps *= scale;
    *east_mps *= scale;
}

fn copter_horizontal_drag_accel(north_mps: f64, east_mps: f64) -> (f64, f64) {
    let speed_mps = (north_mps * north_mps + east_mps * east_mps).sqrt();
    if speed_mps <= f64::EPSILON {
        return (0.0, 0.0);
    }

    let scale = -COPTER_DRAG_ACCEL_PER_SPEED_SQUARED * speed_mps;
    (north_mps * scale, east_mps * scale)
}

fn limit_horizontal_accel(north_mps2: &mut f64, east_mps2: &mut f64, max_accel_mps2: f64) {
    let accel_mps2 = (*north_mps2 * *north_mps2 + *east_mps2 * *east_mps2).sqrt();
    if accel_mps2 <= max_accel_mps2 || accel_mps2 <= f64::EPSILON {
        return;
    }

    let scale = max_accel_mps2 / accel_mps2;
    *north_mps2 *= scale;
    *east_mps2 *= scale;
}

fn limit_horizontal_step(
    step_north_m: &mut f64,
    step_east_m: &mut f64,
    remaining_north_m: f64,
    remaining_east_m: f64,
    remaining_distance_m: f64,
) {
    let step_distance_m = (*step_north_m * *step_north_m + *step_east_m * *step_east_m).sqrt();
    if step_distance_m <= remaining_distance_m
        || step_distance_m <= f64::EPSILON
        || remaining_distance_m <= f64::EPSILON
    {
        return;
    }

    let dot = *step_north_m * remaining_north_m + *step_east_m * remaining_east_m;
    if dot < 0.0 {
        *step_north_m = 0.0;
        *step_east_m = 0.0;
    } else {
        *step_north_m = remaining_north_m;
        *step_east_m = remaining_east_m;
    }
}

fn apply_horizontal_step(
    latitude_deg: &mut f64,
    longitude_deg: &mut f64,
    step_north_m: f64,
    step_east_m: f64,
) {
    let lat_rad = latitude_deg.to_radians();
    let meters_per_deg_lat = 111_320.0;
    let meters_per_deg_lon = 111_320.0 * lat_rad.cos().abs().max(0.01);
    *latitude_deg += step_north_m / meters_per_deg_lat;
    *longitude_deg += step_east_m / meters_per_deg_lon;
}

#[cfg(test)]
mod tests {
    use tokio::sync::{mpsc, watch};

    use super::*;
    use crate::mission::HomePosition;
    use crate::sim::state::{DemoVehicleConfig, SimulatorCore, default_mode};
    use crate::sim::{DemoClock, DemoProfile, DemoVehicleSnapshot};

    fn sim_core(profile: DemoProfile) -> SimulatorCore {
        let home = HomePosition {
            latitude_deg: 42.0,
            longitude_deg: -71.0,
            altitude_m: 100.0,
        };
        let snapshot = DemoVehicleSnapshot {
            time_boot_ms: 0,
            armed: false,
            custom_mode: default_mode(profile),
            home: home.clone(),
            latitude_deg: home.latitude_deg,
            longitude_deg: home.longitude_deg,
            altitude_msl_m: home.altitude_m,
            relative_alt_m: 0.0,
            roll_rad: 0.0,
            pitch_rad: 0.0,
            yaw_rad: 0.0,
            mission_current_wire_seq: 0,
            mission_total_wire_items: 0,
        };
        let (outbound_tx, _outbound_rx) = mpsc::channel(1);
        let (snapshot_tx, _snapshot_rx) = watch::channel(snapshot.clone());
        SimulatorCore::new(
            DemoVehicleConfig {
                profile,
                clock: DemoClock::Manual,
                tick_hz: 10,
                home,
            },
            outbound_tx,
            snapshot_tx,
            snapshot,
        )
    }

    #[test]
    fn disarmed_vehicle_stays_parked() {
        let mut sim = sim_core(DemoProfile::ArduPlane);
        sim.nav_target = Some(NavTarget {
            source: NavSource::Guided,
            latitude_deg: 42.001,
            longitude_deg: -71.0,
            altitude_msl_m: 120.0,
            acceptance_radius_m: ACCEPTANCE_RADIUS_M,
            wire_seq: None,
            disarm_on_reach: false,
        });

        sim.advance_navigation();

        assert_eq!(sim.snapshot.latitude_deg, sim.snapshot.home.latitude_deg);
        assert_eq!(sim.snapshot.longitude_deg, sim.snapshot.home.longitude_deg);
        assert_eq!(sim.velocity.north_mps, 0.0);
        assert_eq!(sim.velocity.east_mps, 0.0);
    }

    #[test]
    fn copter_no_target_holds_position() {
        let mut sim = sim_core(DemoProfile::ArduCopter);
        sim.snapshot.armed = true;

        sim.advance_navigation();

        assert_eq!(sim.snapshot.latitude_deg, sim.snapshot.home.latitude_deg);
        assert_eq!(sim.snapshot.longitude_deg, sim.snapshot.home.longitude_deg);
        assert_eq!(sim.velocity.north_mps, 0.0);
    }

    #[test]
    fn plane_no_target_loiters_forward() {
        let mut sim = sim_core(DemoProfile::ArduPlane);
        sim.snapshot.armed = true;

        sim.advance_navigation();

        assert!(sim.snapshot.latitude_deg > sim.snapshot.home.latitude_deg);
        assert!(sim.snapshot.yaw_rad > 0.0);
        assert!(horizontal_speed_mps(sim.velocity) > 0.0);
    }

    #[test]
    fn quadplane_qloiter_holds_without_target() {
        let mut sim = sim_core(DemoProfile::ArduQuadPlane);
        sim.snapshot.armed = true;
        sim.snapshot.custom_mode = super::super::state::QUADPLANE_QLOITER_MODE;

        sim.advance_navigation();

        assert_eq!(sim.snapshot.latitude_deg, sim.snapshot.home.latitude_deg);
        assert_eq!(sim.snapshot.longitude_deg, sim.snapshot.home.longitude_deg);
        assert_eq!(sim.velocity.north_mps, 0.0);
    }

    #[test]
    fn plane_turns_gradually_toward_target() {
        let mut sim = sim_core(DemoProfile::ArduPlane);
        sim.snapshot.armed = true;
        sim.snapshot.custom_mode = guided_mode(sim.config.profile);
        sim.nav_target = Some(NavTarget {
            source: NavSource::Guided,
            latitude_deg: 42.0,
            longitude_deg: -70.999,
            altitude_msl_m: 100.0,
            acceptance_radius_m: ACCEPTANCE_RADIUS_M,
            wire_seq: None,
            disarm_on_reach: false,
        });

        sim.advance_navigation();

        assert!(sim.snapshot.yaw_rad > 0.0);
        assert!(sim.snapshot.yaw_rad < std::f32::consts::FRAC_PI_2);
        assert!(sim.snapshot.latitude_deg > sim.snapshot.home.latitude_deg);
        assert!(sim.snapshot.longitude_deg > sim.snapshot.home.longitude_deg);
    }

    #[test]
    fn plane_turn_uses_bank_to_change_yaw() {
        let mut sim = sim_core(DemoProfile::ArduPlane);
        sim.snapshot.armed = true;
        sim.snapshot.custom_mode = guided_mode(sim.config.profile);
        sim.nav_target = Some(NavTarget {
            source: NavSource::Guided,
            latitude_deg: 42.0,
            longitude_deg: -70.999,
            altitude_msl_m: 100.0,
            acceptance_radius_m: ACCEPTANCE_RADIUS_M,
            wire_seq: None,
            disarm_on_reach: false,
        });

        sim.advance_navigation();

        let expected_yaw_rate =
            STANDARD_GRAVITY_MPS2 * sim.snapshot.roll_rad.tan() / PLANE_CRUISE_SPEED_MPS as f32;
        assert!(sim.snapshot.roll_rad > 0.0);
        assert!(sim.snapshot.yaw_rad > 0.0);
        assert!((sim.attitude_rates.yaw_rad_per_sec - expected_yaw_rate).abs() < 1e-5);
    }

    #[test]
    fn copter_forward_motion_pitches_down() {
        let mut sim = sim_core(DemoProfile::ArduCopter);
        sim.snapshot.armed = true;
        sim.snapshot.custom_mode = guided_mode(sim.config.profile);
        sim.nav_target = Some(NavTarget {
            source: NavSource::Guided,
            latitude_deg: 42.001,
            longitude_deg: -71.0,
            altitude_msl_m: 100.0,
            acceptance_radius_m: ACCEPTANCE_RADIUS_M,
            wire_seq: None,
            disarm_on_reach: false,
        });

        sim.advance_navigation();

        assert!(sim.snapshot.pitch_rad < 0.0);
        assert!(sim.snapshot.roll_rad.abs() < 1e-6);
        assert!(sim.velocity.north_mps > 0.0);
    }

    #[test]
    fn copter_steady_ten_meter_per_second_forward_flight_pitches_down_for_drag() {
        let mut sim = sim_core(DemoProfile::ArduCopter);
        sim.snapshot.armed = true;
        sim.snapshot.custom_mode = guided_mode(sim.config.profile);
        sim.velocity = VelocityNed {
            north_mps: 10.0,
            east_mps: 0.0,
            down_mps: 0.0,
        };
        sim.nav_target = Some(NavTarget {
            source: NavSource::Guided,
            latitude_deg: 42.01,
            longitude_deg: -71.0,
            altitude_msl_m: 100.0,
            acceptance_radius_m: ACCEPTANCE_RADIUS_M,
            wire_seq: None,
            disarm_on_reach: false,
        });

        for _ in 0..3 {
            sim.advance_navigation();
        }

        let pitch_deg = sim.snapshot.pitch_rad.to_degrees();
        assert!(
            (pitch_deg + 20.0).abs() < 0.25,
            "expected about 20 deg pitch down at 10 m/s, got {pitch_deg} deg"
        );
        assert!((sim.velocity.north_mps - 10.0).abs() < 1e-6);
    }

    #[test]
    fn copter_direction_change_uses_roll_pitch_and_yaw() {
        let mut sim = sim_core(DemoProfile::ArduCopter);
        sim.snapshot.armed = true;
        sim.snapshot.custom_mode = guided_mode(sim.config.profile);
        sim.velocity = VelocityNed {
            north_mps: 4.0,
            east_mps: 0.0,
            down_mps: 0.0,
        };
        sim.nav_target = Some(NavTarget {
            source: NavSource::Guided,
            latitude_deg: 42.0,
            longitude_deg: -70.999,
            altitude_msl_m: 100.0,
            acceptance_radius_m: ACCEPTANCE_RADIUS_M,
            wire_seq: None,
            disarm_on_reach: false,
        });

        sim.advance_navigation();

        assert!(sim.snapshot.roll_rad > 0.0);
        assert!(sim.snapshot.pitch_rad > 0.0);
        assert!(sim.snapshot.yaw_rad > 0.0);
    }

    #[test]
    fn copter_hover_levels_attitude() {
        let mut sim = sim_core(DemoProfile::ArduCopter);
        sim.snapshot.armed = true;
        sim.snapshot.roll_rad = 0.2;
        sim.snapshot.pitch_rad = -0.1;
        sim.velocity = VelocityNed {
            north_mps: 1.0,
            east_mps: 1.0,
            down_mps: 0.0,
        };

        sim.advance_navigation();

        assert_eq!(sim.snapshot.roll_rad, 0.0);
        assert_eq!(sim.snapshot.pitch_rad, 0.0);
        assert_eq!(horizontal_speed_mps(sim.velocity), 0.0);
    }

    #[test]
    fn plane_close_guided_target_completes_without_overshoot() {
        let mut sim = sim_core(DemoProfile::ArduPlane);
        sim.snapshot.armed = true;
        sim.snapshot.custom_mode = guided_mode(sim.config.profile);
        sim.nav_target = Some(NavTarget {
            source: NavSource::Guided,
            latitude_deg: 42.000_005,
            longitude_deg: -71.0,
            altitude_msl_m: 100.0,
            acceptance_radius_m: 0.2,
            wire_seq: None,
            disarm_on_reach: false,
        });

        sim.advance_navigation();

        assert!((sim.snapshot.latitude_deg - 42.000_005).abs() < 1e-12);
        assert_eq!(sim.snapshot.longitude_deg, -71.0);
        assert_eq!(horizontal_speed_mps(sim.velocity), 0.0);
    }

    #[test]
    fn quadplane_close_guided_target_completes_without_overshoot() {
        let mut sim = sim_core(DemoProfile::ArduQuadPlane);
        sim.snapshot.armed = true;
        sim.snapshot.custom_mode = guided_mode(sim.config.profile);
        sim.nav_target = Some(NavTarget {
            source: NavSource::Guided,
            latitude_deg: 42.000_005,
            longitude_deg: -71.0,
            altitude_msl_m: 100.0,
            acceptance_radius_m: 0.2,
            wire_seq: None,
            disarm_on_reach: false,
        });

        sim.advance_navigation();

        assert!((sim.snapshot.latitude_deg - 42.000_005).abs() < 1e-12);
        assert_eq!(sim.snapshot.longitude_deg, -71.0);
        assert_eq!(horizontal_speed_mps(sim.velocity), 0.0);
    }

    #[test]
    fn close_plane_mission_target_sets_pending_reached_sequence() {
        let mut sim = sim_core(DemoProfile::ArduPlane);
        sim.snapshot.armed = true;
        sim.snapshot.custom_mode = auto_mode(sim.config.profile);
        sim.nav_target = Some(NavTarget {
            source: NavSource::Mission,
            latitude_deg: 42.000_005,
            longitude_deg: -71.0,
            altitude_msl_m: 100.0,
            acceptance_radius_m: 0.2,
            wire_seq: Some(7),
            disarm_on_reach: false,
        });

        sim.advance_navigation();

        assert_eq!(sim.pending_reached_wire_seq, Some(7));
        assert!(sim.nav_target.is_none());
    }

    #[test]
    fn copter_target_motion_does_not_overshoot() {
        let mut sim = sim_core(DemoProfile::ArduCopter);
        sim.snapshot.armed = true;
        sim.snapshot.custom_mode = guided_mode(sim.config.profile);
        sim.nav_target = Some(NavTarget {
            source: NavSource::Guided,
            latitude_deg: 42.000_001,
            longitude_deg: -71.0,
            altitude_msl_m: 100.0,
            acceptance_radius_m: 0.2,
            wire_seq: None,
            disarm_on_reach: false,
        });

        sim.advance_navigation();

        assert!(sim.snapshot.latitude_deg <= 42.000_001);
    }
}
