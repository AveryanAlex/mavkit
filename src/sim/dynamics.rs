use crate::dialect;
use crate::mission::HomePosition;

use super::state::{
    ACCEPTANCE_RADIUS_M, DemoVehicleFamily, NavSource, NavTarget, SimulatorCore,
    TAKEOFF_ALT_MARGIN_M, VelocityNed, auto_mode, guided_mode, is_hover_hold_mode,
    profile_vehicle_family,
};

const COPTER_HORIZONTAL_SPEED_MPS: f64 = 4.0;
const COPTER_CLIMB_SPEED_MPS: f64 = 2.0;
const COPTER_DESCENT_SPEED_MPS: f64 = 1.5;
const PLANE_CRUISE_SPEED_MPS: f64 = 12.0;
const QUADPLANE_CRUISE_SPEED_MPS: f64 = 10.0;
const PLANE_CLIMB_SPEED_MPS: f64 = 2.5;
const PLANE_DESCENT_SPEED_MPS: f64 = 1.8;
const PLANE_TURN_RATE_RAD_PER_SEC: f32 = 25.0_f32.to_radians();
const QUADPLANE_TURN_RATE_RAD_PER_SEC: f32 = 35.0_f32.to_radians();
const IDLE_LOITER_TURN_RATE_RAD_PER_SEC: f32 = 18.0_f32.to_radians();

impl SimulatorCore {
    pub(crate) fn advance_navigation(&mut self) {
        if !self.snapshot.armed {
            self.park();
            return;
        }

        let dt_s = 1.0 / f64::from(self.config.tick_hz);
        let Some(target) = self.active_nav_target() else {
            self.advance_idle_behavior(dt_s);
            return;
        };

        match profile_vehicle_family(self.config.profile) {
            DemoVehicleFamily::Copter => self.advance_copter_navigation(target, dt_s),
            DemoVehicleFamily::Plane | DemoVehicleFamily::QuadPlane => {
                self.advance_plane_navigation(target, dt_s)
            }
        }
    }

    fn park(&mut self) {
        self.velocity = VelocityNed {
            north_mps: 0.0,
            east_mps: 0.0,
            down_mps: 0.0,
        };
        self.snapshot.relative_alt_m = self.snapshot.altitude_msl_m - self.snapshot.home.altitude_m;
    }

    fn advance_idle_behavior(&mut self, dt_s: f64) {
        let family = profile_vehicle_family(self.config.profile);
        if family == DemoVehicleFamily::Copter
            || is_hover_hold_mode(self.config.profile, self.snapshot.custom_mode)
        {
            self.park();
            return;
        }

        let turn_rate = match family {
            DemoVehicleFamily::Plane => IDLE_LOITER_TURN_RATE_RAD_PER_SEC,
            DemoVehicleFamily::QuadPlane => IDLE_LOITER_TURN_RATE_RAD_PER_SEC * 1.1,
            DemoVehicleFamily::Copter => 0.0,
        };
        self.snapshot.yaw_rad = wrap_angle(self.snapshot.yaw_rad + turn_rate * dt_s as f32);
        let speed_mps = self.cruise_speed_mps(None);
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

        let horizontal_step_m = (COPTER_HORIZONTAL_SPEED_MPS * dt_s).min(horizontal_distance_m);
        let vertical_limit_m = if up_m >= 0.0 {
            COPTER_CLIMB_SPEED_MPS * dt_s
        } else {
            COPTER_DESCENT_SPEED_MPS * dt_s
        };
        let step_up_m = up_m.clamp(-vertical_limit_m, vertical_limit_m);
        let scale = if horizontal_distance_m > 0.0 {
            horizontal_step_m / horizontal_distance_m
        } else {
            0.0
        };
        let step_north_m = north_m * scale;
        let step_east_m = east_m * scale;
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

        if horizontal_distance_m > 0.2 {
            self.snapshot.yaw_rad = wrap_angle(east_m.atan2(north_m) as f32);
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

        if horizontal_distance_m <= target.acceptance_radius_m
            && up_m.abs() <= target.acceptance_radius_m.max(TAKEOFF_ALT_MARGIN_M)
        {
            self.finish_nav_target(target);
            return;
        }

        let desired_yaw = east_m.atan2(north_m) as f32;
        let max_turn = self.turn_rate_rad_per_sec() * dt_s as f32;
        let yaw_error = wrap_angle(desired_yaw - self.snapshot.yaw_rad);
        let yaw_step = yaw_error.clamp(-max_turn, max_turn);
        self.snapshot.yaw_rad = wrap_angle(self.snapshot.yaw_rad + yaw_step);

        let speed_mps = self.cruise_speed_mps(Some(target));
        let step_north_m = f64::from(self.snapshot.yaw_rad.cos()) * speed_mps * dt_s;
        let step_east_m = f64::from(self.snapshot.yaw_rad.sin()) * speed_mps * dt_s;
        let step_up_m = self.clamped_vertical_step(up_m, dt_s);

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
