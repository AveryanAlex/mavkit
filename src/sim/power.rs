use crate::dialect;

use super::dynamics::horizontal_speed_mps;
use super::state::{DemoVehicleFamily, SimulatorCore, VelocityNed, profile_vehicle_family};

const DEFAULT_BATTERY_CELLS: usize = 3;
const DEFAULT_BATTERY_CAPACITY_MAH: f64 = 5_200.0;
const DEFAULT_INITIAL_REMAINING_PCT: f64 = 82.0;
const CELL_FULL_V: f64 = 4.20;
const CELL_EMPTY_V: f64 = 3.45;
const PACK_INTERNAL_RESISTANCE_OHM: f64 = 0.035;
const DISARMED_CURRENT_A: f64 = 0.35;

#[derive(Debug, Clone, Copy)]
pub(crate) struct PowerState {
    cells: usize,
    capacity_mah: f64,
    consumed_mah: f64,
    current_a: f64,
    voltage_v: f64,
    load_factor: f64,
}

impl Default for PowerState {
    fn default() -> Self {
        let capacity_mah = DEFAULT_BATTERY_CAPACITY_MAH;
        let consumed_mah = capacity_mah * (100.0 - DEFAULT_INITIAL_REMAINING_PCT) / 100.0;
        let mut state = Self {
            cells: DEFAULT_BATTERY_CELLS,
            capacity_mah,
            consumed_mah,
            current_a: DISARMED_CURRENT_A,
            voltage_v: 0.0,
            load_factor: 0.0,
        };
        state.recompute_voltage();
        state
    }
}

impl PowerState {
    pub(crate) fn remaining_pct(&self) -> f64 {
        (100.0 * (1.0 - self.consumed_mah / self.capacity_mah)).clamp(0.0, 100.0)
    }

    pub(crate) fn voltage_mv(&self) -> u16 {
        (self.voltage_v * 1000.0)
            .round()
            .clamp(0.0, f64::from(u16::MAX)) as u16
    }

    pub(crate) fn current_ca(&self) -> i16 {
        (self.current_a * 100.0)
            .round()
            .clamp(0.0, f64::from(i16::MAX)) as i16
    }

    pub(crate) fn consumed_mah(&self) -> i32 {
        self.consumed_mah.round().clamp(0.0, f64::from(i32::MAX)) as i32
    }

    pub(crate) fn consumed_hj(&self) -> i32 {
        let consumed_wh = self.consumed_mah * f64::from(self.voltage_mv()) / 1_000_000.0;
        (consumed_wh * 36.0).round().clamp(0.0, f64::from(i32::MAX)) as i32
    }

    pub(crate) fn time_remaining_s(&self) -> i32 {
        if self.current_a <= 0.0 {
            return 0;
        }

        let remaining_ah = (self.capacity_mah - self.consumed_mah).max(0.0) / 1000.0;
        (remaining_ah / self.current_a * 3600.0)
            .round()
            .clamp(0.0, f64::from(i32::MAX)) as i32
    }

    pub(crate) fn remaining_pct_i8(&self) -> i8 {
        self.remaining_pct().round().clamp(0.0, 100.0) as i8
    }

    pub(crate) fn load_dpercent(&self) -> u16 {
        (150.0 + self.load_factor * 550.0)
            .round()
            .clamp(0.0, 1000.0) as u16
    }

    pub(crate) fn cell_voltages_mv(&self) -> [u16; 10] {
        let mut voltages = [u16::MAX; 10];
        let cell_mv = (self.voltage_v / self.cells as f64 * 1000.0)
            .round()
            .clamp(0.0, f64::from(u16::MAX)) as u16;
        for voltage in voltages.iter_mut().take(self.cells) {
            *voltage = cell_mv;
        }
        voltages
    }

    fn advance(
        &mut self,
        dt_s: f64,
        armed: bool,
        family: DemoVehicleFamily,
        velocity: VelocityNed,
    ) {
        self.load_factor = power_load_factor(armed, family, velocity);
        self.current_a = if armed {
            base_armed_current_a(family) + self.load_factor * dynamic_current_a(family)
        } else {
            DISARMED_CURRENT_A
        };
        self.consumed_mah =
            (self.consumed_mah + self.current_a * dt_s * 1000.0 / 3600.0).min(self.capacity_mah);
        self.recompute_voltage();
    }

    fn recompute_voltage(&mut self) {
        let soc = self.remaining_pct() / 100.0;
        let resting_cell_v = CELL_EMPTY_V + (CELL_FULL_V - CELL_EMPTY_V) * soc.powf(0.65);
        let sag_v = self.current_a * PACK_INTERNAL_RESISTANCE_OHM;
        self.voltage_v = (resting_cell_v * self.cells as f64 - sag_v)
            .max(CELL_EMPTY_V * self.cells as f64 * 0.9);
    }
}

impl SimulatorCore {
    pub(crate) fn advance_power_model(&mut self) {
        let dt_s = 1.0 / f64::from(self.config.tick_hz);
        self.power.advance(
            dt_s,
            self.snapshot.armed,
            profile_vehicle_family(self.config.profile),
            self.velocity,
        );
    }

    pub(crate) fn battery_charge_state(&self) -> dialect::MavBatteryChargeState {
        if self.power.remaining_pct() <= 15.0 {
            dialect::MavBatteryChargeState::MAV_BATTERY_CHARGE_STATE_CRITICAL
        } else if self.power.remaining_pct() <= 30.0 {
            dialect::MavBatteryChargeState::MAV_BATTERY_CHARGE_STATE_LOW
        } else {
            dialect::MavBatteryChargeState::MAV_BATTERY_CHARGE_STATE_OK
        }
    }
}

fn power_load_factor(armed: bool, family: DemoVehicleFamily, velocity: VelocityNed) -> f64 {
    if !armed {
        return 0.0;
    }

    let horizontal = horizontal_speed_mps(velocity);
    let vertical = velocity.down_mps.abs();
    let cruise_speed = match family {
        DemoVehicleFamily::Copter => 4.0,
        DemoVehicleFamily::Plane => 12.0,
        DemoVehicleFamily::QuadPlane => 10.0,
    };
    let vertical_speed = match family {
        DemoVehicleFamily::Copter => 2.0,
        DemoVehicleFamily::Plane | DemoVehicleFamily::QuadPlane => 2.5,
    };

    (0.25
        + 0.55 * (horizontal / cruise_speed).min(1.5)
        + 0.20 * (vertical / vertical_speed).min(1.5))
    .clamp(0.0, 1.5)
}

fn base_armed_current_a(family: DemoVehicleFamily) -> f64 {
    match family {
        DemoVehicleFamily::Copter => 3.5,
        DemoVehicleFamily::Plane => 2.2,
        DemoVehicleFamily::QuadPlane => 3.0,
    }
}

fn dynamic_current_a(family: DemoVehicleFamily) -> f64 {
    match family {
        DemoVehicleFamily::Copter => 16.0,
        DemoVehicleFamily::Plane => 9.0,
        DemoVehicleFamily::QuadPlane => 12.0,
    }
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
                tick_hz: 1,
                home,
            },
            outbound_tx,
            snapshot_tx,
            snapshot,
        )
    }

    #[test]
    fn battery_drains_faster_when_armed_than_disarmed() {
        let mut disarmed = sim_core(DemoProfile::ArduCopter);
        let mut armed = sim_core(DemoProfile::ArduCopter);
        armed.snapshot.armed = true;

        for _ in 0..60 {
            disarmed.advance_power_model();
            armed.advance_power_model();
        }

        assert!(armed.power.consumed_mah > disarmed.power.consumed_mah);
    }

    #[test]
    fn higher_motion_load_increases_current_and_drain() {
        let mut idle = sim_core(DemoProfile::ArduCopter);
        let mut loaded = sim_core(DemoProfile::ArduCopter);
        idle.snapshot.armed = true;
        loaded.snapshot.armed = true;
        loaded.velocity = VelocityNed {
            north_mps: 4.0,
            east_mps: 0.0,
            down_mps: -2.0,
        };

        idle.advance_power_model();
        loaded.advance_power_model();

        assert!(loaded.power.current_a > idle.power.current_a);
        assert!(loaded.power.consumed_mah > idle.power.consumed_mah);
    }
}
