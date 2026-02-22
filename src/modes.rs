use crate::state::{AutopilotType, FlightMode, VehicleType};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum VehicleClass {
    Copter,
    Plane,
    Rover,
    Unknown,
}

fn vehicle_class(vehicle_type: VehicleType) -> VehicleClass {
    match vehicle_type {
        VehicleType::Quadrotor
        | VehicleType::Hexarotor
        | VehicleType::Octorotor
        | VehicleType::Tricopter
        | VehicleType::Coaxial
        | VehicleType::Helicopter => VehicleClass::Copter,
        VehicleType::FixedWing => VehicleClass::Plane,
        VehicleType::GroundRover => VehicleClass::Rover,
        _ => VehicleClass::Unknown,
    }
}

const COPTER_MODES: &[(u32, &str)] = &[
    (0, "STABILIZE"),
    (1, "ACRO"),
    (2, "ALT_HOLD"),
    (3, "AUTO"),
    (4, "GUIDED"),
    (5, "LOITER"),
    (6, "RTL"),
    (7, "CIRCLE"),
    (9, "LAND"),
    (11, "DRIFT"),
    (13, "SPORT"),
    (15, "AUTOTUNE"),
    (16, "POSHOLD"),
    (17, "BRAKE"),
    (18, "THROW"),
    (21, "SMART_RTL"),
];

const PLANE_MODES: &[(u32, &str)] = &[
    (0, "MANUAL"),
    (1, "CIRCLE"),
    (2, "STABILIZE"),
    (3, "TRAINING"),
    (4, "ACRO"),
    (5, "FLY_BY_WIRE_A"),
    (6, "FLY_BY_WIRE_B"),
    (7, "CRUISE"),
    (8, "AUTOTUNE"),
    (10, "AUTO"),
    (11, "RTL"),
    (12, "LOITER"),
    (15, "GUIDED"),
    (17, "QSTABILIZE"),
    (18, "QHOVER"),
    (19, "QLOITER"),
    (20, "QLAND"),
    (21, "QRTL"),
];

const ROVER_MODES: &[(u32, &str)] = &[
    (0, "MANUAL"),
    (1, "ACRO"),
    (3, "STEERING"),
    (4, "HOLD"),
    (5, "LOITER"),
    (6, "FOLLOW"),
    (7, "SIMPLE"),
    (10, "AUTO"),
    (11, "RTL"),
    (12, "SMART_RTL"),
    (15, "GUIDED"),
];

fn mode_table(autopilot: AutopilotType, vehicle_type: VehicleType) -> &'static [(u32, &'static str)] {
    if autopilot != AutopilotType::ArduPilotMega {
        return &[];
    }
    match vehicle_class(vehicle_type) {
        VehicleClass::Copter | VehicleClass::Unknown => COPTER_MODES,
        VehicleClass::Plane => PLANE_MODES,
        VehicleClass::Rover => ROVER_MODES,
    }
}

pub(crate) fn mode_name(autopilot: AutopilotType, vehicle_type: VehicleType, custom_mode: u32) -> String {
    if autopilot != AutopilotType::ArduPilotMega {
        return format!("MODE({custom_mode})");
    }
    let table = mode_table(autopilot, vehicle_type);
    for &(num, name) in table {
        if num == custom_mode {
            return name.to_string();
        }
    }
    format!("UNKNOWN({custom_mode})")
}

pub(crate) fn mode_number(autopilot: AutopilotType, vehicle_type: VehicleType, name: &str) -> Option<u32> {
    let table = mode_table(autopilot, vehicle_type);
    let upper = name.to_uppercase();
    for &(num, mode_name) in table {
        if mode_name == upper {
            return Some(num);
        }
    }
    None
}

pub(crate) fn available_modes(autopilot: AutopilotType, vehicle_type: VehicleType) -> Vec<FlightMode> {
    mode_table(autopilot, vehicle_type)
        .iter()
        .map(|&(num, name)| FlightMode {
            custom_mode: num,
            name: name.to_string(),
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn copter_guided_name() {
        assert_eq!(
            mode_name(AutopilotType::ArduPilotMega, VehicleType::Quadrotor, 4),
            "GUIDED"
        );
    }

    #[test]
    fn copter_guided_number_case_insensitive() {
        assert_eq!(
            mode_number(AutopilotType::ArduPilotMega, VehicleType::Quadrotor, "guided"),
            Some(4)
        );
    }

    #[test]
    fn plane_rtl_name() {
        assert_eq!(
            mode_name(AutopilotType::ArduPilotMega, VehicleType::FixedWing, 11),
            "RTL"
        );
    }

    #[test]
    fn unknown_mode_number() {
        assert_eq!(
            mode_name(AutopilotType::ArduPilotMega, VehicleType::Quadrotor, 999),
            "UNKNOWN(999)"
        );
    }

    #[test]
    fn available_modes_copter_length() {
        let modes = available_modes(AutopilotType::ArduPilotMega, VehicleType::Quadrotor);
        assert_eq!(modes.len(), COPTER_MODES.len());
    }

    #[test]
    fn non_ardupilot_returns_mode_n() {
        assert_eq!(
            mode_name(AutopilotType::Generic, VehicleType::Quadrotor, 4),
            "MODE(4)"
        );
    }

    #[test]
    fn non_ardupilot_available_modes_empty() {
        let modes = available_modes(AutopilotType::Generic, VehicleType::Quadrotor);
        assert!(modes.is_empty());
    }

    #[test]
    fn rover_guided_number() {
        assert_eq!(
            mode_number(AutopilotType::ArduPilotMega, VehicleType::GroundRover, "GUIDED"),
            Some(15)
        );
    }
}
