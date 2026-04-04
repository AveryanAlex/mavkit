use crate::dialect;
use crate::modes::{ModeCatalogSource, ModeDescriptor};
use crate::state::{AutopilotType, VehicleType};
use std::collections::BTreeMap;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum VehicleClass {
    Copter,
    Plane,
    Rover,
    Unknown,
}

pub(super) fn vehicle_class(vehicle_type: VehicleType) -> VehicleClass {
    match vehicle_type {
        VehicleType::Quadrotor
        | VehicleType::Hexarotor
        | VehicleType::Octorotor
        | VehicleType::Tricopter
        | VehicleType::Coaxial
        | VehicleType::Helicopter => VehicleClass::Copter,
        VehicleType::FixedWing | VehicleType::Vtol => VehicleClass::Plane,
        VehicleType::GroundRover => VehicleClass::Rover,
        _ => VehicleClass::Unknown,
    }
}

pub(super) const COPTER_MODES: &[(u32, &str)] = &[
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

pub(super) const PLANE_MODES: &[(u32, &str)] = &[
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

pub(super) const ROVER_MODES: &[(u32, &str)] = &[
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

pub(super) fn mode_table(
    autopilot: AutopilotType,
    vehicle_type: VehicleType,
) -> &'static [(u32, &'static str)] {
    if autopilot != AutopilotType::ArduPilotMega {
        return &[];
    }

    match vehicle_class(vehicle_type) {
        VehicleClass::Copter | VehicleClass::Unknown => COPTER_MODES,
        VehicleClass::Plane => PLANE_MODES,
        VehicleClass::Rover => ROVER_MODES,
    }
}

pub(crate) fn mode_name(
    autopilot: AutopilotType,
    vehicle_type: VehicleType,
    custom_mode: u32,
) -> String {
    mode_table(autopilot, vehicle_type)
        .iter()
        .find_map(|&(num, name)| (num == custom_mode).then_some(name.to_string()))
        .unwrap_or_else(|| format!("MODE({custom_mode})"))
}

pub(crate) fn mode_number(
    autopilot: AutopilotType,
    vehicle_type: VehicleType,
    name: &str,
) -> Option<u32> {
    let upper = name.to_uppercase();
    mode_table(autopilot, vehicle_type)
        .iter()
        .find_map(|&(num, mode_name)| (mode_name == upper).then_some(num))
}

pub(super) fn static_catalog(
    autopilot: AutopilotType,
    vehicle_type: VehicleType,
) -> Vec<ModeDescriptor> {
    mode_table(autopilot, vehicle_type)
        .iter()
        .map(|&(custom_mode, name)| ModeDescriptor {
            custom_mode,
            name: name.to_string(),
            user_selectable: true,
            source: ModeCatalogSource::StaticArduPilotTable,
        })
        .collect()
}

pub(super) fn dynamic_catalog(
    modes: &BTreeMap<u8, dialect::AVAILABLE_MODES_DATA>,
) -> Vec<ModeDescriptor> {
    modes
        .values()
        .map(|mode| ModeDescriptor {
            custom_mode: mode.custom_mode,
            name: available_mode_name(mode),
            user_selectable: !mode
                .properties
                .contains(dialect::MavModeProperty::MAV_MODE_PROPERTY_NOT_USER_SELECTABLE),
            source: ModeCatalogSource::AvailableModes,
        })
        .collect()
}

pub(super) fn available_mode_name(mode: &dialect::AVAILABLE_MODES_DATA) -> String {
    let name = mode.mode_name.to_str().unwrap_or_default().trim();
    if name.is_empty() {
        format!("MODE({})", mode.custom_mode)
    } else {
        name.to_string()
    }
}

pub(super) fn catalog_name_or_fallback(catalog: &[ModeDescriptor], custom_mode: u32) -> String {
    catalog
        .iter()
        .find_map(|mode| (mode.custom_mode == custom_mode).then(|| mode.name.clone()))
        .unwrap_or_else(|| format!("MODE({custom_mode})"))
}
