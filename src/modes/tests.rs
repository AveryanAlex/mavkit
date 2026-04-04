use super::*;
use crate::dialect;
use crate::state::{AutopilotType, VehicleState, VehicleType};

#[test]
fn static_table_population() {
    let (runtime, handle) = test_observer_fixture();

    runtime.handle_vehicle_state(&heartbeat_state(
        AutopilotType::ArduPilotMega,
        VehicleType::Quadrotor,
        5,
    ));

    assert_eq!(
        handle.support().latest(),
        Some(crate::types::SupportState::Supported)
    );

    let catalog = handle
        .catalog()
        .latest()
        .expect("catalog should be populated");
    assert!(catalog.iter().any(|mode| {
        mode.custom_mode == 0
            && mode.name == "STABILIZE"
            && mode.user_selectable
            && mode.source == ModeCatalogSource::StaticArduPilotTable
    }));
    assert!(catalog.iter().any(|mode| {
        mode.custom_mode == 5
            && mode.name == "LOITER"
            && mode.source == ModeCatalogSource::StaticArduPilotTable
    }));

    assert_eq!(
        handle.current().latest(),
        Some(CurrentMode {
            custom_mode: 5,
            name: "LOITER".to_string(),
            intended_custom_mode: None,
            source: CurrentModeSource::Heartbeat,
        })
    );
}

#[test]
fn dynamic_available_modes_replace_static_catalog() {
    let (runtime, handle) = test_observer_fixture();

    runtime.handle_vehicle_state(&heartbeat_state(
        AutopilotType::ArduPilotMega,
        VehicleType::Quadrotor,
        4,
    ));

    runtime.handle_message(&dialect::MavMessage::AVAILABLE_MODES(
        available_modes_message(2, 1, 4, "GUIDED", true),
    ));
    runtime.handle_message(&dialect::MavMessage::AVAILABLE_MODES(
        available_modes_message(2, 2, 27, "AUTO_RTL", false),
    ));

    assert_eq!(
        handle.catalog().latest(),
        Some(vec![
            ModeDescriptor {
                custom_mode: 4,
                name: "GUIDED".to_string(),
                user_selectable: true,
                source: ModeCatalogSource::AvailableModes,
            },
            ModeDescriptor {
                custom_mode: 27,
                name: "AUTO_RTL".to_string(),
                user_selectable: false,
                source: ModeCatalogSource::AvailableModes,
            },
        ])
    );
}

#[test]
fn current_mode_tracks_heartbeat_custom_mode() {
    let (runtime, handle) = test_observer_fixture();

    runtime.handle_vehicle_state(&heartbeat_state(
        AutopilotType::ArduPilotMega,
        VehicleType::Quadrotor,
        4,
    ));
    runtime.handle_vehicle_state(&heartbeat_state(
        AutopilotType::ArduPilotMega,
        VehicleType::Quadrotor,
        6,
    ));

    assert_eq!(
        handle.current().latest(),
        Some(CurrentMode {
            custom_mode: 6,
            name: "RTL".to_string(),
            intended_custom_mode: None,
            source: CurrentModeSource::Heartbeat,
        })
    );
}

#[test]
fn unknown_mode_fallback() {
    let (runtime, handle) = test_observer_fixture();

    runtime.handle_vehicle_state(&heartbeat_state(
        AutopilotType::ArduPilotMega,
        VehicleType::Quadrotor,
        999,
    ));

    assert_eq!(
        handle.current().latest(),
        Some(CurrentMode {
            custom_mode: 999,
            name: "MODE(999)".to_string(),
            intended_custom_mode: None,
            source: CurrentModeSource::Heartbeat,
        })
    );
}

#[test]
fn mode_name_unknown_ardupilot_uses_mode_fallback() {
    assert_eq!(
        mode_name(AutopilotType::ArduPilotMega, VehicleType::Quadrotor, 999),
        "MODE(999)"
    );
}

#[test]
fn mode_number_is_case_insensitive() {
    assert_eq!(
        mode_number(
            AutopilotType::ArduPilotMega,
            VehicleType::GroundRover,
            "guided"
        ),
        Some(15)
    );
}

fn test_observer_fixture() -> (ModeDomain, ModeDomain) {
    let domain = ModeDomain::new();
    (domain.clone(), domain)
}

fn heartbeat_state(
    autopilot: AutopilotType,
    vehicle_type: VehicleType,
    custom_mode: u32,
) -> VehicleState {
    VehicleState {
        custom_mode,
        autopilot,
        vehicle_type,
        heartbeat_received: true,
        ..VehicleState::default()
    }
}

fn available_modes_message(
    number_modes: u8,
    mode_index: u8,
    custom_mode: u32,
    mode_name: &str,
    user_selectable: bool,
) -> dialect::AVAILABLE_MODES_DATA {
    let mut properties = dialect::MavModeProperty::empty();
    if !user_selectable {
        properties |= dialect::MavModeProperty::MAV_MODE_PROPERTY_NOT_USER_SELECTABLE;
    }

    dialect::AVAILABLE_MODES_DATA {
        custom_mode,
        properties,
        number_modes,
        mode_index,
        standard_mode: dialect::MavStandardMode::MAV_STANDARD_MODE_NON_STANDARD,
        mode_name: mode_name.into(),
    }
}

fn current_mode_message(custom_mode: u32, intended_custom_mode: u32) -> dialect::CURRENT_MODE_DATA {
    dialect::CURRENT_MODE_DATA {
        custom_mode,
        intended_custom_mode,
        standard_mode: dialect::MavStandardMode::MAV_STANDARD_MODE_NON_STANDARD,
    }
}

#[test]
fn current_mode_message_updates_current() {
    let (runtime, handle) = test_observer_fixture();

    runtime.handle_vehicle_state(&heartbeat_state(
        AutopilotType::ArduPilotMega,
        VehicleType::Quadrotor,
        4,
    ));

    runtime.handle_message(&dialect::MavMessage::CURRENT_MODE(current_mode_message(
        4, 0,
    )));

    assert_eq!(
        handle.current().latest(),
        Some(CurrentMode {
            custom_mode: 4,
            name: "GUIDED".to_string(),
            intended_custom_mode: None,
            source: CurrentModeSource::CurrentModeMessage,
        })
    );
}

#[test]
fn current_mode_message_with_intended_mode() {
    let (runtime, handle) = test_observer_fixture();

    runtime.handle_vehicle_state(&heartbeat_state(
        AutopilotType::ArduPilotMega,
        VehicleType::Quadrotor,
        6,
    ));

    // Vehicle is in RTL (6) but user intended GUIDED (4)
    runtime.handle_message(&dialect::MavMessage::CURRENT_MODE(current_mode_message(
        6, 4,
    )));

    assert_eq!(
        handle.current().latest(),
        Some(CurrentMode {
            custom_mode: 6,
            name: "RTL".to_string(),
            intended_custom_mode: Some(4),
            source: CurrentModeSource::CurrentModeMessage,
        })
    );
}

#[test]
fn heartbeat_does_not_overwrite_current_mode_source() {
    let (runtime, handle) = test_observer_fixture();

    runtime.handle_vehicle_state(&heartbeat_state(
        AutopilotType::ArduPilotMega,
        VehicleType::Quadrotor,
        6,
    ));

    runtime.handle_message(&dialect::MavMessage::CURRENT_MODE(current_mode_message(
        6, 4,
    )));

    // Another heartbeat with the same mode should not overwrite
    runtime.handle_vehicle_state(&heartbeat_state(
        AutopilotType::ArduPilotMega,
        VehicleType::Quadrotor,
        6,
    ));

    let current = handle.current().latest().unwrap();
    assert_eq!(current.source, CurrentModeSource::CurrentModeMessage);
    assert_eq!(current.intended_custom_mode, Some(4));
}

#[test]
fn heartbeat_with_new_mode_overrides_current_mode_source() {
    let (runtime, handle) = test_observer_fixture();

    runtime.handle_vehicle_state(&heartbeat_state(
        AutopilotType::ArduPilotMega,
        VehicleType::Quadrotor,
        6,
    ));

    runtime.handle_message(&dialect::MavMessage::CURRENT_MODE(current_mode_message(
        6, 4,
    )));

    // Heartbeat with a different mode should override
    runtime.handle_vehicle_state(&heartbeat_state(
        AutopilotType::ArduPilotMega,
        VehicleType::Quadrotor,
        5,
    ));

    let current = handle.current().latest().unwrap();
    assert_eq!(current.custom_mode, 5);
    assert_eq!(current.source, CurrentModeSource::Heartbeat);
    assert_eq!(current.intended_custom_mode, None);
}
