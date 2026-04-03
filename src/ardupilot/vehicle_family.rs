use super::guided::{ArduGuidedKind, ArduPlaneKind};
use crate::state::VehicleType;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum VehicleFamily {
    Copter,
    Plane(ArduPlaneKind),
    Rover,
    Sub,
}

pub(super) fn classify(vehicle_type: VehicleType) -> Option<VehicleFamily> {
    match vehicle_type {
        VehicleType::Quadrotor
        | VehicleType::Hexarotor
        | VehicleType::Octorotor
        | VehicleType::Tricopter
        | VehicleType::Coaxial
        | VehicleType::Helicopter => Some(VehicleFamily::Copter),
        VehicleType::FixedWing => Some(VehicleFamily::Plane(ArduPlaneKind::FixedWing)),
        VehicleType::Vtol => Some(VehicleFamily::Plane(ArduPlaneKind::Vtol)),
        VehicleType::GroundRover => Some(VehicleFamily::Rover),
        VehicleType::Submarine => Some(VehicleFamily::Sub),
        VehicleType::Unknown | VehicleType::Generic => None,
    }
}

pub(super) fn guided_kind(vehicle_type: VehicleType) -> Option<ArduGuidedKind> {
    match classify(vehicle_type) {
        Some(VehicleFamily::Copter) => Some(ArduGuidedKind::Copter),
        Some(VehicleFamily::Plane(_)) => Some(ArduGuidedKind::Plane),
        Some(VehicleFamily::Rover) => Some(ArduGuidedKind::Rover),
        Some(VehicleFamily::Sub) => Some(ArduGuidedKind::Sub),
        None => None,
    }
}

pub(super) fn plane_kind(vehicle_type: VehicleType) -> Option<ArduPlaneKind> {
    match classify(vehicle_type) {
        Some(VehicleFamily::Plane(kind)) => Some(kind),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn classify_maps_vehicle_types_to_expected_families() {
        assert_eq!(
            classify(VehicleType::Quadrotor),
            Some(VehicleFamily::Copter)
        );
        assert_eq!(
            classify(VehicleType::FixedWing),
            Some(VehicleFamily::Plane(ArduPlaneKind::FixedWing))
        );
        assert_eq!(
            classify(VehicleType::Vtol),
            Some(VehicleFamily::Plane(ArduPlaneKind::Vtol))
        );
        assert_eq!(
            classify(VehicleType::GroundRover),
            Some(VehicleFamily::Rover)
        );
        assert_eq!(classify(VehicleType::Submarine), Some(VehicleFamily::Sub));
        assert_eq!(classify(VehicleType::Unknown), None);
        assert_eq!(classify(VehicleType::Generic), None);
    }
}
