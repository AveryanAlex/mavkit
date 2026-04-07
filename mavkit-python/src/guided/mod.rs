mod copter;
mod plane;
mod rover;
mod session;
mod sub;
#[cfg(test)]
mod test_support;
#[cfg(test)]
mod tests;

pub use copter::PyArduCopterGuidedHandle;
pub use plane::{PyArduPlaneGuidedHandle, PyArduPlaneVtolGuidedHandle};
pub use rover::PyArduRoverGuidedHandle;
pub use session::PyArduGuidedSession;
pub use sub::PyArduSubGuidedHandle;

fn geo_point_2d(latitude_deg: f64, longitude_deg: f64) -> mavkit::GeoPoint2d {
    mavkit::GeoPoint2d {
        latitude_deg,
        longitude_deg,
    }
}

fn geo_point_rel_home(
    latitude_deg: f64,
    longitude_deg: f64,
    relative_alt_m: f64,
) -> mavkit::GeoPoint3dRelHome {
    mavkit::GeoPoint3dRelHome {
        latitude_deg,
        longitude_deg,
        relative_alt_m,
    }
}

fn geo_point_msl(
    latitude_deg: f64,
    longitude_deg: f64,
    altitude_msl_m: f64,
) -> mavkit::GeoPoint3dMsl {
    mavkit::GeoPoint3dMsl {
        latitude_deg,
        longitude_deg,
        altitude_msl_m,
    }
}

fn relative_climb_target(relative_climb_m: f32) -> mavkit::RelativeClimbTarget {
    mavkit::RelativeClimbTarget { relative_climb_m }
}

fn sub_goto_depth_target(
    latitude_deg: f64,
    longitude_deg: f64,
    depth_m: f32,
) -> mavkit::SubGotoDepthTarget {
    mavkit::SubGotoDepthTarget {
        point: geo_point_2d(latitude_deg, longitude_deg),
        depth_m,
    }
}

fn session_closed_error() -> mavkit::VehicleError {
    mavkit::VehicleError::OperationConflict {
        conflicting_domain: "ardupilot_guided".to_string(),
        conflicting_op: "session_closed".to_string(),
    }
}

fn guided_family_unavailable_error(label: &str) -> mavkit::VehicleError {
    mavkit::VehicleError::Unsupported(format!(
        "{label} guided controls are unavailable for this session"
    ))
}
