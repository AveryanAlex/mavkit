//! ArduPilot parameter semantics, independent of GCS labels and metadata downloads.
//!
//! Resolve against the parameters actually advertised by the vehicle. Version-only
//! selection is unsafe with vendor backports. `reference` records the firmware
//! source/version used to verify each generation; it is not a support-version gate.

use serde::Serialize;

#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct ParameterEncoding {
    pub family: &'static str,
    pub role: &'static str,
    pub name: &'static str,
    pub raw_per_unit: f64,
    pub unit: &'static str,
    pub sentinels: &'static [f64],
    pub reference: &'static str,
}

impl ParameterEncoding {
    pub fn display_value(&self, raw: f64) -> f64 {
        if self.sentinels.contains(&raw) {
            raw
        } else {
            raw / self.raw_per_unit
        }
    }
    pub fn raw_value(&self, display: f64) -> f64 {
        if self.sentinels.contains(&display) {
            display
        } else {
            display * self.raw_per_unit
        }
    }
}

macro_rules! encoding {
    ($family:literal, $role:literal, $name:literal, $scale:literal, $unit:literal, $sentinels:expr, $reference:literal) => {
        ParameterEncoding {
            family: $family,
            role: $role,
            name: $name,
            raw_per_unit: $scale,
            unit: $unit,
            sentinels: $sentinels,
            reference: $reference,
        }
    };
}

// Modern names precede legacy names. Sources: ArduCopter/Parameters.cpp,
// ArduPlane/Parameters.cpp and Rover/Parameters.cpp at the recorded refs.
pub const RTL_PARAMETER_ENCODINGS: &[ParameterEncoding] = &[
    encoding!(
        "copter",
        "return_altitude",
        "RTL_ALT_M",
        1.0,
        "m",
        &[0.0],
        "master"
    ),
    encoding!(
        "copter",
        "return_altitude",
        "RTL_ALT",
        100.0,
        "m",
        &[0.0],
        "Copter-4.5.7"
    ),
    encoding!(
        "copter",
        "final_altitude",
        "RTL_ALT_FINAL_M",
        1.0,
        "m",
        &[0.0],
        "master"
    ),
    encoding!(
        "copter",
        "final_altitude",
        "RTL_ALT_FINAL",
        100.0,
        "m",
        &[0.0],
        "Copter-4.5.7"
    ),
    encoding!(
        "copter",
        "minimum_climb",
        "RTL_CLIMB_MIN_M",
        1.0,
        "m",
        &[],
        "master"
    ),
    encoding!(
        "copter",
        "minimum_climb",
        "RTL_CLIMB_MIN",
        100.0,
        "m",
        &[],
        "Copter-4.5.7"
    ),
    encoding!(
        "copter",
        "return_speed",
        "RTL_SPEED_MS",
        1.0,
        "m/s",
        &[0.0],
        "master"
    ),
    encoding!(
        "copter",
        "return_speed",
        "RTL_SPEED",
        100.0,
        "m/s",
        &[0.0],
        "Copter-4.5.7"
    ),
    encoding!(
        "copter",
        "loiter_time",
        "RTL_LOIT_TIME",
        1000.0,
        "s",
        &[],
        "Copter-4.5.7"
    ),
    encoding!(
        "plane",
        "return_altitude",
        "RTL_ALTITUDE",
        1.0,
        "m",
        &[-1.0],
        "master"
    ),
    encoding!(
        "plane",
        "return_altitude",
        "ALT_HOLD_RTL",
        100.0,
        "m",
        &[-1.0],
        "Plane-4.5.7"
    ),
    encoding!(
        "rover",
        "return_speed",
        "RTL_SPEED",
        1.0,
        "m/s",
        &[0.0],
        "Rover-4.5.7"
    ),
    encoding!(
        "rover",
        "approach_radius",
        "WP_RADIUS",
        1.0,
        "m",
        &[],
        "Rover-4.5.7"
    ),
];

pub fn resolve_rtl_parameter<'a>(
    family: &str,
    role: &str,
    available: impl IntoIterator<Item = &'a str>,
) -> Option<&'static ParameterEncoding> {
    let names: Vec<_> = available.into_iter().collect();
    RTL_PARAMETER_ENCODINGS
        .iter()
        .find(|entry| entry.family == family && entry.role == role && names.contains(&entry.name))
}

#[derive(Debug, Clone, Copy, Serialize)]
pub struct GpsSource {
    pub value: u8,
    pub name: &'static str,
    pub requires_uart: bool,
}

// AP_GPS::needs_uart and AP_GPS_Params.cpp (master). Unknown future values
// intentionally have no profile: a GCS must not invent a serial requirement.
pub const GPS_SOURCES: &[GpsSource] = &[
    GpsSource {
        value: 0,
        name: "Disabled",
        requires_uart: false,
    },
    GpsSource {
        value: 1,
        name: "Auto",
        requires_uart: true,
    },
    GpsSource {
        value: 2,
        name: "u-blox",
        requires_uart: true,
    },
    GpsSource {
        value: 5,
        name: "NMEA",
        requires_uart: true,
    },
    GpsSource {
        value: 6,
        name: "SiRF",
        requires_uart: true,
    },
    GpsSource {
        value: 7,
        name: "HIL",
        requires_uart: false,
    },
    GpsSource {
        value: 8,
        name: "SwiftNav",
        requires_uart: true,
    },
    GpsSource {
        value: 9,
        name: "DroneCAN",
        requires_uart: false,
    },
    GpsSource {
        value: 10,
        name: "Septentrio",
        requires_uart: true,
    },
    GpsSource {
        value: 11,
        name: "Trimble",
        requires_uart: true,
    },
    GpsSource {
        value: 13,
        name: "ERB",
        requires_uart: true,
    },
    GpsSource {
        value: 14,
        name: "MAVLink",
        requires_uart: false,
    },
    GpsSource {
        value: 15,
        name: "NOVA",
        requires_uart: true,
    },
    GpsSource {
        value: 16,
        name: "Hemisphere",
        requires_uart: true,
    },
    GpsSource {
        value: 17,
        name: "u-blox moving base",
        requires_uart: true,
    },
    GpsSource {
        value: 18,
        name: "u-blox moving rover",
        requires_uart: true,
    },
    GpsSource {
        value: 19,
        name: "MSP",
        requires_uart: false,
    },
    GpsSource {
        value: 20,
        name: "AllyStar",
        requires_uart: true,
    },
    GpsSource {
        value: 21,
        name: "External AHRS",
        requires_uart: false,
    },
    GpsSource {
        value: 22,
        name: "DroneCAN moving base",
        requires_uart: false,
    },
    GpsSource {
        value: 23,
        name: "DroneCAN moving rover",
        requires_uart: false,
    },
    GpsSource {
        value: 24,
        name: "Unicore",
        requires_uart: true,
    },
    GpsSource {
        value: 25,
        name: "Unicore moving base",
        requires_uart: true,
    },
    GpsSource {
        value: 26,
        name: "Septentrio dual",
        requires_uart: true,
    },
];

pub fn gps_source(value: u8) -> Option<&'static GpsSource> {
    GPS_SOURCES.iter().find(|entry| entry.value == value)
}

pub const PRIMARY_GPS_TYPE_NAMES: &[&str] = &["GPS1_TYPE", "GPS_TYPE"];
pub const PRIMARY_GNSS_MODE_NAMES: &[&str] = &["GPS1_GNSS_MODE", "GPS_GNSS_MODE"];

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn family_and_advertised_names_determine_units_without_metadata() {
        let copter = resolve_rtl_parameter("copter", "return_speed", ["RTL_SPEED"]).unwrap();
        let rover = resolve_rtl_parameter("rover", "return_speed", ["RTL_SPEED"]).unwrap();
        assert_eq!(copter.display_value(500.0), 5.0);
        assert_eq!(rover.display_value(5.0), 5.0);
        let climb = resolve_rtl_parameter("copter", "minimum_climb", ["RTL_CLIMB_MIN"]).unwrap();
        assert_eq!(climb.raw_value(3.0), 300.0);
        let plane = resolve_rtl_parameter("plane", "return_altitude", ["ALT_HOLD_RTL"]).unwrap();
        assert_eq!(plane.display_value(-1.0), -1.0);
        assert_eq!(plane.raw_value(-1.0), -1.0);
        assert_eq!(
            resolve_rtl_parameter("copter", "return_altitude", ["RTL_ALT", "RTL_ALT_M"])
                .unwrap()
                .name,
            "RTL_ALT_M"
        );
    }
    #[test]
    fn non_serial_and_unknown_gps_sources_do_not_invent_uart_requirements() {
        for value in [7, 9, 14, 19, 21, 22, 23] {
            assert!(!gps_source(value).unwrap().requires_uart);
        }
        assert!(gps_source(2).unwrap().requires_uart);
        assert!(gps_source(255).is_none());
    }
}
