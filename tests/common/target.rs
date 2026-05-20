use mavkit::SupportState;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
pub enum VehicleProfile {
    Copter,
    Plane,
    QuadPlane,
}

impl VehicleProfile {
    pub fn sitl_target_name(self) -> &'static str {
        match self {
            Self::Copter => "copter",
            Self::Plane => "plane",
            Self::QuadPlane => "quadplane",
        }
    }

    pub fn from_sitl_target(value: &str) -> Option<Self> {
        match value.trim().to_ascii_lowercase().as_str() {
            "copter" | "arducopter" | "sitl_copter" => Some(Self::Copter),
            "plane" | "arduplane" | "sitl_plane" => Some(Self::Plane),
            "quadplane" | "arduquadplane" | "sitl_quadplane" => Some(Self::QuadPlane),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TestTarget {
    #[allow(dead_code)]
    Sitl(VehicleProfile),
    #[cfg(feature = "sim")]
    #[allow(dead_code)]
    Sim(VehicleProfile),
}

#[derive(Debug, Clone, Copy)]
pub struct HomePositionExpectation {
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    pub tolerance_deg: f64,
    pub set_current_max_drift_deg: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct TelemetryExpectations {
    pub home_position: HomePositionExpectation,
    pub battery_voltage_min_v: f64,
    pub battery_voltage_max_v: f64,
    pub min_satellites: u8,
    pub max_stationary_groundspeed_mps: f64,
    pub max_stationary_roll_pitch_deg: f64,
    pub heading_min_deg: f64,
    pub heading_max_deg: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct RuntimeExpectations {
    pub guided_movement: Option<GuidedMovementExpectation>,
    pub auto_mission: Option<AutoMissionExpectation>,
}

#[derive(Debug, Clone, Copy)]
pub struct GuidedMovementExpectation {
    pub kind: GuidedMovementKind,
    pub north_offset_deg: f64,
    pub relative_alt_m: f64,
    pub position_tolerance_deg: f64,
    pub altitude_tolerance_m: f64,
    pub timeout: std::time::Duration,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
pub enum GuidedMovementKind {
    CopterGoto,
    PlaneReposition,
}

#[derive(Debug, Clone, Copy)]
pub struct AutoMissionExpectation {
    pub takeoff_first: bool,
    pub progress_timeout: std::time::Duration,
    pub completion_timeout: std::time::Duration,
    pub progress_min_index: u16,
    pub progress_min_north_deg: f64,
    pub landing_position_tolerance_deg: f64,
    pub landing_altitude_tolerance_m: f64,
}

#[derive(Debug, Clone, Copy)]
pub enum SupportExpectation {
    AnyResolved,
    Known,
    Exact(SupportState),
}

#[derive(Debug, Clone, Copy)]
pub struct SupportExpectations {
    pub ardupilot: SupportExpectation,
    pub modes: SupportExpectation,
    pub command_int: SupportExpectation,
    pub terrain: SupportExpectation,
    pub mission_fence: SupportExpectation,
    pub mission_rally: SupportExpectation,
}

#[cfg(feature = "sim")]
#[derive(Debug, Clone, Copy)]
#[allow(dead_code)]
pub struct InvalidCustomModeExpectation {
    pub custom_mode: u32,
    pub can_assert_unchanged_state: bool,
}

#[cfg(feature = "sim")]
#[derive(Debug, Clone, Copy)]
#[allow(dead_code)]
pub struct ParamProfileExpectations {
    pub expected_present: &'static [&'static str],
    pub expected_absent: &'static [&'static str],
    pub expected_values: &'static [(&'static str, f32)],
}

#[cfg(feature = "sim")]
#[derive(Debug, Clone, Copy)]
#[allow(dead_code)]
pub struct RawMessageExpectations {
    pub required_message_ids: &'static [u32],
}

#[cfg(feature = "sim")]
#[allow(dead_code)]
const RAW_POWER_OUTPUT_MESSAGE_IDS: &[u32] = &[147, 65, 36];

impl TestTarget {
    #[allow(dead_code)]
    pub const SITL_COPTER: Self = Self::Sitl(VehicleProfile::Copter);

    #[allow(dead_code)]
    pub const SITL_PLANE: Self = Self::Sitl(VehicleProfile::Plane);

    #[allow(dead_code)]
    pub const SITL_QUADPLANE: Self = Self::Sitl(VehicleProfile::QuadPlane);

    #[cfg(feature = "sim")]
    #[allow(dead_code)]
    pub const SIM_COPTER: Self = Self::Sim(VehicleProfile::Copter);

    #[cfg(feature = "sim")]
    #[allow(dead_code)]
    pub const SIM_PLANE: Self = Self::Sim(VehicleProfile::Plane);

    #[cfg(feature = "sim")]
    #[allow(dead_code)]
    pub const SIM_QUADPLANE: Self = Self::Sim(VehicleProfile::QuadPlane);

    pub fn sitl_profile(self) -> Option<VehicleProfile> {
        match self {
            Self::Sitl(profile) => Some(profile),
            #[cfg(feature = "sim")]
            Self::Sim(_) => None,
        }
    }

    pub fn active_sitl_profile() -> Result<VehicleProfile, String> {
        let value = std::env::var("MAVKIT_SITL_TARGET").unwrap_or_else(|_| String::from("copter"));
        VehicleProfile::from_sitl_target(&value).ok_or_else(|| {
            format!(
                "unsupported MAVKIT_SITL_TARGET={value:?}; expected copter, plane, or quadplane"
            )
        })
    }

    #[allow(dead_code)]
    pub fn sitl_skip_reason_for_current_env(self) -> Result<Option<String>, String> {
        let Some(expected) = self.sitl_profile() else {
            return Ok(None);
        };
        let active = Self::active_sitl_profile()?;
        Ok((expected != active).then(|| {
            format!(
                "target requires SITL profile {}, but MAVKIT_SITL_TARGET selects {}",
                expected.sitl_target_name(),
                active.sitl_target_name()
            )
        }))
    }

    pub fn expected_modes(self) -> &'static [(u32, &'static str)] {
        match self {
            Self::Sitl(VehicleProfile::Copter) => {
                &[(0, "STABILIZE"), (4, "GUIDED"), (5, "LOITER"), (6, "RTL")]
            }
            Self::Sitl(VehicleProfile::Plane) => {
                &[(0, "MANUAL"), (10, "AUTO"), (11, "RTL"), (15, "GUIDED")]
            }
            Self::Sitl(VehicleProfile::QuadPlane) => &[
                (10, "AUTO"),
                (15, "GUIDED"),
                (17, "QSTABILIZE"),
                (19, "QLOITER"),
            ],
            #[cfg(feature = "sim")]
            Self::Sim(VehicleProfile::Copter) => {
                &[(0, "STABILIZE"), (4, "GUIDED"), (5, "LOITER"), (6, "RTL")]
            }
            #[cfg(feature = "sim")]
            Self::Sim(VehicleProfile::Plane) => {
                &[(0, "MANUAL"), (10, "AUTO"), (11, "RTL"), (15, "GUIDED")]
            }
            #[cfg(feature = "sim")]
            Self::Sim(VehicleProfile::QuadPlane) => &[
                (10, "AUTO"),
                (15, "GUIDED"),
                (17, "QSTABILIZE"),
                (19, "QLOITER"),
            ],
        }
    }

    pub fn guided_mode(self) -> (u32, &'static str) {
        match self {
            Self::Sitl(VehicleProfile::Copter) => (4, "GUIDED"),
            Self::Sitl(VehicleProfile::Plane | VehicleProfile::QuadPlane) => (15, "GUIDED"),
            #[cfg(feature = "sim")]
            Self::Sim(VehicleProfile::Copter) => (4, "GUIDED"),
            #[cfg(feature = "sim")]
            Self::Sim(VehicleProfile::Plane | VehicleProfile::QuadPlane) => (15, "GUIDED"),
        }
    }

    #[allow(dead_code)]
    pub fn auto_mode(self) -> u32 {
        match self {
            Self::Sitl(VehicleProfile::Copter) => 3,
            Self::Sitl(VehicleProfile::Plane | VehicleProfile::QuadPlane) => 10,
            #[cfg(feature = "sim")]
            Self::Sim(VehicleProfile::Copter) => 3,
            #[cfg(feature = "sim")]
            Self::Sim(VehicleProfile::Plane | VehicleProfile::QuadPlane) => 10,
        }
    }

    pub fn runtime_expectations(self) -> RuntimeExpectations {
        match self {
            Self::Sitl(VehicleProfile::Copter) => RuntimeExpectations {
                guided_movement: Some(GuidedMovementExpectation {
                    kind: GuidedMovementKind::CopterGoto,
                    north_offset_deg: 0.000_03,
                    relative_alt_m: 3.0,
                    position_tolerance_deg: 0.000_02,
                    altitude_tolerance_m: 1.5,
                    timeout: std::time::Duration::from_secs(45),
                }),
                auto_mission: Some(AutoMissionExpectation {
                    takeoff_first: true,
                    progress_timeout: std::time::Duration::from_secs(90),
                    completion_timeout: std::time::Duration::from_secs(180),
                    progress_min_index: 1,
                    progress_min_north_deg: 0.000_01,
                    landing_position_tolerance_deg: 0.000_05,
                    landing_altitude_tolerance_m: 1.0,
                }),
            },
            Self::Sitl(VehicleProfile::Plane | VehicleProfile::QuadPlane) => RuntimeExpectations {
                guided_movement: None,
                auto_mission: None,
            },
            #[cfg(feature = "sim")]
            Self::Sim(VehicleProfile::Copter) => RuntimeExpectations {
                guided_movement: Some(GuidedMovementExpectation {
                    kind: GuidedMovementKind::CopterGoto,
                    north_offset_deg: 0.000_05,
                    relative_alt_m: 3.0,
                    position_tolerance_deg: 1e-9,
                    altitude_tolerance_m: 1e-9,
                    timeout: std::time::Duration::from_secs(20),
                }),
                auto_mission: None,
            },
            #[cfg(feature = "sim")]
            Self::Sim(VehicleProfile::Plane | VehicleProfile::QuadPlane) => RuntimeExpectations {
                guided_movement: Some(GuidedMovementExpectation {
                    kind: GuidedMovementKind::PlaneReposition,
                    north_offset_deg: 0.000_05,
                    relative_alt_m: 0.0,
                    position_tolerance_deg: 1e-9,
                    altitude_tolerance_m: 1e-9,
                    timeout: std::time::Duration::from_secs(8),
                }),
                auto_mission: Some(AutoMissionExpectation {
                    takeoff_first: false,
                    progress_timeout: std::time::Duration::from_secs(12),
                    completion_timeout: std::time::Duration::from_secs(60),
                    progress_min_index: 1,
                    progress_min_north_deg: 0.000_02,
                    landing_position_tolerance_deg: 1e-9,
                    landing_altitude_tolerance_m: 1e-9,
                }),
            },
        }
    }

    pub fn hold_mode(self) -> (u32, &'static str) {
        match self {
            Self::Sitl(VehicleProfile::Copter) => (5, "LOITER"),
            Self::Sitl(VehicleProfile::Plane) => (12, "LOITER"),
            Self::Sitl(VehicleProfile::QuadPlane) => (19, "QLOITER"),
            #[cfg(feature = "sim")]
            Self::Sim(VehicleProfile::Copter) => (5, "LOITER"),
            #[cfg(feature = "sim")]
            Self::Sim(VehicleProfile::Plane) => (12, "LOITER"),
            #[cfg(feature = "sim")]
            Self::Sim(VehicleProfile::QuadPlane) => (19, "QLOITER"),
        }
    }

    pub fn mode_names(self) -> (&'static str, &'static str, &'static str) {
        match self {
            Self::Sitl(VehicleProfile::Copter) => ("STABILIZE", "GUIDED", "LOITER"),
            Self::Sitl(VehicleProfile::Plane) => ("MANUAL", "GUIDED", "LOITER"),
            Self::Sitl(VehicleProfile::QuadPlane) => ("QSTABILIZE", "GUIDED", "QLOITER"),
            #[cfg(feature = "sim")]
            Self::Sim(VehicleProfile::Copter) => ("STABILIZE", "GUIDED", "LOITER"),
            #[cfg(feature = "sim")]
            Self::Sim(VehicleProfile::Plane) => ("MANUAL", "GUIDED", "LOITER"),
            #[cfg(feature = "sim")]
            Self::Sim(VehicleProfile::QuadPlane) => ("QSTABILIZE", "GUIDED", "QLOITER"),
        }
    }

    pub fn telemetry_expectations(self) -> TelemetryExpectations {
        TelemetryExpectations {
            home_position: HomePositionExpectation {
                latitude_deg: 42.3898,
                longitude_deg: -71.1476,
                tolerance_deg: 0.01,
                set_current_max_drift_deg: 0.001,
            },
            battery_voltage_min_v: 5.0,
            battery_voltage_max_v: 60.0,
            min_satellites: 1,
            max_stationary_groundspeed_mps: 1.0,
            max_stationary_roll_pitch_deg: 17.0,
            heading_min_deg: 0.0,
            heading_max_deg: 360.0,
        }
    }

    pub fn support_expectations(self) -> SupportExpectations {
        match self {
            Self::Sitl(_) => SupportExpectations {
                ardupilot: SupportExpectation::Exact(SupportState::Supported),
                modes: SupportExpectation::Known,
                command_int: SupportExpectation::Known,
                terrain: SupportExpectation::AnyResolved,
                mission_fence: SupportExpectation::AnyResolved,
                mission_rally: SupportExpectation::Known,
            },
            #[cfg(feature = "sim")]
            Self::Sim(_) => SupportExpectations {
                ardupilot: SupportExpectation::Exact(SupportState::Supported),
                modes: SupportExpectation::Known,
                command_int: SupportExpectation::Exact(SupportState::Unsupported),
                terrain: SupportExpectation::Exact(SupportState::Unsupported),
                mission_fence: SupportExpectation::Exact(SupportState::Unsupported),
                mission_rally: SupportExpectation::Exact(SupportState::Unsupported),
            },
        }
    }

    #[cfg(feature = "sim")]
    #[allow(dead_code)]
    pub fn param_profile_expectations(self) -> Option<ParamProfileExpectations> {
        match self {
            Self::Sitl(_) => None,
            Self::Sim(VehicleProfile::Copter) => None,
            Self::Sim(VehicleProfile::Plane) => Some(ParamProfileExpectations {
                expected_present: &["AIRSPEED_CRUISE", "RTL_ALTITUDE", "Q_ENABLE"],
                expected_absent: &["WPNAV_SPEED", "Q_FRAME_CLASS", "Q_OPTIONS", "Q_RTL_ALT"],
                expected_values: &[("Q_ENABLE", 0.0)],
            }),
            Self::Sim(VehicleProfile::QuadPlane) => Some(ParamProfileExpectations {
                expected_present: &[
                    "AIRSPEED_CRUISE",
                    "RTL_ALTITUDE",
                    "Q_ENABLE",
                    "Q_FRAME_CLASS",
                    "Q_OPTIONS",
                    "Q_RTL_ALT",
                ],
                expected_absent: &["WPNAV_SPEED"],
                expected_values: &[("Q_ENABLE", 1.0)],
            }),
        }
    }

    #[cfg(feature = "sim")]
    #[allow(dead_code)]
    pub fn raw_power_output_expectations(self) -> Option<RawMessageExpectations> {
        match self {
            Self::Sitl(_) => None,
            Self::Sim(_) => Some(RawMessageExpectations {
                required_message_ids: RAW_POWER_OUTPUT_MESSAGE_IDS,
            }),
        }
    }

    #[cfg(feature = "sim")]
    #[allow(dead_code)]
    pub fn invalid_custom_mode_expectation(self) -> Option<InvalidCustomModeExpectation> {
        match self {
            Self::Sitl(_) => None,
            Self::Sim(_) => Some(InvalidCustomModeExpectation {
                custom_mode: 999,
                can_assert_unchanged_state: true,
            }),
        }
    }
}
