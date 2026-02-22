use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct VehicleState {
    pub armed: bool,
    pub custom_mode: u32,
    pub mode_name: String,
    pub system_status: SystemStatus,
    pub vehicle_type: VehicleType,
    pub autopilot: AutopilotType,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Telemetry {
    // Existing
    pub altitude_m: Option<f64>,
    pub speed_mps: Option<f64>,
    pub heading_deg: Option<f64>,
    pub latitude_deg: Option<f64>,
    pub longitude_deg: Option<f64>,
    pub battery_pct: Option<f64>,
    pub gps_fix_type: Option<GpsFixType>,

    // From VFR_HUD
    pub climb_rate_mps: Option<f64>,
    pub throttle_pct: Option<f64>,
    pub airspeed_mps: Option<f64>,

    // From SYS_STATUS
    pub battery_voltage_v: Option<f64>,
    pub battery_current_a: Option<f64>,

    // From GPS_RAW_INT
    pub gps_satellites: Option<u8>,
    pub gps_hdop: Option<f64>,

    // From ATTITUDE
    pub roll_deg: Option<f64>,
    pub pitch_deg: Option<f64>,
    pub yaw_deg: Option<f64>,

    // From NAV_CONTROLLER_OUTPUT
    pub wp_dist_m: Option<f64>,
    pub nav_bearing_deg: Option<f64>,
    pub target_bearing_deg: Option<f64>,
    pub xtrack_error_m: Option<f64>,

    // From TERRAIN_REPORT
    pub terrain_height_m: Option<f64>,
    pub height_above_terrain_m: Option<f64>,

    // From BATTERY_STATUS
    pub battery_voltage_cells: Option<Vec<f64>>,
    pub energy_consumed_wh: Option<f64>,
    pub battery_time_remaining_s: Option<i32>,

    // From RC_CHANNELS
    pub rc_channels: Option<Vec<u16>>,
    pub rc_rssi: Option<u8>,

    // From SERVO_OUTPUT_RAW
    pub servo_outputs: Option<Vec<u16>>,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct MissionState {
    pub current_seq: u16,
    pub total_items: u16,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LinkState {
    Connecting,
    Connected,
    Disconnected,
    Error(String),
}

impl Default for LinkState {
    fn default() -> Self {
        LinkState::Connecting
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct VehicleIdentity {
    pub system_id: u8,
    pub component_id: u8,
    pub autopilot: AutopilotType,
    pub vehicle_type: VehicleType,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct FlightMode {
    pub custom_mode: u32,
    pub name: String,
}

// --- Simple enums mapping from MAVLink values ---

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SystemStatus {
    #[default]
    Unknown,
    Boot,
    Calibrating,
    Standby,
    Active,
    Critical,
    Emergency,
    Poweroff,
}

impl SystemStatus {
    pub(crate) fn from_mav(status: mavlink::common::MavState) -> Self {
        use mavlink::common::MavState;
        match status {
            MavState::MAV_STATE_BOOT => SystemStatus::Boot,
            MavState::MAV_STATE_CALIBRATING => SystemStatus::Calibrating,
            MavState::MAV_STATE_STANDBY => SystemStatus::Standby,
            MavState::MAV_STATE_ACTIVE => SystemStatus::Active,
            MavState::MAV_STATE_CRITICAL => SystemStatus::Critical,
            MavState::MAV_STATE_EMERGENCY => SystemStatus::Emergency,
            MavState::MAV_STATE_POWEROFF => SystemStatus::Poweroff,
            _ => SystemStatus::Unknown,
        }
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum VehicleType {
    #[default]
    Unknown,
    FixedWing,
    Quadrotor,
    Hexarotor,
    Octorotor,
    Tricopter,
    Helicopter,
    Coaxial,
    GroundRover,
    Generic,
}

impl VehicleType {
    pub(crate) fn from_mav(mav_type: mavlink::common::MavType) -> Self {
        use mavlink::common::MavType;
        match mav_type {
            MavType::MAV_TYPE_FIXED_WING => VehicleType::FixedWing,
            MavType::MAV_TYPE_QUADROTOR => VehicleType::Quadrotor,
            MavType::MAV_TYPE_HEXAROTOR => VehicleType::Hexarotor,
            MavType::MAV_TYPE_OCTOROTOR => VehicleType::Octorotor,
            MavType::MAV_TYPE_TRICOPTER => VehicleType::Tricopter,
            MavType::MAV_TYPE_HELICOPTER => VehicleType::Helicopter,
            MavType::MAV_TYPE_COAXIAL => VehicleType::Coaxial,
            MavType::MAV_TYPE_GROUND_ROVER => VehicleType::GroundRover,
            MavType::MAV_TYPE_GENERIC => VehicleType::Generic,
            _ => VehicleType::Unknown,
        }
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AutopilotType {
    #[default]
    Unknown,
    Generic,
    ArduPilotMega,
    Px4,
}

impl AutopilotType {
    pub(crate) fn from_mav(autopilot: mavlink::common::MavAutopilot) -> Self {
        use mavlink::common::MavAutopilot;
        match autopilot {
            MavAutopilot::MAV_AUTOPILOT_GENERIC => AutopilotType::Generic,
            MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA => AutopilotType::ArduPilotMega,
            MavAutopilot::MAV_AUTOPILOT_PX4 => AutopilotType::Px4,
            _ => AutopilotType::Unknown,
        }
    }

    #[allow(dead_code)]
    pub(crate) fn to_mav(self) -> mavlink::common::MavAutopilot {
        use mavlink::common::MavAutopilot;
        match self {
            AutopilotType::Generic => MavAutopilot::MAV_AUTOPILOT_GENERIC,
            AutopilotType::ArduPilotMega => MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
            AutopilotType::Px4 => MavAutopilot::MAV_AUTOPILOT_PX4,
            AutopilotType::Unknown => MavAutopilot::MAV_AUTOPILOT_GENERIC,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StatusMessage {
    pub text: String,
    pub severity: u8,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GpsFixType {
    #[default]
    NoFix,
    Fix2d,
    Fix3d,
    Dgps,
    RtkFloat,
    RtkFixed,
}

impl GpsFixType {
    pub(crate) fn from_raw(fix_type: u8) -> Self {
        match fix_type {
            2 => GpsFixType::Fix2d,
            3 => GpsFixType::Fix3d,
            4 => GpsFixType::Dgps,
            5 => GpsFixType::RtkFloat,
            6 => GpsFixType::RtkFixed,
            _ => GpsFixType::NoFix,
        }
    }
}

/// Internal state for watch channels (writer side).
pub(crate) struct StateWriters {
    pub vehicle_state: tokio::sync::watch::Sender<VehicleState>,
    pub telemetry: tokio::sync::watch::Sender<Telemetry>,
    pub home_position: tokio::sync::watch::Sender<Option<crate::mission::HomePosition>>,
    pub mission_state: tokio::sync::watch::Sender<MissionState>,
    pub link_state: tokio::sync::watch::Sender<LinkState>,
    pub mission_progress: tokio::sync::watch::Sender<Option<crate::mission::TransferProgress>>,
    pub param_store: tokio::sync::watch::Sender<crate::params::ParamStore>,
    pub param_progress: tokio::sync::watch::Sender<crate::params::ParamProgress>,
    pub statustext: tokio::sync::watch::Sender<Option<StatusMessage>>,
}

/// Reader-side channels, cloneable via Arc.
pub(crate) struct StateChannels {
    pub vehicle_state: tokio::sync::watch::Receiver<VehicleState>,
    pub telemetry: tokio::sync::watch::Receiver<Telemetry>,
    pub home_position: tokio::sync::watch::Receiver<Option<crate::mission::HomePosition>>,
    pub mission_state: tokio::sync::watch::Receiver<MissionState>,
    pub link_state: tokio::sync::watch::Receiver<LinkState>,
    pub mission_progress: tokio::sync::watch::Receiver<Option<crate::mission::TransferProgress>>,
    pub param_store: tokio::sync::watch::Receiver<crate::params::ParamStore>,
    pub param_progress: tokio::sync::watch::Receiver<crate::params::ParamProgress>,
    pub statustext: tokio::sync::watch::Receiver<Option<StatusMessage>>,
}

pub(crate) fn create_channels() -> (StateWriters, StateChannels) {
    let (vs_tx, vs_rx) = tokio::sync::watch::channel(VehicleState::default());
    let (telem_tx, telem_rx) = tokio::sync::watch::channel(Telemetry::default());
    let (home_tx, home_rx) = tokio::sync::watch::channel(None);
    let (ms_tx, ms_rx) = tokio::sync::watch::channel(MissionState::default());
    let (ls_tx, ls_rx) = tokio::sync::watch::channel(LinkState::Connecting);
    let (mp_tx, mp_rx) = tokio::sync::watch::channel(None);
    let (ps_tx, ps_rx) = tokio::sync::watch::channel(crate::params::ParamStore::default());
    let (pp_tx, pp_rx) = tokio::sync::watch::channel(crate::params::ParamProgress::default());
    let (st_tx, st_rx) = tokio::sync::watch::channel(None);

    let writers = StateWriters {
        vehicle_state: vs_tx,
        telemetry: telem_tx,
        home_position: home_tx,
        mission_state: ms_tx,
        link_state: ls_tx,
        mission_progress: mp_tx,
        param_store: ps_tx,
        param_progress: pp_tx,
        statustext: st_tx,
    };

    let channels = StateChannels {
        vehicle_state: vs_rx,
        telemetry: telem_rx,
        home_position: home_rx,
        mission_state: ms_rx,
        link_state: ls_rx,
        mission_progress: mp_rx,
        param_store: ps_rx,
        param_progress: pp_rx,
        statustext: st_rx,
    };

    (writers, channels)
}
