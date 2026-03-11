use serde::{Deserialize, Serialize};

/// High-level vehicle state derived from MAVLink HEARTBEAT messages.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct VehicleState {
    pub armed: bool,
    pub custom_mode: u32,
    pub mode_name: String,
    pub system_status: SystemStatus,
    pub vehicle_type: VehicleType,
    pub autopilot: AutopilotType,
    pub system_id: u8,
    pub component_id: u8,
    pub heartbeat_received: bool,
}

// ---------------------------------------------------------------------------
// Telemetry sub-group types
// ---------------------------------------------------------------------------

/// Position and velocity data (from VFR_HUD + GLOBAL_POSITION_INT).
///
/// Subscribe via [`Vehicle::position()`] to receive only position-related updates.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Position {
    pub latitude_deg: Option<f64>,
    pub longitude_deg: Option<f64>,
    pub altitude_m: Option<f64>,
    pub speed_mps: Option<f64>,
    pub airspeed_mps: Option<f64>,
    pub climb_rate_mps: Option<f64>,
    pub heading_deg: Option<f64>,
    pub throttle_pct: Option<f64>,
}

/// Vehicle orientation (from ATTITUDE).
///
/// Subscribe via [`Vehicle::attitude()`] to receive only attitude updates.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Attitude {
    pub roll_deg: Option<f64>,
    pub pitch_deg: Option<f64>,
    pub yaw_deg: Option<f64>,
}

/// Battery power status (from SYS_STATUS + BATTERY_STATUS).
///
/// Subscribe via [`Vehicle::battery()`] to receive only battery updates.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Battery {
    pub remaining_pct: Option<f64>,
    pub voltage_v: Option<f64>,
    pub current_a: Option<f64>,
    pub voltage_cells: Option<Vec<f64>>,
    pub energy_consumed_wh: Option<f64>,
    pub time_remaining_s: Option<i32>,
}

/// GPS quality information (from GPS_RAW_INT).
///
/// Subscribe via [`Vehicle::gps()`] to receive only GPS updates.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Gps {
    pub fix_type: Option<GpsFixType>,
    pub satellites: Option<u8>,
    pub hdop: Option<f64>,
}

/// Navigation controller output (from NAV_CONTROLLER_OUTPUT).
///
/// Subscribe via [`Vehicle::navigation()`] to receive only navigation updates.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Navigation {
    pub wp_dist_m: Option<f64>,
    pub nav_bearing_deg: Option<f64>,
    pub target_bearing_deg: Option<f64>,
    pub xtrack_error_m: Option<f64>,
}

/// Terrain data (from TERRAIN_REPORT).
///
/// Subscribe via [`Vehicle::terrain()`] to receive only terrain updates.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Terrain {
    pub terrain_height_m: Option<f64>,
    pub height_above_terrain_m: Option<f64>,
}

/// RC channels and servo outputs (from RC_CHANNELS + SERVO_OUTPUT_RAW).
///
/// Subscribe via [`Vehicle::rc_channels()`] to receive only RC/servo updates.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct RcChannels {
    pub channels: Option<Vec<u16>>,
    pub rssi: Option<u8>,
    pub servo_outputs: Option<Vec<u16>>,
}

/// Telemetry data aggregated from multiple MAVLink messages.
///
/// This flat struct contains all telemetry fields and notifies on any change.
/// For fine-grained subscriptions, use the per-domain watch channels:
/// [`Vehicle::position()`], [`Vehicle::attitude()`], [`Vehicle::battery()`], etc.
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

/// Current mission execution state reported by the autopilot.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct MissionState {
    pub current_seq: u16,
    pub total_items: u16,
}

/// Connection lifecycle state.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LinkState {
    #[default]
    Connecting,
    Connected,
    Disconnected,
    Error(String),
}

/// Identifies the connected vehicle (system/component IDs and firmware type).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct VehicleIdentity {
    pub system_id: u8,
    pub component_id: u8,
    pub autopilot: AutopilotType,
    pub vehicle_type: VehicleType,
}

/// A named flight mode with its numeric custom_mode value.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct FlightMode {
    pub custom_mode: u32,
    pub name: String,
}

// --- Simple enums mapping from MAVLink values ---

/// MAVLink system status.
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

/// MAVLink vehicle airframe type.
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

/// MAVLink autopilot firmware type.
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
}

/// MAVLink message severity level (MAV_SEVERITY).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MavSeverity {
    Emergency,
    Alert,
    Critical,
    Error,
    Warning,
    Notice,
    Info,
    Debug,
}

impl MavSeverity {
    pub(crate) fn from_mav(severity: mavlink::common::MavSeverity) -> Self {
        use mavlink::common::MavSeverity as MS;
        match severity {
            MS::MAV_SEVERITY_EMERGENCY => MavSeverity::Emergency,
            MS::MAV_SEVERITY_ALERT => MavSeverity::Alert,
            MS::MAV_SEVERITY_CRITICAL => MavSeverity::Critical,
            MS::MAV_SEVERITY_ERROR => MavSeverity::Error,
            MS::MAV_SEVERITY_WARNING => MavSeverity::Warning,
            MS::MAV_SEVERITY_NOTICE => MavSeverity::Notice,
            MS::MAV_SEVERITY_INFO => MavSeverity::Info,
            MS::MAV_SEVERITY_DEBUG => MavSeverity::Debug,
        }
    }
}

impl std::fmt::Display for MavSeverity {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let label = match self {
            MavSeverity::Emergency => "EMERGENCY",
            MavSeverity::Alert => "ALERT",
            MavSeverity::Critical => "CRITICAL",
            MavSeverity::Error => "ERROR",
            MavSeverity::Warning => "WARNING",
            MavSeverity::Notice => "NOTICE",
            MavSeverity::Info => "INFO",
            MavSeverity::Debug => "DEBUG",
        };
        f.write_str(label)
    }
}

/// A STATUSTEXT message received from the autopilot.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StatusMessage {
    pub text: String,
    pub severity: MavSeverity,
}

/// GPS fix quality level.
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

// ---------------------------------------------------------------------------
// Sensor health from SYS_STATUS bitmasks
// ---------------------------------------------------------------------------

/// Identifies a sensor subsystem reported in SYS_STATUS bitmasks.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SensorId {
    Gyro3d,
    Accel3d,
    Mag3d,
    AbsolutePressure,
    Gps,
    OpticalFlow,
    RangeFinder,
    ExternalGroundTruth,
    MotorOutputs,
    RcReceiver,
    PrearmCheck,
    Ahrs,
    Terrain,
    ReverseMotor,
    Logging,
    Battery,
}

impl SensorId {
    /// Sensor IDs and their MAV_SYS_STATUS_SENSOR bitmask values.
    const ALL: &[(SensorId, u32)] = &[
        (SensorId::Gyro3d, 0x01),
        (SensorId::Accel3d, 0x02),
        (SensorId::Mag3d, 0x04),
        (SensorId::AbsolutePressure, 0x08),
        (SensorId::Gps, 0x20),
        (SensorId::OpticalFlow, 0x40),
        (SensorId::RangeFinder, 0x100),
        (SensorId::ExternalGroundTruth, 0x200),
        (SensorId::MotorOutputs, 0x8000),
        (SensorId::RcReceiver, 0x1_0000),
        (SensorId::Ahrs, 0x20_0000),
        (SensorId::Terrain, 0x40_0000),
        (SensorId::ReverseMotor, 0x80_0000),
        (SensorId::Logging, 0x100_0000),
        (SensorId::Battery, 0x200_0000),
        (SensorId::PrearmCheck, 0x1000_0000),
    ];
}

/// Status of a single sensor subsystem.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SensorStatus {
    /// Sensor is present and healthy.
    Healthy,
    /// Sensor is present and enabled but reporting an error.
    Unhealthy,
    /// Sensor is present but not enabled.
    Disabled,
    /// Sensor is not present on this vehicle.
    NotPresent,
}

/// Aggregated sensor health decoded from SYS_STATUS bitmasks.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct SensorHealth {
    pub sensors: Vec<(SensorId, SensorStatus)>,
    /// Convenience: true when prearm check bit is healthy (bit 21).
    pub pre_arm_good: bool,
}

impl SensorHealth {
    /// Decode the three SYS_STATUS bitmask fields into structured sensor health.
    pub fn from_bitmasks(present: u32, enabled: u32, health: u32) -> Self {
        let mut sensors = Vec::new();
        let mut pre_arm_good = false;

        for &(sensor_id, bit) in SensorId::ALL {
            let is_present = present & bit != 0;
            let is_enabled = enabled & bit != 0;
            let is_healthy = health & bit != 0;

            if !is_present {
                continue; // Skip sensors not present on this vehicle
            }

            let status = if !is_enabled {
                SensorStatus::Disabled
            } else if is_healthy {
                SensorStatus::Healthy
            } else {
                SensorStatus::Unhealthy
            };

            if sensor_id == SensorId::PrearmCheck {
                pre_arm_good = is_healthy;
            }

            sensors.push((sensor_id, status));
        }

        SensorHealth {
            sensors,
            pre_arm_good,
        }
    }
}

// ---------------------------------------------------------------------------
// Mag calibration types
// ---------------------------------------------------------------------------

/// Status of a magnetometer calibration.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MagCalStatus {
    #[default]
    NotStarted,
    WaitingToStart,
    RunningStepOne,
    RunningStepTwo,
    Success,
    Failed,
    BadOrientation,
    BadRadius,
}

impl MagCalStatus {
    pub(crate) fn from_mav(status: mavlink::common::MagCalStatus) -> Self {
        use mavlink::common::MagCalStatus as MCS;
        match status {
            MCS::MAG_CAL_NOT_STARTED => MagCalStatus::NotStarted,
            MCS::MAG_CAL_WAITING_TO_START => MagCalStatus::WaitingToStart,
            MCS::MAG_CAL_RUNNING_STEP_ONE => MagCalStatus::RunningStepOne,
            MCS::MAG_CAL_RUNNING_STEP_TWO => MagCalStatus::RunningStepTwo,
            MCS::MAG_CAL_SUCCESS => MagCalStatus::Success,
            MCS::MAG_CAL_FAILED => MagCalStatus::Failed,
            MCS::MAG_CAL_BAD_ORIENTATION => MagCalStatus::BadOrientation,
            MCS::MAG_CAL_BAD_RADIUS => MagCalStatus::BadRadius,
        }
    }
}

/// Progress of a magnetometer calibration in progress.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct MagCalProgress {
    pub compass_id: u8,
    pub completion_pct: u8,
    pub status: MagCalStatus,
    pub attempt: u8,
}

/// Final report from a magnetometer calibration.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct MagCalReport {
    pub compass_id: u8,
    pub status: MagCalStatus,
    pub fitness: f32,
    pub ofs_x: f32,
    pub ofs_y: f32,
    pub ofs_z: f32,
    pub autosaved: bool,
}

/// Set a field and return whether it changed. Used with `send_if_modified`.
pub(crate) fn set_if_changed<T: PartialEq>(field: &mut T, value: T) -> bool {
    if *field != value {
        *field = value;
        true
    } else {
        false
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
    pub sensor_health: tokio::sync::watch::Sender<SensorHealth>,
    // MAG_CAL_PROGRESS (msg 191) is ardupilotmega-only; populated when dialect is added
    #[allow(dead_code)]
    pub mag_cal_progress: tokio::sync::watch::Sender<Option<MagCalProgress>>,
    pub mag_cal_report: tokio::sync::watch::Sender<Option<MagCalReport>>,
    // Per-domain telemetry channels
    pub position: tokio::sync::watch::Sender<Position>,
    pub attitude: tokio::sync::watch::Sender<Attitude>,
    pub battery: tokio::sync::watch::Sender<Battery>,
    pub gps: tokio::sync::watch::Sender<Gps>,
    pub navigation: tokio::sync::watch::Sender<Navigation>,
    pub terrain: tokio::sync::watch::Sender<Terrain>,
    pub rc_channels: tokio::sync::watch::Sender<RcChannels>,
    pub raw_message_tx:
        tokio::sync::broadcast::Sender<(mavlink::MavHeader, mavlink::common::MavMessage)>,
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
    pub sensor_health: tokio::sync::watch::Receiver<SensorHealth>,
    pub mag_cal_progress: tokio::sync::watch::Receiver<Option<MagCalProgress>>,
    pub mag_cal_report: tokio::sync::watch::Receiver<Option<MagCalReport>>,
    // Per-domain telemetry channels
    pub position: tokio::sync::watch::Receiver<Position>,
    pub attitude: tokio::sync::watch::Receiver<Attitude>,
    pub battery: tokio::sync::watch::Receiver<Battery>,
    pub gps: tokio::sync::watch::Receiver<Gps>,
    pub navigation: tokio::sync::watch::Receiver<Navigation>,
    pub terrain: tokio::sync::watch::Receiver<Terrain>,
    pub rc_channels: tokio::sync::watch::Receiver<RcChannels>,
    pub raw_message_tx:
        tokio::sync::broadcast::Sender<(mavlink::MavHeader, mavlink::common::MavMessage)>,
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
    let (sh_tx, sh_rx) = tokio::sync::watch::channel(SensorHealth::default());
    let (mcp_tx, mcp_rx) = tokio::sync::watch::channel(None);
    let (mcr_tx, mcr_rx) = tokio::sync::watch::channel(None);
    // Per-domain telemetry channels
    let (pos_tx, pos_rx) = tokio::sync::watch::channel(Position::default());
    let (att_tx, att_rx) = tokio::sync::watch::channel(Attitude::default());
    let (bat_tx, bat_rx) = tokio::sync::watch::channel(Battery::default());
    let (gps_tx, gps_rx) = tokio::sync::watch::channel(Gps::default());
    let (nav_tx, nav_rx) = tokio::sync::watch::channel(Navigation::default());
    let (ter_tx, ter_rx) = tokio::sync::watch::channel(Terrain::default());
    let (rc_tx, rc_rx) = tokio::sync::watch::channel(RcChannels::default());
    let (raw_msg_tx, _) = tokio::sync::broadcast::channel(256);

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
        sensor_health: sh_tx,
        mag_cal_progress: mcp_tx,
        mag_cal_report: mcr_tx,
        position: pos_tx,
        attitude: att_tx,
        battery: bat_tx,
        gps: gps_tx,
        navigation: nav_tx,
        terrain: ter_tx,
        rc_channels: rc_tx,
        raw_message_tx: raw_msg_tx.clone(),
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
        sensor_health: sh_rx,
        mag_cal_progress: mcp_rx,
        mag_cal_report: mcr_rx,
        position: pos_rx,
        attitude: att_rx,
        battery: bat_rx,
        gps: gps_rx,
        navigation: nav_rx,
        terrain: ter_rx,
        rc_channels: rc_rx,
        raw_message_tx: raw_msg_tx,
    };

    (writers, channels)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sensor_health_all_healthy() {
        let present = 0x01 | 0x02 | 0x04 | 0x20 | 0x1000_0000;
        let enabled = present;
        let health = present;
        let sh = SensorHealth::from_bitmasks(present, enabled, health);
        assert!(sh.pre_arm_good);
        for &(_, status) in &sh.sensors {
            assert_eq!(status, SensorStatus::Healthy);
        }
        assert_eq!(sh.sensors.len(), 5);
    }

    #[test]
    fn sensor_health_gps_unhealthy() {
        let present = (1 << 0) | (1 << 5); // gyro + gps
        let enabled = present;
        let health = 1 << 0; // only gyro healthy, gps unhealthy
        let sh = SensorHealth::from_bitmasks(present, enabled, health);
        let gps = sh
            .sensors
            .iter()
            .find(|(id, _)| *id == SensorId::Gps)
            .unwrap();
        assert_eq!(gps.1, SensorStatus::Unhealthy);
        let gyro = sh
            .sensors
            .iter()
            .find(|(id, _)| *id == SensorId::Gyro3d)
            .unwrap();
        assert_eq!(gyro.1, SensorStatus::Healthy);
    }

    #[test]
    fn sensor_health_compass_not_present() {
        let present = (1 << 0) | (1 << 1); // gyro + accel, no compass
        let enabled = present;
        let health = present;
        let sh = SensorHealth::from_bitmasks(present, enabled, health);
        assert!(
            !sh.sensors.iter().any(|(id, _)| *id == SensorId::Mag3d),
            "compass should not appear when not present"
        );
        assert_eq!(sh.sensors.len(), 2);
    }

    #[test]
    fn sensor_health_prearm_failing() {
        let present = 0x1000_0000 | 0x01;
        let enabled = present;
        let health = 0x01; // gyro healthy, prearm unhealthy
        let sh = SensorHealth::from_bitmasks(present, enabled, health);
        assert!(!sh.pre_arm_good);
        let prearm = sh
            .sensors
            .iter()
            .find(|(id, _)| *id == SensorId::PrearmCheck)
            .unwrap();
        assert_eq!(prearm.1, SensorStatus::Unhealthy);
    }

    #[test]
    fn sensor_health_disabled_sensor() {
        let present = 1 << 6; // optical flow present
        let enabled = 0; // but not enabled
        let health = 0;
        let sh = SensorHealth::from_bitmasks(present, enabled, health);
        let of = sh
            .sensors
            .iter()
            .find(|(id, _)| *id == SensorId::OpticalFlow)
            .unwrap();
        assert_eq!(of.1, SensorStatus::Disabled);
    }
}
