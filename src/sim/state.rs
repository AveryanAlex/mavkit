use std::collections::BTreeMap;

use mavlink::MavHeader;
use tokio::sync::{mpsc, oneshot, watch};

use crate::dialect;
use crate::error::VehicleError;
use crate::geo::{GeoPoint3dMsl, try_latitude_e7, try_longitude_e7};
use crate::mission::{HomePosition, MissionType};

use super::api::{DemoClock, DemoProfile, DemoVehicleSnapshot};
use super::params::{SimParam, seeded_params};
use super::power::PowerState;

pub(crate) const DEFAULT_SYSTEM_ID: u8 = 1;
pub(crate) const DEFAULT_COMPONENT_ID: u8 = 1;
pub(crate) const BROADCAST_ID: u8 = 0;
pub(crate) const DEFAULT_TICK_HZ: u32 = 2;
pub(crate) const DEFAULT_HOME_LAT_DEG: f64 = 42.3898;
pub(crate) const DEFAULT_HOME_LON_DEG: f64 = -71.1476;
pub(crate) const DEFAULT_HOME_ALT_M: f64 = 60.0;
pub(crate) const AVAILABLE_MODES_MESSAGE_ID: u32 = 435;
pub(crate) const AUTOPILOT_VERSION_MESSAGE_ID: u32 = 148;
pub(crate) const GPS_GLOBAL_ORIGIN_MESSAGE_ID: u32 = 49;
pub(crate) const HOME_POSITION_MESSAGE_ID: u32 = 242;
pub(crate) const ACCEPTANCE_RADIUS_M: f64 = 2.0;
pub(crate) const TAKEOFF_ALT_MARGIN_M: f64 = 0.5;

pub(crate) const COPTER_AUTO_MODE: u32 = 3;
pub(crate) const COPTER_GUIDED_MODE: u32 = 4;
pub(crate) const COPTER_LOITER_MODE: u32 = 5;
pub(crate) const PLANE_AUTO_MODE: u32 = 10;
pub(crate) const PLANE_LOITER_MODE: u32 = 12;
pub(crate) const PLANE_GUIDED_MODE: u32 = 15;
pub(crate) const QUADPLANE_QSTABILIZE_MODE: u32 = 17;
pub(crate) const QUADPLANE_QLOITER_MODE: u32 = 19;

#[derive(Debug, Clone)]
pub(crate) struct DemoVehicleConfig {
    pub(crate) profile: DemoProfile,
    pub(crate) clock: DemoClock,
    pub(crate) tick_hz: u32,
    pub(crate) home: HomePosition,
}

#[derive(Debug)]
pub(crate) enum ControlMessage {
    Step {
        reply: oneshot::Sender<Result<DemoVehicleSnapshot, VehicleError>>,
    },
    TeleportTo {
        target: TeleportTarget,
        reply: oneshot::Sender<Result<DemoVehicleSnapshot, VehicleError>>,
    },
    Shutdown {
        reply: oneshot::Sender<Result<(), VehicleError>>,
    },
}

#[derive(Debug, Clone)]
pub(crate) struct MissionStores {
    pub(crate) mission: Vec<dialect::MISSION_ITEM_INT_DATA>,
    pub(crate) fence: Vec<dialect::MISSION_ITEM_INT_DATA>,
    pub(crate) rally: Vec<dialect::MISSION_ITEM_INT_DATA>,
}

#[derive(Debug, Clone)]
pub(crate) struct PendingMissionUpload {
    pub(crate) mission_type: MissionType,
    pub(crate) expected_count: u16,
    pub(crate) items: Vec<dialect::MISSION_ITEM_INT_DATA>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum NavSource {
    Guided,
    Mission,
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct NavTarget {
    pub(crate) source: NavSource,
    pub(crate) latitude_deg: f64,
    pub(crate) longitude_deg: f64,
    pub(crate) altitude_msl_m: f64,
    pub(crate) acceptance_radius_m: f64,
    pub(crate) wire_seq: Option<u16>,
    pub(crate) disarm_on_reach: bool,
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct VelocityNed {
    pub(crate) north_mps: f64,
    pub(crate) east_mps: f64,
    pub(crate) down_mps: f64,
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct TeleportTarget {
    pub(crate) latitude_e7: i32,
    pub(crate) longitude_e7: i32,
    pub(crate) altitude_mm: i32,
}

impl TryFrom<GeoPoint3dMsl> for TeleportTarget {
    type Error = VehicleError;

    fn try_from(position: GeoPoint3dMsl) -> Result<Self, Self::Error> {
        Ok(Self {
            latitude_e7: try_latitude_e7(position.latitude_deg)?,
            longitude_e7: try_longitude_e7(position.longitude_deg)?,
            altitude_mm: altitude_msl_to_mm(position.altitude_msl_m)?,
        })
    }
}

fn altitude_msl_to_mm(value: f64) -> Result<i32, VehicleError> {
    if !value.is_finite() {
        return Err(VehicleError::InvalidParameter(format!(
            "altitude_msl_m must be finite, got {value}"
        )));
    }

    let scaled = (value * 1000.0).round();
    if !(i32::MIN as f64..=i32::MAX as f64).contains(&scaled) {
        return Err(VehicleError::InvalidParameter(format!(
            "altitude_msl_m {value} m overflows i32 millimeter range"
        )));
    }

    Ok(scaled as i32)
}

pub(crate) struct SimulatorCore {
    pub(crate) config: DemoVehicleConfig,
    pub(crate) snapshot: DemoVehicleSnapshot,
    pub(crate) outbound_tx: mpsc::Sender<(MavHeader, dialect::MavMessage)>,
    pub(crate) snapshot_tx: watch::Sender<DemoVehicleSnapshot>,
    pub(crate) boot_time_us: u64,
    // Carries the fractional tick duration lost when converting tick_hz to microseconds.
    pub(crate) tick_time_remainder: u64,
    pub(crate) header_sequence: u8,
    pub(crate) reply_target_system_id: u8,
    pub(crate) reply_target_component_id: u8,
    pub(crate) stream_schedules: BTreeMap<u32, StreamSchedule>,
    pub(crate) params: Vec<SimParam>,
    pub(crate) missions: MissionStores,
    pub(crate) pending_upload: Option<PendingMissionUpload>,
    pub(crate) nav_target: Option<NavTarget>,
    pub(crate) velocity: VelocityNed,
    pub(crate) power: PowerState,
    pub(crate) mission_completed: bool,
    pub(crate) pending_reached_wire_seq: Option<u16>,
    pub(crate) mission_speed_override_mps: Option<f64>,
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct StreamSchedule {
    pub(crate) interval_us: i64,
    pub(crate) elapsed_us: u64,
}

impl StreamSchedule {
    pub(crate) fn new(interval_us: i64) -> Self {
        Self {
            interval_us,
            elapsed_us: 0,
        }
    }

    pub(crate) fn is_enabled(self) -> bool {
        self.interval_us > 0
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum DemoVehicleFamily {
    Copter,
    Plane,
    QuadPlane,
}

impl SimulatorCore {
    pub(crate) fn new(
        config: DemoVehicleConfig,
        outbound_tx: mpsc::Sender<(MavHeader, dialect::MavMessage)>,
        snapshot_tx: watch::Sender<DemoVehicleSnapshot>,
        initial_snapshot: DemoVehicleSnapshot,
    ) -> Self {
        let profile = config.profile;
        let boot_time_us = u64::from(initial_snapshot.time_boot_ms) * 1000;
        Self {
            config,
            snapshot: initial_snapshot,
            outbound_tx,
            snapshot_tx,
            boot_time_us,
            tick_time_remainder: 0,
            header_sequence: 0,
            reply_target_system_id: BROADCAST_ID,
            reply_target_component_id: BROADCAST_ID,
            stream_schedules: BTreeMap::new(),
            params: seeded_params(profile),
            missions: MissionStores {
                mission: Vec::new(),
                fence: Vec::new(),
                rally: Vec::new(),
            },
            pending_upload: None,
            nav_target: None,
            velocity: VelocityNed {
                north_mps: 0.0,
                east_mps: 0.0,
                down_mps: 0.0,
            },
            power: PowerState::default(),
            mission_completed: false,
            pending_reached_wire_seq: None,
            mission_speed_override_mps: None,
        }
    }

    pub(crate) fn publish_snapshot(&self) {
        let _ = self.snapshot_tx.send_replace(self.snapshot.clone());
    }

    pub(crate) fn mission_items(
        &self,
        mission_type: MissionType,
    ) -> &Vec<dialect::MISSION_ITEM_INT_DATA> {
        match mission_type {
            MissionType::Mission => &self.missions.mission,
            MissionType::Fence => &self.missions.fence,
            MissionType::Rally => &self.missions.rally,
        }
    }

    pub(crate) fn replace_mission_items(
        &mut self,
        mission_type: MissionType,
        items: Vec<dialect::MISSION_ITEM_INT_DATA>,
    ) {
        let total = items.len() as u16;
        match mission_type {
            MissionType::Mission => self.missions.mission = items,
            MissionType::Fence => self.missions.fence = items,
            MissionType::Rally => self.missions.rally = items,
        }

        if mission_type == MissionType::Mission {
            self.snapshot.mission_total_wire_items = total;
            self.pending_reached_wire_seq = None;
            if total == 0 || self.snapshot.mission_current_wire_seq >= total {
                self.snapshot.mission_current_wire_seq = 0;
            }
            self.publish_snapshot();
        }
    }

    pub(crate) fn set_reply_target(&mut self, header: MavHeader) {
        self.reply_target_system_id = header.system_id;
        self.reply_target_component_id = header.component_id;
    }

    pub(crate) fn targets_this_system(target_system: u8) -> bool {
        target_system == BROADCAST_ID || target_system == DEFAULT_SYSTEM_ID
    }

    pub(crate) fn targets_this_vehicle(target_system: u8, target_component: u8) -> bool {
        Self::targets_this_system(target_system)
            && (target_component == BROADCAST_ID || target_component == DEFAULT_COMPONENT_ID)
    }
}

pub(crate) fn default_mode(profile: DemoProfile) -> u32 {
    match profile {
        DemoProfile::ArduCopter => COPTER_LOITER_MODE,
        DemoProfile::ArduPlane => PLANE_LOITER_MODE,
        DemoProfile::ArduQuadPlane => QUADPLANE_QLOITER_MODE,
    }
}

pub(crate) fn profile_modes(profile: DemoProfile) -> &'static [(u32, &'static str)] {
    match profile {
        DemoProfile::ArduCopter => &[
            (0, "STABILIZE"),
            (1, "ACRO"),
            (2, "ALT_HOLD"),
            (3, "AUTO"),
            (4, "GUIDED"),
            (5, "LOITER"),
            (6, "RTL"),
            (7, "CIRCLE"),
            (9, "LAND"),
            (21, "SMART_RTL"),
            (27, "AUTO_RTL"),
        ],
        DemoProfile::ArduPlane => &[
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
        ],
        DemoProfile::ArduQuadPlane => &[
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
        ],
    }
}

pub(crate) fn is_supported_custom_mode(profile: DemoProfile, custom_mode: u32) -> bool {
    profile_modes(profile)
        .iter()
        .any(|(mode, _name)| *mode == custom_mode)
}

pub(crate) fn profile_vehicle_type(profile: DemoProfile) -> dialect::MavType {
    match profile {
        DemoProfile::ArduCopter => dialect::MavType::MAV_TYPE_QUADROTOR,
        DemoProfile::ArduPlane => dialect::MavType::MAV_TYPE_FIXED_WING,
        DemoProfile::ArduQuadPlane => dialect::MavType::MAV_TYPE_VTOL_FIXEDROTOR,
    }
}

pub(crate) fn profile_vehicle_family(profile: DemoProfile) -> DemoVehicleFamily {
    match profile {
        DemoProfile::ArduCopter => DemoVehicleFamily::Copter,
        DemoProfile::ArduPlane => DemoVehicleFamily::Plane,
        DemoProfile::ArduQuadPlane => DemoVehicleFamily::QuadPlane,
    }
}

pub(crate) fn auto_mode(profile: DemoProfile) -> u32 {
    match profile {
        DemoProfile::ArduCopter => COPTER_AUTO_MODE,
        DemoProfile::ArduPlane | DemoProfile::ArduQuadPlane => PLANE_AUTO_MODE,
    }
}

pub(crate) fn guided_mode(profile: DemoProfile) -> u32 {
    match profile {
        DemoProfile::ArduCopter => COPTER_GUIDED_MODE,
        DemoProfile::ArduPlane | DemoProfile::ArduQuadPlane => PLANE_GUIDED_MODE,
    }
}

pub(crate) fn is_hover_hold_mode(profile: DemoProfile, custom_mode: u32) -> bool {
    match profile {
        DemoProfile::ArduCopter => custom_mode == COPTER_LOITER_MODE,
        DemoProfile::ArduPlane => false,
        DemoProfile::ArduQuadPlane => {
            matches!(
                custom_mode,
                QUADPLANE_QLOITER_MODE | QUADPLANE_QSTABILIZE_MODE
            )
        }
    }
}

pub(crate) fn health_sensors() -> dialect::MavSysStatusSensor {
    dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_GYRO
        | dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_ACCEL
        | dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_MAG
        | dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE
        | dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_GPS
        | dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_RC_RECEIVER
        | dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_BATTERY
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mission_type_roundtrip_defaults_unknown_to_mission() {
        assert_eq!(
            super::super::mission::from_mav_mission_type(
                dialect::MavMissionType::MAV_MISSION_TYPE_ALL,
            ),
            MissionType::Mission
        );
    }

    #[test]
    fn plane_catalog_does_not_expose_quadplane_q_modes() {
        let plane_modes = profile_modes(DemoProfile::ArduPlane);

        assert!(
            !plane_modes
                .iter()
                .any(|(_mode, name)| name.starts_with('Q'))
        );
        assert!(!is_supported_custom_mode(
            DemoProfile::ArduPlane,
            QUADPLANE_QLOITER_MODE
        ));
    }

    #[test]
    fn quadplane_catalog_exposes_q_modes() {
        let quadplane_modes = profile_modes(DemoProfile::ArduQuadPlane);

        assert!(
            quadplane_modes.iter().any(|(mode, name)| {
                *mode == QUADPLANE_QSTABILIZE_MODE && *name == "QSTABILIZE"
            })
        );
        assert!(
            quadplane_modes
                .iter()
                .any(|(mode, name)| *mode == QUADPLANE_QLOITER_MODE && *name == "QLOITER")
        );
    }
}
