use std::collections::BTreeMap;

use mavlink::MavHeader;
use tokio::sync::{mpsc, oneshot, watch};

use crate::dialect;
use crate::error::VehicleError;
use crate::mission::{HomePosition, MissionType};

use super::api::{DemoClock, DemoProfile, DemoVehicleSnapshot};
use super::params::{SimParam, seeded_params};
use super::power::PowerState;

pub(crate) const DEFAULT_SYSTEM_ID: u8 = 1;
pub(crate) const DEFAULT_COMPONENT_ID: u8 = 1;
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

pub(crate) struct SimulatorCore {
    pub(crate) config: DemoVehicleConfig,
    pub(crate) snapshot: DemoVehicleSnapshot,
    pub(crate) outbound_tx: mpsc::Sender<(MavHeader, dialect::MavMessage)>,
    pub(crate) snapshot_tx: watch::Sender<DemoVehicleSnapshot>,
    pub(crate) header_sequence: u8,
    pub(crate) stream_intervals_us: BTreeMap<u32, i32>,
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
        Self {
            config,
            snapshot: initial_snapshot,
            outbound_tx,
            snapshot_tx,
            header_sequence: 0,
            stream_intervals_us: BTreeMap::new(),
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
        DemoProfile::ArduPlane | DemoProfile::ArduQuadPlane => &[
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
}
