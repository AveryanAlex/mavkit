use crate::dialect;
use crate::error::VehicleError;
use crate::mission::MissionType;

use super::state::{
    DEFAULT_COMPONENT_ID, DEFAULT_SYSTEM_ID, NavSource, PendingMissionUpload, SimulatorCore,
    auto_mode,
};

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum RuntimeMissionItem {
    Navigation,
    DoChangeSpeed { speed_mps: Option<f64> },
    Unsupported { command: dialect::MavCmd },
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
struct MissionRuntimeState {
    speed_override_mps: Option<f64>,
}

impl RuntimeMissionItem {
    fn from_wire(item: &dialect::MISSION_ITEM_INT_DATA) -> Self {
        match item.command {
            dialect::MavCmd::MAV_CMD_NAV_WAYPOINT
            | dialect::MavCmd::MAV_CMD_NAV_TAKEOFF
            | dialect::MavCmd::MAV_CMD_NAV_LAND
            | dialect::MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH => Self::Navigation,
            dialect::MavCmd::MAV_CMD_DO_CHANGE_SPEED => Self::DoChangeSpeed {
                speed_mps: (item.param2.is_finite() && item.param2 > 0.0)
                    .then_some(f64::from(item.param2)),
            },
            command => Self::Unsupported { command },
        }
    }
}

impl MissionRuntimeState {
    fn apply(&mut self, item: RuntimeMissionItem) {
        if let RuntimeMissionItem::DoChangeSpeed { speed_mps } = item {
            self.speed_override_mps = speed_mps;
        }
    }
}

pub(crate) fn to_mav_mission_type(mission_type: MissionType) -> dialect::MavMissionType {
    match mission_type {
        MissionType::Mission => dialect::MavMissionType::MAV_MISSION_TYPE_MISSION,
        MissionType::Fence => dialect::MavMissionType::MAV_MISSION_TYPE_FENCE,
        MissionType::Rally => dialect::MavMissionType::MAV_MISSION_TYPE_RALLY,
    }
}

pub(crate) fn from_mav_mission_type(mission_type: dialect::MavMissionType) -> MissionType {
    match mission_type {
        dialect::MavMissionType::MAV_MISSION_TYPE_FENCE => MissionType::Fence,
        dialect::MavMissionType::MAV_MISSION_TYPE_RALLY => MissionType::Rally,
        _ => MissionType::Mission,
    }
}

impl SimulatorCore {
    pub(crate) async fn handle_mission_count(
        &mut self,
        data: dialect::MISSION_COUNT_DATA,
    ) -> Result<(), VehicleError> {
        if data.count == 0 {
            self.replace_mission_items(from_mav_mission_type(data.mission_type), Vec::new());
            return self
                .send_mission_ack(
                    data.mission_type,
                    dialect::MavMissionResult::MAV_MISSION_ACCEPTED,
                )
                .await;
        }

        self.pending_upload = Some(PendingMissionUpload {
            mission_type: from_mav_mission_type(data.mission_type),
            expected_count: data.count,
            items: Vec::with_capacity(data.count as usize),
        });

        self.send_message(dialect::MavMessage::MISSION_REQUEST_INT(
            dialect::MISSION_REQUEST_INT_DATA {
                seq: 0,
                target_system: DEFAULT_SYSTEM_ID,
                target_component: DEFAULT_COMPONENT_ID,
                mission_type: data.mission_type,
            },
        ))
        .await
    }

    pub(crate) async fn handle_mission_item_int(
        &mut self,
        data: dialect::MISSION_ITEM_INT_DATA,
    ) -> Result<(), VehicleError> {
        let Some(pending) = self.pending_upload.as_ref() else {
            return self
                .send_mission_ack(
                    data.mission_type,
                    dialect::MavMissionResult::MAV_MISSION_ERROR,
                )
                .await;
        };

        let mission_type = to_mav_mission_type(pending.mission_type);
        let next_seq = pending.items.len() as u16;
        let result = if next_seq >= pending.expected_count {
            Some(dialect::MavMissionResult::MAV_MISSION_NO_SPACE)
        } else if data.seq != next_seq {
            Some(dialect::MavMissionResult::MAV_MISSION_INVALID_SEQUENCE)
        } else if data.mission_type != mission_type {
            Some(dialect::MavMissionResult::MAV_MISSION_INVALID)
        } else {
            None
        };

        if let Some(result) = result {
            self.pending_upload = None;
            return self.send_mission_ack(mission_type, result).await;
        }

        let Some(pending) = self.pending_upload.as_mut() else {
            return self
                .send_mission_ack(
                    data.mission_type,
                    dialect::MavMissionResult::MAV_MISSION_ERROR,
                )
                .await;
        };

        pending.items.push(data);
        if pending.items.len() < pending.expected_count as usize {
            let next_seq = pending.items.len() as u16;
            let mission_type = to_mav_mission_type(pending.mission_type);
            self.send_message(dialect::MavMessage::MISSION_REQUEST_INT(
                dialect::MISSION_REQUEST_INT_DATA {
                    seq: next_seq,
                    target_system: DEFAULT_SYSTEM_ID,
                    target_component: DEFAULT_COMPONENT_ID,
                    mission_type,
                },
            ))
            .await
        } else {
            let mission_type = pending.mission_type;
            let items = pending.items.clone();
            self.pending_upload = None;
            self.replace_mission_items(mission_type, items);
            self.mission_completed = false;
            self.mission_speed_override_mps = None;
            self.send_mission_ack(
                to_mav_mission_type(mission_type),
                dialect::MavMissionResult::MAV_MISSION_ACCEPTED,
            )
            .await
        }
    }

    pub(crate) async fn handle_mission_request_list(
        &mut self,
        data: dialect::MISSION_REQUEST_LIST_DATA,
    ) -> Result<(), VehicleError> {
        let items = self.mission_items(from_mav_mission_type(data.mission_type));
        self.send_message(dialect::MavMessage::MISSION_COUNT(
            dialect::MISSION_COUNT_DATA {
                count: items.len() as u16,
                target_system: DEFAULT_SYSTEM_ID,
                target_component: DEFAULT_COMPONENT_ID,
                mission_type: data.mission_type,
                opaque_id: 0,
            },
        ))
        .await
    }

    pub(crate) async fn handle_mission_request_int(
        &mut self,
        data: dialect::MISSION_REQUEST_INT_DATA,
    ) -> Result<(), VehicleError> {
        self.send_requested_mission_item(data.seq, data.mission_type)
            .await
    }

    #[allow(
        deprecated,
        reason = "the demo simulator accepts the legacy mission-request fallback used by current download code"
    )]
    pub(crate) async fn handle_mission_request(
        &mut self,
        data: dialect::MISSION_REQUEST_DATA,
    ) -> Result<(), VehicleError> {
        self.send_requested_mission_item(data.seq, data.mission_type)
            .await
    }

    pub(crate) async fn handle_mission_clear_all(
        &mut self,
        data: dialect::MISSION_CLEAR_ALL_DATA,
    ) -> Result<(), VehicleError> {
        if data.mission_type == dialect::MavMissionType::MAV_MISSION_TYPE_ALL {
            self.replace_mission_items(MissionType::Mission, Vec::new());
            self.replace_mission_items(MissionType::Fence, Vec::new());
            self.replace_mission_items(MissionType::Rally, Vec::new());
        } else {
            self.replace_mission_items(from_mav_mission_type(data.mission_type), Vec::new());
        }
        self.mission_completed = false;
        self.nav_target = None;
        self.pending_reached_wire_seq = None;
        self.mission_speed_override_mps = None;
        self.send_mission_ack(
            data.mission_type,
            dialect::MavMissionResult::MAV_MISSION_ACCEPTED,
        )
        .await
    }

    async fn send_requested_mission_item(
        &mut self,
        seq: u16,
        mission_type: dialect::MavMissionType,
    ) -> Result<(), VehicleError> {
        let item = self
            .mission_items(from_mav_mission_type(mission_type))
            .get(seq as usize)
            .cloned();

        if let Some(item) = item {
            self.send_message(dialect::MavMessage::MISSION_ITEM_INT(item))
                .await
        } else {
            self.send_mission_ack(mission_type, dialect::MavMissionResult::MAV_MISSION_ERROR)
                .await
        }
    }

    pub(crate) async fn drive_mission_state(&mut self) -> Result<(), VehicleError> {
        if !self.snapshot.armed || self.snapshot.custom_mode != auto_mode(self.config.profile) {
            if self
                .nav_target
                .is_some_and(|target| target.source == NavSource::Mission)
            {
                self.nav_target = None;
            }
            return Ok(());
        }

        if self.missions.mission.len() <= 1 || self.mission_completed {
            return Ok(());
        }

        if let Some(reached_seq) = self.pending_reached_wire_seq.take()
            && reached_seq == self.snapshot.mission_current_wire_seq
        {
            let _ = self.advance_mission_item().await?;
            return Ok(());
        }

        if self.snapshot.mission_current_wire_seq == 0 {
            self.snapshot.mission_current_wire_seq = 1;
            self.publish_snapshot();
            self.emit_mission_current().await?;
        }

        loop {
            let Some(item) = self
                .missions
                .mission
                .get(self.snapshot.mission_current_wire_seq as usize)
                .cloned()
            else {
                self.mission_completed = true;
                self.nav_target = None;
                return Ok(());
            };

            if let Some(target) = self.nav_target_for_mission_item(&item) {
                self.nav_target = Some(target);
                return Ok(());
            }

            if self.handle_non_nav_mission_item(&item).await? {
                if !self.advance_mission_item().await? {
                    return Ok(());
                }
                continue;
            }

            if !self.advance_mission_item().await? {
                return Ok(());
            }
        }
    }

    pub(crate) async fn advance_mission_item(&mut self) -> Result<bool, VehicleError> {
        let seq = self.snapshot.mission_current_wire_seq;
        if let Some(item) = self.missions.mission.get(seq as usize)
            && let RuntimeMissionItem::Unsupported { command } = RuntimeMissionItem::from_wire(item)
        {
            self.emit_statustext(format!("Skipping mission cmd {:?} at seq {}", command, seq))
                .await?;
        }

        self.emit_mission_item_reached(seq).await?;
        let next_seq = seq.saturating_add(1);
        if next_seq >= self.missions.mission.len() as u16 {
            self.mission_completed = true;
            self.nav_target = None;
            return Ok(false);
        }

        self.snapshot.mission_current_wire_seq = next_seq;
        self.nav_target = None;
        self.publish_snapshot();
        self.emit_mission_current().await?;
        Ok(true)
    }

    async fn handle_non_nav_mission_item(
        &mut self,
        item: &dialect::MISSION_ITEM_INT_DATA,
    ) -> Result<bool, VehicleError> {
        match RuntimeMissionItem::from_wire(item) {
            RuntimeMissionItem::DoChangeSpeed { speed_mps } => {
                self.mission_speed_override_mps = speed_mps;
                Ok(true)
            }
            RuntimeMissionItem::Navigation | RuntimeMissionItem::Unsupported { .. } => Ok(false),
        }
    }

    pub(crate) fn restore_mission_runtime_state(&mut self) {
        let state = self.mission_runtime_state_before(self.snapshot.mission_current_wire_seq);
        self.mission_speed_override_mps = state.speed_override_mps;
    }

    fn mission_runtime_state_before(&self, wire_seq: u16) -> MissionRuntimeState {
        let mut state = MissionRuntimeState::default();
        for item in self.missions.mission.iter().take(wire_seq as usize) {
            state.apply(RuntimeMissionItem::from_wire(item));
        }
        state
    }
}

#[cfg(test)]
mod tests {
    use tokio::sync::{mpsc, watch};

    use super::*;
    use crate::mission::HomePosition;
    use crate::sim::state::{
        ACCEPTANCE_RADIUS_M, DemoVehicleConfig, NavTarget, profile_vehicle_type,
    };
    use crate::sim::{DemoClock, DemoProfile, DemoVehicleSnapshot};

    fn sim_core(
        profile: DemoProfile,
    ) -> (
        SimulatorCore,
        mpsc::Receiver<(mavlink::MavHeader, dialect::MavMessage)>,
    ) {
        let home = HomePosition {
            latitude_deg: 42.0,
            longitude_deg: -71.0,
            altitude_m: 100.0,
        };
        let snapshot = DemoVehicleSnapshot {
            time_boot_ms: 0,
            armed: true,
            custom_mode: auto_mode(profile),
            home: home.clone(),
            latitude_deg: home.latitude_deg,
            longitude_deg: home.longitude_deg,
            altitude_msl_m: home.altitude_m,
            relative_alt_m: 0.0,
            roll_rad: 0.0,
            pitch_rad: 0.0,
            yaw_rad: 0.0,
            mission_current_wire_seq: 1,
            mission_total_wire_items: 0,
        };
        let (outbound_tx, outbound_rx) = mpsc::channel(8);
        let (snapshot_tx, _snapshot_rx) = watch::channel(snapshot.clone());
        (
            SimulatorCore::new(
                DemoVehicleConfig {
                    profile,
                    clock: DemoClock::Manual,
                    tick_hz: 10,
                    home,
                },
                outbound_tx,
                snapshot_tx,
                snapshot,
            ),
            outbound_rx,
        )
    }

    fn mission_item(seq: u16, command: dialect::MavCmd) -> dialect::MISSION_ITEM_INT_DATA {
        dialect::MISSION_ITEM_INT_DATA {
            seq,
            command,
            frame: dialect::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
            current: 0,
            autocontinue: 1,
            target_system: 1,
            target_component: 1,
            x: 420_000_000,
            y: -710_000_000,
            z: 20.0,
            mission_type: dialect::MavMissionType::MAV_MISSION_TYPE_MISSION,
            ..dialect::MISSION_ITEM_INT_DATA::default()
        }
    }

    fn mission_count(
        count: u16,
        mission_type: dialect::MavMissionType,
    ) -> dialect::MISSION_COUNT_DATA {
        dialect::MISSION_COUNT_DATA {
            count,
            target_system: DEFAULT_SYSTEM_ID,
            target_component: DEFAULT_COMPONENT_ID,
            mission_type,
            opaque_id: 0,
        }
    }

    fn recv_mission_request(
        outbound_rx: &mut mpsc::Receiver<(mavlink::MavHeader, dialect::MavMessage)>,
    ) -> dialect::MISSION_REQUEST_INT_DATA {
        match outbound_rx.try_recv().unwrap().1 {
            dialect::MavMessage::MISSION_REQUEST_INT(data) => data,
            message => panic!("expected MISSION_REQUEST_INT, got {message:?}"),
        }
    }

    fn recv_mission_ack(
        outbound_rx: &mut mpsc::Receiver<(mavlink::MavHeader, dialect::MavMessage)>,
    ) -> dialect::MISSION_ACK_DATA {
        match outbound_rx.try_recv().unwrap().1 {
            dialect::MavMessage::MISSION_ACK(data) => data,
            message => panic!("expected MISSION_ACK, got {message:?}"),
        }
    }

    fn mission_clear_all(mission_type: dialect::MavMissionType) -> dialect::MISSION_CLEAR_ALL_DATA {
        dialect::MISSION_CLEAR_ALL_DATA {
            target_system: DEFAULT_SYSTEM_ID,
            target_component: DEFAULT_COMPONENT_ID,
            mission_type,
        }
    }

    #[tokio::test]
    async fn mission_upload_accepts_ordered_items() {
        let (mut sim, mut outbound_rx) = sim_core(DemoProfile::ArduCopter);

        sim.handle_mission_count(mission_count(
            2,
            dialect::MavMissionType::MAV_MISSION_TYPE_MISSION,
        ))
        .await
        .unwrap();
        let request = recv_mission_request(&mut outbound_rx);
        assert_eq!(request.seq, 0);

        sim.handle_mission_item_int(mission_item(0, dialect::MavCmd::MAV_CMD_NAV_WAYPOINT))
            .await
            .unwrap();
        let request = recv_mission_request(&mut outbound_rx);
        assert_eq!(request.seq, 1);

        sim.handle_mission_item_int(mission_item(1, dialect::MavCmd::MAV_CMD_NAV_WAYPOINT))
            .await
            .unwrap();
        let ack = recv_mission_ack(&mut outbound_rx);
        assert_eq!(ack.mavtype, dialect::MavMissionResult::MAV_MISSION_ACCEPTED);
        assert!(sim.pending_upload.is_none());
        assert_eq!(sim.missions.mission.len(), 2);
        assert_eq!(sim.missions.mission[1].seq, 1);
    }

    #[tokio::test]
    async fn mission_upload_rejects_invalid_seq_and_clears_pending() {
        let (mut sim, mut outbound_rx) = sim_core(DemoProfile::ArduCopter);

        sim.handle_mission_count(mission_count(
            2,
            dialect::MavMissionType::MAV_MISSION_TYPE_MISSION,
        ))
        .await
        .unwrap();
        let _ = recv_mission_request(&mut outbound_rx);

        sim.handle_mission_item_int(mission_item(1, dialect::MavCmd::MAV_CMD_NAV_WAYPOINT))
            .await
            .unwrap();
        let ack = recv_mission_ack(&mut outbound_rx);
        assert_eq!(
            ack.mavtype,
            dialect::MavMissionResult::MAV_MISSION_INVALID_SEQUENCE
        );
        assert!(sim.pending_upload.is_none());
        assert!(sim.missions.mission.is_empty());
    }

    #[tokio::test]
    async fn mission_upload_rejects_wrong_mission_type_and_clears_pending() {
        let (mut sim, mut outbound_rx) = sim_core(DemoProfile::ArduCopter);

        sim.handle_mission_count(mission_count(
            1,
            dialect::MavMissionType::MAV_MISSION_TYPE_MISSION,
        ))
        .await
        .unwrap();
        let _ = recv_mission_request(&mut outbound_rx);

        let mut item = mission_item(0, dialect::MavCmd::MAV_CMD_NAV_WAYPOINT);
        item.mission_type = dialect::MavMissionType::MAV_MISSION_TYPE_FENCE;
        sim.handle_mission_item_int(item).await.unwrap();

        let ack = recv_mission_ack(&mut outbound_rx);
        assert_eq!(ack.mavtype, dialect::MavMissionResult::MAV_MISSION_INVALID);
        assert_eq!(
            ack.mission_type,
            dialect::MavMissionType::MAV_MISSION_TYPE_MISSION
        );
        assert!(sim.pending_upload.is_none());
        assert!(sim.missions.mission.is_empty());
    }

    #[tokio::test]
    async fn mission_upload_rejects_overrun_and_clears_pending() {
        let (mut sim, mut outbound_rx) = sim_core(DemoProfile::ArduCopter);
        sim.pending_upload = Some(PendingMissionUpload {
            mission_type: MissionType::Mission,
            expected_count: 1,
            items: vec![mission_item(0, dialect::MavCmd::MAV_CMD_NAV_WAYPOINT)],
        });

        sim.handle_mission_item_int(mission_item(1, dialect::MavCmd::MAV_CMD_NAV_WAYPOINT))
            .await
            .unwrap();

        let ack = recv_mission_ack(&mut outbound_rx);
        assert_eq!(ack.mavtype, dialect::MavMissionResult::MAV_MISSION_NO_SPACE);
        assert!(sim.pending_upload.is_none());
        assert!(sim.missions.mission.is_empty());
    }

    #[tokio::test]
    async fn zero_count_upload_clears_pending_and_rejects_stray_item() {
        let (mut sim, mut outbound_rx) = sim_core(DemoProfile::ArduCopter);

        sim.handle_mission_count(mission_count(
            0,
            dialect::MavMissionType::MAV_MISSION_TYPE_MISSION,
        ))
        .await
        .unwrap();
        let ack = recv_mission_ack(&mut outbound_rx);
        assert_eq!(ack.mavtype, dialect::MavMissionResult::MAV_MISSION_ACCEPTED);
        assert!(sim.pending_upload.is_none());

        sim.handle_mission_item_int(mission_item(0, dialect::MavCmd::MAV_CMD_NAV_WAYPOINT))
            .await
            .unwrap();
        let ack = recv_mission_ack(&mut outbound_rx);
        assert_eq!(ack.mavtype, dialect::MavMissionResult::MAV_MISSION_ERROR);
        assert!(sim.pending_upload.is_none());
        assert!(sim.missions.mission.is_empty());
    }

    #[tokio::test]
    async fn mission_clear_all_with_all_clears_all_stores_and_runtime_state() {
        let (mut sim, mut outbound_rx) = sim_core(DemoProfile::ArduCopter);
        sim.missions.mission = vec![mission_item(0, dialect::MavCmd::MAV_CMD_NAV_WAYPOINT)];
        sim.missions.fence = vec![mission_item(
            0,
            dialect::MavCmd::MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
        )];
        sim.missions.rally = vec![mission_item(0, dialect::MavCmd::MAV_CMD_NAV_RALLY_POINT)];
        sim.snapshot.mission_total_wire_items = 3;
        sim.snapshot.mission_current_wire_seq = 2;
        sim.mission_completed = true;
        sim.nav_target = Some(NavTarget {
            source: NavSource::Mission,
            latitude_deg: sim.snapshot.latitude_deg,
            longitude_deg: sim.snapshot.longitude_deg,
            altitude_msl_m: sim.snapshot.altitude_msl_m,
            acceptance_radius_m: ACCEPTANCE_RADIUS_M,
            wire_seq: Some(2),
            disarm_on_reach: false,
        });
        sim.pending_reached_wire_seq = Some(2);
        sim.mission_speed_override_mps = Some(12.0);

        sim.handle_mission_clear_all(mission_clear_all(
            dialect::MavMissionType::MAV_MISSION_TYPE_ALL,
        ))
        .await
        .unwrap();

        let ack = recv_mission_ack(&mut outbound_rx);
        assert_eq!(ack.mavtype, dialect::MavMissionResult::MAV_MISSION_ACCEPTED);
        assert_eq!(
            ack.mission_type,
            dialect::MavMissionType::MAV_MISSION_TYPE_ALL
        );
        assert!(sim.missions.mission.is_empty());
        assert!(sim.missions.fence.is_empty());
        assert!(sim.missions.rally.is_empty());
        assert_eq!(sim.snapshot.mission_total_wire_items, 0);
        assert_eq!(sim.snapshot.mission_current_wire_seq, 0);
        assert!(!sim.mission_completed);
        assert!(sim.nav_target.is_none());
        assert!(sim.pending_reached_wire_seq.is_none());
        assert!(sim.mission_speed_override_mps.is_none());
    }

    #[tokio::test]
    async fn mission_clear_all_with_specific_type_keeps_other_stores() {
        let (mut sim, mut outbound_rx) = sim_core(DemoProfile::ArduCopter);
        sim.missions.mission = vec![mission_item(0, dialect::MavCmd::MAV_CMD_NAV_WAYPOINT)];
        sim.missions.fence = vec![mission_item(
            0,
            dialect::MavCmd::MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
        )];
        sim.missions.rally = vec![mission_item(0, dialect::MavCmd::MAV_CMD_NAV_RALLY_POINT)];

        sim.handle_mission_clear_all(mission_clear_all(
            dialect::MavMissionType::MAV_MISSION_TYPE_FENCE,
        ))
        .await
        .unwrap();

        let ack = recv_mission_ack(&mut outbound_rx);
        assert_eq!(ack.mavtype, dialect::MavMissionResult::MAV_MISSION_ACCEPTED);
        assert_eq!(
            ack.mission_type,
            dialect::MavMissionType::MAV_MISSION_TYPE_FENCE
        );
        assert_eq!(sim.missions.mission.len(), 1);
        assert!(sim.missions.fence.is_empty());
        assert_eq!(sim.missions.rally.len(), 1);
    }

    #[tokio::test]
    async fn change_speed_updates_subsequent_auto_motion() {
        let (mut sim, _outbound_rx) = sim_core(DemoProfile::ArduPlane);
        let mut speed = mission_item(1, dialect::MavCmd::MAV_CMD_DO_CHANGE_SPEED);
        speed.param2 = 23.5;
        sim.missions.mission = vec![
            mission_item(0, dialect::MavCmd::MAV_CMD_NAV_WAYPOINT),
            speed,
            mission_item(2, dialect::MavCmd::MAV_CMD_NAV_WAYPOINT),
        ];

        sim.drive_mission_state().await.unwrap();

        assert_eq!(sim.mission_speed_override_mps, Some(23.5));
        assert_eq!(sim.snapshot.mission_current_wire_seq, 2);
        assert!(sim.nav_target.is_some());
    }

    #[tokio::test]
    async fn mission_current_jump_restores_prior_change_speed() {
        let (mut sim, _outbound_rx) = sim_core(DemoProfile::ArduPlane);
        let mut speed = mission_item(1, dialect::MavCmd::MAV_CMD_DO_CHANGE_SPEED);
        speed.param2 = 19.25;
        sim.missions.mission = vec![
            mission_item(0, dialect::MavCmd::MAV_CMD_NAV_WAYPOINT),
            speed,
            mission_item(2, dialect::MavCmd::MAV_CMD_NAV_WAYPOINT),
        ];
        sim.mission_speed_override_mps = None;

        sim.handle_inbound_message(dialect::MavMessage::COMMAND_LONG(
            dialect::COMMAND_LONG_DATA {
                command: dialect::MavCmd::MAV_CMD_DO_SET_MISSION_CURRENT,
                param1: 2.0,
                ..dialect::COMMAND_LONG_DATA::default()
            },
        ))
        .await
        .unwrap();

        assert_eq!(sim.snapshot.mission_current_wire_seq, 2);
        assert_eq!(sim.mission_speed_override_mps, Some(19.25));
    }

    #[test]
    fn mission_runtime_state_clears_and_restores_around_change_speed() {
        let (mut sim, _outbound_rx) = sim_core(DemoProfile::ArduPlane);
        let mut speed = mission_item(1, dialect::MavCmd::MAV_CMD_DO_CHANGE_SPEED);
        speed.param2 = 17.0;
        sim.missions.mission = vec![
            mission_item(0, dialect::MavCmd::MAV_CMD_NAV_WAYPOINT),
            speed,
            mission_item(2, dialect::MavCmd::MAV_CMD_NAV_WAYPOINT),
        ];
        sim.mission_speed_override_mps = Some(17.0);

        sim.snapshot.mission_current_wire_seq = 1;
        sim.restore_mission_runtime_state();
        assert_eq!(sim.mission_speed_override_mps, None);

        sim.snapshot.mission_current_wire_seq = 2;
        sim.restore_mission_runtime_state();
        assert_eq!(sim.mission_speed_override_mps, Some(17.0));
    }

    #[test]
    fn runtime_mission_item_preserves_unsupported_command_structure() {
        let unsupported = mission_item(4, dialect::MavCmd::MAV_CMD_DO_SET_SERVO);

        assert_eq!(
            RuntimeMissionItem::from_wire(&unsupported),
            RuntimeMissionItem::Unsupported {
                command: dialect::MavCmd::MAV_CMD_DO_SET_SERVO,
            }
        );
    }

    #[tokio::test]
    async fn unsupported_command_advances_mission() {
        let (mut sim, _outbound_rx) = sim_core(DemoProfile::ArduCopter);
        let unsupported = mission_item(1, dialect::MavCmd::MAV_CMD_DO_SET_SERVO);
        sim.missions.mission = vec![
            mission_item(0, dialect::MavCmd::MAV_CMD_NAV_WAYPOINT),
            unsupported,
        ];

        sim.drive_mission_state().await.unwrap();

        assert!(sim.mission_completed);
        assert!(sim.nav_target.is_none());
    }

    #[test]
    fn rtl_mission_item_targets_home() {
        let (sim, _outbound_rx) = sim_core(DemoProfile::ArduPlane);
        let item = mission_item(3, dialect::MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH);

        let target = sim.nav_target_for_mission_item(&item).unwrap();

        assert_eq!(target.latitude_deg, sim.snapshot.home.latitude_deg);
        assert_eq!(target.longitude_deg, sim.snapshot.home.longitude_deg);
        assert_eq!(target.wire_seq, Some(3));
    }

    #[test]
    fn land_target_disarms_and_zeros_relative_altitude_when_reached() {
        let (mut sim, _outbound_rx) = sim_core(DemoProfile::ArduPlane);
        sim.snapshot.altitude_msl_m = 101.0;
        sim.snapshot.relative_alt_m = 1.0;
        let target = NavTarget {
            source: NavSource::Mission,
            latitude_deg: sim.snapshot.latitude_deg,
            longitude_deg: sim.snapshot.longitude_deg,
            altitude_msl_m: sim.snapshot.home.altitude_m + 0.2,
            acceptance_radius_m: ACCEPTANCE_RADIUS_M,
            wire_seq: Some(2),
            disarm_on_reach: true,
        };

        sim.nav_target = Some(target);
        sim.advance_navigation();

        assert!(!sim.snapshot.armed);
        assert_eq!(sim.snapshot.relative_alt_m, 0.0);
        assert_eq!(sim.snapshot.altitude_msl_m, sim.snapshot.home.altitude_m);
    }

    #[test]
    fn profile_vehicle_type_covers_plane_variants() {
        assert_eq!(
            profile_vehicle_type(DemoProfile::ArduPlane),
            dialect::MavType::MAV_TYPE_FIXED_WING
        );
        assert_eq!(
            profile_vehicle_type(DemoProfile::ArduQuadPlane),
            dialect::MavType::MAV_TYPE_VTOL_FIXEDROTOR
        );
    }
}
