use crate::command::Command;
use crate::dialect;
use crate::observation::{ObservationHandle, ObservationWriter};
use crate::state::{AutopilotType, StateChannels, VehicleState, VehicleType};
use crate::types::SupportState;
use crate::vehicle::VehicleInner;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};
use tokio::sync::{broadcast, mpsc, oneshot};

pub use crate::state::FlightMode;

const AVAILABLE_MODES_MSG_ID: f32 = 435.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum VehicleClass {
    Copter,
    Plane,
    Rover,
    Unknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
/// Source used to build the current mode catalog.
pub enum ModeCatalogSource {
    AvailableModes,
    StaticArduPilotTable,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
/// One flight mode entry users can inspect or request.
pub struct ModeDescriptor {
    pub custom_mode: u32,
    pub name: String,
    pub user_selectable: bool,
    pub source: ModeCatalogSource,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
/// Source used to determine the currently active mode.
pub enum CurrentModeSource {
    Heartbeat,
    CurrentModeMessage,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
/// Current mode snapshot, including optional pending target mode.
pub struct CurrentMode {
    pub custom_mode: u32,
    pub name: String,
    pub intended_custom_mode: Option<u32>,
    pub source: CurrentModeSource,
}

#[derive(Clone)]
/// Accessor for mode support, catalog, and current mode observations.
pub struct ModesHandle<'a> {
    inner: &'a VehicleInner,
}

impl<'a> ModesHandle<'a> {
    pub(crate) fn new(inner: &'a VehicleInner) -> Self {
        Self { inner }
    }

    pub fn support(&self) -> ObservationHandle<SupportState> {
        self.seed_from_vehicle_state();
        self.inner.modes.support()
    }

    pub fn catalog(&self) -> ObservationHandle<Vec<ModeDescriptor>> {
        self.seed_from_vehicle_state();
        self.inner.modes.catalog()
    }

    pub fn current(&self) -> ObservationHandle<CurrentMode> {
        self.seed_from_vehicle_state();
        self.inner.modes.current()
    }

    fn seed_from_vehicle_state(&self) {
        let state = self.inner.stores.vehicle_state.borrow().clone();
        self.inner.modes.seed_from_vehicle_state(&state);
    }

    fn snapshot(&self) -> Vec<FlightMode> {
        self.catalog()
            .latest()
            .unwrap_or_default()
            .into_iter()
            .map(|mode| FlightMode {
                custom_mode: mode.custom_mode,
                name: mode.name,
            })
            .collect()
    }

    pub fn len(&self) -> usize {
        self.snapshot().len()
    }

    pub fn is_empty(&self) -> bool {
        self.snapshot().is_empty()
    }

    pub fn iter(&self) -> std::vec::IntoIter<FlightMode> {
        self.snapshot().into_iter()
    }
}

impl IntoIterator for ModesHandle<'_> {
    type Item = FlightMode;
    type IntoIter = std::vec::IntoIter<FlightMode>;

    fn into_iter(self) -> Self::IntoIter {
        self.snapshot().into_iter()
    }
}

impl IntoIterator for &ModesHandle<'_> {
    type Item = FlightMode;
    type IntoIter = std::vec::IntoIter<FlightMode>;

    fn into_iter(self) -> Self::IntoIter {
        self.snapshot().into_iter()
    }
}

#[derive(Clone)]
pub(crate) struct ModeDomain {
    inner: Arc<ModeDomainInner>,
}

struct ModeDomainInner {
    support_writer: ObservationWriter<SupportState>,
    support: ObservationHandle<SupportState>,
    catalog_writer: ObservationWriter<Vec<ModeDescriptor>>,
    catalog: ObservationHandle<Vec<ModeDescriptor>>,
    current_writer: ObservationWriter<CurrentMode>,
    current: ObservationHandle<CurrentMode>,
    tracker: Mutex<ModeTracker>,
}

#[derive(Debug, Default)]
struct ModeTracker {
    identity: Option<(AutopilotType, VehicleType)>,
    catalog: Vec<ModeDescriptor>,
    current_custom_mode: Option<u32>,
    last_current: Option<CurrentMode>,
    last_monitor_seq: Option<u8>,
    pending_available_modes: BTreeMap<u8, dialect::AVAILABLE_MODES_DATA>,
    expected_available_modes: Option<u8>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ModeAction {
    None,
    RequestAvailableModes,
}

impl ModeDomain {
    pub(crate) fn new() -> Self {
        let (support_writer, support) = ObservationHandle::watch();
        let (catalog_writer, catalog) = ObservationHandle::watch();
        let (current_writer, current) = ObservationHandle::watch();
        let _ = support_writer.publish(SupportState::Unknown);

        Self {
            inner: Arc::new(ModeDomainInner {
                support_writer,
                support,
                catalog_writer,
                catalog,
                current_writer,
                current,
                tracker: Mutex::new(ModeTracker::default()),
            }),
        }
    }

    pub(crate) fn start(&self, stores: &StateChannels, command_tx: mpsc::Sender<Command>) {
        let inner = self.inner.clone();
        let mut vehicle_state_rx = stores.vehicle_state.clone();
        let mut raw_rx = stores.raw_message_tx.subscribe();

        inner.handle_vehicle_state(&vehicle_state_rx.borrow().clone());

        tokio::spawn(async move {
            loop {
                tokio::select! {
                    changed = vehicle_state_rx.changed() => {
                        if changed.is_err() {
                            break;
                        }
                        inner.handle_vehicle_state(&vehicle_state_rx.borrow_and_update().clone());
                    }
                    raw = raw_rx.recv() => {
                        match raw {
                            Ok((_, message)) => {
                                if inner.handle_message(&message) == ModeAction::RequestAvailableModes {
                                    request_available_modes(&command_tx).await;
                                }
                            }
                            Err(broadcast::error::RecvError::Lagged(_)) => continue,
                            Err(broadcast::error::RecvError::Closed) => break,
                        }
                    }
                }
            }
        });
    }

    pub(crate) fn seed_from_vehicle_state(&self, state: &VehicleState) {
        self.inner.handle_vehicle_state(state);
    }

    pub(crate) fn support(&self) -> ObservationHandle<SupportState> {
        self.inner.support.clone()
    }

    pub(crate) fn catalog(&self) -> ObservationHandle<Vec<ModeDescriptor>> {
        self.inner.catalog.clone()
    }

    pub(crate) fn current(&self) -> ObservationHandle<CurrentMode> {
        self.inner.current.clone()
    }

    #[cfg(test)]
    fn handle_vehicle_state(&self, state: &VehicleState) {
        self.inner.handle_vehicle_state(state);
    }

    #[cfg(test)]
    fn handle_message(&self, message: &dialect::MavMessage) -> ModeAction {
        self.inner.handle_message(message)
    }
}

impl ModeDomainInner {
    fn handle_vehicle_state(&self, state: &VehicleState) {
        let mut tracker = self.tracker.lock().unwrap();
        tracker.handle_vehicle_state(state, self);
    }

    fn handle_message(&self, message: &dialect::MavMessage) -> ModeAction {
        let mut tracker = self.tracker.lock().unwrap();
        tracker.handle_message(message, self)
    }
}

impl ModeTracker {
    fn handle_vehicle_state(&mut self, state: &VehicleState, domain: &ModeDomainInner) {
        if !state.heartbeat_received {
            return;
        }

        let _ = domain.support_writer.publish(SupportState::Supported);

        let identity = (state.autopilot, state.vehicle_type);
        if self.identity != Some(identity) {
            self.identity = Some(identity);
            self.last_monitor_seq = None;
            self.pending_available_modes.clear();
            self.expected_available_modes = None;
            self.publish_catalog(static_catalog(identity.0, identity.1), domain);
        }

        self.current_custom_mode = Some(state.custom_mode);
        self.publish_current(domain);
    }

    fn handle_message(
        &mut self,
        message: &dialect::MavMessage,
        domain: &ModeDomainInner,
    ) -> ModeAction {
        match message {
            dialect::MavMessage::AVAILABLE_MODES(data) => {
                self.expected_available_modes = Some(data.number_modes);
                self.pending_available_modes
                    .insert(data.mode_index, data.clone());

                let expected = usize::from(data.number_modes);
                if expected == 0 || self.pending_available_modes.len() >= expected {
                    let catalog = dynamic_catalog(&self.pending_available_modes);
                    self.pending_available_modes.clear();
                    self.expected_available_modes = None;
                    self.publish_catalog(catalog, domain);
                }

                ModeAction::None
            }
            dialect::MavMessage::AVAILABLE_MODES_MONITOR(data) => {
                let should_request = self.last_monitor_seq.is_some_and(|seq| seq != data.seq);
                self.last_monitor_seq = Some(data.seq);
                self.pending_available_modes.clear();
                self.expected_available_modes = None;

                if should_request {
                    ModeAction::RequestAvailableModes
                } else {
                    ModeAction::None
                }
            }
            dialect::MavMessage::CURRENT_MODE(_data) => ModeAction::None,
            _ => ModeAction::None,
        }
    }

    fn publish_catalog(&mut self, next: Vec<ModeDescriptor>, domain: &ModeDomainInner) {
        if self.catalog != next {
            self.catalog = next.clone();
            let _ = domain.catalog_writer.publish(next);
        }

        self.publish_current(domain);
    }

    fn publish_current(&mut self, domain: &ModeDomainInner) {
        let Some(custom_mode) = self.current_custom_mode else {
            return;
        };

        let next = CurrentMode {
            custom_mode,
            name: catalog_name_or_fallback(&self.catalog, custom_mode),
            intended_custom_mode: None,
            source: CurrentModeSource::Heartbeat,
        };

        if self.last_current.as_ref() != Some(&next) {
            self.last_current = Some(next.clone());
            let _ = domain.current_writer.publish(next);
        }
    }
}

fn vehicle_class(vehicle_type: VehicleType) -> VehicleClass {
    match vehicle_type {
        VehicleType::Quadrotor
        | VehicleType::Hexarotor
        | VehicleType::Octorotor
        | VehicleType::Tricopter
        | VehicleType::Coaxial
        | VehicleType::Helicopter => VehicleClass::Copter,
        VehicleType::FixedWing | VehicleType::Vtol => VehicleClass::Plane,
        VehicleType::GroundRover => VehicleClass::Rover,
        _ => VehicleClass::Unknown,
    }
}

const COPTER_MODES: &[(u32, &str)] = &[
    (0, "STABILIZE"),
    (1, "ACRO"),
    (2, "ALT_HOLD"),
    (3, "AUTO"),
    (4, "GUIDED"),
    (5, "LOITER"),
    (6, "RTL"),
    (7, "CIRCLE"),
    (9, "LAND"),
    (11, "DRIFT"),
    (13, "SPORT"),
    (15, "AUTOTUNE"),
    (16, "POSHOLD"),
    (17, "BRAKE"),
    (18, "THROW"),
    (21, "SMART_RTL"),
];

const PLANE_MODES: &[(u32, &str)] = &[
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
];

const ROVER_MODES: &[(u32, &str)] = &[
    (0, "MANUAL"),
    (1, "ACRO"),
    (3, "STEERING"),
    (4, "HOLD"),
    (5, "LOITER"),
    (6, "FOLLOW"),
    (7, "SIMPLE"),
    (10, "AUTO"),
    (11, "RTL"),
    (12, "SMART_RTL"),
    (15, "GUIDED"),
];

fn mode_table(
    autopilot: AutopilotType,
    vehicle_type: VehicleType,
) -> &'static [(u32, &'static str)] {
    if autopilot != AutopilotType::ArduPilotMega {
        return &[];
    }

    match vehicle_class(vehicle_type) {
        VehicleClass::Copter | VehicleClass::Unknown => COPTER_MODES,
        VehicleClass::Plane => PLANE_MODES,
        VehicleClass::Rover => ROVER_MODES,
    }
}

pub(crate) fn mode_name(
    autopilot: AutopilotType,
    vehicle_type: VehicleType,
    custom_mode: u32,
) -> String {
    mode_table(autopilot, vehicle_type)
        .iter()
        .find_map(|&(num, name)| (num == custom_mode).then_some(name.to_string()))
        .unwrap_or_else(|| format!("MODE({custom_mode})"))
}

pub(crate) fn mode_number(
    autopilot: AutopilotType,
    vehicle_type: VehicleType,
    name: &str,
) -> Option<u32> {
    let upper = name.to_uppercase();
    mode_table(autopilot, vehicle_type)
        .iter()
        .find_map(|&(num, mode_name)| (mode_name == upper).then_some(num))
}

fn static_catalog(autopilot: AutopilotType, vehicle_type: VehicleType) -> Vec<ModeDescriptor> {
    mode_table(autopilot, vehicle_type)
        .iter()
        .map(|&(custom_mode, name)| ModeDescriptor {
            custom_mode,
            name: name.to_string(),
            user_selectable: true,
            source: ModeCatalogSource::StaticArduPilotTable,
        })
        .collect()
}

fn dynamic_catalog(modes: &BTreeMap<u8, dialect::AVAILABLE_MODES_DATA>) -> Vec<ModeDescriptor> {
    modes
        .values()
        .map(|mode| ModeDescriptor {
            custom_mode: mode.custom_mode,
            name: available_mode_name(mode),
            user_selectable: !mode
                .properties
                .contains(dialect::MavModeProperty::MAV_MODE_PROPERTY_NOT_USER_SELECTABLE),
            source: ModeCatalogSource::AvailableModes,
        })
        .collect()
}

fn available_mode_name(mode: &dialect::AVAILABLE_MODES_DATA) -> String {
    let name = mode.mode_name.to_str().unwrap_or_default().trim();
    if name.is_empty() {
        format!("MODE({})", mode.custom_mode)
    } else {
        name.to_string()
    }
}

fn catalog_name_or_fallback(catalog: &[ModeDescriptor], custom_mode: u32) -> String {
    catalog
        .iter()
        .find_map(|mode| (mode.custom_mode == custom_mode).then(|| mode.name.clone()))
        .unwrap_or_else(|| format!("MODE({custom_mode})"))
}

async fn request_available_modes(command_tx: &mpsc::Sender<Command>) {
    let (reply_tx, _reply_rx) = oneshot::channel();
    let _ = command_tx
        .send(Command::RawCommandLong {
            command: dialect::MavCmd::MAV_CMD_REQUEST_MESSAGE,
            params: [AVAILABLE_MODES_MSG_ID, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            reply: reply_tx,
        })
        .await;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dialect;

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
}
