use crate::command::Command;
use crate::dialect;
use crate::modes::catalog::{catalog_name_or_fallback, dynamic_catalog, static_catalog};
use crate::modes::{CurrentMode, CurrentModeSource, ModeDescriptor};
use crate::observation::{ObservationHandle, ObservationWriter};
use crate::state::{AutopilotType, StateChannels, VehicleState, VehicleType};
use crate::types::SupportState;
use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};
use tokio::sync::{broadcast, mpsc, oneshot};
use tokio::task::JoinHandle;

const AVAILABLE_MODES_MSG_ID: f32 = 435.0;

#[derive(Clone)]
pub(crate) struct ModeDomain {
    inner: Arc<ModeDomainInner>,
}

pub(super) struct ModeDomainInner {
    support_writer: ObservationWriter<SupportState>,
    support: ObservationHandle<SupportState>,
    catalog_writer: ObservationWriter<Vec<ModeDescriptor>>,
    catalog: ObservationHandle<Vec<ModeDescriptor>>,
    current_writer: ObservationWriter<CurrentMode>,
    current: ObservationHandle<CurrentMode>,
    tracker: Mutex<ModeTracker>,
}

#[derive(Debug, Default)]
pub(super) struct ModeTracker {
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

    pub(crate) fn start(
        &self,
        stores: &StateChannels,
        command_tx: mpsc::Sender<Command>,
    ) -> JoinHandle<()> {
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
        })
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

    pub(crate) fn close(&self) {
        self.inner.support_writer.close();
        self.inner.catalog_writer.close();
        self.inner.current_writer.close();
    }

    #[cfg(test)]
    pub(super) fn handle_vehicle_state(&self, state: &VehicleState) {
        self.inner.handle_vehicle_state(state);
    }

    #[cfg(test)]
    pub(super) fn handle_message(&self, message: &dialect::MavMessage) {
        let _ = self.inner.handle_message(message);
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
                self.pending_available_modes.insert(data.mode_index, data.clone());

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
            dialect::MavMessage::CURRENT_MODE(data) => {
                self.current_custom_mode = Some(data.custom_mode);
                let intended = if data.intended_custom_mode == 0 {
                    None
                } else {
                    Some(data.intended_custom_mode)
                };

                let next = CurrentMode {
                    custom_mode: data.custom_mode,
                    name: catalog_name_or_fallback(&self.catalog, data.custom_mode),
                    intended_custom_mode: intended,
                    source: CurrentModeSource::CurrentModeMessage,
                };

                if self.last_current.as_ref() != Some(&next) {
                    self.last_current = Some(next.clone());
                    let _ = domain.current_writer.publish(next);
                }

                ModeAction::None
            }
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

        // If we already have a CURRENT_MODE-sourced snapshot for the same mode,
        // preserve it — it carries intended_custom_mode that heartbeat lacks.
        if let Some(ref last) = self.last_current
            && last.custom_mode == custom_mode
            && last.source == CurrentModeSource::CurrentModeMessage
        {
            // Re-publish with updated name in case catalog changed.
            let refreshed = CurrentMode {
                name: catalog_name_or_fallback(&self.catalog, custom_mode),
                ..last.clone()
            };
            if self.last_current.as_ref() != Some(&refreshed) {
                self.last_current = Some(refreshed.clone());
                let _ = domain.current_writer.publish(refreshed);
            }
            return;
        }

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
