use crate::event_loop::{InitManager, InitSnapshot};
use crate::info::logic::{
    firmware_from_init_state, hardware_from_init_state, persistent_identity,
    unique_ids_from_init_state,
};
use crate::info::{FirmwareInfo, HardwareInfo, PersistentIdentity, UniqueIds};
use crate::observation::{ObservationHandle, ObservationWriter};
use crate::runtime::{self, TaskHandle};
use crate::shared_state::recover_lock;
use crate::state::{StateChannels, VehicleState};
use std::sync::{Arc, Mutex};

#[derive(Clone)]
pub(crate) struct InfoDomain {
    inner: Arc<InfoDomainInner>,
}

struct InfoDomainInner {
    firmware_writer: ObservationWriter<FirmwareInfo>,
    firmware: ObservationHandle<FirmwareInfo>,
    hardware_writer: ObservationWriter<HardwareInfo>,
    hardware: ObservationHandle<HardwareInfo>,
    unique_ids_writer: ObservationWriter<UniqueIds>,
    unique_ids: ObservationHandle<UniqueIds>,
    persistent_identity_writer: ObservationWriter<PersistentIdentity>,
    persistent_identity: ObservationHandle<PersistentIdentity>,
    tracker: Mutex<InfoTracker>,
}

#[derive(Default)]
struct InfoTracker {
    vehicle_state: VehicleState,
    init_snapshot: InitSnapshot,
    firmware: Option<FirmwareInfo>,
    hardware: Option<HardwareInfo>,
    unique_ids: Option<UniqueIds>,
    persistent_identity: Option<PersistentIdentity>,
}

impl InfoDomain {
    pub(crate) fn new() -> Self {
        let (firmware_writer, firmware) = ObservationHandle::watch();
        let (hardware_writer, hardware) = ObservationHandle::watch();
        let (unique_ids_writer, unique_ids) = ObservationHandle::watch();
        let (persistent_identity_writer, persistent_identity) = ObservationHandle::watch();

        Self {
            inner: Arc::new(InfoDomainInner {
                firmware_writer,
                firmware,
                hardware_writer,
                hardware,
                unique_ids_writer,
                unique_ids,
                persistent_identity_writer,
                persistent_identity,
                tracker: Mutex::new(InfoTracker::default()),
            }),
        }
    }

    pub(crate) fn start(&self, stores: &StateChannels, init_manager: &InitManager) -> TaskHandle {
        let inner = self.inner.clone();
        let mut vehicle_state_rx = stores.vehicle_state.clone();
        let mut init_rx = init_manager.subscribe();

        inner.handle_vehicle_state(&vehicle_state_rx.borrow().clone());
        inner.handle_init_snapshot(&init_rx.borrow().clone());

        runtime::spawn(async move {
            loop {
                tokio::select! {
                    changed = vehicle_state_rx.changed() => {
                        if changed.is_err() {
                            break;
                        }
                        inner.handle_vehicle_state(&vehicle_state_rx.borrow_and_update().clone());
                    }
                    changed = init_rx.changed() => {
                        if changed.is_err() {
                            break;
                        }
                        inner.handle_init_snapshot(&init_rx.borrow_and_update().clone());
                    }
                }
            }
        })
    }

    pub(crate) fn firmware(&self) -> ObservationHandle<FirmwareInfo> {
        self.inner.firmware.clone()
    }

    pub(crate) fn hardware(&self) -> ObservationHandle<HardwareInfo> {
        self.inner.hardware.clone()
    }

    pub(crate) fn unique_ids(&self) -> ObservationHandle<UniqueIds> {
        self.inner.unique_ids.clone()
    }

    pub(crate) fn persistent_identity(&self) -> ObservationHandle<PersistentIdentity> {
        self.inner.persistent_identity.clone()
    }

    pub(crate) fn close(&self) {
        self.inner.firmware_writer.close();
        self.inner.hardware_writer.close();
        self.inner.unique_ids_writer.close();
        self.inner.persistent_identity_writer.close();
    }

    #[cfg(test)]
    fn handle_vehicle_state(&self, vehicle_state: &VehicleState) {
        self.inner.handle_vehicle_state(vehicle_state);
    }

    #[cfg(test)]
    fn handle_init_snapshot(&self, init_snapshot: &InitSnapshot) {
        self.inner.handle_init_snapshot(init_snapshot);
    }
}

impl InfoDomainInner {
    fn handle_vehicle_state(&self, vehicle_state: &VehicleState) {
        self.update_tracker(|tracker| {
            tracker.vehicle_state = vehicle_state.clone();
        });
    }

    fn handle_init_snapshot(&self, init_snapshot: &InitSnapshot) {
        self.update_tracker(|tracker| {
            tracker.init_snapshot = init_snapshot.clone();
        });
    }

    fn update_tracker(&self, edit: impl FnOnce(&mut InfoTracker)) {
        let mut tracker = recover_lock(&self.tracker);
        edit(&mut tracker);
        self.recompute(&mut tracker);
    }

    fn recompute(&self, tracker: &mut InfoTracker) {
        let firmware = firmware_from_init_state(&tracker.init_snapshot.autopilot_version);
        publish_option_if_changed(&mut tracker.firmware, firmware, &self.firmware_writer);

        let hardware = hardware_from_init_state(&tracker.init_snapshot.autopilot_version);
        publish_option_if_changed(&mut tracker.hardware, hardware, &self.hardware_writer);

        let unique_ids = unique_ids_from_init_state(&tracker.init_snapshot.autopilot_version);
        publish_option_if_changed(
            &mut tracker.unique_ids,
            unique_ids.clone(),
            &self.unique_ids_writer,
        );

        let identity = persistent_identity(&tracker.vehicle_state, unique_ids.as_ref());
        publish_value_if_changed(
            &mut tracker.persistent_identity,
            identity,
            &self.persistent_identity_writer,
        );
    }
}

fn publish_option_if_changed<T: Clone + Send + Sync + PartialEq + 'static>(
    slot: &mut Option<T>,
    next: Option<T>,
    writer: &ObservationWriter<T>,
) {
    if *slot == next {
        return;
    }

    *slot = next.clone();
    if let Some(value) = next {
        let _ = writer.publish(value);
    }
}

fn publish_value_if_changed<T: Clone + Send + Sync + PartialEq + 'static>(
    slot: &mut Option<T>,
    next: T,
    writer: &ObservationWriter<T>,
) {
    if slot.as_ref() == Some(&next) {
        return;
    }

    *slot = Some(next.clone());
    let _ = writer.publish(next);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dialect;
    use crate::event_loop::InitState;
    use std::time::Duration;

    fn vehicle_state(system_id: u8, component_id: u8) -> VehicleState {
        VehicleState {
            system_id,
            component_id,
            ..VehicleState::default()
        }
    }

    #[tokio::test]
    async fn persistent_identity_tracks_pending_then_ready_identity() {
        let domain = InfoDomain::new();
        let mut sub = domain.persistent_identity().subscribe();
        let ready = PersistentIdentity::Ready {
            canonical_id: "uid64:0123456789abcdef".into(),
            aliases: vec!["uid64:0123456789abcdef".into()],
        };

        domain.handle_vehicle_state(&vehicle_state(1, 1));

        assert_eq!(
            tokio::time::timeout(Duration::from_millis(100), sub.recv())
                .await
                .expect("pending identity should be published")
                .expect("identity subscription should remain open"),
            PersistentIdentity::Pending {
                system_id: 1,
                component_id: 1,
            }
        );

        domain.handle_init_snapshot(&InitSnapshot {
            autopilot_version: InitState::Available(dialect::AUTOPILOT_VERSION_DATA {
                uid: 0x0123_4567_89ab_cdef,
                ..dialect::AUTOPILOT_VERSION_DATA::default()
            }),
            ..InitSnapshot::default()
        });

        assert_eq!(
            tokio::time::timeout(Duration::from_millis(100), sub.recv())
                .await
                .expect("ready identity should be published")
                .expect("identity subscription should remain open"),
            ready,
        );

        domain.handle_vehicle_state(&vehicle_state(9, 9));

        assert!(
            tokio::time::timeout(Duration::from_millis(100), sub.recv())
                .await
                .is_err(),
            "expected ready identity to stay stable after later vehicle-state updates"
        );
        assert_eq!(domain.persistent_identity().latest(), Some(ready));
    }
}
