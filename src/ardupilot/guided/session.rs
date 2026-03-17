use super::{
    ArduCopterGuidedHandle, ArduGuidedKind, ArduPlaneGuidedHandle, ArduPlaneKind,
    ArduRoverGuidedHandle, ArduSubGuidedHandle, GuidedSpecific,
};
use crate::command::Command;
use crate::error::VehicleError;
use crate::state::LinkState;
use crate::vehicle::VehicleInner;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use tokio::sync::mpsc;
use tokio_util::sync::CancellationToken;

#[derive(Clone, Debug)]
pub(crate) struct GuidedLeaseScope {
    inner: Arc<GuidedLeaseScopeInner>,
}

#[derive(Debug)]
struct GuidedLeaseScopeInner {
    active_lease: Mutex<Option<u64>>,
    next_lease_id: AtomicU64,
}

impl GuidedLeaseScope {
    pub(crate) fn new() -> Self {
        Self {
            inner: Arc::new(GuidedLeaseScopeInner {
                active_lease: Mutex::new(None),
                next_lease_id: AtomicU64::new(1),
            }),
        }
    }

    pub(crate) fn acquire(&self) -> Result<u64, VehicleError> {
        let mut active = self.inner.active_lease.lock().unwrap();
        if active.is_some() {
            return Err(VehicleError::OperationConflict {
                conflicting_domain: "ardupilot_guided".to_string(),
                conflicting_op: "session_active".to_string(),
            });
        }

        let lease_id = self.inner.next_lease_id.fetch_add(1, Ordering::Relaxed);
        *active = Some(lease_id);
        Ok(lease_id)
    }

    pub(crate) fn release(&self, lease_id: u64) {
        let mut active = self.inner.active_lease.lock().unwrap();
        if active
            .as_ref()
            .is_some_and(|active_id| *active_id == lease_id)
        {
            *active = None;
        }
    }
}

#[derive(Debug, Clone)]
/// Exclusive ArduPilot GUIDED-mode lease and command session.
pub struct ArduGuidedSession {
    inner: Arc<ArduGuidedSessionInner>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum GuidedTerminalReason {
    LinkDropped,
    GuidedModeLost,
}

#[derive(Debug)]
struct ArduGuidedSessionInner {
    lease_scope: GuidedLeaseScope,
    lease_id: u64,
    kind: ArduGuidedKind,
    plane_kind: Option<ArduPlaneKind>,
    guided_mode_custom_mode: u32,
    command_tx: mpsc::Sender<Command>,
    target_system: u8,
    target_component: u8,
    released: AtomicBool,
    monitor_cancel: CancellationToken,
    terminal_reason: Mutex<Option<GuidedTerminalReason>>,
}

pub(crate) struct GuidedSessionInit {
    pub(crate) lease_scope: GuidedLeaseScope,
    pub(crate) lease_id: u64,
    pub(crate) kind: ArduGuidedKind,
    pub(crate) plane_kind: Option<ArduPlaneKind>,
    pub(crate) guided_mode_custom_mode: u32,
    pub(crate) command_tx: mpsc::Sender<Command>,
    pub(crate) target_system: u8,
    pub(crate) target_component: u8,
}

impl ArduGuidedSession {
    pub(crate) fn new(vehicle: &VehicleInner, init: GuidedSessionInit) -> Self {
        let inner = Arc::new(ArduGuidedSessionInner {
            lease_scope: init.lease_scope,
            lease_id: init.lease_id,
            kind: init.kind,
            plane_kind: init.plane_kind,
            guided_mode_custom_mode: init.guided_mode_custom_mode,
            command_tx: init.command_tx,
            target_system: init.target_system,
            target_component: init.target_component,
            released: AtomicBool::new(false),
            monitor_cancel: CancellationToken::new(),
            terminal_reason: Mutex::new(None),
        });
        inner.spawn_terminal_monitor(vehicle);

        Self { inner }
    }

    pub fn kind(&self) -> ArduGuidedKind {
        self.inner.kind
    }

    pub fn specific(&self) -> GuidedSpecific<'_> {
        match self.kind() {
            ArduGuidedKind::Copter => GuidedSpecific::Copter(ArduCopterGuidedHandle::new(self)),
            ArduGuidedKind::Plane => GuidedSpecific::Plane(ArduPlaneGuidedHandle::new(self)),
            ArduGuidedKind::Rover => GuidedSpecific::Rover(ArduRoverGuidedHandle::new(self)),
            ArduGuidedKind::Sub => GuidedSpecific::Sub(ArduSubGuidedHandle::new(self)),
        }
    }

    pub async fn close(self) -> Result<(), VehicleError> {
        self.inner.release_local_lease();
        Ok(())
    }

    pub(crate) fn ensure_active(&self) -> Result<(), VehicleError> {
        self.inner.ensure_active()
    }

    pub(crate) fn command_tx(&self) -> mpsc::Sender<Command> {
        self.inner.command_tx.clone()
    }

    pub(crate) fn plane_kind(&self) -> Option<ArduPlaneKind> {
        self.inner.plane_kind
    }

    pub(crate) fn target(&self) -> (u8, u8) {
        (self.inner.target_system, self.inner.target_component)
    }

    #[cfg(test)]
    pub(crate) fn terminal_reason_for_test(&self) -> Option<&'static str> {
        self.inner
            .terminal_reason
            .lock()
            .unwrap()
            .as_ref()
            .map(GuidedTerminalReason::as_str)
    }
}

impl Drop for ArduGuidedSession {
    fn drop(&mut self) {
        self.inner.release_local_lease();
    }
}

impl ArduGuidedSessionInner {
    fn ensure_active(&self) -> Result<(), VehicleError> {
        if self.released.load(Ordering::Acquire) {
            return Err(VehicleError::OperationConflict {
                conflicting_domain: "ardupilot_guided".to_string(),
                conflicting_op: "session_closed".to_string(),
            });
        }

        if self.terminal_reason.lock().unwrap().is_some() {
            return Err(VehicleError::OperationConflict {
                conflicting_domain: "ardupilot_guided".to_string(),
                conflicting_op: "session_terminal".to_string(),
            });
        }

        Ok(())
    }

    fn spawn_terminal_monitor(self: &Arc<Self>, vehicle: &VehicleInner) {
        let session = self.clone();
        let mut link_state_rx = vehicle.stores.link_state.clone();
        let current_mode = vehicle.modes.current();
        let mut current_mode_sub = current_mode.subscribe();

        tokio::spawn(async move {
            if !matches!(link_state_rx.borrow().clone(), LinkState::Connected) {
                session.mark_terminal(GuidedTerminalReason::LinkDropped);
                return;
            }

            if current_mode.latest().map(|mode| mode.custom_mode)
                != Some(session.guided_mode_custom_mode)
            {
                session.mark_terminal(GuidedTerminalReason::GuidedModeLost);
                return;
            }

            loop {
                tokio::select! {
                    _ = session.monitor_cancel.cancelled() => break,
                    changed = link_state_rx.changed() => {
                        if changed.is_err() {
                            session.mark_terminal(GuidedTerminalReason::LinkDropped);
                            break;
                        }

                        if !matches!(link_state_rx.borrow_and_update().clone(), LinkState::Connected) {
                            session.mark_terminal(GuidedTerminalReason::LinkDropped);
                            break;
                        }
                    }
                    observed = current_mode_sub.recv() => {
                        match observed {
                            Some(mode) if mode.custom_mode == session.guided_mode_custom_mode => {}
                            Some(_) | None => {
                                session.mark_terminal(GuidedTerminalReason::GuidedModeLost);
                                break;
                            }
                        }
                    }
                }
            }
        });
    }

    fn mark_terminal(&self, reason: GuidedTerminalReason) {
        let mut terminal_reason = self.terminal_reason.lock().unwrap();
        if terminal_reason.is_none() {
            *terminal_reason = Some(reason);
        }
    }

    fn release_local_lease(&self) {
        if self.released.swap(true, Ordering::AcqRel) {
            return;
        }

        self.monitor_cancel.cancel();
        self.lease_scope.release(self.lease_id);
    }
}

impl GuidedTerminalReason {
    #[cfg(test)]
    fn as_str(&self) -> &'static str {
        match self {
            GuidedTerminalReason::LinkDropped => "link_dropped",
            GuidedTerminalReason::GuidedModeLost => "guided_mode_lost",
        }
    }
}
