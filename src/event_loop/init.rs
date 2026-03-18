use super::{SharedConnection, VehicleTarget, send_message};
use crate::config::{InitDomainPolicy, VehicleConfig};
use crate::dialect::{self, MavCmd};
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::sync::watch;
use tokio_util::sync::CancellationToken;

const AUTOPILOT_VERSION_MSG_ID: f32 = 148.0;
const GPS_GLOBAL_ORIGIN_MSG_ID: f32 = 49.0;
const AVAILABLE_MODES_MSG_ID: f32 = 435.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum InitUnavailableReason {
    SilenceBudgetExhausted,
}

#[derive(Debug, Clone, PartialEq)]
pub(crate) enum InitState<T> {
    Unknown,
    Requesting { attempt: u8 },
    Available(T),
    Unavailable { reason: InitUnavailableReason },
}

#[derive(Debug, Clone, PartialEq)]
pub(crate) struct InitSnapshot {
    pub autopilot_version: InitState<dialect::AUTOPILOT_VERSION_DATA>,
    pub available_modes: InitState<Vec<dialect::AVAILABLE_MODES_DATA>>,
    pub home_position: InitState<dialect::HOME_POSITION_DATA>,
    pub gps_global_origin: InitState<dialect::GPS_GLOBAL_ORIGIN_DATA>,
}

impl Default for InitSnapshot {
    fn default() -> Self {
        Self {
            autopilot_version: InitState::Unknown,
            available_modes: InitState::Unknown,
            home_position: InitState::Unknown,
            gps_global_origin: InitState::Unknown,
        }
    }
}

#[derive(Debug)]
struct InitInner {
    started: bool,
    snapshot: InitSnapshot,
}

#[derive(Debug, Clone)]
pub(crate) struct InitManager {
    config: VehicleConfig,
    inner: Arc<Mutex<InitInner>>,
    snapshot_tx: watch::Sender<InitSnapshot>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum InitDomain {
    AutopilotVersion,
    AvailableModes,
    HomePosition,
    GpsGlobalOrigin,
}

impl InitManager {
    pub(crate) fn new(config: VehicleConfig) -> Self {
        let snapshot = InitSnapshot::default();
        let (snapshot_tx, _) = watch::channel(snapshot.clone());

        Self {
            config,
            inner: Arc::new(Mutex::new(InitInner {
                started: false,
                snapshot,
            })),
            snapshot_tx,
        }
    }

    pub(crate) fn subscribe(&self) -> watch::Receiver<InitSnapshot> {
        self.snapshot_tx.subscribe()
    }

    pub(super) fn start(
        &self,
        connection: SharedConnection,
        target: VehicleTarget,
        cancel: CancellationToken,
    ) {
        let should_start = {
            let mut inner = self.inner.lock().unwrap();
            if inner.started {
                false
            } else {
                inner.started = true;
                true
            }
        };

        if !should_start {
            return;
        }

        self.spawn_domain(
            InitDomain::AutopilotVersion,
            connection.clone(),
            target,
            cancel.clone(),
        );
        self.spawn_domain(
            InitDomain::AvailableModes,
            connection.clone(),
            target,
            cancel.clone(),
        );
        if self.config.auto_request_home {
            self.spawn_domain(
                InitDomain::HomePosition,
                connection.clone(),
                target,
                cancel.clone(),
            );
        }
        self.spawn_domain(InitDomain::GpsGlobalOrigin, connection, target, cancel);
    }

    pub(super) fn handle_message(&self, message: &dialect::MavMessage) {
        let mut inner = self.inner.lock().unwrap();

        match message {
            dialect::MavMessage::AUTOPILOT_VERSION(data) => {
                inner.snapshot.autopilot_version = InitState::Available(data.clone());
            }
            dialect::MavMessage::AVAILABLE_MODES(data) => {
                match &mut inner.snapshot.available_modes {
                    InitState::Available(modes) => {
                        if let Some(existing) = modes
                            .iter_mut()
                            .find(|existing| existing.mode_index == data.mode_index)
                        {
                            *existing = data.clone();
                        } else {
                            modes.push(data.clone());
                        }
                    }
                    _ => {
                        inner.snapshot.available_modes = InitState::Available(vec![data.clone()]);
                    }
                }
            }
            dialect::MavMessage::HOME_POSITION(data) => {
                inner.snapshot.home_position = InitState::Available(data.clone());
            }
            dialect::MavMessage::GPS_GLOBAL_ORIGIN(data) => {
                inner.snapshot.gps_global_origin = InitState::Available(data.clone());
            }
            _ => {}
        }

        self.snapshot_tx.send_replace(inner.snapshot.clone());
    }

    #[cfg(test)]
    pub(super) fn snapshot(&self) -> InitSnapshot {
        self.inner.lock().unwrap().snapshot.clone()
    }

    fn spawn_domain(
        &self,
        domain: InitDomain,
        connection: SharedConnection,
        target: VehicleTarget,
        cancel: CancellationToken,
    ) {
        let Some(policy) = self.policy_for(domain).cloned() else {
            return;
        };

        if !policy.enabled || policy.max_attempts == 0 {
            return;
        }

        let manager = self.clone();
        let config = self.config.clone();
        tokio::spawn(async move {
            let domain_task = tokio::spawn(async move {
                manager
                    .run_domain(domain, policy, connection, target, config, cancel)
                    .await;
            });

            if let Err(err) = domain_task.await
                && err.is_panic()
            {
                tracing::error!(domain = ?domain, "init domain task panicked: {err}");
            }
        });
    }

    async fn run_domain(
        &self,
        domain: InitDomain,
        policy: InitDomainPolicy,
        connection: SharedConnection,
        target: VehicleTarget,
        config: VehicleConfig,
        cancel: CancellationToken,
    ) {
        let max_attempts = policy.max_attempts;
        let retry_delay = retry_delay(&policy);

        for attempt in 1..=max_attempts {
            if self.is_terminal(domain) {
                return;
            }

            self.set_requesting(domain, attempt);
            let _ = request_domain(&connection, &config, &target, domain).await;

            tokio::select! {
                _ = cancel.cancelled() => return,
                _ = tokio::time::sleep(retry_delay) => {}
            }
        }

        self.finish_on_silence(domain);
    }

    fn policy_for(&self, domain: InitDomain) -> Option<&InitDomainPolicy> {
        match domain {
            InitDomain::AutopilotVersion => Some(&self.config.init_policy.autopilot_version),
            InitDomain::AvailableModes => Some(&self.config.init_policy.available_modes),
            InitDomain::HomePosition => Some(&self.config.init_policy.home),
            InitDomain::GpsGlobalOrigin => Some(&self.config.init_policy.origin),
        }
    }

    fn is_terminal(&self, domain: InitDomain) -> bool {
        let inner = self.inner.lock().unwrap();
        match domain {
            InitDomain::AutopilotVersion => matches!(
                inner.snapshot.autopilot_version,
                InitState::Available(_) | InitState::Unavailable { .. }
            ),
            InitDomain::AvailableModes => matches!(
                inner.snapshot.available_modes,
                InitState::Available(_) | InitState::Unavailable { .. }
            ),
            InitDomain::HomePosition => matches!(
                inner.snapshot.home_position,
                InitState::Available(_) | InitState::Unavailable { .. }
            ),
            InitDomain::GpsGlobalOrigin => matches!(
                inner.snapshot.gps_global_origin,
                InitState::Available(_) | InitState::Unavailable { .. }
            ),
        }
    }

    fn set_requesting(&self, domain: InitDomain, attempt: u8) {
        let mut inner = self.inner.lock().unwrap();
        match domain {
            InitDomain::AutopilotVersion => {
                inner.snapshot.autopilot_version = InitState::Requesting { attempt }
            }
            InitDomain::AvailableModes => {
                inner.snapshot.available_modes = InitState::Requesting { attempt }
            }
            InitDomain::HomePosition => {
                inner.snapshot.home_position = InitState::Requesting { attempt }
            }
            InitDomain::GpsGlobalOrigin => {
                inner.snapshot.gps_global_origin = InitState::Requesting { attempt }
            }
        }

        self.snapshot_tx.send_replace(inner.snapshot.clone());
    }

    fn finish_on_silence(&self, domain: InitDomain) {
        let mut inner = self.inner.lock().unwrap();
        match domain {
            InitDomain::AutopilotVersion => finish_state_on_silence(
                &mut inner.snapshot.autopilot_version,
                marks_unavailable_on_silence(domain),
            ),
            InitDomain::AvailableModes => finish_state_on_silence(
                &mut inner.snapshot.available_modes,
                marks_unavailable_on_silence(domain),
            ),
            InitDomain::HomePosition => finish_state_on_silence(
                &mut inner.snapshot.home_position,
                marks_unavailable_on_silence(domain),
            ),
            InitDomain::GpsGlobalOrigin => finish_state_on_silence(
                &mut inner.snapshot.gps_global_origin,
                marks_unavailable_on_silence(domain),
            ),
        }

        self.snapshot_tx.send_replace(inner.snapshot.clone());
    }
}

fn finish_state_on_silence<T>(state: &mut InitState<T>, unavailable_on_silence: bool) {
    if !matches!(state, InitState::Requesting { .. }) {
        return;
    }

    *state = if unavailable_on_silence {
        InitState::Unavailable {
            reason: InitUnavailableReason::SilenceBudgetExhausted,
        }
    } else {
        InitState::Unknown
    };
}

fn marks_unavailable_on_silence(domain: InitDomain) -> bool {
    matches!(
        domain,
        InitDomain::AutopilotVersion | InitDomain::AvailableModes
    )
}

fn retry_delay(policy: &InitDomainPolicy) -> Duration {
    if policy.max_attempts == 0 {
        Duration::ZERO
    } else {
        policy.budget / u32::from(policy.max_attempts)
    }
}

async fn request_domain(
    connection: &SharedConnection,
    config: &VehicleConfig,
    target: &VehicleTarget,
    domain: InitDomain,
) -> Result<(), crate::error::VehicleError> {
    let message = match domain {
        InitDomain::HomePosition => request_home_position_command(target),
        InitDomain::AutopilotVersion => {
            request_message_command(target, AUTOPILOT_VERSION_MSG_ID, 0.0)
        }
        InitDomain::AvailableModes => request_message_command(target, AVAILABLE_MODES_MSG_ID, 0.0),
        InitDomain::GpsGlobalOrigin => {
            request_message_command(target, GPS_GLOBAL_ORIGIN_MSG_ID, 0.0)
        }
    };

    send_message(connection.as_ref(), config, message).await
}

fn request_message_command(
    target: &VehicleTarget,
    message_id: f32,
    param2: f32,
) -> dialect::MavMessage {
    dialect::MavMessage::COMMAND_LONG(dialect::COMMAND_LONG_DATA {
        target_system: target.system_id,
        target_component: target.component_id,
        command: MavCmd::MAV_CMD_REQUEST_MESSAGE,
        confirmation: 0,
        param1: message_id,
        param2,
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
    })
}

#[allow(deprecated)]
fn request_home_position_command(target: &VehicleTarget) -> dialect::MavMessage {
    dialect::MavMessage::COMMAND_LONG(dialect::COMMAND_LONG_DATA {
        target_system: target.system_id,
        target_component: target.component_id,
        command: MavCmd::MAV_CMD_GET_HOME_POSITION,
        confirmation: 0,
        param1: 0.0,
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
    })
}
