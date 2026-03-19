pub mod file;
pub mod operations;
pub mod types;

pub use file::{format_param_file, parse_param_file};
pub use operations::{ParamDownloadOp, ParamWriteBatchOp};
pub use types::{Param, ParamState, ParamStore, ParamType, ParamWriteResult};

use crate::command::Command;
use crate::error::VehicleError;
use crate::mission::{MissionProtocolScope, OperationReservation, send_domain_command};
use crate::observation::{ObservationHandle, ObservationSubscription, ObservationWriter};
use crate::types::{ParamOperationKind, ParamOperationProgress, SyncState};
use crate::vehicle::VehicleInner;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::sync::{oneshot, watch};
use tokio_util::sync::CancellationToken;

#[derive(Clone)]
pub(crate) struct ParamsDomain {
    inner: Arc<ParamsDomainInner>,
}

struct ParamsDomainInner {
    state_writer: ObservationWriter<ParamState>,
    state: ObservationHandle<ParamState>,
    latest_state: Mutex<ParamState>,
}

impl ParamsDomain {
    pub(crate) fn new() -> Self {
        let (state_writer, state) = ObservationHandle::watch();
        let latest = ParamState::default();
        let _ = state_writer.publish(latest.clone());

        Self {
            inner: Arc::new(ParamsDomainInner {
                state_writer,
                state,
                latest_state: Mutex::new(latest),
            }),
        }
    }

    pub(crate) fn state(&self) -> ObservationHandle<ParamState> {
        self.inner.state.clone()
    }

    pub(crate) fn begin_operation(
        &self,
        scope: &MissionProtocolScope,
        kind: ParamOperationKind,
        op_name: &'static str,
    ) -> Result<OperationReservation, VehicleError> {
        let reservation = scope.begin_operation("params", op_name)?;
        self.update_state(|state| {
            state.active_op = Some(kind);
        });
        Ok(reservation)
    }

    pub(crate) fn finish_operation(&self, scope: &MissionProtocolScope, op_id: u64) {
        scope.finish_operation(op_id);
        self.update_state(|state| {
            state.active_op = None;
        });
    }

    pub(crate) fn note_download_success(&self, store: ParamStore) {
        self.update_state(|state| {
            state.store = Some(store);
            state.sync = SyncState::Current;
        });
    }

    pub(crate) fn note_write_result(&self, store: ParamStore, result: &ParamWriteResult) {
        self.update_state(|state| {
            state.store = Some(store);
            state.sync = if result.success {
                SyncState::Current
            } else {
                SyncState::PossiblyStale
            };
        });
    }

    pub(crate) fn note_batch_result(&self, store: ParamStore, results: &[ParamWriteResult]) {
        let all_ok = results.iter().all(|result| result.success);
        self.update_state(|state| {
            state.store = Some(store);
            state.sync = if all_ok {
                SyncState::Current
            } else {
                SyncState::PossiblyStale
            };
        });
    }

    pub(crate) fn note_operation_error(&self) {
        self.update_state(|state| {
            state.sync = SyncState::PossiblyStale;
        });
    }

    fn update_state(&self, edit: impl FnOnce(&mut ParamState)) {
        let mut latest = self.inner.latest_state.lock().unwrap();
        let mut next = latest.clone();
        edit(&mut next);
        if *latest != next {
            *latest = next.clone();
            let _ = self.inner.state_writer.publish(next);
        }
    }
}

pub(crate) fn spawn_param_progress_bridge(
    mut param_progress_rx: watch::Receiver<Option<ParamOperationProgress>>,
    progress_writer: ObservationWriter<ParamOperationProgress>,
    stop: CancellationToken,
) -> tokio::task::JoinHandle<()> {
    tokio::spawn(async move {
        let _ = param_progress_rx.borrow_and_update();

        loop {
            tokio::select! {
                _ = stop.cancelled() => break,
                changed = param_progress_rx.changed() => {
                    if changed.is_err() {
                        break;
                    }
                    if let Some(progress) = param_progress_rx.borrow_and_update().clone() {
                        let _ = progress_writer.publish(progress);
                    }
                }
            }
        }
    })
}

/// Accessor for parameter cache state and parameter operations.
pub struct ParamsHandle<'a> {
    inner: &'a VehicleInner,
}

impl<'a> ParamsHandle<'a> {
    pub(crate) fn new(inner: &'a VehicleInner) -> Self {
        Self { inner }
    }

    /// Look up a single cached parameter by name from the last download.
    ///
    /// Returns `None` if no download has completed or if the name is absent.
    pub fn get(&self, name: &str) -> Option<Param> {
        self.inner
            .params
            .state()
            .latest()
            .and_then(|state| state.store)
            .and_then(|store| store.get(name).cloned())
    }

    pub fn latest(&self) -> Option<ParamState> {
        self.inner.params.state().latest()
    }

    pub async fn wait(&self) -> ParamState {
        self.inner.params.state().wait().await.unwrap_or_default()
    }

    pub async fn wait_timeout(&self, timeout: Duration) -> Result<ParamState, VehicleError> {
        self.inner.params.state().wait_timeout(timeout).await
    }

    pub fn subscribe(&self) -> ObservationSubscription<ParamState> {
        self.inner.params.state().subscribe()
    }

    pub fn download_all(&self) -> Result<ParamDownloadOp, VehicleError> {
        let domain = self.inner.params.clone();
        let reservation = domain.begin_operation(
            &self.inner.mission_protocol,
            ParamOperationKind::DownloadAll,
            "download_all",
        )?;
        let (progress_writer, progress) = ObservationHandle::watch();
        let _ = progress_writer.publish(ParamOperationProgress::Downloading {
            received: 0,
            expected: None,
        });
        let (result_tx, result_rx) = oneshot::channel();

        let op =
            operations::ParamOperationHandle::new(progress, result_rx, reservation.cancel.clone());

        let command_tx = self.inner.command_tx.clone();
        let param_progress_rx = self.inner.stores.param_progress.clone();
        let mission_protocol = self.inner.mission_protocol.clone();
        let op_id = reservation.id;
        let cancel = reservation.cancel;

        tokio::spawn(async move {
            let progress_stop = CancellationToken::new();
            let progress_task = spawn_param_progress_bridge(
                param_progress_rx,
                progress_writer.clone(),
                progress_stop.clone(),
            );

            let command_result = tokio::select! {
                _ = cancel.cancelled() => Err(VehicleError::Cancelled),
                result = send_domain_command(command_tx, |reply| Command::ParamDownloadAll { reply }) => result,
            };

            progress_stop.cancel();
            let _ = progress_task.await;

            let result = match command_result {
                Ok(store) => {
                    domain.note_download_success(store.clone());
                    Ok(store)
                }
                Err(err) => {
                    domain.note_operation_error();
                    Err(err)
                }
            };

            let final_phase = match &result {
                Ok(_) => ParamOperationProgress::Completed,
                Err(VehicleError::Cancelled) => ParamOperationProgress::Cancelled,
                Err(_) => ParamOperationProgress::Failed,
            };
            let _ = progress_writer.publish(final_phase);

            domain.finish_operation(&mission_protocol, op_id);
            let _ = result_tx.send(result);
        });

        Ok(op)
    }

    pub fn write_batch(
        &self,
        batch: Vec<(String, f32)>,
    ) -> Result<ParamWriteBatchOp, VehicleError> {
        if batch.is_empty() {
            return Err(VehicleError::InvalidParameter(
                "write_batch requires at least one parameter".to_string(),
            ));
        }

        let total = u16::try_from(batch.len()).map_err(|_| {
            VehicleError::InvalidParameter(format!(
                "write_batch has {} params, exceeding the {} protocol limit",
                batch.len(),
                u16::MAX
            ))
        })?;
        let first_name = batch[0].0.clone();
        let domain = self.inner.params.clone();
        let reservation = domain.begin_operation(
            &self.inner.mission_protocol,
            ParamOperationKind::WriteBatch,
            "write_batch",
        )?;
        let (progress_writer, progress) = ObservationHandle::watch();
        let _ = progress_writer.publish(ParamOperationProgress::Writing {
            index: 0,
            total,
            name: first_name,
        });
        let (result_tx, result_rx) = oneshot::channel();

        let op =
            operations::ParamOperationHandle::new(progress, result_rx, reservation.cancel.clone());

        let command_tx = self.inner.command_tx.clone();
        let param_progress_rx = self.inner.stores.param_progress.clone();
        let param_store_rx = self.inner.stores.param_store.clone();
        let mission_protocol = self.inner.mission_protocol.clone();
        let op_id = reservation.id;
        let cancel = reservation.cancel;

        tokio::spawn(async move {
            let progress_stop = CancellationToken::new();
            let progress_task = spawn_param_progress_bridge(
                param_progress_rx,
                progress_writer.clone(),
                progress_stop.clone(),
            );

            let command_result = tokio::select! {
                _ = cancel.cancelled() => Err(VehicleError::Cancelled),
                result = send_domain_command(command_tx, |reply| Command::ParamWriteBatch {
                    params: batch,
                    reply,
                }) => result,
            };

            progress_stop.cancel();
            let _ = progress_task.await;

            let result = match command_result {
                Ok(results) => {
                    let store = param_store_rx.borrow().clone();
                    domain.note_batch_result(store, &results);
                    Ok(results)
                }
                Err(err) => {
                    domain.note_operation_error();
                    Err(err)
                }
            };

            let final_phase = match &result {
                Ok(results) if results.iter().all(|result| result.success) => {
                    ParamOperationProgress::Completed
                }
                Ok(_) => ParamOperationProgress::Failed,
                Err(VehicleError::Cancelled) => ParamOperationProgress::Cancelled,
                Err(_) => ParamOperationProgress::Failed,
            };
            let _ = progress_writer.publish(final_phase);

            domain.finish_operation(&mission_protocol, op_id);
            let _ = result_tx.send(result);
        });

        Ok(op)
    }

    pub async fn write(&self, name: &str, value: f32) -> Result<ParamWriteResult, VehicleError> {
        let reservation = self
            .inner
            .mission_protocol
            .begin_operation("params", "write")?;
        let result =
            send_domain_command(self.inner.command_tx.clone(), |reply| Command::ParamWrite {
                name: name.to_string(),
                value,
                reply,
            })
            .await;
        self.inner.mission_protocol.finish_operation(reservation.id);

        match result {
            Ok(write_result) => {
                let store = self.inner.stores.param_store.borrow().clone();
                self.inner.params.note_write_result(store, &write_result);
                Ok(write_result)
            }
            Err(err) => {
                self.inner.params.note_operation_error();
                Err(err)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Vehicle;
    use crate::config::VehicleConfig;
    use crate::dialect;
    use crate::test_support::MockConnection;
    use crate::types::SyncState;
    use mavlink::MavHeader;
    use std::time::Duration;
    use tokio::sync::mpsc;
    use tokio::time::timeout;

    #[test]
    fn download_progress() {
        let progress = ParamOperationProgress::Downloading {
            received: 12,
            expected: Some(42),
        };

        assert_eq!(
            progress,
            ParamOperationProgress::Downloading {
                received: 12,
                expected: Some(42),
            }
        );
    }

    fn default_header() -> MavHeader {
        MavHeader {
            system_id: 1,
            component_id: 1,
            sequence: 0,
        }
    }

    fn heartbeat_msg() -> dialect::MavMessage {
        dialect::MavMessage::HEARTBEAT(dialect::HEARTBEAT_DATA {
            custom_mode: 0,
            mavtype: dialect::MavType::MAV_TYPE_QUADROTOR,
            autopilot: dialect::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode: dialect::MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            system_status: dialect::MavState::MAV_STATE_STANDBY,
            mavlink_version: 3,
        })
    }

    fn param_value_msg(
        name: &str,
        value: f32,
        param_index: u16,
        param_count: u16,
    ) -> dialect::MavMessage {
        dialect::MavMessage::PARAM_VALUE(dialect::PARAM_VALUE_DATA {
            param_id: name.into(),
            param_value: value,
            param_type: dialect::MavParamType::MAV_PARAM_TYPE_REAL32,
            param_count,
            param_index,
        })
    }

    fn fast_config() -> VehicleConfig {
        let mut config = VehicleConfig {
            connect_timeout: Duration::from_millis(200),
            command_timeout: Duration::from_millis(50),
            command_completion_timeout: Duration::from_millis(200),
            auto_request_home: false,
            ..VehicleConfig::default()
        };
        config.retry_policy.max_retries = 0;
        config
    }

    async fn connect_mock_vehicle() -> (Vehicle, mpsc::Sender<(MavHeader, dialect::MavMessage)>) {
        let (msg_tx, msg_rx) = mpsc::channel(32);
        let (conn, _sent) = MockConnection::new(msg_rx);
        let connect_task =
            tokio::spawn(
                async move { Vehicle::from_connection(Box::new(conn), fast_config()).await },
            );

        msg_tx
            .send((default_header(), heartbeat_msg()))
            .await
            .expect("heartbeat should be delivered");

        let vehicle = timeout(Duration::from_millis(400), connect_task)
            .await
            .expect("connect should finish")
            .expect("connect task should join")
            .expect("vehicle should connect");

        (vehicle, msg_tx)
    }

    #[tokio::test]
    async fn partial_download_preserves() {
        let (vehicle, msg_tx) = connect_mock_vehicle().await;

        let first_op = vehicle
            .params()
            .download_all()
            .expect("first download op should start");
        let first_wait = tokio::spawn(async move { first_op.wait().await });
        tokio::time::sleep(Duration::from_millis(20)).await;

        msg_tx
            .send((default_header(), param_value_msg("BASE_A", 1.0, 0, 2)))
            .await
            .expect("first param should be delivered");
        msg_tx
            .send((default_header(), param_value_msg("BASE_B", 2.0, 1, 2)))
            .await
            .expect("second param should be delivered");

        let baseline = timeout(Duration::from_secs(4), first_wait)
            .await
            .expect("first download should complete")
            .expect("first download task should join")
            .expect("first download should succeed");
        assert_eq!(baseline.expected_count, 2);
        assert_eq!(baseline.params.len(), 2);

        let second_op = vehicle
            .params()
            .download_all()
            .expect("second download op should start");
        let second_wait = tokio::spawn(async move { second_op.wait().await });
        tokio::time::sleep(Duration::from_millis(20)).await;

        msg_tx
            .send((default_header(), param_value_msg("BASE_A", 9.0, 0, 2)))
            .await
            .expect("partial param should be delivered");
        msg_tx
            .send((
                default_header(),
                param_value_msg("OUT_OF_RANGE", 77.0, 999, 2),
            ))
            .await
            .expect("invalid index param should be delivered");

        let second_result = timeout(Duration::from_secs(7), second_wait)
            .await
            .expect("second download should resolve")
            .expect("second download task should join");
        assert!(matches!(second_result, Err(VehicleError::Timeout(_))));

        let latest = vehicle
            .params()
            .latest()
            .expect("params state should be available");
        assert_eq!(latest.store, Some(baseline.clone()));
        assert_eq!(latest.sync, SyncState::PossiblyStale);
        let cached_store = latest
            .store
            .clone()
            .expect("cached store should be preserved after partial timeout");
        assert_eq!(cached_store, baseline);

        vehicle
            .disconnect()
            .await
            .expect("disconnect should succeed");
    }
}
