use super::operations::{self, ParamDownloadOp, ParamWriteBatchOp};
use super::types::{Param, ParamState, ParamWriteResult};
use crate::command::Command;
use crate::error::VehicleError;
use crate::observation::{ObservationHandle, ObservationSubscription};
use crate::operation::{run_with_progress_bridge, send_domain_command};
use crate::types::{ParamOperationKind, ParamOperationProgress};
use crate::vehicle::VehicleInner;
use std::time::Duration;
use tokio::sync::oneshot;

/// Accessor for parameter cache state and parameter operations.
///
/// Obtained from [`Vehicle::params`](crate::Vehicle::params).
///
/// Parameter names follow ArduPilot's convention: uppercase ASCII, up to 16 characters
/// (e.g. `"ARMING_CHECK"`, `"ATC_RAT_PIT_P"`). Names are matched exactly and are
/// case-sensitive on the wire.
///
/// # Conflict model
///
/// Parameter operations (`download_all`, `write_batch`, `write`) share the same
/// `MissionProtocolScope` as mission/fence/rally
/// transfers. Starting a param operation while any other protocol operation is active
/// returns [`VehicleError::OperationConflict`] immediately.
pub struct ParamsHandle<'a> {
    inner: &'a VehicleInner,
}

impl<'a> ParamsHandle<'a> {
    pub(crate) fn new(inner: &'a VehicleInner) -> Self {
        Self { inner }
    }

    /// Look up a single cached parameter by name from the last completed download.
    ///
    /// Returns `None` if no download has completed yet or if the name is absent.
    /// The value is stored as `f32` regardless of the underlying MAVLink parameter type.
    pub fn get(&self, name: &str) -> Option<Param> {
        self.inner
            .params
            .state()
            .latest()
            .and_then(|state| state.store)
            .and_then(|store| store.get(name).cloned())
    }

    /// Returns the most recently observed parameter state, or `None` if no update has arrived.
    pub fn latest(&self) -> Option<ParamState> {
        self.inner.params.state().latest()
    }

    /// Returns the current cached parameter state immediately when one is already available;
    /// otherwise waits for the first parameter state observed after the call.
    ///
    /// Returns the default state if the vehicle disconnects before any parameter state becomes
    /// available.
    pub async fn wait(&self) -> ParamState {
        self.inner.params.state().wait().await.unwrap_or_default()
    }

    /// Like [`wait`](Self::wait), but returns the current cached parameter state immediately when
    /// one is already available.
    ///
    /// If no parameter state is cached yet, this waits for the first observed parameter state and
    /// returns [`VehicleError::Timeout`] if it does not arrive within `timeout`. Returns
    /// [`VehicleError::Disconnected`] if the vehicle disconnects before any parameter state
    /// becomes available.
    pub async fn wait_timeout(&self, timeout: Duration) -> Result<ParamState, VehicleError> {
        self.inner.params.state().wait_timeout(timeout).await
    }

    /// Subscribes to an async stream of parameter state updates.
    pub fn subscribe(&self) -> ObservationSubscription<ParamState> {
        self.inner.params.state().subscribe()
    }

    /// Begins downloading all parameters from the vehicle.
    ///
    /// Returns a [`ParamDownloadOp`] handle. On success, the resolved [`crate::params::ParamStore`] contains all
    /// parameters indexed by name. On failure the previous cache is preserved but marked as
    /// [`SyncState::PossiblyStale`](crate::types::SyncState::PossiblyStale).
    ///
    /// The download can time out if the vehicle stops sending `PARAM_VALUE` messages before the
    /// expected count is reached.
    ///
    /// Returns [`VehicleError::OperationConflict`] immediately if any protocol operation is
    /// already active.
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
            let command_result = run_with_progress_bridge(
                param_progress_rx,
                progress_writer.clone(),
                cancel,
                std::convert::identity,
                send_domain_command(command_tx, |reply| Command::ParamDownloadAll { reply }),
            )
            .await;

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

    /// Writes multiple parameters in a single operation.
    ///
    /// Each entry is `(name, value)`. The vehicle processes writes sequentially and returns a
    /// `PARAM_VALUE` echo for each. The resolved `Vec<ParamWriteResult>` has one entry per input
    /// parameter in the same order. If any write fails, the overall sync state is marked
    /// [`SyncState::PossiblyStale`](crate::types::SyncState::PossiblyStale).
    ///
    /// Returns [`VehicleError::InvalidParameter`] if the batch is empty or exceeds the
    /// `u16::MAX` protocol limit.
    ///
    /// Returns [`VehicleError::OperationConflict`] immediately if any protocol operation is
    /// already active.
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
            let command_result = run_with_progress_bridge(
                param_progress_rx,
                progress_writer.clone(),
                cancel,
                std::convert::identity,
                send_domain_command(command_tx, |reply| Command::ParamWriteBatch {
                    params: batch,
                    reply,
                }),
            )
            .await;

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

    /// Writes a single parameter and waits for the vehicle's `PARAM_VALUE` echo.
    ///
    /// Returns [`VehicleError::OperationConflict`] immediately if any protocol operation is
    /// already active. Unlike `write_batch`, this method does not require a prior download
    /// and does not return an operation handle — it awaits completion directly.
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
    use crate::dialect;
    use crate::test_support::{
        ConnectedVehicleHarness, ConnectedVehicleOptions, default_header, fast_vehicle_test_config,
        heartbeat,
    };
    use crate::types::SyncState;
    use std::time::Duration;
    use tokio::sync::mpsc;
    use tokio::time::timeout;

    #[test]
    fn download_progress_variants() {
        let downloading = ParamOperationProgress::Downloading {
            received: 12,
            expected: Some(42),
        };
        let downloading_unknown = ParamOperationProgress::Downloading {
            received: 0,
            expected: None,
        };
        let failed = ParamOperationProgress::Failed;

        assert!(matches!(
            downloading,
            ParamOperationProgress::Downloading {
                received: 12,
                expected: Some(42)
            }
        ));
        assert!(matches!(
            downloading_unknown,
            ParamOperationProgress::Downloading {
                received: 0,
                expected: None
            }
        ));
        assert!(matches!(failed, ParamOperationProgress::Failed));
        assert_ne!(downloading, failed);
        assert_ne!(downloading, downloading_unknown);
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

    fn fast_config() -> crate::config::VehicleConfig {
        let mut config = fast_vehicle_test_config();
        config.connect_timeout = Duration::from_millis(200);
        config.command_completion_timeout = Duration::from_millis(200);
        config.retry_policy.max_retries = 0;
        config
    }

    async fn connect_mock_vehicle() -> (
        Vehicle,
        mpsc::Sender<(mavlink::MavHeader, dialect::MavMessage)>,
    ) {
        let harness = ConnectedVehicleHarness::connect(ConnectedVehicleOptions {
            config: fast_config(),
            heartbeat_message: heartbeat(false, 0),
            join_timeout: Duration::from_millis(400),
            message_capacity: 32,
            ..ConnectedVehicleOptions::default()
        })
        .await;

        (harness.vehicle, harness.msg_tx)
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
