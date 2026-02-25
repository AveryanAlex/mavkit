use super::{VehicleTarget, get_target, send_message, update_state, update_vehicle_target};
use crate::config::VehicleConfig;
use crate::error::VehicleError;
use crate::params::{
    Param, ParamProgress, ParamStore, ParamTransferPhase, ParamType, ParamWriteResult,
};
use crate::state::StateWriters;
use mavlink::AsyncMavConnection;
use mavlink::common::{self, MavParamType};
use std::collections::{HashMap, HashSet};
use std::time::Duration;
use tokio_util::sync::CancellationToken;
use tracing::{debug, warn};

pub(super) fn from_mav_param_type(mav: MavParamType) -> ParamType {
    match mav {
        MavParamType::MAV_PARAM_TYPE_UINT8 => ParamType::Uint8,
        MavParamType::MAV_PARAM_TYPE_INT8 => ParamType::Int8,
        MavParamType::MAV_PARAM_TYPE_UINT16 => ParamType::Uint16,
        MavParamType::MAV_PARAM_TYPE_INT16 => ParamType::Int16,
        MavParamType::MAV_PARAM_TYPE_UINT32 => ParamType::Uint32,
        MavParamType::MAV_PARAM_TYPE_INT32 => ParamType::Int32,
        _ => ParamType::Real32,
    }
}

pub(super) fn to_mav_param_type(pt: ParamType) -> MavParamType {
    match pt {
        ParamType::Uint8 => MavParamType::MAV_PARAM_TYPE_UINT8,
        ParamType::Int8 => MavParamType::MAV_PARAM_TYPE_INT8,
        ParamType::Uint16 => MavParamType::MAV_PARAM_TYPE_UINT16,
        ParamType::Int16 => MavParamType::MAV_PARAM_TYPE_INT16,
        ParamType::Uint32 => MavParamType::MAV_PARAM_TYPE_UINT32,
        ParamType::Int32 => MavParamType::MAV_PARAM_TYPE_INT32,
        ParamType::Real32 => MavParamType::MAV_PARAM_TYPE_REAL32,
    }
}

pub(super) fn param_id_to_string(param_id: &mavlink::types::CharArray<16>) -> String {
    param_id.to_str().unwrap_or("").to_string()
}

pub(super) fn string_to_param_id(name: &str) -> mavlink::types::CharArray<16> {
    name.into()
}

// ---------------------------------------------------------------------------
// Parameter Download All
// ---------------------------------------------------------------------------

pub(super) async fn handle_param_download_all(
    connection: &(dyn AsyncMavConnection<common::MavMessage> + Sync + Send),
    writers: &StateWriters,
    vehicle_target: &mut Option<VehicleTarget>,
    config: &VehicleConfig,
    cancel: &CancellationToken,
) -> Result<ParamStore, VehicleError> {
    let target = get_target(vehicle_target)?;

    // Reset progress
    let _ = writers.param_progress.send(ParamProgress {
        phase: ParamTransferPhase::Downloading,
        received: 0,
        expected: 0,
    });

    // Send PARAM_REQUEST_LIST
    send_message(
        connection,
        config,
        common::MavMessage::PARAM_REQUEST_LIST(common::PARAM_REQUEST_LIST_DATA {
            target_system: target.system_id,
            target_component: target.component_id,
        }),
    )
    .await?;

    let mut params: HashMap<String, Param> = HashMap::new();
    let mut received_indices: HashSet<u16> = HashSet::new();
    let mut expected_count: u16 = 0;
    let mut count_known = false;
    let mut last_progress_update = 0u16;
    let max_retries = 3u32;
    let mut retries = 0u32;

    loop {
        let timeout = Duration::from_secs(2);
        let deadline = tokio::time::sleep(timeout);
        tokio::pin!(deadline);

        let mut got_new = false;

        loop {
            tokio::select! {
                biased;
                _ = cancel.cancelled() => {
                    let _ = writers.param_progress.send(ParamProgress {
                        phase: ParamTransferPhase::Failed,
                        received: params.len() as u16,
                        expected: expected_count,
                    });
                    return Err(VehicleError::Cancelled);
                }
                _ = &mut deadline => break,
                result = connection.recv() => {
                    let (header, msg) = result
                        .map_err(|err| VehicleError::Io(std::io::Error::other(err.to_string())))?;
                    update_vehicle_target(vehicle_target, &header, &msg);
                    update_state(&header, &msg, writers, vehicle_target);

                    if let common::MavMessage::PARAM_VALUE(data) = &msg {
                        let name = param_id_to_string(&data.param_id);
                        if name.is_empty() {
                            continue;
                        }

                        if !count_known && data.param_count > 0 {
                            expected_count = data.param_count;
                            count_known = true;
                        }

                        if received_indices.insert(data.param_index) {
                            got_new = true;
                            params.insert(name.clone(), Param {
                                name,
                                value: data.param_value,
                                param_type: from_mav_param_type(data.param_type),
                                index: data.param_index,
                            });
                        }

                        // Update progress every 50 params
                        let received = params.len() as u16;
                        if received - last_progress_update >= 50 || received >= expected_count {
                            last_progress_update = received;
                            let _ = writers.param_progress.send(ParamProgress {
                                phase: ParamTransferPhase::Downloading,
                                received,
                                expected: expected_count,
                            });
                        }

                        // Reset deadline on new data
                        deadline.as_mut().reset(tokio::time::Instant::now() + Duration::from_secs(2));
                    }
                }
            }
        }

        // Timeout reached — check if we're done
        let received = params.len() as u16;
        if count_known && received >= expected_count {
            break; // Done
        }

        if !got_new {
            retries += 1;
            if retries > max_retries {
                // Accept partial if we have more than 50% of expected
                if count_known && received > expected_count / 2 {
                    warn!(
                        "param download: accepting partial {}/{} after {} retries",
                        received, expected_count, max_retries
                    );
                    break;
                }
                let _ = writers.param_progress.send(ParamProgress {
                    phase: ParamTransferPhase::Failed,
                    received,
                    expected: expected_count,
                });
                return Err(VehicleError::Timeout);
            }
        } else {
            retries = 0;
        }

        // Request missing indices
        if count_known {
            let mut missing_requested = 0u32;
            for idx in 0..expected_count {
                if !received_indices.contains(&idx) {
                    send_message(
                        connection,
                        config,
                        common::MavMessage::PARAM_REQUEST_READ(common::PARAM_REQUEST_READ_DATA {
                            param_index: idx as i16,
                            target_system: target.system_id,
                            target_component: target.component_id,
                            param_id: string_to_param_id(""),
                        }),
                    )
                    .await?;
                    missing_requested += 1;
                    if missing_requested >= 10 {
                        break; // Don't flood, request in batches
                    }
                }
            }
            debug!(
                "param download: requested {} missing params (retry {})",
                missing_requested, retries
            );
        }
    }

    let store = ParamStore {
        params,
        expected_count,
    };

    let _ = writers.param_store.send(store.clone());
    let _ = writers.param_progress.send(ParamProgress {
        phase: ParamTransferPhase::Completed,
        received: store.params.len() as u16,
        expected: expected_count,
    });

    Ok(store)
}

// ---------------------------------------------------------------------------
// Parameter Write
// ---------------------------------------------------------------------------

pub(super) async fn handle_param_write(
    name: &str,
    value: f32,
    connection: &(dyn AsyncMavConnection<common::MavMessage> + Sync + Send),
    writers: &StateWriters,
    vehicle_target: &mut Option<VehicleTarget>,
    config: &VehicleConfig,
    cancel: &CancellationToken,
) -> Result<Param, VehicleError> {
    let target = get_target(vehicle_target)?;

    // Look up current param_type from store, or default to Real32
    let param_type = {
        let store = writers.param_store.borrow();
        store
            .params
            .get(name)
            .map(|p| p.param_type)
            .unwrap_or(ParamType::Real32)
    };

    let retry_policy = &config.retry_policy;

    for _attempt in 0..=retry_policy.max_retries {
        send_message(
            connection,
            config,
            common::MavMessage::PARAM_SET(common::PARAM_SET_DATA {
                param_value: value,
                target_system: target.system_id,
                target_component: target.component_id,
                param_id: string_to_param_id(name),
                param_type: to_mav_param_type(param_type),
            }),
        )
        .await?;

        let timeout = Duration::from_millis(retry_policy.request_timeout_ms);
        let deadline = tokio::time::sleep(timeout);
        tokio::pin!(deadline);

        loop {
            tokio::select! {
                biased;
                _ = cancel.cancelled() => return Err(VehicleError::Cancelled),
                _ = &mut deadline => break, // retry
                result = connection.recv() => {
                    let (header, msg) = result
                        .map_err(|err| VehicleError::Io(std::io::Error::other(err.to_string())))?;
                    update_vehicle_target(vehicle_target, &header, &msg);
                    update_state(&header, &msg, writers, vehicle_target);

                    if let common::MavMessage::PARAM_VALUE(data) = &msg {
                        let received_name = param_id_to_string(&data.param_id);
                        if received_name == name {
                            let confirmed = Param {
                                name: received_name.clone(),
                                value: data.param_value,
                                param_type: from_mav_param_type(data.param_type),
                                index: data.param_index,
                            };

                            // Update store
                            writers.param_store.send_modify(|store| {
                                store.params.insert(received_name, confirmed.clone());
                            });

                            return Ok(confirmed);
                        }
                    }
                }
            }
        }
    }

    Err(VehicleError::Timeout)
}

// ---------------------------------------------------------------------------
// Parameter Batch Write
// ---------------------------------------------------------------------------

pub(super) async fn handle_param_write_batch(
    params: Vec<(String, f32)>,
    connection: &(dyn AsyncMavConnection<common::MavMessage> + Sync + Send),
    writers: &StateWriters,
    vehicle_target: &mut Option<VehicleTarget>,
    config: &VehicleConfig,
    cancel: &CancellationToken,
) -> Result<Vec<ParamWriteResult>, VehicleError> {
    let total = params.len() as u16;
    let mut results = Vec::with_capacity(params.len());

    let _ = writers.param_progress.send(ParamProgress {
        phase: ParamTransferPhase::Writing,
        received: 0,
        expected: total,
    });

    for (i, (name, value)) in params.into_iter().enumerate() {
        let result = handle_param_write(
            &name,
            value,
            connection,
            writers,
            vehicle_target,
            config,
            cancel,
        )
        .await;
        match result {
            Ok(confirmed) => {
                results.push(ParamWriteResult {
                    name,
                    requested_value: value,
                    confirmed_value: confirmed.value,
                    success: true,
                });
            }
            Err(VehicleError::Cancelled) => {
                let _ = writers.param_progress.send(ParamProgress {
                    phase: ParamTransferPhase::Failed,
                    received: i as u16,
                    expected: total,
                });
                return Err(VehicleError::Cancelled);
            }
            Err(_) => {
                results.push(ParamWriteResult {
                    name,
                    requested_value: value,
                    confirmed_value: 0.0,
                    success: false,
                });
            }
        }

        let _ = writers.param_progress.send(ParamProgress {
            phase: ParamTransferPhase::Writing,
            received: (i + 1) as u16,
            expected: total,
        });
    }

    let all_ok = results.iter().all(|r| r.success);
    let _ = writers.param_progress.send(ParamProgress {
        phase: if all_ok {
            ParamTransferPhase::Completed
        } else {
            ParamTransferPhase::Failed
        },
        received: results.iter().filter(|r| r.success).count() as u16,
        expected: total,
    });

    Ok(results)
}
