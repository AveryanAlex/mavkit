use super::{CommandContext, get_target, recv_routed, send_message};
use crate::dialect::{self, MavParamType};
use crate::error::VehicleError;
use crate::params::{Param, ParamStore, ParamType, ParamWriteResult};
use crate::types::ParamOperationProgress;
use std::collections::{HashMap, HashSet};
use std::time::Duration;
use tracing::{debug, warn};

const PARAM_WRITE_TOLERANCE: f32 = 0.01;

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
    ctx: &mut CommandContext,
) -> Result<ParamStore, VehicleError> {
    let target = get_target(&ctx.vehicle_target)?;

    ctx.writers
        .param_progress
        .send_replace(Some(ParamOperationProgress::Downloading {
            received: 0,
            expected: None,
        }));

    // Send PARAM_REQUEST_LIST
    send_message(
        ctx.connection.as_ref(),
        &ctx.config,
        dialect::MavMessage::PARAM_REQUEST_LIST(dialect::PARAM_REQUEST_LIST_DATA {
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
    let max_retries = u32::from(ctx.config.retry_policy.max_retries);
    let mut retries = 0u32;

    loop {
        let timeout = Duration::from_secs(2);
        let deadline = tokio::time::sleep(timeout);
        tokio::pin!(deadline);
        let cancel = ctx.cancel.clone();

        let mut got_new = false;

        loop {
            tokio::select! {
                biased;
                _ = cancel.cancelled() => {
                    ctx.writers
                        .param_progress
                        .send_replace(Some(ParamOperationProgress::Cancelled));
                    return Err(VehicleError::Cancelled);
                }
                _ = &mut deadline => break,
                result = recv_routed(&mut ctx.inbound_rx) => {
                    let (_, msg) = result?;

                    if let dialect::MavMessage::PARAM_VALUE(data) = &msg {
                        let name = param_id_to_string(&data.param_id);
                        if name.is_empty() {
                            continue;
                        }

                        if !count_known && data.param_count > 0 {
                            expected_count = data.param_count;
                            count_known = true;
                        }

                        if count_known && data.param_index >= expected_count {
                            continue;
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
                            ctx.writers
                                .param_progress
                                .send_replace(Some(ParamOperationProgress::Downloading {
                                    received,
                                    expected: count_known.then_some(expected_count),
                                }));
                        }

                        // Reset deadline on new data
                        deadline.as_mut().reset(tokio::time::Instant::now() + Duration::from_secs(2));
                    }
                }
            }
        }

        // Timeout reached — check if we're done
        let received = params.len() as u16;
        let download_complete = count_known
            && received >= expected_count
            && (0..expected_count).all(|idx| received_indices.contains(&idx));
        if download_complete {
            break; // Done
        }

        if !got_new {
            retries += 1;
            if retries > max_retries {
                warn!(
                    "param download failed: received {}/{} after {} retries",
                    received, expected_count, max_retries
                );
                ctx.writers
                    .param_progress
                    .send_replace(Some(ParamOperationProgress::Failed));
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
                        ctx.connection.as_ref(),
                        &ctx.config,
                        dialect::MavMessage::PARAM_REQUEST_READ(dialect::PARAM_REQUEST_READ_DATA {
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

    ctx.writers.param_store.send_replace(store.clone());
    ctx.writers
        .param_progress
        .send_replace(Some(ParamOperationProgress::Completed));

    Ok(store)
}

// ---------------------------------------------------------------------------
// Parameter Write
// ---------------------------------------------------------------------------

pub(super) async fn handle_param_write(
    name: &str,
    value: f32,
    ctx: &mut CommandContext,
) -> Result<ParamWriteResult, VehicleError> {
    let target = get_target(&ctx.vehicle_target)?;

    // Look up current param_type from store, or default to Real32
    let param_type = {
        let store = ctx.writers.param_store.borrow();
        store
            .params
            .get(name)
            .map(|p| p.param_type)
            .unwrap_or(ParamType::Real32)
    };

    let max_retries = ctx.config.retry_policy.max_retries;
    let request_timeout_ms = ctx.config.retry_policy.request_timeout_ms;

    for _attempt in 0..=max_retries {
        send_message(
            ctx.connection.as_ref(),
            &ctx.config,
            dialect::MavMessage::PARAM_SET(dialect::PARAM_SET_DATA {
                param_value: value,
                target_system: target.system_id,
                target_component: target.component_id,
                param_id: string_to_param_id(name),
                param_type: to_mav_param_type(param_type),
            }),
        )
        .await?;

        let timeout = Duration::from_millis(request_timeout_ms);
        let deadline = tokio::time::sleep(timeout);
        tokio::pin!(deadline);
        let cancel = ctx.cancel.clone();

        loop {
            tokio::select! {
                biased;
                _ = cancel.cancelled() => return Err(VehicleError::Cancelled),
                _ = &mut deadline => break, // retry
                result = recv_routed(&mut ctx.inbound_rx) => {
                    let (_, msg) = result?;

                    if let dialect::MavMessage::PARAM_VALUE(data) = &msg {
                        let received_name = param_id_to_string(&data.param_id);
                        if received_name == name {
                            let confirmed = Param {
                                name: received_name.clone(),
                                value: data.param_value,
                                param_type: from_mav_param_type(data.param_type),
                                index: data.param_index,
                            };

                            // Update store
                            ctx.writers.param_store.send_modify(|store| {
                                store.params.insert(received_name, confirmed.clone());
                            });

                            return Ok(ParamWriteResult {
                                name: confirmed.name,
                                requested_value: value,
                                confirmed_value: confirmed.value,
                                success: (confirmed.value - value).abs() <= PARAM_WRITE_TOLERANCE,
                            });
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
    ctx: &mut CommandContext,
) -> Result<Vec<ParamWriteResult>, VehicleError> {
    let total = params.len() as u16;
    let mut results = Vec::with_capacity(params.len());

    for (i, (name, value)) in params.into_iter().enumerate() {
        ctx.writers
            .param_progress
            .send_replace(Some(ParamOperationProgress::Writing {
                index: i as u16,
                total,
                name: name.clone(),
            }));

        let result = handle_param_write(&name, value, ctx).await;
        match result {
            Ok(write_result) => {
                results.push(write_result);
            }
            Err(VehicleError::Cancelled) => {
                ctx.writers
                    .param_progress
                    .send_replace(Some(ParamOperationProgress::Cancelled));
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
    }

    let all_ok = results.iter().all(|r| r.success);
    ctx.writers.param_progress.send_replace(Some(if all_ok {
        ParamOperationProgress::Completed
    } else {
        ParamOperationProgress::Failed
    }));

    Ok(results)
}
