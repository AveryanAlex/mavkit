use crate::common::backend::{disconnect, setup_backend_vehicle};
use crate::common::target::TestTarget;
use mavkit::{ParamOperationProgress, ParamStore, Vehicle};
use std::time::Duration;

async fn download_all_params(vehicle: &Vehicle) -> Result<ParamStore, String> {
    vehicle
        .params()
        .download_all()
        .map_err(|err| err.to_string())?
        .wait()
        .await
        .map_err(|err| err.to_string())
}

pub async fn param_download_all_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;

    let result: Result<(), String> = async {
        let store = download_all_params(vehicle).await?;

        if store.params.is_empty() {
            return Err(String::from("expected non-empty param store"));
        }
        if store.expected_count == 0 {
            return Err(String::from("expected non-zero expected_count"));
        }
        if store.params.len() != store.expected_count as usize {
            return Err(format!(
                "param count mismatch: got {} params but expected_count is {}",
                store.params.len(),
                store.expected_count
            ));
        }
        if !store.params.contains_key("SYSID_THISMAV") {
            return Err(String::from("missing SYSID_THISMAV in downloaded params"));
        }

        Ok(())
    }
    .await;

    disconnect(backend).await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

pub async fn param_write_and_readback_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;

    let result: Result<(), String> = async {
        let store = download_all_params(vehicle).await?;

        let original = store
            .params
            .get("SR0_EXTRA1")
            .ok_or_else(|| String::from("SR0_EXTRA1 not found in params"))?
            .value;

        let new_value = if (original - 4.0).abs() < 0.01 {
            10.0
        } else {
            4.0
        };

        let confirmed = vehicle
            .params()
            .write("SR0_EXTRA1", new_value)
            .await
            .map_err(|err| err.to_string())?;

        if !confirmed.success {
            return Err(String::from(
                "single-parameter write did not report success",
            ));
        }
        if (confirmed.confirmed_value - new_value).abs() > 0.01 {
            return Err(format!(
                "write confirmation mismatch: requested {new_value}, got {}",
                confirmed.confirmed_value
            ));
        }

        let store = download_all_params(vehicle).await?;
        let readback = store
            .params
            .get("SR0_EXTRA1")
            .ok_or_else(|| String::from("SR0_EXTRA1 missing after write"))?
            .value;

        if (readback - new_value).abs() > 0.01 {
            return Err(format!(
                "readback mismatch: expected {new_value}, got {readback}"
            ));
        }

        vehicle
            .params()
            .write("SR0_EXTRA1", original)
            .await
            .map_err(|err| err.to_string())?;

        Ok(())
    }
    .await;

    disconnect(backend).await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

pub async fn param_progress_during_download_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;

    let result: Result<(), String> = async {
        let op = vehicle
            .params()
            .download_all()
            .map_err(|err| err.to_string())?;

        if !op
            .latest()
            .is_some_and(|progress| matches!(progress, ParamOperationProgress::Downloading { .. }))
        {
            let mut progress_stream = op.subscribe();
            let deadline = tokio::time::sleep(Duration::from_secs(10));
            tokio::pin!(deadline);
            loop {
                tokio::select! {
                    _ = &mut deadline => {
                        return Err(String::from("timed out waiting for param download progress"));
                    }
                    observed = progress_stream.recv() => {
                        let progress = observed.ok_or_else(|| String::from("param progress stream closed before download progress"))?;
                        if matches!(progress, ParamOperationProgress::Downloading { .. }) {
                            break;
                        }
                    }
                }
            }
        }

        let store = op.wait().await.map_err(|err| err.to_string())?;
        if store.expected_count == 0 {
            return Err(String::from(
                "expected non-zero expected count in downloaded store",
            ));
        }

        let progress = op.latest();
        if !matches!(progress, Some(ParamOperationProgress::Completed)) {
            return Err(format!(
                "expected completed param progress after download, got {progress:?}"
            ));
        }

        Ok(())
    }
    .await;

    disconnect(backend).await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

pub async fn param_write_batch_and_readback_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;

    let result: Result<(), String> = async {
        let store = download_all_params(vehicle).await?;

        let params_to_write: Vec<(&str, f32)> = vec![
            ("SR0_EXTRA1", 2.0),
            ("SR0_EXTRA2", 2.0),
            ("SR0_EXTRA3", 2.0),
        ];

        let originals: Vec<(String, f32)> = params_to_write
            .iter()
            .map(|(name, _)| {
                let val = store
                    .params
                    .get(*name)
                    .unwrap_or_else(|| panic!("{name} not found"))
                    .value;
                (name.to_string(), val)
            })
            .collect();

        let batch: Vec<(String, f32)> = params_to_write
            .iter()
            .map(|(name, val)| (name.to_string(), *val))
            .collect();

        let results = vehicle
            .params()
            .write_batch(batch)
            .map_err(|e| e.to_string())?
            .wait()
            .await
            .map_err(|e| e.to_string())?;

        if results.len() != params_to_write.len() {
            return Err(format!(
                "expected {} results, got {}",
                params_to_write.len(),
                results.len()
            ));
        }

        for result in &results {
            if !result.success {
                return Err(format!("batch write failed for {}", result.name));
            }
            if (result.confirmed_value - result.requested_value).abs() > 0.01 {
                return Err(format!(
                    "batch write mismatch for {}: requested {}, got {}",
                    result.name, result.requested_value, result.confirmed_value
                ));
            }
        }

        let store = download_all_params(vehicle).await?;
        for (name, expected) in &params_to_write {
            let actual = store
                .params
                .get(*name)
                .ok_or(format!("{name} missing after batch write"))?
                .value;
            if (actual - expected).abs() > 0.01 {
                return Err(format!(
                    "readback mismatch for {name}: expected {expected}, got {actual}"
                ));
            }
        }

        vehicle
            .params()
            .write_batch(originals)
            .map_err(|e| e.to_string())?
            .wait()
            .await
            .map_err(|e| e.to_string())?;

        Ok(())
    }
    .await;

    disconnect(backend).await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

pub async fn param_store_watch_updates_on_write_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;

    let result: Result<(), String> = async {
        download_all_params(vehicle).await?;

        let original_store = vehicle
            .params()
            .latest()
            .and_then(|state| state.store)
            .ok_or("parameter store unavailable after download")?;

        let original = original_store
            .params
            .get("SR0_EXTRA1")
            .ok_or("SR0_EXTRA1 not found")?
            .value;

        let new_value = if (original - 7.0).abs() < 0.01 {
            3.0
        } else {
            7.0
        };

        vehicle
            .params()
            .write("SR0_EXTRA1", new_value)
            .await
            .map_err(|e| e.to_string())?;

        tokio::time::sleep(Duration::from_millis(200)).await;
        let store = vehicle
            .params()
            .latest()
            .and_then(|state| state.store)
            .ok_or("parameter store unavailable after write")?;
        let readback = store
            .params
            .get("SR0_EXTRA1")
            .ok_or("SR0_EXTRA1 missing from watch store")?
            .value;

        if (readback - new_value).abs() > 0.01 {
            return Err(format!(
                "watch store not updated: expected {new_value}, got {readback}"
            ));
        }

        vehicle
            .params()
            .write("SR0_EXTRA1", original)
            .await
            .map_err(|e| e.to_string())?;

        Ok(())
    }
    .await;

    disconnect(backend).await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

pub async fn param_download_twice_is_consistent_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;

    let result: Result<(), String> = async {
        let first = download_all_params(vehicle).await?;
        let second = download_all_params(vehicle).await?;

        if first.expected_count != second.expected_count {
            return Err(format!(
                "expected_count changed between downloads: {} vs {}",
                first.expected_count, second.expected_count
            ));
        }

        if first.params.len() != second.params.len() {
            return Err(format!(
                "param count changed between downloads: {} vs {}",
                first.params.len(),
                second.params.len()
            ));
        }

        for (name, param) in &first.params {
            let other = second
                .params
                .get(name)
                .ok_or(format!("{name} missing in second download"))?;
            if (param.value - other.value).abs() > 0.001 {
                return Err(format!(
                    "{name} value differs: {} vs {}",
                    param.value, other.value
                ));
            }
        }

        Ok(())
    }
    .await;

    disconnect(backend).await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

pub async fn param_write_nonexistent_returns_error_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;

    let result: Result<(), String> = async {
        download_all_params(vehicle).await?;

        if let Ok(confirmed) = vehicle.params().write("ZZZZ_NONEXISTENT_PARAM", 42.0).await {
            if confirmed.success {
                return Err("write to nonexistent param unexpectedly reported success".into());
            }
        }

        Ok(())
    }
    .await;

    disconnect(backend).await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

pub async fn param_subscribe_emits_on_download_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;

    let result: Result<(), String> = async {
        let mut sub = vehicle.params().subscribe();

        let op = vehicle.params().download_all().map_err(|e| e.to_string())?;

        let deadline = tokio::time::sleep(Duration::from_secs(15));
        tokio::pin!(deadline);

        let received = tokio::select! {
            _ = &mut deadline => false,
            item = sub.recv() => item.is_some(),
        };

        let _ = op.wait().await;

        if !received {
            return Err("subscription did not emit any update during param download".into());
        }

        Ok(())
    }
    .await;

    disconnect(backend).await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

#[cfg(feature = "sim")]
#[allow(dead_code)]
pub async fn param_profile_specifics_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;

    let result: Result<(), String> = async {
        let expectations = target
            .param_profile_expectations()
            .ok_or_else(|| format!("no profile-specific param expectations for {target:?}"))?;
        let store = download_all_params(vehicle).await?;

        for name in expectations.expected_present {
            if !store.params.contains_key(*name) {
                return Err(format!("expected profile param {name} to be present"));
            }
        }

        for name in expectations.expected_absent {
            if store.params.contains_key(*name) {
                return Err(format!("expected profile param {name} to be absent"));
            }
        }

        for (name, expected_value) in expectations.expected_values {
            let param = store
                .params
                .get(*name)
                .ok_or_else(|| format!("expected profile param {name} to be present"))?;
            if (param.value - expected_value).abs() > f32::EPSILON {
                return Err(format!(
                    "expected profile param {name} to be {expected_value}, got {}",
                    param.value
                ));
            }
        }

        Ok(())
    }
    .await;

    disconnect(backend).await;
    if let Err(err) = result {
        panic!("{err}");
    }
}
