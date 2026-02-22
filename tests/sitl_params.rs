#[allow(dead_code)]
mod common;

use mavkit::{ParamTransferPhase, Vehicle};
use std::time::Duration;

async fn wait_for_param_progress<F>(vehicle: &Vehicle, mut predicate: F, timeout: Duration)
where
    F: FnMut(&mavkit::ParamProgress) -> bool,
{
    let mut rx = vehicle.param_progress();
    let deadline = tokio::time::sleep(timeout);
    tokio::pin!(deadline);
    loop {
        tokio::select! {
            _ = &mut deadline => panic!("timed out waiting for param progress"),
            result = rx.changed() => {
                result.expect("watch channel closed");
                let progress = rx.borrow().clone();
                if predicate(&progress) {
                    return;
                }
            }
        }
    }
}

#[tokio::test]
#[ignore = "requires ArduPilot SITL endpoint"]
async fn sitl_param_download_all() {
    let vehicle = common::setup_sitl_vehicle().await;

    let result: Result<(), String> = async {
        let store = vehicle
            .params()
            .download_all()
            .await
            .map_err(|e| e.to_string())?;

        if store.params.is_empty() {
            return Err("expected non-empty param store".into());
        }
        if store.expected_count == 0 {
            return Err("expected non-zero expected_count".into());
        }
        if store.params.len() != store.expected_count as usize {
            return Err(format!(
                "param count mismatch: got {} params but expected_count is {}",
                store.params.len(),
                store.expected_count
            ));
        }

        // Verify a param that every ArduPilot vehicle has
        if !store.params.contains_key("SYSID_THISMAV") {
            return Err("missing SYSID_THISMAV in downloaded params".into());
        }

        Ok(())
    }
    .await;

    let _ = vehicle.disconnect().await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

#[tokio::test]
#[ignore = "requires ArduPilot SITL endpoint"]
async fn sitl_param_write_and_readback() {
    let vehicle = common::setup_sitl_vehicle().await;

    let result: Result<(), String> = async {
        let store = vehicle
            .params()
            .download_all()
            .await
            .map_err(|e| e.to_string())?;

        let original = store
            .params
            .get("SR0_EXTRA1")
            .ok_or("SR0_EXTRA1 not found in params")?
            .value;

        // Toggle to a different value
        let new_value = if (original - 4.0).abs() < 0.01 {
            10.0
        } else {
            4.0
        };

        let confirmed = vehicle
            .params()
            .write("SR0_EXTRA1".into(), new_value)
            .await
            .map_err(|e| e.to_string())?;

        if (confirmed.value - new_value).abs() > 0.01 {
            return Err(format!(
                "write confirmation mismatch: requested {new_value}, got {}",
                confirmed.value
            ));
        }

        // Download again and verify persistence
        let store = vehicle
            .params()
            .download_all()
            .await
            .map_err(|e| e.to_string())?;

        let readback = store
            .params
            .get("SR0_EXTRA1")
            .ok_or("SR0_EXTRA1 missing after write")?
            .value;

        if (readback - new_value).abs() > 0.01 {
            return Err(format!(
                "readback mismatch: expected {new_value}, got {readback}"
            ));
        }

        // Restore original
        vehicle
            .params()
            .write("SR0_EXTRA1".into(), original)
            .await
            .map_err(|e| e.to_string())?;

        Ok(())
    }
    .await;

    let _ = vehicle.disconnect().await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

#[tokio::test]
#[ignore = "requires ArduPilot SITL endpoint"]
async fn sitl_param_write_batch_and_readback() {
    let vehicle = common::setup_sitl_vehicle().await;

    let result: Result<(), String> = async {
        let store = vehicle
            .params()
            .download_all()
            .await
            .map_err(|e| e.to_string())?;

        let params_to_write: Vec<(&str, f32)> = vec![
            ("SR0_EXTRA1", 2.0),
            ("SR0_EXTRA2", 2.0),
            ("SR0_EXTRA3", 2.0),
        ];

        // Save originals for restore
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

        // Verify via full download
        let store = vehicle
            .params()
            .download_all()
            .await
            .map_err(|e| e.to_string())?;

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

        // Restore originals
        vehicle
            .params()
            .write_batch(originals)
            .await
            .map_err(|e| e.to_string())?;

        Ok(())
    }
    .await;

    let _ = vehicle.disconnect().await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

#[tokio::test]
#[ignore = "requires ArduPilot SITL endpoint"]
async fn sitl_param_progress_during_download() {
    let vehicle = common::setup_sitl_vehicle().await;

    let result: Result<(), String> = async {
        let download = tokio::spawn({
            let vehicle = vehicle.clone();
            async move {
                vehicle
                    .params()
                    .download_all()
                    .await
                    .map_err(|e| e.to_string())
            }
        });

        // Wait for downloading phase to appear
        wait_for_param_progress(
            &vehicle,
            |p| p.phase == ParamTransferPhase::Downloading,
            Duration::from_secs(10),
        )
        .await;

        let store = download.await.unwrap()?;

        // After download completes, progress should reflect completion
        let progress = vehicle.param_progress().borrow().clone();
        if progress.expected == 0 {
            return Err("expected non-zero expected count in progress".into());
        }
        if progress.expected != store.expected_count {
            return Err(format!(
                "progress expected count ({}) doesn't match store ({})",
                progress.expected, store.expected_count
            ));
        }

        Ok(())
    }
    .await;

    let _ = vehicle.disconnect().await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

#[tokio::test]
#[ignore = "requires ArduPilot SITL endpoint"]
async fn sitl_param_store_watch_updates_on_write() {
    let vehicle = common::setup_sitl_vehicle().await;

    let result: Result<(), String> = async {
        // Populate the store first
        vehicle
            .params()
            .download_all()
            .await
            .map_err(|e| e.to_string())?;

        let original = vehicle
            .param_store()
            .borrow()
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
            .write("SR0_EXTRA1".into(), new_value)
            .await
            .map_err(|e| e.to_string())?;

        // The watch channel should reflect the updated value
        tokio::time::sleep(Duration::from_millis(200)).await;
        let store = vehicle.param_store().borrow().clone();
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

        // Restore
        vehicle
            .params()
            .write("SR0_EXTRA1".into(), original)
            .await
            .map_err(|e| e.to_string())?;

        Ok(())
    }
    .await;

    let _ = vehicle.disconnect().await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

#[tokio::test]
#[ignore = "requires ArduPilot SITL endpoint"]
async fn sitl_param_download_twice_is_consistent() {
    let vehicle = common::setup_sitl_vehicle().await;

    let result: Result<(), String> = async {
        let first = vehicle
            .params()
            .download_all()
            .await
            .map_err(|e| e.to_string())?;

        let second = vehicle
            .params()
            .download_all()
            .await
            .map_err(|e| e.to_string())?;

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

        // Values should be identical (no writes in between)
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

    let _ = vehicle.disconnect().await;
    if let Err(err) = result {
        panic!("{err}");
    }
}
