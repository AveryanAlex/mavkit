#[allow(dead_code)]
mod common;

use std::collections::HashSet;
use std::time::Duration;

#[tokio::test]
#[ignore = "requires ArduPilot SITL endpoint"]
async fn sitl_modes_catalog_entries_have_names_and_ids() {
    let vehicle = common::setup_sitl_vehicle().await;

    let result: Result<(), String> = async {
        let catalog = vehicle
            .available_modes()
            .catalog()
            .wait_timeout(Duration::from_secs(10))
            .await
            .map_err(|e| e.to_string())?;

        if catalog.is_empty() {
            return Err(String::from("catalog is empty"));
        }

        for entry in &catalog {
            if entry.name.is_empty() {
                return Err(format!(
                    "catalog entry with custom_mode={} has an empty name",
                    entry.custom_mode
                ));
            }
        }

        let mut seen = HashSet::new();
        for entry in &catalog {
            if !seen.insert(entry.custom_mode) {
                return Err(format!(
                    "duplicate custom_mode {} in catalog",
                    entry.custom_mode
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

#[tokio::test]
#[ignore = "requires ArduPilot SITL endpoint"]
async fn sitl_modes_current_mode_stream_updates_on_switch() {
    let vehicle = common::setup_sitl_vehicle().await;

    let result: Result<(), String> = async {
        vehicle
            .set_mode_by_name("STABILIZE", true)
            .await
            .map_err(|e| e.to_string())?;
        common::wait_for_mode_name(&vehicle, "STABILIZE", Duration::from_secs(10)).await?;

        vehicle
            .set_mode_by_name("GUIDED", true)
            .await
            .map_err(|e| e.to_string())?;
        common::wait_for_mode_name(&vehicle, "GUIDED", Duration::from_secs(10)).await?;

        vehicle
            .set_mode_by_name("LOITER", true)
            .await
            .map_err(|e| e.to_string())?;
        common::wait_for_mode_name(&vehicle, "LOITER", Duration::from_secs(10)).await?;

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
async fn sitl_modes_set_invalid_name_returns_error() {
    let vehicle = common::setup_sitl_vehicle().await;

    let result: Result<(), String> = async {
        let res = vehicle.set_mode_by_name("NONEXISTENT_MODE_XYZ", true).await;
        if res.is_ok() {
            return Err(String::from(
                "expected set_mode_by_name with invalid name to return Err, but got Ok",
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
