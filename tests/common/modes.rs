use crate::common::wait::{wait_for_mode, wait_for_mode_name};
use crate::{TestTarget, disconnect, setup_backend_vehicle};
use std::time::Duration;

pub async fn mode_catalog_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;

    let result: Result<(), String> = async {
        let catalog = vehicle
            .available_modes()
            .catalog()
            .wait_timeout(Duration::from_secs(10))
            .await
            .map_err(|err| err.to_string())?;

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

        let mut seen = std::collections::HashSet::new();
        for entry in &catalog {
            if !seen.insert(entry.custom_mode) {
                return Err(format!(
                    "duplicate custom_mode {} in catalog",
                    entry.custom_mode
                ));
            }
        }

        if catalog.len() < 10 {
            return Err(format!("expected at least 10 modes, got {}", catalog.len()));
        }

        for &(id, name) in target.expected_modes() {
            if !catalog
                .iter()
                .any(|mode| mode.custom_mode == id && mode.name == name)
            {
                return Err(format!("missing {name} mode"));
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

pub async fn set_flight_mode_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;

    let result: Result<(), String> = async {
        let (guided_id, guided_name) = target.guided_mode();
        let (hold_id, hold_name) = target.hold_mode();

        vehicle
            .set_mode(guided_id)
            .await
            .map_err(|err| err.to_string())?;
        wait_for_mode(vehicle, guided_id, Duration::from_secs(10)).await;
        let mode = vehicle
            .available_modes()
            .current()
            .latest()
            .ok_or_else(|| format!("mode state unavailable after {guided_name} transition"))?;
        if mode.name != guided_name {
            return Err(format!("expected {guided_name}, got {}", mode.name));
        }

        vehicle
            .set_mode(hold_id)
            .await
            .map_err(|err| err.to_string())?;
        wait_for_mode(vehicle, hold_id, Duration::from_secs(10)).await;
        let mode = vehicle
            .available_modes()
            .current()
            .latest()
            .ok_or_else(|| format!("mode state unavailable after {hold_name} transition"))?;
        if mode.name != hold_name {
            return Err(format!("expected {hold_name}, got {}", mode.name));
        }

        Ok(())
    }
    .await;

    disconnect(backend).await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

pub async fn set_mode_by_name_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;

    let result: Result<(), String> = async {
        let (start_name, guided_name, hold_name) = target.mode_names();

        vehicle
            .set_mode_by_name(start_name)
            .await
            .map_err(|err| err.to_string())?;
        wait_for_mode_name(vehicle, start_name, Duration::from_secs(10)).await?;

        vehicle
            .set_mode_by_name(guided_name)
            .await
            .map_err(|err| err.to_string())?;
        wait_for_mode_name(vehicle, guided_name, Duration::from_secs(10)).await?;

        vehicle
            .set_mode_by_name(hold_name)
            .await
            .map_err(|err| err.to_string())?;
        wait_for_mode_name(vehicle, hold_name, Duration::from_secs(10)).await?;

        Ok(())
    }
    .await;

    disconnect(backend).await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

pub async fn set_invalid_name_returns_error_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;

    let result: Result<(), String> = async {
        let res = vehicle.set_mode_by_name("NONEXISTENT_MODE_XYZ").await;
        if res.is_ok() {
            return Err(String::from(
                "expected set_mode_by_name with invalid name to return Err, but got Ok",
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
