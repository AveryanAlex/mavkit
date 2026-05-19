use crate::common::wait::{wait_for_armed, wait_for_mode};
use crate::{TestTarget, disconnect, setup_backend_vehicle};
use mavkit::{LinkState, SupportState, Vehicle};
use std::time::Duration;

async fn wait_for_home_position(vehicle: &Vehicle, timeout: Duration) {
    vehicle
        .telemetry()
        .home()
        .wait_timeout(timeout)
        .await
        .expect("timed out waiting for home position");
}

pub async fn arm_with_retries(
    vehicle: &Vehicle,
    force: bool,
    timeout: Duration,
) -> Result<(), String> {
    let deadline = tokio::time::Instant::now() + timeout;
    let mut last_err = String::from("arm timed out");
    loop {
        if tokio::time::Instant::now() > deadline {
            return Err(last_err);
        }
        match if force {
            vehicle.force_arm().await
        } else {
            vehicle.arm().await
        } {
            Ok(()) => return Ok(()),
            Err(err) => {
                last_err = err.to_string();
                tokio::time::sleep(Duration::from_secs(1)).await;
            }
        }
    }
}

pub async fn force_arm_disarm_cycle_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;

    let result: Result<(), String> = async {
        vehicle.set_mode(0).await.map_err(|err| err.to_string())?;
        wait_for_mode(vehicle, 0, Duration::from_secs(10)).await;

        arm_with_retries(vehicle, false, Duration::from_secs(30)).await?;
        wait_for_armed(vehicle, true, Duration::from_secs(10)).await;

        vehicle.disarm().await.map_err(|err| err.to_string())?;
        wait_for_armed(vehicle, false, Duration::from_secs(10)).await;

        Ok(())
    }
    .await;

    disconnect(backend).await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

pub async fn home_position_watch_populates_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;
    let result: Result<(), String> = async {
        wait_for_home_position(vehicle, Duration::from_secs(20)).await;
        Ok(())
    }
    .await;

    disconnect(backend).await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

pub async fn support_discovery_reports_ardupilot_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;
    let result: Result<(), String> = async {
        let ardupilot_support = vehicle
            .support()
            .ardupilot()
            .wait_timeout(Duration::from_secs(10))
            .await
            .map_err(|e| e.to_string())?;

        if ardupilot_support != SupportState::Supported {
            return Err(format!(
                "expected ardupilot support discovery to be Supported, got {ardupilot_support:?}"
            ));
        }

        let modes_support = vehicle
            .available_modes()
            .support()
            .wait_timeout(Duration::from_secs(10))
            .await
            .map_err(|e| e.to_string())?;

        if modes_support == SupportState::Unknown {
            return Err(String::from(
                "expected mode support to resolve to a concrete state",
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

pub async fn set_home_current_updates_home_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;
    let result: Result<(), String> = async {
        wait_for_home_position(vehicle, Duration::from_secs(20)).await;

        let original = vehicle
            .telemetry()
            .home()
            .latest()
            .ok_or("no home position after wait")?;

        vehicle
            .set_home_current()
            .await
            .map_err(|e| e.to_string())?;

        tokio::time::sleep(Duration::from_millis(500)).await;

        let updated = vehicle
            .telemetry()
            .home()
            .latest()
            .ok_or("no home position after set_home_current")?;

        if !updated.value.latitude_deg.is_finite() || !updated.value.longitude_deg.is_finite() {
            return Err("updated home coordinates are not finite".into());
        }

        let lat_diff = (updated.value.latitude_deg - original.value.latitude_deg).abs();
        let lon_diff = (updated.value.longitude_deg - original.value.longitude_deg).abs();
        if lat_diff >= 0.001 || lon_diff >= 0.001 {
            return Err(format!(
                "home drifted too far: lat_diff={lat_diff}, lon_diff={lon_diff}"
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

pub async fn disconnect_transitions_link_state_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;
    let result: Result<(), String> = async {
        let state = vehicle
            .link()
            .state()
            .latest()
            .ok_or("link state not available")?;
        if state != LinkState::Connected {
            return Err(format!(
                "expected Connected before disconnect, got {state:?}"
            ));
        }

        vehicle.disconnect().await.map_err(|e| e.to_string())?;

        let state_after = vehicle
            .link()
            .state()
            .latest()
            .ok_or("link state not available after disconnect")?;
        if state_after == LinkState::Connected {
            return Err("link state still Connected after disconnect".into());
        }

        Ok(())
    }
    .await;

    if let Err(err) = result {
        panic!("{err}");
    }
}
