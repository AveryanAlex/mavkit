#[allow(dead_code)]
mod common;

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

#[tokio::test]
#[ignore = "requires ArduPilot SITL endpoint"]
async fn sitl_set_mode_by_name() {
    let vehicle = common::setup_sitl_vehicle().await;
    let result: Result<(), String> = async {
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
async fn sitl_home_position_watch_populates() {
    let vehicle = common::setup_sitl_vehicle().await;
    let result: Result<(), String> = async {
        wait_for_home_position(&vehicle, Duration::from_secs(20)).await;
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
async fn sitl_support_discovery_reports_ardupilot() {
    let vehicle = common::setup_sitl_vehicle().await;
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

    let _ = vehicle.disconnect().await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

#[tokio::test]
#[ignore = "requires ArduPilot SITL endpoint"]
async fn sitl_set_home_current_updates_home() {
    let vehicle = common::setup_sitl_vehicle().await;
    let result: Result<(), String> = async {
        wait_for_home_position(&vehicle, Duration::from_secs(20)).await;

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

    let _ = vehicle.disconnect().await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

#[tokio::test]
#[ignore = "requires ArduPilot SITL endpoint"]
async fn sitl_disconnect_transitions_link_state() {
    let vehicle = common::setup_sitl_vehicle().await;
    let result: Result<(), String> = async {
        let state = vehicle
            .link()
            .state()
            .latest()
            .ok_or("link state not available")?;
        if state != LinkState::Connected {
            return Err(format!("expected Connected before disconnect, got {state:?}"));
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

    // Vehicle already disconnected inside the test body.
    if let Err(err) = result {
        panic!("{err}");
    }
}
