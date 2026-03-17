#[allow(dead_code)]
mod common;

use mavkit::{SupportState, Vehicle};
use std::time::Duration;

async fn wait_for_home_position(vehicle: &Vehicle, timeout: Duration) {
    vehicle
        .telemetry()
        .home()
        .wait_timeout(timeout)
        .await
        .expect("timed out waiting for home position");
}

async fn wait_for_mode_name(
    vehicle: &Vehicle,
    expected_name: &str,
    timeout: Duration,
) -> Result<(), String> {
    let current_mode = vehicle.available_modes().current();
    if current_mode
        .latest()
        .is_some_and(|mode| mode.name == expected_name)
    {
        return Ok(());
    }

    let mut subscription = current_mode.subscribe();
    let deadline = tokio::time::sleep(timeout);
    tokio::pin!(deadline);
    loop {
        tokio::select! {
            _ = &mut deadline => {
                return Err(format!("timed out waiting for mode {expected_name}"));
            }
            observed = subscription.recv() => {
                let mode = observed.ok_or_else(|| String::from("mode observation stream closed"))?;
                if mode.name == expected_name {
                    return Ok(());
                }
            }
        }
    }
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
        wait_for_mode_name(&vehicle, "GUIDED", Duration::from_secs(10)).await?;

        vehicle
            .set_mode_by_name("LOITER", true)
            .await
            .map_err(|e| e.to_string())?;
        wait_for_mode_name(&vehicle, "LOITER", Duration::from_secs(10)).await?;

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
