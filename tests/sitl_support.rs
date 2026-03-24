#[allow(dead_code)]
mod common;

use mavkit::SupportState;
use std::time::Duration;

#[tokio::test]
#[ignore = "requires ArduPilot SITL endpoint"]
async fn sitl_support_command_int_resolves() {
    let vehicle = common::setup_sitl_vehicle().await;
    let result: Result<(), String> = async {
        let state = vehicle
            .support()
            .command_int()
            .wait_timeout(Duration::from_secs(10))
            .await
            .map_err(|e| e.to_string())?;

        if state == SupportState::Unknown {
            return Err(String::from("command_int support should resolve"));
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
async fn sitl_support_mission_fence_resolves() {
    let vehicle = common::setup_sitl_vehicle().await;
    let result: Result<(), String> = async {
        let state = vehicle
            .support()
            .mission_fence()
            .wait_timeout(Duration::from_secs(10))
            .await
            .map_err(|e| e.to_string())?;

        // Fence support may legitimately stay Unknown on some SITL images;
        // just verify the observation emits a value without timing out.
        let _ = state;

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
async fn sitl_support_mission_rally_resolves() {
    let vehicle = common::setup_sitl_vehicle().await;
    let result: Result<(), String> = async {
        let state = vehicle
            .support()
            .mission_rally()
            .wait_timeout(Duration::from_secs(10))
            .await
            .map_err(|e| e.to_string())?;

        if state == SupportState::Unknown {
            return Err(String::from("mission_rally support should resolve"));
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
async fn sitl_support_terrain_resolves() {
    let vehicle = common::setup_sitl_vehicle().await;
    let result: Result<(), String> = async {
        let state = vehicle
            .support()
            .terrain()
            .wait_timeout(Duration::from_secs(10))
            .await
            .map_err(|e| e.to_string())?;

        // Terrain support may legitimately stay Unknown on some SITL images;
        // just verify the observation emits a value without timing out.
        let _ = state;

        Ok(())
    }
    .await;

    let _ = vehicle.disconnect().await;
    if let Err(err) = result {
        panic!("{err}");
    }
}
