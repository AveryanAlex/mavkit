#[allow(dead_code)]
mod common;

use mavkit::Vehicle;
use std::time::Duration;

async fn wait_for_home_position(vehicle: &Vehicle, timeout: Duration) {
    let mut rx = vehicle.home_position();
    let deadline = tokio::time::sleep(timeout);
    tokio::pin!(deadline);
    loop {
        tokio::select! {
            _ = &mut deadline => panic!("timed out waiting for home position"),
            result = rx.changed() => {
                result.expect("watch channel closed");
                if rx.borrow().is_some() {
                    return;
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
            .set_mode_by_name("GUIDED")
            .await
            .map_err(|e| e.to_string())?;
        common::wait_for_state(
            &vehicle,
            |s| s.mode_name == "GUIDED",
            Duration::from_secs(10),
        )
        .await;

        vehicle
            .set_mode_by_name("LOITER")
            .await
            .map_err(|e| e.to_string())?;
        common::wait_for_state(
            &vehicle,
            |s| s.mode_name == "LOITER",
            Duration::from_secs(10),
        )
        .await;

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
