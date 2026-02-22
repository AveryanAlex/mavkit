#[allow(dead_code)]
mod common;

use mavkit::{HomePosition, MissionPlan, MissionType, Vehicle};
use std::time::Duration;

// ---------------------------------------------------------------------------
// Mission roundtrip tests
// ---------------------------------------------------------------------------

fn sample_plan_mission() -> MissionPlan {
    common::sample_plan_mission(
        Some(HomePosition {
            latitude_deg: 47.397742,
            longitude_deg: 8.545594,
            altitude_m: 0.0,
        }),
        3,
    )
}

// ---------------------------------------------------------------------------
// Mission roundtrip tests
// ---------------------------------------------------------------------------

#[tokio::test]
#[ignore = "requires ArduPilot SITL endpoint"]
async fn sitl_roundtrip_mission_type_mission() {
    common::run_roundtrip_case(sample_plan_mission()).await;
}

#[tokio::test]
#[ignore = "requires ArduPilot SITL endpoint"]
async fn sitl_roundtrip_mission_type_fence() {
    common::run_roundtrip_case(MissionPlan {
        mission_type: MissionType::Fence,
        home: None,
        items: Vec::new(),
    })
    .await;
}

#[tokio::test]
#[ignore = "requires ArduPilot SITL endpoint"]
async fn sitl_roundtrip_mission_type_rally() {
    common::run_roundtrip_case(MissionPlan {
        mission_type: MissionType::Rally,
        home: None,
        items: Vec::new(),
    })
    .await;
}

// ---------------------------------------------------------------------------
// Vehicle command tests
// ---------------------------------------------------------------------------

async fn setup_sitl_vehicle() -> Vehicle {
    common::setup_sitl_vehicle().await
}

#[tokio::test]
#[ignore = "requires ArduPilot SITL endpoint"]
async fn sitl_force_arm_disarm_cycle() {
    let vehicle = setup_sitl_vehicle().await;

    let result: Result<(), String> = async {
        // Arm in STABILIZE.
        vehicle.set_mode(0).await.map_err(|e| e.to_string())?;
        common::wait_for_state(&vehicle, |s| s.custom_mode == 0, Duration::from_secs(10)).await;

        common::arm_with_retries(&vehicle, false, Duration::from_secs(30)).await?;
        common::wait_for_state(&vehicle, |s| s.armed, Duration::from_secs(10)).await;

        vehicle.disarm(false).await.map_err(|e| e.to_string())?;
        common::wait_for_state(&vehicle, |s| !s.armed, Duration::from_secs(10)).await;

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
async fn sitl_set_flight_mode() {
    let vehicle = setup_sitl_vehicle().await;

    let result: Result<(), String> = async {
        // Set GUIDED (custom_mode=4)
        vehicle.set_mode(4).await.map_err(|e| e.to_string())?;
        common::wait_for_state(&vehicle, |s| s.custom_mode == 4, Duration::from_secs(10)).await;
        {
            let state = vehicle.state().borrow().clone();
            if state.mode_name != "GUIDED" {
                return Err(format!("expected GUIDED, got {}", state.mode_name));
            }
        }

        // Set LOITER (custom_mode=5)
        vehicle.set_mode(5).await.map_err(|e| e.to_string())?;
        common::wait_for_state(&vehicle, |s| s.custom_mode == 5, Duration::from_secs(10)).await;
        {
            let state = vehicle.state().borrow().clone();
            if state.mode_name != "LOITER" {
                return Err(format!("expected LOITER, got {}", state.mode_name));
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
async fn sitl_takeoff_and_land() {
    let vehicle = setup_sitl_vehicle().await;

    let result: Result<(), String> = async {
        vehicle.set_mode(4).await.map_err(|e| e.to_string())?; // GUIDED
        common::arm_with_retries(&vehicle, false, Duration::from_secs(30)).await?;
        vehicle.takeoff(1.0).await.map_err(|e| e.to_string())?;

        common::wait_for_state(&vehicle, |s| s.armed, Duration::from_secs(15)).await;

        tokio::time::sleep(Duration::from_secs(5)).await;

        vehicle.set_mode(9).await.map_err(|e| e.to_string())?; // LAND

        // Wait for auto-disarm
        common::wait_for_state(&vehicle, |s| !s.armed, Duration::from_secs(60)).await;

        Ok(())
    }
    .await;

    let _ = vehicle.disarm(true).await;
    let _ = vehicle.disconnect().await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

#[tokio::test]
#[ignore = "requires ArduPilot SITL endpoint"]
async fn sitl_guided_goto() {
    let vehicle = setup_sitl_vehicle().await;

    let result: Result<(), String> = async {
        vehicle.set_mode(4).await.map_err(|e| e.to_string())?; // GUIDED
        common::arm_with_retries(&vehicle, false, Duration::from_secs(30)).await?;
        vehicle.takeoff(1.0).await.map_err(|e| e.to_string())?;
        common::wait_for_state(&vehicle, |s| s.armed, Duration::from_secs(15)).await;

        tokio::time::sleep(Duration::from_secs(5)).await;

        vehicle
            .goto(42.390_000, -71.147_000, 20.0)
            .await
            .map_err(|e| e.to_string())?;

        tokio::time::sleep(Duration::from_secs(3)).await;

        vehicle.disarm(true).await.map_err(|e| e.to_string())?;
        Ok(())
    }
    .await;

    let _ = vehicle.disarm(true).await;
    let _ = vehicle.disconnect().await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

#[tokio::test]
#[ignore = "requires ArduPilot SITL endpoint"]
async fn sitl_get_available_modes() {
    let vehicle = setup_sitl_vehicle().await;

    let result: Result<(), String> = async {
        let modes = vehicle.available_modes();

        if modes.len() < 10 {
            return Err(format!(
                "expected at least 10 copter modes, got {}",
                modes.len()
            ));
        }

        let has_mode = |name: &str, id: u32| -> bool {
            modes.iter().any(|m| m.custom_mode == id && m.name == name)
        };

        if !has_mode("STABILIZE", 0) {
            return Err(String::from("missing STABILIZE mode"));
        }
        if !has_mode("GUIDED", 4) {
            return Err(String::from("missing GUIDED mode"));
        }
        if !has_mode("LOITER", 5) {
            return Err(String::from("missing LOITER mode"));
        }
        if !has_mode("RTL", 6) {
            return Err(String::from("missing RTL mode"));
        }

        Ok(())
    }
    .await;

    let _ = vehicle.disconnect().await;
    if let Err(err) = result {
        panic!("{err}");
    }
}
