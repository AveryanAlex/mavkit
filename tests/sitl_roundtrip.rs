#[allow(dead_code)]
mod common;

use mavkit::{FencePlan, MissionPlan, MissionType, RallyPlan, Vehicle};
use std::time::Duration;

fn sample_plan_mission() -> MissionPlan {
    common::sample_plan_mission(3)
}

async fn setup_sitl_vehicle() -> Vehicle {
    common::setup_sitl_vehicle().await
}

#[tokio::test]
#[ignore = "requires ArduPilot SITL endpoint"]
async fn sitl_roundtrip_mission_type_mission() {
    common::run_roundtrip_case(sample_plan_mission()).await;
}

#[tokio::test]
#[ignore = "requires ArduPilot SITL endpoint"]
async fn sitl_roundtrip_mission_type_fence() {
    let vehicle = setup_sitl_vehicle().await;
    let result: Result<(), String> = async {
        let plan = FencePlan {
            return_point: None,
            regions: Vec::new(),
        };

        let clear = vehicle.fence().clear().map_err(|e| e.to_string())?;
        if let Err(err) = clear.wait().await {
            if common::is_optional_type_unsupported(MissionType::Fence, &err) {
                eprintln!("Skipping Fence roundtrip: {err}");
                return Ok(());
            }
            return Err(format!("failed to clear fence before upload: {err}"));
        }

        let upload = vehicle
            .fence()
            .upload(plan.clone())
            .map_err(|e| e.to_string())?;
        if let Err(err) = upload.wait().await {
            if common::is_optional_type_unsupported(MissionType::Fence, &err) {
                eprintln!("Skipping Fence roundtrip: {err}");
                return Ok(());
            }
            return Err(format!("failed to upload fence plan: {err}"));
        }

        let download = vehicle.fence().download().map_err(|e| e.to_string())?;
        let downloaded = match download.wait().await {
            Ok(plan) => plan,
            Err(err) => {
                if common::is_optional_type_unsupported(MissionType::Fence, &err) {
                    eprintln!("Skipping Fence roundtrip: {err}");
                    return Ok(());
                }
                return Err(format!("failed to download fence plan: {err}"));
            }
        };

        if downloaded != plan {
            return Err(format!(
                "fence roundtrip mismatch: expected {:?}, got {:?}",
                plan, downloaded
            ));
        }

        vehicle
            .fence()
            .clear()
            .map_err(|e| e.to_string())?
            .wait()
            .await
            .map_err(|e| format!("failed to clear fence after roundtrip: {e}"))?;

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
async fn sitl_roundtrip_mission_type_rally() {
    let vehicle = setup_sitl_vehicle().await;
    let result: Result<(), String> = async {
        let plan = RallyPlan { points: Vec::new() };

        let clear = vehicle.rally().clear().map_err(|e| e.to_string())?;
        if let Err(err) = clear.wait().await {
            if common::is_optional_type_unsupported(MissionType::Rally, &err) {
                eprintln!("Skipping Rally roundtrip: {err}");
                return Ok(());
            }
            return Err(format!("failed to clear rally before upload: {err}"));
        }

        let upload = vehicle
            .rally()
            .upload(plan.clone())
            .map_err(|e| e.to_string())?;
        if let Err(err) = upload.wait().await {
            if common::is_optional_type_unsupported(MissionType::Rally, &err) {
                eprintln!("Skipping Rally roundtrip: {err}");
                return Ok(());
            }
            return Err(format!("failed to upload rally plan: {err}"));
        }

        let download = vehicle.rally().download().map_err(|e| e.to_string())?;
        let downloaded = match download.wait().await {
            Ok(plan) => plan,
            Err(err) => {
                if common::is_optional_type_unsupported(MissionType::Rally, &err) {
                    eprintln!("Skipping Rally roundtrip: {err}");
                    return Ok(());
                }
                return Err(format!("failed to download rally plan: {err}"));
            }
        };

        if downloaded != plan {
            return Err(format!(
                "rally roundtrip mismatch: expected {:?}, got {:?}",
                plan, downloaded
            ));
        }

        vehicle
            .rally()
            .clear()
            .map_err(|e| e.to_string())?
            .wait()
            .await
            .map_err(|e| format!("failed to clear rally after roundtrip: {e}"))?;

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
async fn sitl_force_arm_disarm_cycle() {
    let vehicle = setup_sitl_vehicle().await;

    let result: Result<(), String> = async {
        vehicle.set_mode(0, true).await.map_err(|e| e.to_string())?;
        common::wait_for_mode(&vehicle, 0, Duration::from_secs(10)).await;

        common::arm_with_retries(&vehicle, false, Duration::from_secs(30)).await?;
        common::wait_for_armed(&vehicle, true, Duration::from_secs(10)).await;

        vehicle.disarm(false).await.map_err(|e| e.to_string())?;
        common::wait_for_armed(&vehicle, false, Duration::from_secs(10)).await;

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
        vehicle.set_mode(4, true).await.map_err(|e| e.to_string())?;
        common::wait_for_mode(&vehicle, 4, Duration::from_secs(10)).await;
        {
            let mode = vehicle
                .available_modes()
                .current()
                .latest()
                .ok_or_else(|| String::from("mode state unavailable after GUIDED transition"))?;
            if mode.name != "GUIDED" {
                return Err(format!("expected GUIDED, got {}", mode.name));
            }
        }

        vehicle.set_mode(5, true).await.map_err(|e| e.to_string())?;
        common::wait_for_mode(&vehicle, 5, Duration::from_secs(10)).await;
        {
            let mode = vehicle
                .available_modes()
                .current()
                .latest()
                .ok_or_else(|| String::from("mode state unavailable after LOITER transition"))?;
            if mode.name != "LOITER" {
                return Err(format!("expected LOITER, got {}", mode.name));
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
async fn sitl_telemetry_position_observation_available() {
    let vehicle = setup_sitl_vehicle().await;
    let result: Result<(), String> = async {
        let sample = vehicle
            .telemetry()
            .position()
            .global()
            .wait_timeout(Duration::from_secs(10))
            .await
            .map_err(|e| e.to_string())?;

        if !sample.value.latitude_deg.is_finite() || !sample.value.longitude_deg.is_finite() {
            return Err(String::from(
                "telemetry position contained non-finite coordinates",
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
async fn sitl_get_available_modes() {
    let vehicle = setup_sitl_vehicle().await;

    let result: Result<(), String> = async {
        let modes = vehicle.available_modes();
        let catalog = modes
            .catalog()
            .wait_timeout(Duration::from_secs(10))
            .await
            .map_err(|e| e.to_string())?;

        if catalog.len() < 10 {
            return Err(format!(
                "expected at least 10 copter modes, got {}",
                catalog.len()
            ));
        }

        let has_mode = |name: &str, id: u32| -> bool {
            catalog
                .iter()
                .any(|mode| mode.custom_mode == id && mode.name == name)
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
