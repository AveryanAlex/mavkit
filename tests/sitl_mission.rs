#[allow(dead_code)]
mod common;

use std::time::Duration;

#[tokio::test]
#[ignore = "requires ArduPilot SITL endpoint"]
async fn sitl_roundtrip_mission_no_home() {
    common::run_roundtrip_case(common::sample_plan_mission(3)).await;
}

#[tokio::test]
#[ignore = "requires ArduPilot SITL endpoint"]
async fn sitl_roundtrip_mission_empty_items_with_home() {
    let plan = common::sample_plan_mission(0);
    common::run_roundtrip_case(plan).await;
}

#[tokio::test]
#[ignore = "requires ArduPilot SITL endpoint"]
async fn sitl_roundtrip_mission_empty_items_no_home() {
    common::run_roundtrip_case(common::sample_plan_mission(0)).await;
}

#[tokio::test]
#[ignore = "requires ArduPilot SITL endpoint"]
async fn sitl_clear_then_download_mission_is_empty() {
    let vehicle = common::setup_sitl_vehicle().await;
    let result: Result<(), String> = async {
        vehicle
            .mission()
            .clear()
            .map_err(|e| e.to_string())?
            .wait()
            .await
            .map_err(|e| e.to_string())?;

        tokio::time::sleep(Duration::from_millis(500)).await;

        let downloaded = common::download_with_retries(&vehicle)
            .await
            .map_err(|e| e.to_string())?;
        assert!(
            downloaded.items.is_empty(),
            "expected empty mission after clear"
        );
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
async fn sitl_set_current_updates_mission_state() {
    let vehicle = common::setup_sitl_vehicle().await;

    let result: Result<(), String> = async {
        let plan = common::sample_plan_mission(3);
        vehicle
            .mission()
            .clear()
            .map_err(|e| e.to_string())?
            .wait()
            .await
            .map_err(|e| e.to_string())?;
        vehicle
            .mission()
            .upload(plan)
            .map_err(|e| e.to_string())?
            .wait()
            .await
            .map_err(|e| e.to_string())?;

        tokio::time::sleep(Duration::from_millis(500)).await;

        vehicle
            .mission()
            .set_current(1)
            .await
            .map_err(|e| e.to_string())?;

        common::wait_for_mission_state(
            &vehicle,
            |s| s.current_index == Some(1),
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
