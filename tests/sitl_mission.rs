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

        let downloaded = common::download_with_retries(&vehicle).await?;
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

#[tokio::test]
#[ignore = "requires ArduPilot SITL endpoint"]
async fn sitl_roundtrip_mission_20_items() {
    common::run_roundtrip_case(common::sample_plan_mission(20)).await;
}

#[tokio::test]
#[ignore = "requires ArduPilot SITL endpoint"]
async fn sitl_upload_overwrites_previous_mission() {
    let vehicle = common::setup_sitl_vehicle().await;

    let result: Result<(), String> = async {
        vehicle
            .mission()
            .clear()
            .map_err(|e| e.to_string())?
            .wait()
            .await
            .map_err(|e| e.to_string())?;

        let plan_a = common::sample_plan_mission(3);
        vehicle
            .mission()
            .upload(plan_a)
            .map_err(|e| e.to_string())?
            .wait()
            .await
            .map_err(|e| e.to_string())?;

        let plan_b = common::sample_plan_mission(5);
        vehicle
            .mission()
            .upload(plan_b)
            .map_err(|e| e.to_string())?
            .wait()
            .await
            .map_err(|e| e.to_string())?;

        tokio::time::sleep(Duration::from_millis(500)).await;

        let downloaded = common::download_with_retries(&vehicle).await?;
        assert_eq!(
            downloaded.items.len(),
            5,
            "expected plan_b (5 items) to overwrite plan_a, got {} items",
            downloaded.items.len()
        );

        vehicle
            .mission()
            .clear()
            .map_err(|e| e.to_string())?
            .wait()
            .await
            .map_err(|e| e.to_string())?;

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
async fn sitl_mission_state_reflects_uploaded_count() {
    let vehicle = common::setup_sitl_vehicle().await;

    let result: Result<(), String> = async {
        vehicle
            .mission()
            .clear()
            .map_err(|e| e.to_string())?
            .wait()
            .await
            .map_err(|e| e.to_string())?;

        let plan = common::sample_plan_mission(4);
        vehicle
            .mission()
            .upload(plan)
            .map_err(|e| e.to_string())?
            .wait()
            .await
            .map_err(|e| e.to_string())?;

        common::wait_for_mission_state(
            &vehicle,
            |s| s.plan.as_ref().is_some_and(|p| p.items.len() == 4),
            Duration::from_secs(10),
        )
        .await;

        vehicle
            .mission()
            .clear()
            .map_err(|e| e.to_string())?
            .wait()
            .await
            .map_err(|e| e.to_string())?;

        Ok(())
    }
    .await;

    let _ = vehicle.disconnect().await;
    if let Err(err) = result {
        panic!("{err}");
    }
}
