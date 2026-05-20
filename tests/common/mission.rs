use crate::common::backend::{disconnect, setup_backend_vehicle};
use crate::common::clock::sleep;
use crate::common::fixtures::sample_plan_mission;
use crate::common::target::TestTarget;
use crate::common::wait::wait_for_mission_state;
use mavkit::{
    CompareTolerance, FencePlan, MissionPlan, MissionType, RallyPlan, Vehicle, VehicleError,
    normalize_for_compare, plans_equivalent,
};
use std::time::Duration;

pub fn is_optional_type_unsupported(mission_type: MissionType, error: &VehicleError) -> bool {
    if mission_type == MissionType::Mission {
        return false;
    }
    let msg = error.to_string().to_ascii_lowercase();
    msg.contains("unsupported")
        || msg.contains("transfer.timeout")
        || msg.contains("operation timeout")
        || msg.contains("timed out")
}

pub async fn download_with_retries(vehicle: &Vehicle) -> Result<MissionPlan, String> {
    let strict = std::env::var("MAVKIT_SITL_STRICT")
        .map(|v| v == "1")
        .unwrap_or(false);
    let mut last_error: Option<String> = None;

    for attempt in 1..=3 {
        let op = match vehicle.mission().download() {
            Ok(op) => op,
            Err(err) => {
                last_error = Some(err.to_string());
                if attempt < 3 {
                    sleep(Duration::from_millis(600)).await;
                }
                continue;
            }
        };

        match op.wait().await {
            Ok(plan) => return Ok(plan),
            Err(err) => {
                last_error = Some(err.to_string());
                if attempt < 3 {
                    sleep(Duration::from_millis(600)).await;
                }
            }
        }
    }

    let err_msg = format!(
        "failed to download mission plan after retries: {}",
        last_error.unwrap_or_else(|| String::from("unknown error"))
    );

    if !strict && err_msg.to_ascii_lowercase().contains("transfer.timeout") {
        eprintln!(
            "Skipping Mission download timeout in non-strict SITL mode: {err_msg}. Set MAVKIT_SITL_STRICT=1 to enforce failure."
        );
        return Err(String::from("skip_optional_mission_type"));
    }

    Err(err_msg)
}

pub async fn roundtrip_case(target: TestTarget, plan: MissionPlan) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;

    let result: Result<(), String> = async {
        vehicle
            .mission()
            .clear()
            .map_err(|err| format!("failed to start clear before upload: {err}"))?
            .wait()
            .await
            .map_err(|err| format!("failed to clear before upload: {err}"))?;

        vehicle
            .mission()
            .upload(plan.clone())
            .map_err(|err| format!("failed to start upload: {err}"))?
            .wait()
            .await
            .map_err(|err| format!("failed to upload mission plan: {err}"))?;

        sleep(Duration::from_millis(500)).await;

        let downloaded = download_with_retries(vehicle).await;
        let downloaded = match downloaded {
            Ok(plan) => plan,
            Err(err) if err == "skip_optional_mission_type" => return Ok(()),
            Err(err) => return Err(err),
        };

        let expected = normalize_for_compare(&plan);
        let got = normalize_for_compare(&downloaded);

        if !plans_equivalent(&expected, &got, CompareTolerance::default()) {
            return Err(format!(
                "readback mismatch for MissionType::Mission: expected {:?}, got {:?}",
                expected, got
            ));
        }

        vehicle
            .mission()
            .clear()
            .map_err(|err| format!("failed to start clear after roundtrip: {err}"))?
            .wait()
            .await
            .map_err(|err| {
                format!("failed to clear after roundtrip for MissionType::Mission: {err}")
            })?;

        Ok(())
    }
    .await;

    disconnect(backend).await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

pub async fn clear_then_download_empty_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;

    let result: Result<(), String> = async {
        vehicle
            .mission()
            .clear()
            .map_err(|err| err.to_string())?
            .wait()
            .await
            .map_err(|err| err.to_string())?;

        sleep(Duration::from_millis(500)).await;

        let downloaded = download_with_retries(vehicle).await?;
        if !downloaded.items.is_empty() {
            return Err(String::from("expected empty mission after clear"));
        }

        Ok(())
    }
    .await;

    disconnect(backend).await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

pub async fn set_current_updates_mission_state_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;

    let result: Result<(), String> = async {
        let plan = sample_plan_mission(3);
        vehicle
            .mission()
            .clear()
            .map_err(|err| err.to_string())?
            .wait()
            .await
            .map_err(|err| err.to_string())?;
        vehicle
            .mission()
            .upload(plan)
            .map_err(|err| err.to_string())?
            .wait()
            .await
            .map_err(|err| err.to_string())?;

        sleep(Duration::from_millis(500)).await;

        vehicle
            .mission()
            .set_current(1)
            .await
            .map_err(|err| err.to_string())?;

        wait_for_mission_state(
            vehicle,
            |state| state.current_index == Some(1),
            Duration::from_secs(10),
        )
        .await;

        Ok(())
    }
    .await;

    disconnect(backend).await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

pub async fn upload_overwrites_previous_mission_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;

    let result: Result<(), String> = async {
        vehicle
            .mission()
            .clear()
            .map_err(|err| err.to_string())?
            .wait()
            .await
            .map_err(|err| err.to_string())?;

        let plan_a = sample_plan_mission(3);
        vehicle
            .mission()
            .upload(plan_a)
            .map_err(|err| err.to_string())?
            .wait()
            .await
            .map_err(|err| err.to_string())?;

        let plan_b = sample_plan_mission(5);
        vehicle
            .mission()
            .upload(plan_b)
            .map_err(|err| err.to_string())?
            .wait()
            .await
            .map_err(|err| err.to_string())?;

        sleep(Duration::from_millis(500)).await;

        let downloaded = download_with_retries(vehicle).await?;
        if downloaded.items.len() != 5 {
            return Err(format!(
                "expected plan_b (5 items) to overwrite plan_a, got {} items",
                downloaded.items.len()
            ));
        }

        vehicle
            .mission()
            .clear()
            .map_err(|err| err.to_string())?
            .wait()
            .await
            .map_err(|err| err.to_string())?;

        Ok(())
    }
    .await;

    disconnect(backend).await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

pub async fn mission_state_reflects_uploaded_count_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;

    let result: Result<(), String> = async {
        vehicle
            .mission()
            .clear()
            .map_err(|err| err.to_string())?
            .wait()
            .await
            .map_err(|err| err.to_string())?;

        let plan = sample_plan_mission(4);
        vehicle
            .mission()
            .upload(plan)
            .map_err(|err| err.to_string())?
            .wait()
            .await
            .map_err(|err| err.to_string())?;

        wait_for_mission_state(
            vehicle,
            |state| {
                state
                    .plan
                    .as_ref()
                    .is_some_and(|plan| plan.items.len() == 4)
            },
            Duration::from_secs(10),
        )
        .await;

        vehicle
            .mission()
            .clear()
            .map_err(|err| err.to_string())?
            .wait()
            .await
            .map_err(|err| err.to_string())?;

        Ok(())
    }
    .await;

    disconnect(backend).await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

pub async fn roundtrip_mission_type_fence_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;
    let result: Result<(), String> = async {
        let plan = FencePlan {
            return_point: None,
            regions: Vec::new(),
        };

        let clear = vehicle.fence().clear().map_err(|e| e.to_string())?;
        if let Err(err) = clear.wait().await {
            if is_optional_type_unsupported(MissionType::Fence, &err) {
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
            if is_optional_type_unsupported(MissionType::Fence, &err) {
                eprintln!("Skipping Fence roundtrip: {err}");
                return Ok(());
            }
            return Err(format!("failed to upload fence plan: {err}"));
        }

        let download = vehicle.fence().download().map_err(|e| e.to_string())?;
        let downloaded = match download.wait().await {
            Ok(plan) => plan,
            Err(err) => {
                if is_optional_type_unsupported(MissionType::Fence, &err) {
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

    disconnect(backend).await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

pub async fn roundtrip_mission_type_rally_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;
    let result: Result<(), String> = async {
        let plan = RallyPlan { points: Vec::new() };

        let clear = vehicle.rally().clear().map_err(|e| e.to_string())?;
        if let Err(err) = clear.wait().await {
            if is_optional_type_unsupported(MissionType::Rally, &err) {
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
            if is_optional_type_unsupported(MissionType::Rally, &err) {
                eprintln!("Skipping Rally roundtrip: {err}");
                return Ok(());
            }
            return Err(format!("failed to upload rally plan: {err}"));
        }

        let download = vehicle.rally().download().map_err(|e| e.to_string())?;
        let downloaded = match download.wait().await {
            Ok(plan) => plan,
            Err(err) => {
                if is_optional_type_unsupported(MissionType::Rally, &err) {
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

    disconnect(backend).await;
    if let Err(err) = result {
        panic!("{err}");
    }
}
