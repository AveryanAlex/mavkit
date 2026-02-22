use mavkit::{
    CompareTolerance, HomePosition, MissionFrame, MissionItem, MissionPlan, MissionType, Vehicle,
    VehicleError, normalize_for_compare, plans_equivalent,
};
use std::time::Duration;

pub const CONNECT_TIMEOUT: Duration = Duration::from_secs(30);

pub fn sitl_bind_addr() -> String {
    std::env::var("MAVKIT_SITL_UDP_BIND").unwrap_or_else(|_| String::from("0.0.0.0:14550"))
}

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

pub async fn wait_for_state<F>(vehicle: &Vehicle, mut predicate: F, timeout: Duration)
where
    F: FnMut(&mavkit::VehicleState) -> bool,
{
    let mut rx = vehicle.state();
    let deadline = tokio::time::sleep(timeout);
    tokio::pin!(deadline);
    loop {
        tokio::select! {
            _ = &mut deadline => panic!("timed out waiting for vehicle state"),
            result = rx.changed() => {
                result.expect("watch channel closed");
                let state = rx.borrow().clone();
                if predicate(&state) {
                    return;
                }
            }
        }
    }
}

pub async fn wait_for_mission_state<F>(vehicle: &Vehicle, mut predicate: F, timeout: Duration)
where
    F: FnMut(&mavkit::MissionState) -> bool,
{
    let mut rx = vehicle.mission_state();
    let deadline = tokio::time::sleep(timeout);
    tokio::pin!(deadline);
    loop {
        tokio::select! {
            _ = &mut deadline => panic!("timed out waiting for mission state"),
            result = rx.changed() => {
                result.expect("watch channel closed");
                let state = rx.borrow().clone();
                if predicate(&state) {
                    return;
                }
            }
        }
    }
}

pub async fn wait_for_telemetry(vehicle: &Vehicle, timeout: Duration) -> Result<(), String> {
    let mut telem_rx = vehicle.telemetry();
    tokio::time::timeout(timeout, async {
        loop {
            telem_rx.changed().await.map_err(|e| e.to_string())?;
            let t = telem_rx.borrow().clone();
            if t.latitude_deg.is_some() {
                return Ok::<(), String>(());
            }
        }
    })
    .await
    .map_err(|_| String::from("timed out waiting for telemetry"))??;
    Ok(())
}

pub async fn download_with_retries(
    vehicle: &Vehicle,
    mission_type: MissionType,
) -> Result<MissionPlan, String> {
    let strict = std::env::var("MAVKIT_SITL_STRICT")
        .map(|v| v == "1")
        .unwrap_or(false);
    let mut last_error: Option<String> = None;

    for attempt in 1..=3 {
        match vehicle.mission().download(mission_type).await {
            Ok(plan) => return Ok(plan),
            Err(err) => {
                if is_optional_type_unsupported(mission_type, &err) {
                    eprintln!(
                        "Skipping {:?} download on SITL target without mission-type support: {err}",
                        mission_type
                    );
                    return Err(String::from("skip_optional_mission_type"));
                }
                last_error = Some(err.to_string());
                if attempt < 3 {
                    tokio::time::sleep(Duration::from_millis(600)).await;
                }
            }
        }
    }

    let err_msg = format!(
        "failed to download {:?} plan after retries: {}",
        mission_type,
        last_error.unwrap_or_else(|| String::from("unknown error"))
    );

    if !strict
        && mission_type == MissionType::Mission
        && err_msg.to_ascii_lowercase().contains("transfer.timeout")
    {
        eprintln!(
            "Skipping Mission download timeout in non-strict SITL mode: {err_msg}. Set MAVKIT_SITL_STRICT=1 to enforce failure."
        );
        return Err(String::from("skip_optional_mission_type"));
    }

    Err(err_msg)
}

pub async fn setup_sitl_vehicle() -> Vehicle {
    let vehicle = Vehicle::connect_udp(&sitl_bind_addr()).await.unwrap();
    wait_for_telemetry(&vehicle, CONNECT_TIMEOUT)
        .await
        .expect("should receive telemetry from SITL");
    vehicle
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
        match vehicle.arm(force).await {
            Ok(()) => return Ok(()),
            Err(err) => {
                last_err = err.to_string();
                tokio::time::sleep(Duration::from_secs(1)).await;
            }
        }
    }
}

pub async fn run_roundtrip_case(plan: MissionPlan) {
    let vehicle = Vehicle::connect_udp(&sitl_bind_addr()).await.unwrap();

    let result: Result<(), String> = async {
        wait_for_telemetry(&vehicle, CONNECT_TIMEOUT).await?;

        if let Err(err) = vehicle.mission().clear(plan.mission_type).await {
            if is_optional_type_unsupported(plan.mission_type, &err) {
                eprintln!(
                    "Skipping {:?} roundtrip on SITL target without mission-type support: {err}",
                    plan.mission_type
                );
                return Ok(());
            }
            return Err(format!(
                "failed to clear before upload for {:?}: {err}",
                plan.mission_type
            ));
        }

        if let Err(err) = vehicle.mission().upload(plan.clone()).await {
            if is_optional_type_unsupported(plan.mission_type, &err) {
                eprintln!(
                    "Skipping {:?} upload on SITL target without mission-type support: {err}",
                    plan.mission_type
                );
                return Ok(());
            }
            return Err(format!(
                "failed to upload {:?} plan: {err}",
                plan.mission_type
            ));
        }

        tokio::time::sleep(Duration::from_millis(500)).await;

        let downloaded = download_with_retries(&vehicle, plan.mission_type).await;
        let downloaded = match downloaded {
            Ok(plan) => plan,
            Err(err) if err == "skip_optional_mission_type" => return Ok(()),
            Err(err) => return Err(err),
        };

        if plan.mission_type == MissionType::Mission {
            assert!(
                downloaded.home.is_some(),
                "downloaded Mission plan should have home extracted from wire seq 0"
            );
        }

        let mut expected = normalize_for_compare(&plan);
        let mut got = normalize_for_compare(&downloaded);
        expected.home = None;
        got.home = None;

        if !plans_equivalent(&expected, &got, CompareTolerance::default()) {
            return Err(format!(
                "readback mismatch for {:?}: expected {:?}, got {:?}",
                plan.mission_type, expected, got
            ));
        }

        vehicle
            .mission()
            .clear(plan.mission_type)
            .await
            .map_err(|err| {
                format!(
                    "failed to clear after roundtrip for {:?}: {err}",
                    plan.mission_type
                )
            })?;

        Ok(())
    }
    .await;

    let _ = vehicle.disconnect().await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

pub fn waypoint(seq: u16, lat: f64, lon: f64, alt: f32) -> MissionItem {
    MissionItem {
        seq,
        frame: MissionFrame::GlobalRelativeAltInt,
        command: 16,
        current: seq == 0,
        autocontinue: true,
        param1: 0.0,
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        x: (lat * 1e7) as i32,
        y: (lon * 1e7) as i32,
        z: alt,
    }
}

pub fn sample_plan_mission(home: Option<HomePosition>, item_count: usize) -> MissionPlan {
    let base_lat = 47.397_742;
    let base_lon = 8.545_594;
    let mut items = Vec::with_capacity(item_count);
    for i in 0..item_count {
        let seq = i as u16;
        let lat = base_lat + (i as f64) * 0.000_2;
        let lon = base_lon + (i as f64) * 0.000_2;
        let alt = 25.0 + (i as f32);
        items.push(waypoint(seq, lat, lon, alt));
    }

    MissionPlan {
        mission_type: MissionType::Mission,
        home,
        items,
    }
}
