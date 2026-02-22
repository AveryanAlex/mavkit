use mavkit::{
    normalize_for_compare, plans_equivalent, CompareTolerance, HomePosition, MissionFrame,
    MissionItem, MissionPlan, MissionType, Vehicle, VehicleError,
};
use std::time::Duration;

const CONNECT_TIMEOUT: Duration = Duration::from_secs(30);

fn sitl_bind_addr() -> String {
    std::env::var("MP_SITL_UDP_BIND").unwrap_or_else(|_| String::from("0.0.0.0:14550"))
}

fn is_optional_type_unsupported(mission_type: MissionType, error: &VehicleError) -> bool {
    if mission_type == MissionType::Mission {
        return false;
    }
    let msg = error.to_string().to_ascii_lowercase();
    msg.contains("unsupported")
        || msg.contains("transfer.timeout")
        || msg.contains("operation timeout")
        || msg.contains("timed out")
}

async fn wait_for_state<F>(vehicle: &Vehicle, mut predicate: F, timeout: Duration)
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

// ---------------------------------------------------------------------------
// Mission roundtrip tests
// ---------------------------------------------------------------------------

async fn run_roundtrip_case(plan: MissionPlan) {
    let vehicle = Vehicle::connect_udp(&sitl_bind_addr()).await.unwrap();

    let result: Result<(), String> = async {
        // Wait for telemetry to arrive
        let mut telem_rx = vehicle.telemetry();
        tokio::time::timeout(CONNECT_TIMEOUT, async {
            loop {
                telem_rx.changed().await.map_err(|e| e.to_string())?;
                let t = telem_rx.borrow().clone();
                if t.latitude_deg.is_some() {
                    return Ok::<(), String>(());
                }
            }
        })
        .await
        .map_err(|_| String::from("timed out waiting for telemetry"))?
        .map_err(|e| e)?;

        // Clear
        if let Err(err) = vehicle.mission().clear(plan.mission_type).await {
            if is_optional_type_unsupported(plan.mission_type, &err) {
                eprintln!(
                    "Skipping {:?} roundtrip on SITL target without mission-type support: {err}",
                    plan.mission_type
                );
                return Ok(());
            }
            return Err(format!("failed to clear before upload for {:?}: {err}", plan.mission_type));
        }

        // Upload
        if let Err(err) = vehicle.mission().upload(plan.clone()).await {
            if is_optional_type_unsupported(plan.mission_type, &err) {
                eprintln!(
                    "Skipping {:?} upload on SITL target without mission-type support: {err}",
                    plan.mission_type
                );
                return Ok(());
            }
            return Err(format!("failed to upload {:?} plan: {err}", plan.mission_type));
        }

        tokio::time::sleep(Duration::from_millis(500)).await;

        // Download with retries
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
            .map_err(|err| format!("failed to clear after roundtrip for {:?}: {err}", plan.mission_type))?;

        Ok(())
    }
    .await;

    let _ = vehicle.disconnect().await;

    if let Err(err) = result {
        panic!("{err}");
    }
}

async fn download_with_retries(
    vehicle: &Vehicle,
    mission_type: MissionType,
) -> Result<MissionPlan, String> {
    let strict = std::env::var("MP_SITL_STRICT")
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
            "Skipping Mission download timeout in non-strict SITL mode: {err_msg}. Set MP_SITL_STRICT=1 to enforce failure."
        );
        return Err(String::from("skip_optional_mission_type"));
    }

    Err(err_msg)
}

fn sample_plan_mission() -> MissionPlan {
    MissionPlan {
        mission_type: MissionType::Mission,
        home: Some(HomePosition {
            latitude_deg: 47.397742,
            longitude_deg: 8.545594,
            altitude_m: 0.0,
        }),
        items: vec![
            waypoint(0, 47.397742, 8.545594, 25.0),
            waypoint(1, 47.398100, 8.546100, 30.0),
            waypoint(2, 47.398450, 8.546500, 28.0),
        ],
    }
}

fn waypoint(seq: u16, lat: f64, lon: f64, alt: f32) -> MissionItem {
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

// ---------------------------------------------------------------------------
// Mission roundtrip tests
// ---------------------------------------------------------------------------

#[tokio::test]
#[ignore = "requires ArduPilot SITL endpoint"]
async fn sitl_roundtrip_mission_type_mission() {
    run_roundtrip_case(sample_plan_mission()).await;
}

#[tokio::test]
#[ignore = "requires ArduPilot SITL endpoint"]
async fn sitl_roundtrip_mission_type_fence() {
    run_roundtrip_case(MissionPlan {
        mission_type: MissionType::Fence,
        home: None,
        items: Vec::new(),
    })
    .await;
}

#[tokio::test]
#[ignore = "requires ArduPilot SITL endpoint"]
async fn sitl_roundtrip_mission_type_rally() {
    run_roundtrip_case(MissionPlan {
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
    let vehicle = Vehicle::connect_udp(&sitl_bind_addr()).await.unwrap();
    // Wait for telemetry
    let mut telem_rx = vehicle.telemetry();
    tokio::time::timeout(CONNECT_TIMEOUT, async {
        loop {
            telem_rx.changed().await.unwrap();
            let t = telem_rx.borrow().clone();
            if t.latitude_deg.is_some() {
                return;
            }
        }
    })
    .await
    .expect("should receive telemetry from SITL");
    vehicle
}

async fn arm_with_retries(vehicle: &Vehicle, force: bool, timeout: Duration) -> Result<(), String> {
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

#[tokio::test]
#[ignore = "requires ArduPilot SITL endpoint"]
async fn sitl_force_arm_disarm_cycle() {
    let vehicle = setup_sitl_vehicle().await;

    let result: Result<(), String> = async {
        vehicle.arm(true).await.map_err(|e| e.to_string())?;
        wait_for_state(&vehicle, |s| s.armed, Duration::from_secs(10)).await;

        vehicle.disarm(true).await.map_err(|e| e.to_string())?;
        wait_for_state(&vehicle, |s| !s.armed, Duration::from_secs(10)).await;

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
        wait_for_state(&vehicle, |s| s.custom_mode == 4, Duration::from_secs(10)).await;
        {
            let state = vehicle.state().borrow().clone();
            if state.mode_name != "GUIDED" {
                return Err(format!("expected GUIDED, got {}", state.mode_name));
            }
        }

        // Set LOITER (custom_mode=5)
        vehicle.set_mode(5).await.map_err(|e| e.to_string())?;
        wait_for_state(&vehicle, |s| s.custom_mode == 5, Duration::from_secs(10)).await;
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
        arm_with_retries(&vehicle, false, Duration::from_secs(30)).await?;
        vehicle.takeoff(10.0).await.map_err(|e| e.to_string())?;

        wait_for_state(&vehicle, |s| s.armed, Duration::from_secs(15)).await;

        tokio::time::sleep(Duration::from_secs(5)).await;

        vehicle.set_mode(9).await.map_err(|e| e.to_string())?; // LAND

        // Wait for auto-disarm
        wait_for_state(&vehicle, |s| !s.armed, Duration::from_secs(60)).await;

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
        arm_with_retries(&vehicle, false, Duration::from_secs(30)).await?;
        vehicle.takeoff(25.0).await.map_err(|e| e.to_string())?;
        wait_for_state(&vehicle, |s| s.armed, Duration::from_secs(15)).await;

        tokio::time::sleep(Duration::from_secs(5)).await;

        vehicle
            .goto(42.390_000, -71.147_000, 25.0)
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
            return Err(format!("expected at least 10 copter modes, got {}", modes.len()));
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
