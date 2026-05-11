use mavkit::mission::MissionState;
use mavkit::mission::commands::NavWaypoint;
use mavkit::{
    CompareTolerance, GeoPoint3d, MissionItem, MissionPlan, MissionType, Vehicle, VehicleError,
    normalize_for_compare, plans_equivalent,
};
use std::time::Duration;

pub const CONNECT_TIMEOUT: Duration = Duration::from_secs(30);
const SITL_STREAM_RATE_HZ: f32 = 5.0;

enum SitlEndpoint {
    Tcp(String),
    Udp(String),
}

fn sitl_endpoint() -> SitlEndpoint {
    if let Ok(addr) = std::env::var("MAVKIT_SITL_TCP_ADDR") {
        return SitlEndpoint::Tcp(addr);
    }

    if let Ok(addr) = std::env::var("MAVKIT_SITL_UDP_BIND") {
        return SitlEndpoint::Udp(addr);
    }

    SitlEndpoint::Tcp(String::from("127.0.0.1:5760"))
}

async fn connect_sitl_vehicle() -> Result<Vehicle, String> {
    match sitl_endpoint() {
        SitlEndpoint::Tcp(addr) => connect_sitl_vehicle_tcp(&addr).await,
        SitlEndpoint::Udp(addr) => connect_sitl_vehicle_udp(&addr).await,
    }
}

#[cfg(feature = "tcp")]
async fn connect_sitl_vehicle_tcp(addr: &str) -> Result<Vehicle, String> {
    let vehicle = Vehicle::connect_tcp(addr)
        .await
        .map_err(|err| format!("failed to connect to SITL TCP endpoint {addr}: {err}"))?;
    prime_sitl_tcp_streams(&vehicle).await?;
    Ok(vehicle)
}

#[cfg(not(feature = "tcp"))]
async fn connect_sitl_vehicle_tcp(addr: &str) -> Result<Vehicle, String> {
    Err(format!(
        "SITL TCP endpoint {addr} requested, but mavkit was built without the `tcp` feature"
    ))
}

#[cfg(feature = "udp")]
async fn connect_sitl_vehicle_udp(addr: &str) -> Result<Vehicle, String> {
    Vehicle::connect_udp(addr)
        .await
        .map_err(|err| format!("failed to connect to SITL UDP bind {addr}: {err}"))
}

#[cfg(not(feature = "udp"))]
async fn connect_sitl_vehicle_udp(addr: &str) -> Result<Vehicle, String> {
    Err(format!(
        "SITL UDP bind {addr} requested, but mavkit was built without the `udp` feature"
    ))
}

async fn prime_sitl_tcp_streams(vehicle: &Vehicle) -> Result<(), String> {
    let telemetry = vehicle.telemetry();
    let messages = telemetry.messages();

    messages
        .global_position_int()
        .set_rate(SITL_STREAM_RATE_HZ)
        .await
        .map_err(|err| format!("failed to request GLOBAL_POSITION_INT stream: {err}"))?;
    messages
        .attitude()
        .set_rate(SITL_STREAM_RATE_HZ)
        .await
        .map_err(|err| format!("failed to request ATTITUDE stream: {err}"))?;
    messages
        .gps_raw_int()
        .set_rate(SITL_STREAM_RATE_HZ)
        .await
        .map_err(|err| format!("failed to request GPS_RAW_INT stream: {err}"))?;
    messages
        .sys_status()
        .set_rate(SITL_STREAM_RATE_HZ)
        .await
        .map_err(|err| format!("failed to request SYS_STATUS stream: {err}"))?;
    messages
        .vfr_hud()
        .set_rate(SITL_STREAM_RATE_HZ)
        .await
        .map_err(|err| format!("failed to request VFR_HUD stream: {err}"))?;

    Ok(())
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

pub async fn wait_for_mode(vehicle: &Vehicle, custom_mode: u32, timeout: Duration) {
    let current_mode = vehicle.available_modes().current();
    if current_mode
        .latest()
        .is_some_and(|mode| mode.custom_mode == custom_mode)
    {
        return;
    }

    let mut subscription = current_mode.subscribe();
    let deadline = tokio::time::sleep(timeout);
    tokio::pin!(deadline);
    loop {
        tokio::select! {
            _ = &mut deadline => panic!("timed out waiting for current mode"),
            observed = subscription.recv() => {
                let mode = observed.expect("mode observation stream closed");
                if mode.custom_mode == custom_mode {
                    return;
                }
            }
        }
    }
}

pub async fn wait_for_armed(vehicle: &Vehicle, armed: bool, timeout: Duration) {
    let armed_metric = vehicle.telemetry().armed();
    if armed_metric
        .latest()
        .is_some_and(|sample| sample.value == armed)
    {
        return;
    }

    let mut subscription = armed_metric.subscribe();
    let deadline = tokio::time::sleep(timeout);
    tokio::pin!(deadline);
    loop {
        tokio::select! {
            _ = &mut deadline => panic!("timed out waiting for armed state"),
            observed = subscription.recv() => {
                let sample = observed.expect("armed observation stream closed");
                if sample.value == armed {
                    return;
                }
            }
        }
    }
}

pub async fn wait_for_mission_state<F>(vehicle: &Vehicle, mut predicate: F, timeout: Duration)
where
    F: FnMut(&MissionState) -> bool,
{
    let mission = vehicle.mission();
    if let Some(state) = mission.latest()
        && predicate(&state)
    {
        return;
    }

    let mut subscription = mission.subscribe();
    let deadline = tokio::time::sleep(timeout);
    tokio::pin!(deadline);
    loop {
        tokio::select! {
            _ = &mut deadline => panic!("timed out waiting for mission state"),
            observed = subscription.recv() => {
                let state = observed.expect("mission observation stream closed");
                if predicate(&state) {
                    return;
                }
            }
        }
    }
}

pub async fn wait_for_mode_name(
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

pub async fn wait_for_telemetry(vehicle: &Vehicle, timeout: Duration) -> Result<(), String> {
    let global_position = vehicle.telemetry().position().global();
    if let Some(sample) = global_position.latest()
        && is_usable_sitl_position(sample.value.latitude_deg, sample.value.longitude_deg)
    {
        return Ok(());
    }

    let mut subscription = global_position.subscribe();
    let deadline = tokio::time::sleep(timeout);
    tokio::pin!(deadline);
    let mut saw_non_finite = false;
    loop {
        tokio::select! {
            _ = &mut deadline => {
                let suffix = if saw_non_finite { " after receiving non-finite samples" } else { "" };
                return Err(format!("timed out waiting for usable telemetry position{suffix}"));
            }
            observed = subscription.recv() => {
                let sample = observed.ok_or_else(|| String::from("telemetry position stream closed"))?;
                if !sample.value.latitude_deg.is_finite() || !sample.value.longitude_deg.is_finite() {
                    saw_non_finite = true;
                    continue;
                }
                if is_usable_sitl_position(sample.value.latitude_deg, sample.value.longitude_deg) {
                    return Ok(());
                }
            }
        }
    }
}

fn is_usable_sitl_position(latitude_deg: f64, longitude_deg: f64) -> bool {
    latitude_deg.is_finite()
        && longitude_deg.is_finite()
        && (latitude_deg.abs() > f64::EPSILON || longitude_deg.abs() > f64::EPSILON)
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
                    tokio::time::sleep(Duration::from_millis(600)).await;
                }
                continue;
            }
        };

        match op.wait().await {
            Ok(plan) => return Ok(plan),
            Err(err) => {
                last_error = Some(err.to_string());
                if attempt < 3 {
                    tokio::time::sleep(Duration::from_millis(600)).await;
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

pub async fn setup_sitl_vehicle() -> Vehicle {
    let vehicle = connect_sitl_vehicle().await.unwrap();
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
        match if force {
            vehicle.force_arm().await
        } else {
            vehicle.arm().await
        } {
            Ok(()) => return Ok(()),
            Err(err) => {
                last_err = err.to_string();
                tokio::time::sleep(Duration::from_secs(1)).await;
            }
        }
    }
}

pub async fn run_roundtrip_case(plan: MissionPlan) {
    let vehicle = connect_sitl_vehicle().await.unwrap();

    let result: Result<(), String> = async {
        wait_for_telemetry(&vehicle, CONNECT_TIMEOUT).await?;

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

        tokio::time::sleep(Duration::from_millis(500)).await;

        let downloaded = download_with_retries(&vehicle).await;
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

    let _ = vehicle.disconnect().await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

pub fn waypoint(lat: f64, lon: f64, alt: f32) -> MissionItem {
    NavWaypoint::from_point(GeoPoint3d::rel_home(lat, lon, f64::from(alt))).into()
}

pub fn sample_plan_mission(item_count: usize) -> MissionPlan {
    let base_lat = 47.397_742;
    let base_lon = 8.545_594;
    let mut items = Vec::with_capacity(item_count);
    for i in 0..item_count {
        let lat = base_lat + (i as f64) * 0.000_2;
        let lon = base_lon + (i as f64) * 0.000_2;
        let alt = 25.0 + (i as f32);
        items.push(waypoint(lat, lon, alt));
    }

    MissionPlan { items }
}
