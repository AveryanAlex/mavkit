use crate::common::target::TestTarget;
#[cfg(feature = "sim")]
use crate::common::target::VehicleProfile;
use crate::common::wait::wait_for_telemetry;
use mavkit::Vehicle;
#[cfg(feature = "sim")]
use mavkit::VehicleConfig;
#[cfg(feature = "sim")]
use mavkit::sim::{DemoClock, DemoProfile, DemoVehicle};
use std::time::Duration;

#[cfg(feature = "sim")]
use mavkit::sim::DemoVehicleHandle;

pub struct BackendVehicle {
    pub vehicle: Vehicle,
    #[cfg(feature = "sim")]
    pub(crate) demo_handle: Option<DemoVehicleHandle>,
    #[cfg(feature = "sim")]
    manual_tick_hz: Option<u32>,
}

pub async fn setup_backend_vehicle(target: TestTarget) -> BackendVehicle {
    match target {
        TestTarget::Sitl(_) => setup_sitl_backend_vehicle(target).await,
        #[cfg(feature = "sim")]
        TestTarget::Sim(_) => {
            setup_demo_backend_vehicle_with_clock(target, DemoClock::RealTime, 10).await
        }
    }
}

#[cfg(feature = "sim")]
#[allow(dead_code)]
pub async fn setup_manual_backend_vehicle(target: TestTarget, tick_hz: u32) -> BackendVehicle {
    setup_demo_backend_vehicle_with_clock(target, DemoClock::Manual, tick_hz).await
}

pub async fn setup_runtime_backend_vehicle(target: TestTarget) -> BackendVehicle {
    match target {
        TestTarget::Sitl(_) => setup_sitl_backend_vehicle(target).await,
        #[cfg(feature = "sim")]
        TestTarget::Sim(_) => setup_manual_backend_vehicle(target, 10).await,
    }
}

#[cfg(feature = "sim")]
async fn setup_demo_backend_vehicle_with_clock(
    target: TestTarget,
    clock: DemoClock,
    tick_hz: u32,
) -> BackendVehicle {
    let profile = match target {
        TestTarget::Sim(VehicleProfile::Copter) => DemoProfile::ArduCopter,
        TestTarget::Sim(VehicleProfile::Plane) => DemoProfile::ArduPlane,
        TestTarget::Sim(VehicleProfile::QuadPlane) => DemoProfile::ArduQuadPlane,
        TestTarget::Sitl(_) => panic!("SITL target cannot be started as a demo backend"),
    };

    let config = VehicleConfig {
        connect_timeout: Duration::from_secs(5),
        command_timeout: Duration::from_secs(3),
        command_completion_timeout: Duration::from_secs(5),
        transfer_timeout: Duration::from_secs(10),
        ..VehicleConfig::default()
    };

    let (vehicle, demo_handle) = DemoVehicle::builder()
        .profile(profile)
        .clock(clock)
        .tick_hz(tick_hz)
        .connect(config)
        .await
        .expect("should connect demo vehicle");
    wait_for_telemetry(&vehicle, Duration::from_secs(10))
        .await
        .expect("should receive telemetry from demo simulator");

    BackendVehicle {
        vehicle,
        demo_handle: Some(demo_handle),
        manual_tick_hz: matches!(clock, DemoClock::Manual).then_some(tick_hz),
    }
}

async fn setup_sitl_backend_vehicle(target: TestTarget) -> BackendVehicle {
    const CONNECT_TIMEOUT: Duration = Duration::from_secs(30);

    validate_sitl_target_matches_env(target).expect("SITL target/profile mismatch");

    let vehicle = connect_sitl_vehicle().await.unwrap();
    wait_for_telemetry(&vehicle, CONNECT_TIMEOUT)
        .await
        .expect("should receive telemetry from SITL");

    BackendVehicle {
        vehicle,
        #[cfg(feature = "sim")]
        demo_handle: None,
        #[cfg(feature = "sim")]
        manual_tick_hz: None,
    }
}

fn validate_sitl_target_matches_env(target: TestTarget) -> Result<(), String> {
    let expected = target
        .sitl_profile()
        .ok_or_else(|| format!("{target:?} is not a SITL target"))?;
    let active = TestTarget::active_sitl_profile()?;
    if expected == active {
        return Ok(());
    }

    Err(format!(
        "{target:?} requires MAVKIT_SITL_TARGET={}, but current SITL endpoint is selected as {}",
        expected.sitl_target_name(),
        active.sitl_target_name()
    ))
}

pub async fn wait_for_runtime_condition<F>(
    _backend: &BackendVehicle,
    timeout: Duration,
    description: &str,
    mut condition: F,
) -> Result<(), String>
where
    F: FnMut() -> Result<bool, String>,
{
    if condition()? {
        return Ok(());
    }

    #[cfg(feature = "sim")]
    if let (Some(demo_handle), Some(tick_hz)) = (&_backend.demo_handle, _backend.manual_tick_hz) {
        let max_steps = (timeout.as_secs_f64() * f64::from(tick_hz)).ceil() as usize;
        for _ in 0..max_steps.max(1) {
            demo_handle.step().await.map_err(|err| {
                format!("manual demo step failed while waiting for {description}: {err}")
            })?;
            if condition()? {
                return Ok(());
            }
        }

        return Err(format!("timed out waiting for {description}"));
    }

    let deadline = tokio::time::Instant::now() + timeout;
    loop {
        if tokio::time::Instant::now() >= deadline {
            return Err(format!("timed out waiting for {description}"));
        }
        tokio::time::sleep(Duration::from_millis(200)).await;
        if condition()? {
            return Ok(());
        }
    }
}

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

#[cfg(feature = "tcp")]
async fn prime_sitl_tcp_streams(vehicle: &Vehicle) -> Result<(), String> {
    const SITL_STREAM_RATE_HZ: f32 = 5.0;

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

pub async fn disconnect(backend: BackendVehicle) {
    let _ = backend.vehicle.disconnect().await;
    #[cfg(feature = "sim")]
    if let Some(demo_handle) = backend.demo_handle.as_ref() {
        let _ = demo_handle.shutdown().await;
    }
}
