#![cfg(all(target_arch = "wasm32", feature = "sim"))]

use mavkit::sim::{DemoClock, DemoVehicle, DemoVehicleHandle};
use mavkit::{AutopilotType, InitPolicyConfig, LinkState, Vehicle, VehicleConfig};
use std::time::Duration;
use wasm_bindgen_test::wasm_bindgen_test;

const REALTIME_POLL_INTERVAL_MS: u32 = 25;
const REALTIME_POLL_ATTEMPTS: usize = 60;

fn wasm_sim_config() -> VehicleConfig {
    let mut init_policy = InitPolicyConfig::default();
    init_policy.autopilot_version.enabled = false;
    init_policy.available_modes.enabled = false;
    init_policy.home.enabled = false;
    init_policy.origin.enabled = false;

    VehicleConfig {
        connect_timeout: Duration::from_secs(2),
        command_timeout: Duration::from_secs(2),
        command_completion_timeout: Duration::from_secs(2),
        transfer_timeout: Duration::from_secs(5),
        init_policy,
        auto_request_home: false,
        ..VehicleConfig::default()
    }
}

async fn connect_demo(clock: DemoClock, tick_hz: u32) -> (Vehicle, DemoVehicleHandle) {
    DemoVehicle::builder()
        .clock(clock)
        .tick_hz(tick_hz)
        .connect(wasm_sim_config())
        .await
        .expect("demo vehicle should connect on wasm")
}

async fn shutdown_demo(vehicle: Vehicle, handle: DemoVehicleHandle) {
    let _ = vehicle.disconnect().await;
    let _ = handle.shutdown().await;
}

async fn wait_for_snapshot_time_after(
    handle: &DemoVehicleHandle,
    previous_time_boot_ms: u32,
) -> u32 {
    for _ in 0..REALTIME_POLL_ATTEMPTS {
        let snapshot = handle.snapshot();
        if snapshot.time_boot_ms > previous_time_boot_ms {
            return snapshot.time_boot_ms;
        }

        gloo_timers::future::TimeoutFuture::new(REALTIME_POLL_INTERVAL_MS).await;
    }

    panic!("timed out waiting for realtime simulator clock to advance");
}

#[wasm_bindgen_test]
async fn manual_clock_connects_and_steps() {
    let (vehicle, handle) = connect_demo(DemoClock::Manual, 20).await;

    assert_eq!(vehicle.identity().autopilot, AutopilotType::ArduPilotMega);
    let link_state = vehicle
        .link()
        .state()
        .latest()
        .expect("link state should be populated after connect");
    assert!(matches!(link_state, LinkState::Connected));

    let position = vehicle
        .telemetry()
        .position()
        .global()
        .wait_timeout(Duration::from_secs(2))
        .await
        .expect("bootstrap position telemetry should arrive")
        .value;
    assert!(position.latitude_deg.is_finite());
    assert!(position.longitude_deg.is_finite());

    let before_time_boot_ms = handle.snapshot().time_boot_ms;
    let stepped = handle
        .step()
        .await
        .expect("manual simulator step should advance");
    assert!(stepped.time_boot_ms > before_time_boot_ms);

    shutdown_demo(vehicle, handle).await;
}

#[wasm_bindgen_test]
async fn realtime_clock_advances_with_runtime_sleep() {
    let (vehicle, handle) = connect_demo(DemoClock::RealTime, 20).await;

    let before_time_boot_ms = handle.snapshot().time_boot_ms;
    let after_time_boot_ms = wait_for_snapshot_time_after(&handle, before_time_boot_ms).await;
    assert!(after_time_boot_ms > before_time_boot_ms);

    shutdown_demo(vehicle, handle).await;
}

#[wasm_bindgen_test]
async fn manual_clock_processes_command_roundtrip() {
    let (vehicle, handle) = connect_demo(DemoClock::Manual, 10).await;

    vehicle
        .force_arm()
        .await
        .expect("force arm should be acknowledged by the simulator");
    assert!(handle.snapshot().armed);

    vehicle
        .set_mode(4)
        .await
        .expect("set mode should be acknowledged and observed");
    assert_eq!(handle.snapshot().custom_mode, 4);

    shutdown_demo(vehicle, handle).await;
}
