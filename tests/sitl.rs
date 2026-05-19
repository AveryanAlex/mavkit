mod common;

#[path = "common/support.rs"]
mod support;

use common::backend::BackendVehicle;
use common::fixtures::sample_plan_mission;
use common::wait::wait_for_telemetry;
use mavkit::Vehicle;
use std::time::Duration;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct TestTarget;

impl TestTarget {
    const SITL_COPTER: Self = Self;

    fn expected_modes(self) -> &'static [(u32, &'static str)] {
        &[(0, "STABILIZE"), (4, "GUIDED"), (5, "LOITER"), (6, "RTL")]
    }

    fn guided_mode(self) -> (u32, &'static str) {
        (4, "GUIDED")
    }

    fn hold_mode(self) -> (u32, &'static str) {
        (5, "LOITER")
    }

    fn mode_names(self) -> (&'static str, &'static str, &'static str) {
        ("STABILIZE", "GUIDED", "LOITER")
    }
}

async fn setup_backend_vehicle(_: TestTarget) -> BackendVehicle {
    const CONNECT_TIMEOUT: Duration = Duration::from_secs(30);
    #[cfg(feature = "tcp")]
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

    async fn connect_sitl_vehicle() -> Result<Vehicle, String> {
        match sitl_endpoint() {
            SitlEndpoint::Tcp(addr) => connect_sitl_vehicle_tcp(&addr).await,
            SitlEndpoint::Udp(addr) => connect_sitl_vehicle_udp(&addr).await,
        }
    }

    let vehicle = connect_sitl_vehicle().await.unwrap();
    wait_for_telemetry(&vehicle, CONNECT_TIMEOUT)
        .await
        .expect("should receive telemetry from SITL");

    BackendVehicle {
        vehicle,
        #[cfg(feature = "sim")]
        demo_handle: None,
    }
}

async fn disconnect(backend: BackendVehicle) {
    common::backend::disconnect(backend).await;
}

macro_rules! sitl_case {
    ($name:ident, $path:path $(, $arg:expr )* $(,)?) => {
        #[tokio::test]
        #[ignore = "requires ArduPilot SITL endpoint"]
        async fn $name() {
            $path($($arg),*).await;
        }
    };
}

sitl_case!(
    sitl_roundtrip_mission_no_home,
    common::mission::roundtrip_case,
    TestTarget::SITL_COPTER,
    sample_plan_mission(3)
);
sitl_case!(
    sitl_roundtrip_mission_empty_items_with_home,
    common::mission::roundtrip_case,
    TestTarget::SITL_COPTER,
    sample_plan_mission(0)
);
sitl_case!(
    sitl_roundtrip_mission_empty_items_no_home,
    common::mission::roundtrip_case,
    TestTarget::SITL_COPTER,
    sample_plan_mission(0)
);
sitl_case!(
    sitl_clear_then_download_mission_is_empty,
    common::mission::clear_then_download_empty_case,
    TestTarget::SITL_COPTER
);
sitl_case!(
    sitl_set_current_updates_mission_state,
    common::mission::set_current_updates_mission_state_case,
    TestTarget::SITL_COPTER
);
sitl_case!(
    sitl_roundtrip_mission_20_items,
    common::mission::roundtrip_case,
    TestTarget::SITL_COPTER,
    sample_plan_mission(20)
);
sitl_case!(
    sitl_upload_overwrites_previous_mission,
    common::mission::upload_overwrites_previous_mission_case,
    TestTarget::SITL_COPTER
);
sitl_case!(
    sitl_mission_state_reflects_uploaded_count,
    common::mission::mission_state_reflects_uploaded_count_case,
    TestTarget::SITL_COPTER
);
sitl_case!(
    sitl_roundtrip_mission_type_fence,
    common::mission::roundtrip_mission_type_fence_case,
    TestTarget::SITL_COPTER
);
sitl_case!(
    sitl_roundtrip_mission_type_rally,
    common::mission::roundtrip_mission_type_rally_case,
    TestTarget::SITL_COPTER
);

sitl_case!(
    sitl_set_mode_by_name,
    common::modes::set_mode_by_name_case,
    TestTarget::SITL_COPTER
);
sitl_case!(
    sitl_home_position_watch_populates,
    common::commands::home_position_watch_populates_case,
    TestTarget::SITL_COPTER
);
sitl_case!(
    sitl_support_discovery_reports_ardupilot,
    common::commands::support_discovery_reports_ardupilot_case,
    TestTarget::SITL_COPTER
);
sitl_case!(
    sitl_set_home_current_updates_home,
    common::commands::set_home_current_updates_home_case,
    TestTarget::SITL_COPTER
);
sitl_case!(
    sitl_disconnect_transitions_link_state,
    common::commands::disconnect_transitions_link_state_case,
    TestTarget::SITL_COPTER
);
sitl_case!(
    sitl_force_arm_disarm_cycle,
    common::commands::force_arm_disarm_cycle_case,
    TestTarget::SITL_COPTER
);

sitl_case!(
    sitl_modes_catalog_entries_have_names_and_ids,
    common::modes::mode_catalog_case,
    TestTarget::SITL_COPTER
);
sitl_case!(
    sitl_modes_current_mode_stream_updates_on_switch,
    common::modes::set_mode_by_name_case,
    TestTarget::SITL_COPTER
);
sitl_case!(
    sitl_modes_set_invalid_name_returns_error,
    common::modes::set_invalid_name_returns_error_case,
    TestTarget::SITL_COPTER
);
sitl_case!(
    sitl_get_available_modes,
    common::modes::mode_catalog_case,
    TestTarget::SITL_COPTER
);
sitl_case!(
    sitl_set_flight_mode,
    common::modes::set_flight_mode_case,
    TestTarget::SITL_COPTER
);

sitl_case!(
    sitl_param_download_all,
    common::params::param_download_all_case,
    TestTarget::SITL_COPTER
);
sitl_case!(
    sitl_param_write_and_readback,
    common::params::param_write_and_readback_case,
    TestTarget::SITL_COPTER
);
sitl_case!(
    sitl_param_write_batch_and_readback,
    common::params::param_write_batch_and_readback_case,
    TestTarget::SITL_COPTER
);
sitl_case!(
    sitl_param_progress_during_download,
    common::params::param_progress_during_download_case,
    TestTarget::SITL_COPTER
);
sitl_case!(
    sitl_param_store_watch_updates_on_write,
    common::params::param_store_watch_updates_on_write_case,
    TestTarget::SITL_COPTER
);
sitl_case!(
    sitl_param_download_twice_is_consistent,
    common::params::param_download_twice_is_consistent_case,
    TestTarget::SITL_COPTER
);
sitl_case!(
    sitl_param_write_nonexistent_returns_error,
    common::params::param_write_nonexistent_returns_error_case,
    TestTarget::SITL_COPTER
);
sitl_case!(
    sitl_param_subscribe_emits_on_download,
    common::params::param_subscribe_emits_on_download_case,
    TestTarget::SITL_COPTER
);

sitl_case!(
    sitl_telemetry_attitude_euler_available,
    common::telemetry::telemetry_attitude_euler_available_case,
    TestTarget::SITL_COPTER
);
sitl_case!(
    sitl_telemetry_battery_voltage_available,
    common::telemetry::telemetry_battery_voltage_available_case,
    TestTarget::SITL_COPTER
);
sitl_case!(
    sitl_telemetry_gps_quality_available,
    common::telemetry::telemetry_gps_quality_available_case,
    TestTarget::SITL_COPTER
);
sitl_case!(
    sitl_telemetry_groundspeed_available,
    common::telemetry::telemetry_groundspeed_available_case,
    TestTarget::SITL_COPTER
);
sitl_case!(
    sitl_telemetry_heading_available,
    common::telemetry::telemetry_heading_available_case,
    TestTarget::SITL_COPTER
);
sitl_case!(
    sitl_telemetry_sensor_health_available,
    common::telemetry::telemetry_sensor_health_available_case,
    TestTarget::SITL_COPTER
);
sitl_case!(
    sitl_telemetry_position_near_sitl_home,
    common::telemetry::telemetry_position_near_home_case,
    TestTarget::SITL_COPTER
);
sitl_case!(
    sitl_basic_telemetry_suite,
    common::telemetry::basic_telemetry_case,
    TestTarget::SITL_COPTER
);
sitl_case!(
    sitl_telemetry_position_observation_available,
    common::telemetry::telemetry_position_observation_available_case,
    TestTarget::SITL_COPTER
);

sitl_case!(
    sitl_support_command_int_resolves,
    support::sitl_support_command_int_resolves
);
sitl_case!(
    sitl_support_mission_fence_resolves,
    support::sitl_support_mission_fence_resolves
);
sitl_case!(
    sitl_support_mission_rally_resolves,
    support::sitl_support_mission_rally_resolves
);
sitl_case!(
    sitl_support_terrain_resolves,
    support::sitl_support_terrain_resolves
);

sitl_case!(
    sitl_raw_subscribe_receives_heartbeats,
    common::raw::raw_subscribe_receives_heartbeats_case,
    TestTarget::SITL_COPTER
);
sitl_case!(
    sitl_raw_request_message_autopilot_version,
    common::raw::raw_request_message_autopilot_version_case,
    TestTarget::SITL_COPTER
);
sitl_case!(
    sitl_raw_subscribe_unfiltered_receives_multiple_types,
    common::raw::raw_subscribe_unfiltered_receives_multiple_types_case,
    TestTarget::SITL_COPTER
);

sitl_case!(
    sitl_vehicle_identity_is_ardupilot,
    common::identity::vehicle_identity_is_ardupilot_case,
    TestTarget::SITL_COPTER
);
sitl_case!(
    sitl_firmware_info_populates,
    common::identity::firmware_info_populates_case,
    TestTarget::SITL_COPTER
);
sitl_case!(
    sitl_link_state_connected_after_setup,
    common::identity::link_state_connected_after_setup_case,
    TestTarget::SITL_COPTER
);
sitl_case!(
    sitl_persistent_identity_resolves,
    common::identity::persistent_identity_resolves_case,
    TestTarget::SITL_COPTER
);
