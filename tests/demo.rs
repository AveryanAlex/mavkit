#![cfg(feature = "sim")]

mod common;

use common::backend::BackendVehicle;
use common::fixtures::sample_plan_mission;
use common::wait::wait_for_telemetry;
use mavkit::VehicleConfig;
use mavkit::dialect;
use mavkit::sim::DemoProfile;
use mavkit::sim::{DemoClock, DemoVehicle};
use mavkit::{CommandResult, VehicleError};
use std::time::Duration;
use tokio_stream::StreamExt;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum VehicleProfile {
    Copter,
    Plane,
    QuadPlane,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct TestTarget {
    profile: VehicleProfile,
}

impl TestTarget {
    const SIM_COPTER: Self = Self {
        profile: VehicleProfile::Copter,
    };

    const SIM_PLANE: Self = Self {
        profile: VehicleProfile::Plane,
    };

    const SIM_QUADPLANE: Self = Self {
        profile: VehicleProfile::QuadPlane,
    };

    fn expected_modes(self) -> &'static [(u32, &'static str)] {
        match self.profile {
            VehicleProfile::Copter => &[(0, "STABILIZE"), (4, "GUIDED"), (5, "LOITER"), (6, "RTL")],
            VehicleProfile::Plane => &[(0, "MANUAL"), (10, "AUTO"), (11, "RTL"), (15, "GUIDED")],
            VehicleProfile::QuadPlane => &[
                (10, "AUTO"),
                (15, "GUIDED"),
                (17, "QSTABILIZE"),
                (19, "QLOITER"),
            ],
        }
    }

    fn guided_mode(self) -> (u32, &'static str) {
        match self.profile {
            VehicleProfile::Copter => (4, "GUIDED"),
            VehicleProfile::Plane | VehicleProfile::QuadPlane => (15, "GUIDED"),
        }
    }

    fn hold_mode(self) -> (u32, &'static str) {
        match self.profile {
            VehicleProfile::Copter => (5, "LOITER"),
            VehicleProfile::Plane => (12, "LOITER"),
            VehicleProfile::QuadPlane => (19, "QLOITER"),
        }
    }

    fn mode_names(self) -> (&'static str, &'static str, &'static str) {
        match self.profile {
            VehicleProfile::Copter => ("STABILIZE", "GUIDED", "LOITER"),
            VehicleProfile::Plane => ("MANUAL", "GUIDED", "LOITER"),
            VehicleProfile::QuadPlane => ("QSTABILIZE", "GUIDED", "QLOITER"),
        }
    }
}

async fn setup_backend_vehicle(target: TestTarget) -> BackendVehicle {
    let profile = match target.profile {
        VehicleProfile::Copter => DemoProfile::ArduCopter,
        VehicleProfile::Plane => DemoProfile::ArduPlane,
        VehicleProfile::QuadPlane => DemoProfile::ArduQuadPlane,
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
        .clock(DemoClock::RealTime)
        .tick_hz(10)
        .connect(config)
        .await
        .expect("should connect demo vehicle");
    wait_for_telemetry(&vehicle, Duration::from_secs(10))
        .await
        .expect("should receive telemetry from demo simulator");

    BackendVehicle {
        vehicle,
        demo_handle: Some(demo_handle),
    }
}

async fn disconnect(backend: BackendVehicle) {
    common::backend::disconnect(backend).await;
}

async fn mode_catalog_names(target: TestTarget) -> Vec<String> {
    let backend = setup_backend_vehicle(target).await;
    let result = backend
        .vehicle
        .available_modes()
        .catalog()
        .wait_timeout(Duration::from_secs(10))
        .await
        .map(|catalog| {
            catalog
                .into_iter()
                .map(|mode| mode.name)
                .collect::<Vec<_>>()
        });

    disconnect(backend).await;
    result.expect("mode catalog should be available")
}

macro_rules! demo_case {
    ($name:ident, $path:path $(, $arg:expr )* $(,)?) => {
        #[tokio::test]
        async fn $name() {
            $path($($arg),*).await;
        }
    };
}

async fn demo_param_profile_specific_case(
    target: TestTarget,
    expected_present: &[&str],
    expected_absent: &[&str],
    expected_values: &[(&str, f32)],
) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;

    let result: Result<(), String> = async {
        let store = vehicle
            .params()
            .download_all()
            .map_err(|err| err.to_string())?
            .wait()
            .await
            .map_err(|err| err.to_string())?;

        for name in expected_present {
            if !store.params.contains_key(*name) {
                return Err(format!("expected profile param {name} to be present"));
            }
        }

        for name in expected_absent {
            if store.params.contains_key(*name) {
                return Err(format!("expected profile param {name} to be absent"));
            }
        }

        for (name, expected_value) in expected_values {
            let param = store
                .params
                .get(*name)
                .ok_or_else(|| format!("expected profile param {name} to be present"))?;
            if (param.value - expected_value).abs() > f32::EPSILON {
                return Err(format!(
                    "expected profile param {name} to be {expected_value}, got {}",
                    param.value
                ));
            }
        }

        Ok(())
    }
    .await;

    disconnect(backend).await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

async fn demo_raw_subscribe_receives_power_and_output_messages_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;
    let result: Result<(), String> = async {
        let stream = vehicle.raw().subscribe();
        tokio::pin!(stream);
        let mut seen_ids = std::collections::HashSet::new();
        let deadline = tokio::time::sleep(Duration::from_secs(5));
        tokio::pin!(deadline);

        loop {
            tokio::select! {
                _ = &mut deadline => break,
                msg = stream.next() => {
                    let msg = msg.ok_or_else(|| String::from("raw subscription stream closed"))?;
                    seen_ids.insert(msg.message_id);
                    if seen_ids.contains(&147) && seen_ids.contains(&65) && seen_ids.contains(&36) {
                        break;
                    }
                }
            }
        }

        for expected_id in [147, 65, 36] {
            if !seen_ids.contains(&expected_id) {
                return Err(format!(
                    "expected raw message_id {expected_id}, saw {seen_ids:?}"
                ));
            }
        }

        Ok(())
    }
    .await;

    disconnect(backend).await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

demo_case!(
    demo_clear_then_download_mission_is_empty,
    common::mission::clear_then_download_empty_case,
    TestTarget::SIM_COPTER
);
demo_case!(
    demo_set_current_updates_mission_state,
    common::mission::set_current_updates_mission_state_case,
    TestTarget::SIM_COPTER
);
demo_case!(
    demo_upload_overwrites_previous_mission,
    common::mission::upload_overwrites_previous_mission_case,
    TestTarget::SIM_COPTER
);
demo_case!(
    demo_mission_state_reflects_uploaded_count,
    common::mission::mission_state_reflects_uploaded_count_case,
    TestTarget::SIM_COPTER
);
demo_case!(
    demo_roundtrip_mission_no_home,
    common::mission::roundtrip_case,
    TestTarget::SIM_COPTER,
    sample_plan_mission(3)
);
demo_case!(
    demo_roundtrip_mission_20_items,
    common::mission::roundtrip_case,
    TestTarget::SIM_COPTER,
    sample_plan_mission(20)
);
demo_case!(
    demo_plane_roundtrip_mission_no_home,
    common::mission::roundtrip_case,
    TestTarget::SIM_PLANE,
    sample_plan_mission(3)
);
demo_case!(
    demo_quadplane_roundtrip_mission_no_home,
    common::mission::roundtrip_case,
    TestTarget::SIM_QUADPLANE,
    sample_plan_mission(3)
);
demo_case!(
    demo_roundtrip_mission_type_fence,
    common::mission::roundtrip_mission_type_fence_case,
    TestTarget::SIM_COPTER
);
demo_case!(
    demo_roundtrip_mission_type_rally,
    common::mission::roundtrip_mission_type_rally_case,
    TestTarget::SIM_COPTER
);

demo_case!(
    demo_force_arm_disarm_cycle,
    common::commands::force_arm_disarm_cycle_case,
    TestTarget::SIM_COPTER
);
demo_case!(
    demo_set_mode_by_name,
    common::modes::set_mode_by_name_case,
    TestTarget::SIM_COPTER
);
demo_case!(
    demo_home_position_watch_populates,
    common::commands::home_position_watch_populates_case,
    TestTarget::SIM_COPTER
);
demo_case!(
    demo_support_discovery_reports_ardupilot,
    common::commands::support_discovery_reports_ardupilot_case,
    TestTarget::SIM_COPTER
);
demo_case!(
    demo_set_home_current_updates_home,
    common::commands::set_home_current_updates_home_case,
    TestTarget::SIM_COPTER
);
demo_case!(
    demo_disconnect_transitions_link_state,
    common::commands::disconnect_transitions_link_state_case,
    TestTarget::SIM_COPTER
);

demo_case!(
    demo_modes_catalog_entries_have_names_and_ids,
    common::modes::mode_catalog_case,
    TestTarget::SIM_COPTER
);
demo_case!(
    demo_modes_current_mode_stream_updates_on_switch,
    common::modes::set_mode_by_name_case,
    TestTarget::SIM_COPTER
);
demo_case!(
    demo_modes_set_invalid_name_returns_error,
    common::modes::set_invalid_name_returns_error_case,
    TestTarget::SIM_COPTER
);
demo_case!(
    demo_plane_modes_catalog_entries_have_names_and_ids,
    common::modes::mode_catalog_case,
    TestTarget::SIM_PLANE
);
demo_case!(
    demo_quadplane_modes_catalog_entries_have_names_and_ids,
    common::modes::mode_catalog_case,
    TestTarget::SIM_QUADPLANE
);
demo_case!(
    demo_plane_modes_current_mode_stream_updates_on_switch,
    common::modes::set_mode_by_name_case,
    TestTarget::SIM_PLANE
);
demo_case!(
    demo_quadplane_modes_current_mode_stream_updates_on_switch,
    common::modes::set_mode_by_name_case,
    TestTarget::SIM_QUADPLANE
);
demo_case!(
    demo_set_flight_mode_by_id,
    common::modes::set_flight_mode_case,
    TestTarget::SIM_COPTER
);

#[tokio::test]
async fn demo_set_invalid_custom_mode_rejected_without_state_change() {
    let backend = setup_backend_vehicle(TestTarget::SIM_COPTER).await;
    let vehicle = &backend.vehicle;
    let result: Result<(), String> = async {
        let demo_handle = backend
            .demo_handle
            .as_ref()
            .ok_or_else(|| String::from("demo handle unavailable"))?;
        let before = demo_handle.snapshot().custom_mode;
        let err = vehicle
            .set_mode(999)
            .await
            .expect_err("invalid custom mode should be rejected");

        match err {
            VehicleError::CommandRejected { command, result } => {
                if command != dialect::MavCmd::MAV_CMD_DO_SET_MODE as u16 {
                    return Err(format!(
                        "expected DO_SET_MODE rejection, got command {command}"
                    ));
                }
                if result != CommandResult::Denied {
                    return Err(format!("expected denied result, got {result}"));
                }
            }
            other => return Err(format!("expected command rejection, got {other}")),
        }

        let after = demo_handle.snapshot().custom_mode;
        if after != before {
            return Err(format!(
                "invalid mode changed simulator custom_mode from {before} to {after}"
            ));
        }

        Ok(())
    }
    .await;

    disconnect(backend).await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

#[tokio::test]
async fn demo_plane_and_quadplane_mode_catalogs_are_profile_specific() {
    let plane_modes = mode_catalog_names(TestTarget::SIM_PLANE).await;
    let quadplane_modes = mode_catalog_names(TestTarget::SIM_QUADPLANE).await;

    for q_mode in ["QSTABILIZE", "QHOVER", "QLOITER", "QLAND", "QRTL"] {
        assert!(
            !plane_modes.iter().any(|mode| mode == q_mode),
            "plane catalog unexpectedly exposed {q_mode}"
        );
        assert!(
            quadplane_modes.iter().any(|mode| mode == q_mode),
            "quadplane catalog did not expose {q_mode}"
        );
    }
}

demo_case!(
    demo_param_download_all,
    common::params::param_download_all_case,
    TestTarget::SIM_COPTER
);
demo_case!(
    demo_param_write_and_readback,
    common::params::param_write_and_readback_case,
    TestTarget::SIM_COPTER
);
demo_case!(
    demo_param_write_batch_and_readback,
    common::params::param_write_batch_and_readback_case,
    TestTarget::SIM_COPTER
);
demo_case!(
    demo_param_progress_during_download,
    common::params::param_progress_during_download_case,
    TestTarget::SIM_COPTER
);
demo_case!(
    demo_param_store_watch_updates_on_write,
    common::params::param_store_watch_updates_on_write_case,
    TestTarget::SIM_COPTER
);
demo_case!(
    demo_param_download_twice_is_consistent,
    common::params::param_download_twice_is_consistent_case,
    TestTarget::SIM_COPTER
);
demo_case!(
    demo_param_write_nonexistent_returns_error,
    common::params::param_write_nonexistent_returns_error_case,
    TestTarget::SIM_COPTER
);
demo_case!(
    demo_param_subscribe_emits_on_download,
    common::params::param_subscribe_emits_on_download_case,
    TestTarget::SIM_COPTER
);
demo_case!(
    demo_plane_param_profile_specifics,
    demo_param_profile_specific_case,
    TestTarget::SIM_PLANE,
    &["AIRSPEED_CRUISE", "RTL_ALTITUDE", "Q_ENABLE"],
    &["WPNAV_SPEED", "Q_FRAME_CLASS", "Q_OPTIONS", "Q_RTL_ALT"],
    &[("Q_ENABLE", 0.0)]
);
demo_case!(
    demo_quadplane_param_profile_specifics,
    demo_param_profile_specific_case,
    TestTarget::SIM_QUADPLANE,
    &[
        "AIRSPEED_CRUISE",
        "RTL_ALTITUDE",
        "Q_ENABLE",
        "Q_FRAME_CLASS",
        "Q_OPTIONS",
        "Q_RTL_ALT"
    ],
    &["WPNAV_SPEED"],
    &[("Q_ENABLE", 1.0)]
);

demo_case!(
    demo_telemetry_attitude_euler_available,
    common::telemetry::telemetry_attitude_euler_available_case,
    TestTarget::SIM_COPTER
);
demo_case!(
    demo_telemetry_battery_voltage_available,
    common::telemetry::telemetry_battery_voltage_available_case,
    TestTarget::SIM_COPTER
);
demo_case!(
    demo_telemetry_gps_quality_available,
    common::telemetry::telemetry_gps_quality_available_case,
    TestTarget::SIM_COPTER
);
demo_case!(
    demo_telemetry_groundspeed_available,
    common::telemetry::telemetry_groundspeed_available_case,
    TestTarget::SIM_COPTER
);
demo_case!(
    demo_telemetry_heading_available,
    common::telemetry::telemetry_heading_available_case,
    TestTarget::SIM_COPTER
);
demo_case!(
    demo_telemetry_sensor_health_available,
    common::telemetry::telemetry_sensor_health_available_case,
    TestTarget::SIM_COPTER
);
demo_case!(
    demo_telemetry_position_near_home,
    common::telemetry::telemetry_position_near_home_case,
    TestTarget::SIM_COPTER
);
demo_case!(
    demo_basic_telemetry_suite,
    common::telemetry::basic_telemetry_case,
    TestTarget::SIM_COPTER
);
demo_case!(
    demo_telemetry_position_observation_available,
    common::telemetry::telemetry_position_observation_available_case,
    TestTarget::SIM_COPTER
);

demo_case!(
    demo_raw_subscribe_receives_heartbeats,
    common::raw::raw_subscribe_receives_heartbeats_case,
    TestTarget::SIM_COPTER
);
demo_case!(
    demo_raw_request_message_autopilot_version,
    common::raw::raw_request_message_autopilot_version_case,
    TestTarget::SIM_COPTER
);
demo_case!(
    demo_raw_subscribe_unfiltered_receives_multiple_types,
    common::raw::raw_subscribe_unfiltered_receives_multiple_types_case,
    TestTarget::SIM_COPTER
);
demo_case!(
    demo_raw_subscribe_receives_power_and_output_messages,
    demo_raw_subscribe_receives_power_and_output_messages_case,
    TestTarget::SIM_COPTER
);

demo_case!(
    demo_vehicle_identity_is_ardupilot,
    common::identity::vehicle_identity_is_ardupilot_case,
    TestTarget::SIM_COPTER
);
demo_case!(
    demo_firmware_info_populates,
    common::identity::firmware_info_populates_case,
    TestTarget::SIM_COPTER
);
demo_case!(
    demo_link_state_connected_after_setup,
    common::identity::link_state_connected_after_setup_case,
    TestTarget::SIM_COPTER
);
demo_case!(
    demo_persistent_identity_resolves,
    common::identity::persistent_identity_resolves_case,
    TestTarget::SIM_COPTER
);
