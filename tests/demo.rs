#![cfg(feature = "sim")]

mod common;

use common::backend::{disconnect, setup_backend_vehicle};
use common::clock::{Instant, sleep};
use common::fixtures::sample_plan_mission;
use common::target::TestTarget;
use mavkit::{GeoPoint3dMsl, GlobalPosition};
use std::time::Duration;
#[cfg(target_arch = "wasm32")]
use wasm_bindgen_test::wasm_bindgen_test;

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
        #[cfg_attr(not(target_arch = "wasm32"), tokio::test)]
        #[cfg_attr(target_arch = "wasm32", wasm_bindgen_test)]
        async fn $name() {
            $path($($arg),*).await;
        }
    };
}

fn global_position_matches_msl(position: &GlobalPosition, target: &GeoPoint3dMsl) -> bool {
    (position.latitude_deg - target.latitude_deg).abs() <= 1e-7
        && (position.longitude_deg - target.longitude_deg).abs() <= 1e-7
        && (position.altitude_msl_m - target.altitude_msl_m).abs() <= 0.001
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
    demo_plane_auto_mission_progresses_through_rtl_and_land,
    common::runtime::auto_mission_progresses_through_rtl_and_land_case,
    TestTarget::SIM_PLANE
);
demo_case!(
    demo_quadplane_auto_mission_progresses_through_rtl_and_land,
    common::runtime::auto_mission_progresses_through_rtl_and_land_case,
    TestTarget::SIM_QUADPLANE
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
    demo_support_command_int_resolves,
    common::support::support_command_int_case,
    TestTarget::SIM_COPTER
);
demo_case!(
    demo_support_mission_fence_resolves,
    common::support::support_mission_fence_case,
    TestTarget::SIM_COPTER
);
demo_case!(
    demo_support_mission_rally_resolves,
    common::support::support_mission_rally_case,
    TestTarget::SIM_COPTER
);
demo_case!(
    demo_support_terrain_resolves,
    common::support::support_terrain_case,
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
demo_case!(
    demo_plane_guided_reposition_reaches_target,
    common::runtime::guided_movement_reaches_target_case,
    TestTarget::SIM_PLANE
);
demo_case!(
    demo_quadplane_guided_reposition_reaches_target,
    common::runtime::guided_movement_reaches_target_case,
    TestTarget::SIM_QUADPLANE
);

#[cfg_attr(not(target_arch = "wasm32"), tokio::test)]
#[cfg_attr(target_arch = "wasm32", wasm_bindgen_test)]
async fn demo_teleport_to_updates_snapshot_and_telemetry_without_step() {
    let backend = common::backend::setup_manual_backend_vehicle(TestTarget::SIM_COPTER, 10).await;
    let result: Result<(), String> = async {
        let vehicle = &backend.vehicle;
        let handle = backend
            .demo_handle
            .as_ref()
            .ok_or_else(|| String::from("demo handle missing for simulator target"))?;
        let target = GeoPoint3dMsl {
            latitude_deg: 42.390_123_4,
            longitude_deg: -71.148_765_4,
            altitude_msl_m: 73.25,
        };
        let before = handle.snapshot();

        let snapshot = handle
            .teleport_to(target.clone())
            .await
            .map_err(|err| err.to_string())?;

        if snapshot.time_boot_ms != before.time_boot_ms {
            return Err(format!(
                "teleport advanced manual clock from {} to {}",
                before.time_boot_ms, snapshot.time_boot_ms
            ));
        }
        if snapshot.home != before.home {
            return Err(String::from("teleport unexpectedly changed home position"));
        }
        if !global_position_matches_msl(
            &GlobalPosition {
                latitude_deg: snapshot.latitude_deg,
                longitude_deg: snapshot.longitude_deg,
                altitude_msl_m: snapshot.altitude_msl_m,
                relative_alt_m: snapshot.relative_alt_m,
            },
            &target,
        ) {
            return Err(format!(
                "snapshot did not move to target: snapshot={snapshot:?}, target={target:?}"
            ));
        }

        let expected_relative_alt_m = target.altitude_msl_m - before.home.altitude_m;
        if (snapshot.relative_alt_m - expected_relative_alt_m).abs() > 0.001 {
            return Err(format!(
                "relative altitude not recomputed: got {}, expected {}",
                snapshot.relative_alt_m, expected_relative_alt_m
            ));
        }

        let deadline = Instant::now() + Duration::from_secs(2);
        loop {
            if let Some(sample) = vehicle.telemetry().position().global().latest()
                && global_position_matches_msl(&sample.value, &target)
            {
                if (sample.value.relative_alt_m - expected_relative_alt_m).abs() > 0.001 {
                    return Err(format!(
                        "telemetry relative altitude not recomputed: got {}, expected {}",
                        sample.value.relative_alt_m, expected_relative_alt_m
                    ));
                }
                break;
            }

            if Instant::now() >= deadline {
                return Err(format!(
                    "timed out waiting for teleported telemetry; latest={:?}",
                    vehicle.telemetry().position().global().latest()
                ));
            }

            sleep(Duration::from_millis(20)).await;
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
    demo_set_invalid_custom_mode_rejected_without_state_change,
    common::modes::set_invalid_custom_mode_rejected_case,
    TestTarget::SIM_COPTER
);

#[cfg_attr(not(target_arch = "wasm32"), tokio::test)]
#[cfg_attr(target_arch = "wasm32", wasm_bindgen_test)]
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
    common::params::param_profile_specifics_case,
    TestTarget::SIM_PLANE
);
demo_case!(
    demo_quadplane_param_profile_specifics,
    common::params::param_profile_specifics_case,
    TestTarget::SIM_QUADPLANE
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
    common::raw::raw_subscribe_receives_power_and_output_messages_case,
    TestTarget::SIM_COPTER
);
demo_case!(
    demo_plane_raw_subscribe_receives_power_and_output_messages,
    common::raw::raw_subscribe_receives_power_and_output_messages_case,
    TestTarget::SIM_PLANE
);
demo_case!(
    demo_quadplane_raw_subscribe_receives_power_and_output_messages,
    common::raw::raw_subscribe_receives_power_and_output_messages_case,
    TestTarget::SIM_QUADPLANE
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
    demo_firmware_version_populates,
    common::identity::firmware_version_populates_case,
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
