use super::*;
use crate::dialect;
use crate::state::create_channels;
use crate::telemetry::TelemetryHandle;
use crate::test_support::{assert_approx, default_header};
use crate::types::SensorHealthState;
use std::time::Duration;

fn exact_50_char_text(value: &str) -> mavlink::types::CharArray<50> {
    assert_eq!(value.len(), 50, "test chunk must be exactly 50 chars");
    value.into()
}

#[test]
fn metric_source_map_global_position_and_vfr_hud() {
    let (writers, channels) = create_channels();
    let telemetry = TelemetryHandle::new(&channels.telemetry_handles);
    let target: Option<VehicleTarget> = None;

    let global_position =
        dialect::MavMessage::GLOBAL_POSITION_INT(dialect::GLOBAL_POSITION_INT_DATA {
            time_boot_ms: 250,
            lat: 473_977_420,
            lon: 85_455_940,
            alt: 510_000,
            relative_alt: 50_000,
            vx: 120,
            vy: 50,
            vz: -80,
            hdg: 27_000,
        });
    update_state(&default_header(), &global_position, &writers, &target);

    let global_sample = telemetry.position().global().latest().unwrap();
    assert_eq!(
        global_sample.source,
        crate::TelemetryMessageKind::GlobalPositionInt
    );
    assert_approx(global_sample.value.latitude_deg, 47.397742);
    assert_approx(global_sample.value.longitude_deg, 8.545594);
    assert!((global_sample.value.altitude_msl_m - 510.0).abs() < 0.001);
    assert!((global_sample.value.relative_alt_m - 50.0).abs() < 0.001);

    let vfr_hud = dialect::MavMessage::VFR_HUD(dialect::VFR_HUD_DATA {
        airspeed: 12.5,
        groundspeed: 10.0,
        heading: 180,
        throttle: 55,
        alt: 100.0,
        climb: 2.5,
    });
    update_state(&default_header(), &vfr_hud, &writers, &target);

    assert_eq!(
        telemetry
            .position()
            .groundspeed_mps()
            .latest()
            .unwrap()
            .source,
        crate::TelemetryMessageKind::VfrHud
    );
    assert_eq!(
        telemetry
            .position()
            .groundspeed_mps()
            .latest()
            .unwrap()
            .value,
        10.0
    );
    assert_eq!(
        telemetry.position().heading_deg().latest().unwrap().value,
        180.0
    );
    assert_eq!(
        telemetry
            .position()
            .climb_rate_mps()
            .latest()
            .unwrap()
            .value,
        2.5
    );
    assert_eq!(
        telemetry.position().airspeed_mps().latest().unwrap().value,
        12.5
    );
    assert_eq!(
        telemetry.position().throttle_pct().latest().unwrap().value,
        55.0
    );
}

#[test]
fn battery_voltage_prefers_primary_battery_status_with_sys_status_fallback() {
    let (writers, channels) = create_channels();
    let telemetry = TelemetryHandle::new(&channels.telemetry_handles);
    let target: Option<VehicleTarget> = None;

    let sys_status = dialect::MavMessage::SYS_STATUS(dialect::SYS_STATUS_DATA {
        onboard_control_sensors_present: dialect::MavSysStatusSensor::empty(),
        onboard_control_sensors_enabled: dialect::MavSysStatusSensor::empty(),
        onboard_control_sensors_health: dialect::MavSysStatusSensor::empty(),
        load: 0,
        voltage_battery: 12_600,
        current_battery: 520,
        battery_remaining: 71,
        drop_rate_comm: 0,
        errors_comm: 0,
        errors_count1: 0,
        errors_count2: 0,
        errors_count3: 0,
        errors_count4: 0,
        onboard_control_sensors_present_extended: dialect::MavSysStatusSensorExtended::empty(),
        onboard_control_sensors_enabled_extended: dialect::MavSysStatusSensorExtended::empty(),
        onboard_control_sensors_health_extended: dialect::MavSysStatusSensorExtended::empty(),
    });
    update_state(&default_header(), &sys_status, &writers, &target);

    let sys_fallback_sample = telemetry.battery().voltage_v().latest().unwrap();
    assert_eq!(
        sys_fallback_sample.source,
        crate::TelemetryMessageKind::SysStatus
    );
    assert!((sys_fallback_sample.value - 12.6).abs() < 0.001);

    let mut ignored_instance_cells = [u16::MAX; 10];
    ignored_instance_cells[0] = 4300;
    ignored_instance_cells[1] = 4200;
    ignored_instance_cells[2] = 4100;
    let battery_status_non_primary =
        dialect::MavMessage::BATTERY_STATUS(dialect::BATTERY_STATUS_DATA {
            current_consumed: -1,
            energy_consumed: -1,
            temperature: i16::MAX,
            voltages: ignored_instance_cells,
            current_battery: 600,
            id: 1,
            battery_function: dialect::MavBatteryFunction::MAV_BATTERY_FUNCTION_UNKNOWN,
            mavtype: dialect::MavBatteryType::MAV_BATTERY_TYPE_UNKNOWN,
            battery_remaining: 68,
            time_remaining: 0,
            charge_state: dialect::MavBatteryChargeState::MAV_BATTERY_CHARGE_STATE_OK,
            voltages_ext: [0; 4],
            mode: dialect::MavBatteryMode::MAV_BATTERY_MODE_UNKNOWN,
            fault_bitmask: dialect::MavBatteryFault::empty(),
        });
    update_state(
        &default_header(),
        &battery_status_non_primary,
        &writers,
        &target,
    );

    let still_sys_sample = telemetry.battery().voltage_v().latest().unwrap();
    assert_eq!(
        still_sys_sample.source,
        crate::TelemetryMessageKind::SysStatus
    );
    assert!((still_sys_sample.value - 12.6).abs() < 0.001);

    let mut cells = [u16::MAX; 10];
    cells[0] = 4200;
    cells[1] = 4150;
    cells[2] = 4100;
    let battery_status_primary =
        dialect::MavMessage::BATTERY_STATUS(dialect::BATTERY_STATUS_DATA {
            current_consumed: 0,
            energy_consumed: 7200,
            temperature: 0,
            voltages: cells,
            current_battery: 550,
            id: 0,
            battery_function: dialect::MavBatteryFunction::MAV_BATTERY_FUNCTION_UNKNOWN,
            mavtype: dialect::MavBatteryType::MAV_BATTERY_TYPE_UNKNOWN,
            battery_remaining: 66,
            time_remaining: 1800,
            charge_state: dialect::MavBatteryChargeState::MAV_BATTERY_CHARGE_STATE_OK,
            voltages_ext: [0; 4],
            mode: dialect::MavBatteryMode::MAV_BATTERY_MODE_UNKNOWN,
            fault_bitmask: dialect::MavBatteryFault::empty(),
        });
    update_state(
        &default_header(),
        &battery_status_primary,
        &writers,
        &target,
    );

    let primary_sample = telemetry.battery().voltage_v().latest().unwrap();
    assert_eq!(
        primary_sample.source,
        crate::TelemetryMessageKind::BatteryStatus
    );
    assert!((primary_sample.value - 12.45).abs() < 0.001);

    let sys_status_new = dialect::MavMessage::SYS_STATUS(dialect::SYS_STATUS_DATA {
        voltage_battery: 11_100,
        current_battery: 250,
        battery_remaining: 51,
        ..dialect::SYS_STATUS_DATA::default()
    });
    update_state(&default_header(), &sys_status_new, &writers, &target);

    let after_primary_seen_sample = telemetry.battery().voltage_v().latest().unwrap();
    assert_eq!(
        after_primary_seen_sample.source,
        crate::TelemetryMessageKind::BatteryStatus
    );
    assert!((after_primary_seen_sample.value - 12.45).abs() < 0.001);
}

#[test]
fn gps_hdop_conversion_uses_centi_units_and_sentinel() {
    let (writers, channels) = create_channels();
    let telemetry = TelemetryHandle::new(&channels.telemetry_handles);
    let target: Option<VehicleTarget> = None;

    let with_hdop = dialect::MavMessage::GPS_RAW_INT(dialect::GPS_RAW_INT_DATA {
        time_usec: 1_700_000_000_000_000,
        fix_type: dialect::GpsFixType::GPS_FIX_TYPE_3D_FIX,
        lat: 473_977_420,
        lon: 85_455_940,
        alt: 510_000,
        eph: 150,
        epv: u16::MAX,
        vel: u16::MAX,
        cog: u16::MAX,
        satellites_visible: 10,
        alt_ellipsoid: 0,
        h_acc: 0,
        v_acc: 0,
        vel_acc: 0,
        hdg_acc: 0,
        yaw: 0,
    });
    update_state(&default_header(), &with_hdop, &writers, &target);

    let hdop_sample = telemetry.gps().quality().latest().unwrap();
    assert_eq!(hdop_sample.source, crate::TelemetryMessageKind::GpsRawInt);
    assert_eq!(hdop_sample.value.hdop, Some(1.5));

    let sentinel_hdop = dialect::MavMessage::GPS_RAW_INT(dialect::GPS_RAW_INT_DATA {
        eph: u16::MAX,
        ..dialect::GPS_RAW_INT_DATA::default()
    });
    update_state(&default_header(), &sentinel_hdop, &writers, &target);

    let sentinel_sample = telemetry.gps().quality().latest().unwrap();
    assert_eq!(sentinel_sample.value.hdop, None);
}

#[test]
fn grouped_metric_handles_refresh_from_expected_sources() {
    let (writers, channels) = create_channels();
    let telemetry = TelemetryHandle::new(&channels.telemetry_handles);
    let target: Option<VehicleTarget> = None;

    let gps_raw = dialect::MavMessage::GPS_RAW_INT(dialect::GPS_RAW_INT_DATA {
        time_usec: 1_700_000_000_000_000,
        fix_type: dialect::GpsFixType::GPS_FIX_TYPE_3D_FIX,
        lat: 473_977_420,
        lon: 85_455_940,
        alt: 510_000,
        eph: 150,
        satellites_visible: 12,
        ..dialect::GPS_RAW_INT_DATA::default()
    });
    update_state(&default_header(), &gps_raw, &writers, &target);

    let gps_quality = telemetry.gps().quality().latest().unwrap();
    assert_eq!(gps_quality.source, crate::TelemetryMessageKind::GpsRawInt);
    assert_eq!(gps_quality.value.hdop, Some(1.5));

    let gps_position = telemetry.gps().position_msl().latest().unwrap();
    assert_eq!(gps_position.source, crate::TelemetryMessageKind::GpsRawInt);
    assert!((gps_position.value.latitude_deg - 47.397742).abs() < 0.0001);
    assert!((gps_position.value.longitude_deg - 8.545594).abs() < 0.0001);
    assert!((gps_position.value.altitude_msl_m - 510.0).abs() < 0.001);

    let attitude = dialect::MavMessage::ATTITUDE(dialect::ATTITUDE_DATA {
        time_boot_ms: 250,
        roll: 0.1,
        pitch: -0.2,
        yaw: 1.3,
        ..dialect::ATTITUDE_DATA::default()
    });
    update_state(&default_header(), &attitude, &writers, &target);

    let euler = telemetry.attitude().euler().latest().unwrap();
    assert_eq!(euler.source, crate::TelemetryMessageKind::Attitude);
    assert!((euler.value.roll_deg - f64::from(0.1f32.to_degrees())).abs() < 0.001);
    assert!((euler.value.pitch_deg - f64::from((-0.2f32).to_degrees())).abs() < 0.001);
    assert!((euler.value.yaw_deg - f64::from(1.3f32.to_degrees())).abs() < 0.001);

    let nav_output =
        dialect::MavMessage::NAV_CONTROLLER_OUTPUT(dialect::NAV_CONTROLLER_OUTPUT_DATA {
            wp_dist: 33,
            nav_bearing: 210,
            target_bearing: 205,
            xtrack_error: 4.5,
            ..dialect::NAV_CONTROLLER_OUTPUT_DATA::default()
        });
    update_state(&default_header(), &nav_output, &writers, &target);

    let waypoint = telemetry.navigation().waypoint().latest().unwrap();
    assert_eq!(
        waypoint.source,
        crate::TelemetryMessageKind::NavControllerOutput
    );
    assert_eq!(waypoint.value.distance_m, 33.0);
    assert_eq!(waypoint.value.bearing_deg, 210.0);

    let guidance = telemetry.navigation().guidance().latest().unwrap();
    assert_eq!(
        guidance.source,
        crate::TelemetryMessageKind::NavControllerOutput
    );
    assert_eq!(guidance.value.bearing_deg, 205.0);
    assert!((guidance.value.cross_track_error_m - 4.5).abs() < 0.001);

    let terrain_report = dialect::MavMessage::TERRAIN_REPORT(dialect::TERRAIN_REPORT_DATA {
        terrain_height: 120.5,
        current_height: 32.0,
        ..dialect::TERRAIN_REPORT_DATA::default()
    });
    update_state(&default_header(), &terrain_report, &writers, &target);

    let terrain = telemetry.terrain().clearance().latest().unwrap();
    assert_eq!(terrain.source, crate::TelemetryMessageKind::TerrainReport);
    assert!((terrain.value.terrain_height_m - 120.5).abs() < 0.001);
    assert!((terrain.value.height_above_terrain_m - 32.0).abs() < 0.001);

    let mut cell_mv = [u16::MAX; 10];
    cell_mv[0] = 4200;
    cell_mv[1] = 4150;
    cell_mv[2] = 4100;
    let primary_battery = dialect::MavMessage::BATTERY_STATUS(dialect::BATTERY_STATUS_DATA {
        id: 0,
        voltages: cell_mv,
        ..dialect::BATTERY_STATUS_DATA::default()
    });
    update_state(&default_header(), &primary_battery, &writers, &target);

    let cells = telemetry.battery().cells().latest().unwrap();
    assert_eq!(cells.source, crate::TelemetryMessageKind::BatteryStatus);
    assert_eq!(cells.value.voltages_v, vec![4.2, 4.15, 4.1]);
}

#[test]
fn home_and_origin_messages_update_grouped_geo_observations() {
    let (writers, channels) = create_channels();
    let telemetry = TelemetryHandle::new(&channels.telemetry_handles);
    let target: Option<VehicleTarget> = None;

    let home = dialect::MavMessage::HOME_POSITION(dialect::HOME_POSITION_DATA {
        latitude: 473_977_420,
        longitude: 85_455_940,
        altitude: 510_000,
        time_usec: 1_700_000_000_000_000,
        ..dialect::HOME_POSITION_DATA::default()
    });
    update_state(&default_header(), &home, &writers, &target);

    let home_sample = telemetry.home().latest().unwrap();
    assert_eq!(
        home_sample.source,
        crate::TelemetryMessageKind::HomePosition
    );
    assert!((home_sample.value.latitude_deg - 47.397742).abs() < 0.0001);
    assert!((home_sample.value.longitude_deg - 8.545594).abs() < 0.0001);
    assert!((home_sample.value.altitude_msl_m - 510.0).abs() < 0.001);

    let origin = dialect::MavMessage::GPS_GLOBAL_ORIGIN(dialect::GPS_GLOBAL_ORIGIN_DATA {
        latitude: 473_981_230,
        longitude: 85_463_210,
        altitude: 505_500,
        time_usec: 1_700_000_010_000_000,
    });
    update_state(&default_header(), &origin, &writers, &target);

    let origin_sample = telemetry.origin().latest().unwrap();
    assert_eq!(
        origin_sample.source,
        crate::TelemetryMessageKind::GpsGlobalOrigin
    );
    assert!((origin_sample.value.latitude_deg - 47.398123).abs() < 0.0001);
    assert!((origin_sample.value.longitude_deg - 8.546321).abs() < 0.0001);
    assert!((origin_sample.value.altitude_msl_m - 505.5).abs() < 0.001);
}

#[test]
fn message_handles_receive_state_update_publications() {
    let (writers, channels) = create_channels();
    let telemetry = TelemetryHandle::new(&channels.telemetry_handles);
    let target: Option<VehicleTarget> = None;

    let vfr_hud = dialect::MavMessage::VFR_HUD(dialect::VFR_HUD_DATA {
        groundspeed: 10.0,
        ..dialect::VFR_HUD_DATA::default()
    });
    update_state(&default_header(), &vfr_hud, &writers, &target);
    assert_eq!(
        telemetry
            .messages()
            .vfr_hud()
            .latest()
            .unwrap()
            .value
            .groundspeed,
        10.0
    );

    let local_position =
        dialect::MavMessage::LOCAL_POSITION_NED(dialect::LOCAL_POSITION_NED_DATA {
            time_boot_ms: 250,
            x: 1.0,
            y: 2.0,
            z: -3.0,
            ..dialect::LOCAL_POSITION_NED_DATA::default()
        });
    update_state(&default_header(), &local_position, &writers, &target);
    let local_sample = telemetry.messages().local_position_ned().latest().unwrap();
    assert_eq!(local_sample.value.x, 1.0);
    assert_eq!(local_sample.value.y, 2.0);
    assert_eq!(local_sample.value.z, -3.0);

    let battery_status = dialect::MavMessage::BATTERY_STATUS(dialect::BATTERY_STATUS_DATA {
        id: 2,
        battery_remaining: 61,
        ..dialect::BATTERY_STATUS_DATA::default()
    });
    update_state(&default_header(), &battery_status, &writers, &target);
    assert_eq!(
        telemetry
            .messages()
            .battery_status(2)
            .latest()
            .unwrap()
            .value
            .battery_remaining,
        61
    );

    let servo_output = dialect::MavMessage::SERVO_OUTPUT_RAW(dialect::SERVO_OUTPUT_RAW_DATA {
        port: 1,
        servo1_raw: 1100,
        ..dialect::SERVO_OUTPUT_RAW_DATA::default()
    });
    update_state(&default_header(), &servo_output, &writers, &target);
    assert_eq!(
        telemetry
            .messages()
            .servo_output_raw(1)
            .latest()
            .unwrap()
            .value
            .servo1_raw,
        1100
    );

    let status_text = dialect::MavMessage::STATUSTEXT(dialect::STATUSTEXT_DATA {
        text: "PreArm: Check fence".into(),
        ..dialect::STATUSTEXT_DATA::default()
    });
    update_state(&default_header(), &status_text, &writers, &target);
    assert_eq!(
        telemetry
            .messages()
            .status_text()
            .latest()
            .unwrap()
            .value
            .text,
        "PreArm: Check fence"
    );
}

#[tokio::test]
async fn statustext_reassembly_combines_chunks_by_source_and_id() {
    let (writers, channels) = create_channels();
    let telemetry = TelemetryHandle::new(&channels.telemetry_handles);
    let target: Option<VehicleTarget> = None;
    let mut subscription = telemetry.messages().status_text().subscribe();
    let header = MavHeader {
        system_id: 42,
        component_id: 99,
        sequence: 7,
    };

    update_state(
        &header,
        &dialect::MavMessage::STATUSTEXT(dialect::STATUSTEXT_DATA {
            severity: dialect::MavSeverity::MAV_SEVERITY_WARNING,
            text: exact_50_char_text("01234567890123456789012345678901234567890123456789"),
            id: 5,
            chunk_seq: 0,
        }),
        &writers,
        &target,
    );

    assert!(
        tokio::time::timeout(Duration::from_millis(25), subscription.recv())
            .await
            .is_err()
    );

    update_state(
        &header,
        &dialect::MavMessage::STATUSTEXT(dialect::STATUSTEXT_DATA {
            severity: dialect::MavSeverity::MAV_SEVERITY_WARNING,
            text: "abcdefghij".into(),
            id: 5,
            chunk_seq: 1,
        }),
        &writers,
        &target,
    );

    let sample = tokio::time::timeout(Duration::from_millis(250), subscription.recv())
        .await
        .expect("assembled status text should be emitted")
        .expect("assembled status text sample should exist");
    assert_eq!(
        sample.value.text,
        "01234567890123456789012345678901234567890123456789abcdefghij"
    );
    assert_eq!(sample.value.id, 5);
}

#[tokio::test]
async fn statustext_flushes_incomplete_chunks_after_timeout() {
    let (writers, channels) = create_channels();
    let telemetry = TelemetryHandle::new(&channels.telemetry_handles);
    let target: Option<VehicleTarget> = None;
    let mut subscription = telemetry.messages().status_text().subscribe();
    let header = MavHeader {
        system_id: 21,
        component_id: 2,
        sequence: 3,
    };

    update_state(
        &header,
        &dialect::MavMessage::STATUSTEXT(dialect::STATUSTEXT_DATA {
            severity: dialect::MavSeverity::MAV_SEVERITY_ERROR,
            text: exact_50_char_text("98765432109876543210987654321098765432109876543210"),
            id: 9,
            chunk_seq: 0,
        }),
        &writers,
        &target,
    );

    tokio::task::yield_now().await;
    assert!(
        tokio::time::timeout(Duration::from_millis(1), subscription.recv())
            .await
            .is_err()
    );

    tokio::time::sleep(Duration::from_millis(2_100)).await;

    let sample = tokio::time::timeout(Duration::from_millis(50), subscription.recv())
        .await
        .expect("timed flush should emit pending status text")
        .expect("flushed status text sample should exist");
    assert_eq!(
        sample.value.text,
        "98765432109876543210987654321098765432109876543210"
    );
    assert_eq!(sample.value.id, 9);
}

#[test]
fn sensor_health_mapping_sets_all_ten_summary_fields() {
    let (writers, channels) = create_channels();
    let telemetry = TelemetryHandle::new(&channels.telemetry_handles);
    let target: Option<VehicleTarget> = None;

    let present = dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_GYRO
        | dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_ACCEL
        | dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_MAG
        | dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE
        | dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_GPS
        | dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE
        | dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_RC_RECEIVER
        | dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_BATTERY
        | dialect::MavSysStatusSensor::MAV_SYS_STATUS_TERRAIN
        | dialect::MavSysStatusSensor::MAV_SYS_STATUS_GEOFENCE;
    let enabled = dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_GYRO
        | dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_ACCEL
        | dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_MAG
        | dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE
        | dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_GPS
        | dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_BATTERY
        | dialect::MavSysStatusSensor::MAV_SYS_STATUS_GEOFENCE;
    let healthy = dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_GYRO
        | dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_ACCEL
        | dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE
        | dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_GPS
        | dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_BATTERY
        | dialect::MavSysStatusSensor::MAV_SYS_STATUS_GEOFENCE;

    update_state(
        &default_header(),
        &dialect::MavMessage::SYS_STATUS(dialect::SYS_STATUS_DATA {
            onboard_control_sensors_present: present,
            onboard_control_sensors_enabled: enabled,
            onboard_control_sensors_health: healthy,
            ..dialect::SYS_STATUS_DATA::default()
        }),
        &writers,
        &target,
    );

    let summary = telemetry.sensor_health().latest().unwrap().value;
    assert_eq!(summary.gyro, SensorHealthState::Healthy);
    assert_eq!(summary.accel, SensorHealthState::Healthy);
    assert_eq!(summary.mag, SensorHealthState::Unhealthy);
    assert_eq!(summary.baro, SensorHealthState::Healthy);
    assert_eq!(summary.gps, SensorHealthState::Healthy);
    assert_eq!(summary.airspeed, SensorHealthState::Disabled);
    assert_eq!(summary.rc_receiver, SensorHealthState::Disabled);
    assert_eq!(summary.battery, SensorHealthState::Healthy);
    assert_eq!(summary.terrain, SensorHealthState::Disabled);
    assert_eq!(summary.geofence, SensorHealthState::Healthy);
}

#[test]
fn mag_cal_progress_message_updates_progress_channel() {
    let (writers, channels) = create_channels();
    let target: Option<VehicleTarget> = None;

    assert_eq!(*channels.mag_cal_progress.borrow(), None);

    update_state(
        &default_header(),
        &dialect::MavMessage::MAG_CAL_PROGRESS(dialect::MAG_CAL_PROGRESS_DATA {
            compass_id: 2,
            completion_pct: 73,
            cal_status: dialect::MagCalStatus::MAG_CAL_RUNNING_STEP_ONE,
            attempt: 4,
            ..dialect::MAG_CAL_PROGRESS_DATA::default()
        }),
        &writers,
        &target,
    );

    assert_eq!(
        *channels.mag_cal_progress.borrow(),
        Some(crate::state::MagCalProgress {
            compass_id: 2,
            completion_pct: 73,
            status: crate::state::MagCalStatus::RunningStepOne,
            attempt: 4,
        })
    );
}
