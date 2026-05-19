use crate::{TestTarget, disconnect, setup_backend_vehicle};
use mavkit::SensorHealthState;
use std::time::Duration;

pub async fn basic_telemetry_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;

    let result: Result<(), String> = async {
        let position = vehicle
            .telemetry()
            .position()
            .global()
            .wait_timeout(Duration::from_secs(10))
            .await
            .map_err(|err| err.to_string())?;
        if !position.value.latitude_deg.is_finite() || !position.value.longitude_deg.is_finite() {
            return Err(String::from(
                "telemetry position contained non-finite coordinates",
            ));
        }

        let attitude = vehicle
            .telemetry()
            .attitude()
            .euler()
            .wait_timeout(Duration::from_secs(10))
            .await
            .map_err(|err| err.to_string())?;
        if !attitude.value.roll_deg.is_finite()
            || !attitude.value.pitch_deg.is_finite()
            || !attitude.value.yaw_deg.is_finite()
        {
            return Err(String::from("attitude contained non-finite values"));
        }

        let battery = vehicle
            .telemetry()
            .battery()
            .voltage_v()
            .wait_timeout(Duration::from_secs(10))
            .await
            .map_err(|err| err.to_string())?;
        if !(5.0..=60.0).contains(&battery.value) {
            return Err(format!("battery voltage out of range: {} V", battery.value));
        }

        let gps = vehicle
            .telemetry()
            .gps()
            .quality()
            .wait_timeout(Duration::from_secs(10))
            .await
            .map_err(|err| err.to_string())?;
        if gps.value.satellites.unwrap_or(0) == 0 {
            return Err(String::from("GPS reports 0 satellites"));
        }

        let groundspeed = vehicle
            .telemetry()
            .position()
            .groundspeed_mps()
            .wait_timeout(Duration::from_secs(10))
            .await
            .map_err(|err| err.to_string())?;
        if !groundspeed.value.is_finite() {
            return Err(String::from("groundspeed is not finite"));
        }

        let heading = vehicle
            .telemetry()
            .position()
            .heading_deg()
            .wait_timeout(Duration::from_secs(10))
            .await
            .map_err(|err| err.to_string())?;
        if !(0.0..=360.0).contains(&heading.value) {
            return Err(format!("heading out of range: {}", heading.value));
        }

        let sensor_health = vehicle
            .telemetry()
            .sensor_health()
            .wait_timeout(Duration::from_secs(10))
            .await
            .map_err(|err| err.to_string())?;
        if sensor_health.value.gyro != SensorHealthState::Healthy {
            return Err(format!("gyro not healthy: {:?}", sensor_health.value.gyro));
        }
        if sensor_health.value.accel != SensorHealthState::Healthy {
            return Err(format!(
                "accel not healthy: {:?}",
                sensor_health.value.accel
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

pub async fn telemetry_attitude_euler_available_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;
    let result: Result<(), String> = async {
        let sample = vehicle
            .telemetry()
            .attitude()
            .euler()
            .wait_timeout(Duration::from_secs(10))
            .await
            .map_err(|e| e.to_string())?;

        if !sample.value.roll_deg.is_finite()
            || !sample.value.pitch_deg.is_finite()
            || !sample.value.yaw_deg.is_finite()
        {
            return Err(String::from("attitude contained non-finite values"));
        }

        if sample.value.roll_deg.abs() > 17.0 || sample.value.pitch_deg.abs() > 17.0 {
            return Err(format!(
                "attitude roll/pitch unexpectedly large for stationary vehicle: roll={}, pitch={}",
                sample.value.roll_deg, sample.value.pitch_deg
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

pub async fn telemetry_battery_voltage_available_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;
    let result: Result<(), String> = async {
        let sample = vehicle
            .telemetry()
            .battery()
            .voltage_v()
            .wait_timeout(Duration::from_secs(10))
            .await
            .map_err(|e| e.to_string())?;

        if sample.value < 5.0 || sample.value > 60.0 {
            return Err(format!("battery voltage out of range: {} V", sample.value));
        }

        Ok(())
    }
    .await;

    disconnect(backend).await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

pub async fn telemetry_gps_quality_available_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;
    let result: Result<(), String> = async {
        let sample = vehicle
            .telemetry()
            .gps()
            .quality()
            .wait_timeout(Duration::from_secs(10))
            .await
            .map_err(|e| e.to_string())?;

        if sample.value.satellites.unwrap_or(0) == 0 {
            return Err(String::from("GPS reports 0 satellites in SITL"));
        }

        Ok(())
    }
    .await;

    disconnect(backend).await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

pub async fn telemetry_groundspeed_available_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;
    let result: Result<(), String> = async {
        let sample = vehicle
            .telemetry()
            .position()
            .groundspeed_mps()
            .wait_timeout(Duration::from_secs(10))
            .await
            .map_err(|e| e.to_string())?;

        if !sample.value.is_finite() {
            return Err(String::from("groundspeed is not finite"));
        }
        if sample.value > 1.0 {
            return Err(format!(
                "groundspeed unexpectedly high for stationary vehicle: {} m/s",
                sample.value
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

pub async fn telemetry_heading_available_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;
    let result: Result<(), String> = async {
        let sample = vehicle
            .telemetry()
            .position()
            .heading_deg()
            .wait_timeout(Duration::from_secs(10))
            .await
            .map_err(|e| e.to_string())?;

        if sample.value < 0.0 || sample.value > 360.0 {
            return Err(format!("heading out of range: {}", sample.value));
        }

        Ok(())
    }
    .await;

    disconnect(backend).await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

pub async fn telemetry_sensor_health_available_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;
    let result: Result<(), String> = async {
        let sample = vehicle
            .telemetry()
            .sensor_health()
            .wait_timeout(Duration::from_secs(10))
            .await
            .map_err(|e| e.to_string())?;

        if sample.value.gyro != SensorHealthState::Healthy {
            return Err(format!("gyro not healthy: {:?}", sample.value.gyro));
        }
        if sample.value.accel != SensorHealthState::Healthy {
            return Err(format!("accel not healthy: {:?}", sample.value.accel));
        }

        Ok(())
    }
    .await;

    disconnect(backend).await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

pub async fn telemetry_position_near_home_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;
    let result: Result<(), String> = async {
        let sample = vehicle
            .telemetry()
            .position()
            .global()
            .wait_timeout(Duration::from_secs(10))
            .await
            .map_err(|e| e.to_string())?;

        let lat_diff = (sample.value.latitude_deg - 42.3898).abs();
        let lon_diff = (sample.value.longitude_deg - (-71.1476)).abs();
        if lat_diff > 0.01 || lon_diff > 0.01 {
            return Err(format!(
                "position too far from SITL home: lat={}, lon={} (expected ~42.39, ~-71.15)",
                sample.value.latitude_deg, sample.value.longitude_deg
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

pub async fn telemetry_position_observation_available_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;
    let result: Result<(), String> = async {
        let sample = vehicle
            .telemetry()
            .position()
            .global()
            .wait_timeout(Duration::from_secs(10))
            .await
            .map_err(|e| e.to_string())?;

        if !sample.value.latitude_deg.is_finite() || !sample.value.longitude_deg.is_finite() {
            return Err(String::from(
                "telemetry position contained non-finite coordinates",
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
