use mavkit::Vehicle;
use mavkit::mission::MissionState;
use std::time::Duration;

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
