use crate::{TestTarget, disconnect, setup_backend_vehicle};
use mavkit::SupportState;
use std::time::Duration;

pub async fn sitl_support_command_int_resolves() {
    let backend = setup_backend_vehicle(TestTarget::SITL_COPTER).await;
    let vehicle = &backend.vehicle;
    let result: Result<(), String> = async {
        let state = vehicle
            .support()
            .command_int()
            .wait_timeout(Duration::from_secs(10))
            .await
            .map_err(|e| e.to_string())?;

        if state == SupportState::Unknown {
            return Err(String::from("command_int support should resolve"));
        }

        Ok(())
    }
    .await;

    disconnect(backend).await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

pub async fn sitl_support_mission_fence_resolves() {
    let backend = setup_backend_vehicle(TestTarget::SITL_COPTER).await;
    let vehicle = &backend.vehicle;
    let result: Result<(), String> = async {
        let state = vehicle
            .support()
            .mission_fence()
            .wait_timeout(Duration::from_secs(10))
            .await
            .map_err(|e| e.to_string())?;

        let _ = state;

        Ok(())
    }
    .await;

    disconnect(backend).await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

pub async fn sitl_support_mission_rally_resolves() {
    let backend = setup_backend_vehicle(TestTarget::SITL_COPTER).await;
    let vehicle = &backend.vehicle;
    let result: Result<(), String> = async {
        let state = vehicle
            .support()
            .mission_rally()
            .wait_timeout(Duration::from_secs(10))
            .await
            .map_err(|e| e.to_string())?;

        if state == SupportState::Unknown {
            return Err(String::from("mission_rally support should resolve"));
        }

        Ok(())
    }
    .await;

    disconnect(backend).await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

pub async fn sitl_support_terrain_resolves() {
    let backend = setup_backend_vehicle(TestTarget::SITL_COPTER).await;
    let vehicle = &backend.vehicle;
    let result: Result<(), String> = async {
        let state = vehicle
            .support()
            .terrain()
            .wait_timeout(Duration::from_secs(10))
            .await
            .map_err(|e| e.to_string())?;

        let _ = state;

        Ok(())
    }
    .await;

    disconnect(backend).await;
    if let Err(err) = result {
        panic!("{err}");
    }
}
