use crate::{TestTarget, disconnect, setup_backend_vehicle};
use mavkit::{AutopilotType, LinkState};
use std::time::Duration;

pub async fn vehicle_identity_is_ardupilot_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;
    let result: Result<(), String> = async {
        let identity = vehicle.identity();

        if identity.autopilot != AutopilotType::ArduPilotMega {
            return Err(format!(
                "expected ArduPilotMega autopilot, got {:?}",
                identity.autopilot
            ));
        }

        if identity.system_id == 0 {
            return Err(String::from("system_id should not be 0"));
        }

        Ok(())
    }
    .await;

    disconnect(backend).await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

pub async fn firmware_info_populates_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;
    let result: Result<(), String> = async {
        let firmware = vehicle
            .info()
            .firmware()
            .wait_timeout(Duration::from_secs(10))
            .await
            .map_err(|e| e.to_string())?;

        if firmware.version.is_none() && firmware.git_hash.is_none() {
            return Err(String::from(
                "firmware info has neither version nor git_hash",
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

pub async fn link_state_connected_after_setup_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;
    let result: Result<(), String> = async {
        let state = vehicle
            .link()
            .state()
            .latest()
            .ok_or_else(|| String::from("link state not yet available"))?;

        if !matches!(state, LinkState::Connected) {
            return Err(format!("expected Connected link state, got {state:?}"));
        }

        Ok(())
    }
    .await;

    disconnect(backend).await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

pub async fn persistent_identity_resolves_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;
    let result: Result<(), String> = async {
        let _identity = vehicle
            .info()
            .persistent_identity()
            .wait_timeout(Duration::from_secs(10))
            .await
            .map_err(|e| e.to_string())?;

        Ok(())
    }
    .await;

    disconnect(backend).await;
    if let Err(err) = result {
        panic!("{err}");
    }
}
