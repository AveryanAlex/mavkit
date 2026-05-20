use crate::common::backend::{disconnect, setup_backend_vehicle};
use crate::common::target::{SupportExpectation, TestTarget};
use mavkit::{ObservationHandle, SupportState, Vehicle};
use std::time::Duration;

pub fn check_support_expectation(
    name: &str,
    state: SupportState,
    expectation: SupportExpectation,
) -> Result<(), String> {
    match expectation {
        SupportExpectation::AnyResolved => Ok(()),
        SupportExpectation::Known if state != SupportState::Unknown => Ok(()),
        SupportExpectation::Known => Err(format!("{name} support should resolve to a known state")),
        SupportExpectation::Exact(expected) if state == expected => Ok(()),
        SupportExpectation::Exact(expected) => Err(format!(
            "expected {name} support to be {expected:?}, got {state:?}"
        )),
    }
}

async fn support_expectation_case(
    target: TestTarget,
    name: &str,
    handle: impl FnOnce(&Vehicle) -> ObservationHandle<SupportState>,
    expectation: SupportExpectation,
) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;
    let result: Result<(), String> = async {
        let state = handle(vehicle)
            .wait_timeout(Duration::from_secs(10))
            .await
            .map_err(|e| e.to_string())?;

        check_support_expectation(name, state, expectation)
    }
    .await;

    disconnect(backend).await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

pub async fn support_command_int_case(target: TestTarget) {
    support_expectation_case(
        target,
        "command_int",
        |vehicle| vehicle.support().command_int(),
        target.support_expectations().command_int,
    )
    .await;
}

pub async fn support_mission_fence_case(target: TestTarget) {
    support_expectation_case(
        target,
        "mission_fence",
        |vehicle| vehicle.support().mission_fence(),
        target.support_expectations().mission_fence,
    )
    .await;
}

pub async fn support_mission_rally_case(target: TestTarget) {
    support_expectation_case(
        target,
        "mission_rally",
        |vehicle| vehicle.support().mission_rally(),
        target.support_expectations().mission_rally,
    )
    .await;
}

pub async fn support_terrain_case(target: TestTarget) {
    support_expectation_case(
        target,
        "terrain",
        |vehicle| vehicle.support().terrain(),
        target.support_expectations().terrain,
    )
    .await;
}
