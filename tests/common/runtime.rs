use crate::common::backend::{
    disconnect, setup_runtime_backend_vehicle, wait_for_runtime_condition,
};
use crate::common::commands::arm_with_retries;
use crate::common::target::{GuidedMovementKind, TestTarget};
use mavkit::ardupilot::{GuidedSpecific, RelativeClimbTarget};
use mavkit::mission::commands::{NavCommand, NavLand, NavTakeoff, NavWaypoint};
use mavkit::{GeoPoint3d, GeoPoint3dRelHome, GlobalPosition, MissionItem, MissionPlan, Vehicle};
use std::time::Duration;

fn short_runtime_mission(home_lat: f64, home_lon: f64, takeoff_first: bool) -> MissionPlan {
    let mut items = Vec::new();
    if takeoff_first {
        items.push(MissionItem::from(NavTakeoff::from_point(
            GeoPoint3d::rel_home(home_lat, home_lon, 3.0),
        )));
    }

    items.extend([
        MissionItem::from(NavWaypoint::from_point(GeoPoint3d::rel_home(
            home_lat + 0.000_04,
            home_lon,
            3.0,
        ))),
        MissionItem::from(NavWaypoint::from_point(GeoPoint3d::rel_home(
            home_lat + 0.000_08,
            home_lon,
            3.0,
        ))),
        MissionItem::from(NavCommand::ReturnToLaunch),
        MissionItem::from(NavLand::from_point(GeoPoint3d::rel_home(
            home_lat, home_lon, 0.0,
        ))),
    ]);

    MissionPlan { items }
}

fn latest_position(vehicle: &Vehicle) -> Option<GlobalPosition> {
    vehicle
        .telemetry()
        .position()
        .global()
        .latest()
        .map(|sample| sample.value)
}

fn latest_armed(vehicle: &Vehicle) -> Option<bool> {
    vehicle
        .telemetry()
        .armed()
        .latest()
        .map(|sample| sample.value)
}

fn position_near(
    position: &GlobalPosition,
    latitude_deg: f64,
    longitude_deg: f64,
    relative_alt_m: f64,
    position_tolerance_deg: f64,
    altitude_tolerance_m: f64,
) -> bool {
    (position.latitude_deg - latitude_deg).abs() <= position_tolerance_deg
        && (position.longitude_deg - longitude_deg).abs() <= position_tolerance_deg
        && (position.relative_alt_m - relative_alt_m).abs() <= altitude_tolerance_m
}

pub async fn guided_movement_reaches_target_case(target: TestTarget) {
    let expectations = target.runtime_expectations();
    let Some(expectation) = expectations.guided_movement else {
        eprintln!("Skipping guided movement runtime case: {target:?} does not advertise support");
        return;
    };

    let backend = setup_runtime_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;
    let result: Result<(), String> = async {
        let start = vehicle
            .telemetry()
            .position()
            .global()
            .wait_timeout(Duration::from_secs(10))
            .await
            .map_err(|err| err.to_string())?
            .value;
        let target_lat = start.latitude_deg + expectation.north_offset_deg;
        let target_lon = start.longitude_deg;

        vehicle.force_arm().await.map_err(|err| err.to_string())?;
        let guided = vehicle
            .ardupilot()
            .guided()
            .await
            .map_err(|err| err.to_string())?;

        wait_for_runtime_condition(&backend, Duration::from_secs(10), "guided mode", || {
            Ok(vehicle
                .available_modes()
                .current()
                .latest()
                .is_some_and(|mode| mode.custom_mode == target.guided_mode().0))
        })
        .await?;

        match (expectation.kind, guided.specific()) {
            (GuidedMovementKind::CopterGoto, GuidedSpecific::Copter(copter)) => {
                copter
                    .takeoff(RelativeClimbTarget {
                        relative_climb_m: expectation.relative_alt_m as f32,
                    })
                    .await
                    .map_err(|err| err.to_string())?;
                wait_for_runtime_condition(
                    &backend,
                    expectation.timeout,
                    "guided takeoff altitude",
                    || {
                        Ok(latest_position(vehicle).is_some_and(|position| {
                            position.relative_alt_m
                                >= expectation.relative_alt_m - expectation.altitude_tolerance_m
                        }))
                    },
                )
                .await?;
                copter
                    .goto(GeoPoint3dRelHome {
                        latitude_deg: target_lat,
                        longitude_deg: target_lon,
                        relative_alt_m: expectation.relative_alt_m,
                    })
                    .await
                    .map_err(|err| err.to_string())?;
            }
            (GuidedMovementKind::PlaneReposition, GuidedSpecific::Plane(plane)) => {
                plane
                    .reposition_rel_home(GeoPoint3dRelHome {
                        latitude_deg: target_lat,
                        longitude_deg: target_lon,
                        relative_alt_m: expectation.relative_alt_m,
                    })
                    .await
                    .map_err(|err| err.to_string())?;
            }
            (expected, actual) => {
                return Err(format!(
                    "target advertised guided movement {expected:?}, got guided handle {actual:?}"
                ));
            }
        }

        wait_for_runtime_condition(
            &backend,
            expectation.timeout,
            "guided movement target",
            || {
                Ok(latest_position(vehicle).is_some_and(|position| {
                    position_near(
                        &position,
                        target_lat,
                        target_lon,
                        expectation.relative_alt_m,
                        expectation.position_tolerance_deg,
                        expectation.altitude_tolerance_m,
                    )
                }))
            },
        )
        .await?;

        let final_position = latest_position(vehicle)
            .ok_or_else(|| String::from("position unavailable after guided movement"))?;
        if final_position.latitude_deg <= start.latitude_deg {
            return Err(format!(
                "expected guided movement north from {}, got {}",
                start.latitude_deg, final_position.latitude_deg
            ));
        }

        guided.close().await.map_err(|err| err.to_string())?;
        Ok(())
    }
    .await;

    disconnect(backend).await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

pub async fn auto_mission_progresses_through_rtl_and_land_case(target: TestTarget) {
    let expectations = target.runtime_expectations();
    let Some(expectation) = expectations.auto_mission else {
        eprintln!("Skipping AUTO mission runtime case: {target:?} does not advertise support");
        return;
    };

    let backend = setup_runtime_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;
    let result: Result<(), String> = async {
        let home = vehicle
            .telemetry()
            .home()
            .wait_timeout(Duration::from_secs(20))
            .await
            .map_err(|err| err.to_string())?
            .value;
        let start = vehicle
            .telemetry()
            .position()
            .global()
            .wait_timeout(Duration::from_secs(10))
            .await
            .map_err(|err| err.to_string())?
            .value;
        let plan = short_runtime_mission(
            home.latitude_deg,
            home.longitude_deg,
            expectation.takeoff_first,
        );
        let last_mission_index = plan.items.len().saturating_sub(1) as u16;

        vehicle
            .mission()
            .clear()
            .map_err(|err| err.to_string())?
            .wait()
            .await
            .map_err(|err| err.to_string())?;
        vehicle
            .mission()
            .upload(plan)
            .map_err(|err| err.to_string())?
            .wait()
            .await
            .map_err(|err| err.to_string())?;
        arm_with_retries(vehicle, true, Duration::from_secs(30)).await?;
        vehicle
            .set_mode(target.auto_mode())
            .await
            .map_err(|err| err.to_string())?;

        wait_for_runtime_condition(&backend, Duration::from_secs(10), "AUTO mode", || {
            Ok(vehicle
                .available_modes()
                .current()
                .latest()
                .is_some_and(|mode| mode.custom_mode == target.auto_mode()))
        })
        .await?;

        wait_for_runtime_condition(
            &backend,
            expectation.progress_timeout,
            "AUTO mission progress",
            || {
                let progressed = vehicle.mission().latest().is_some_and(|state| {
                    state.current_index >= Some(expectation.progress_min_index)
                });
                let moved_north = latest_position(vehicle).is_some_and(|position| {
                    position.latitude_deg > start.latitude_deg + expectation.progress_min_north_deg
                });
                Ok(progressed && moved_north)
            },
        )
        .await?;

        wait_for_runtime_condition(
            &backend,
            expectation.completion_timeout,
            "RTL and landing completion",
            || {
                let completed_or_landing = vehicle
                    .mission()
                    .latest()
                    .is_some_and(|state| state.current_index >= Some(last_mission_index));
                let landed = latest_position(vehicle).is_some_and(|position| {
                    position_near(
                        &position,
                        home.latitude_deg,
                        home.longitude_deg,
                        0.0,
                        expectation.landing_position_tolerance_deg,
                        expectation.landing_altitude_tolerance_m,
                    )
                });
                Ok(completed_or_landing && landed && latest_armed(vehicle) == Some(false))
            },
        )
        .await?;

        Ok(())
    }
    .await;

    disconnect(backend).await;
    if let Err(err) = result {
        panic!("{err}");
    }
}
