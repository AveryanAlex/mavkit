use mavkit::{
    CompareTolerance, GeoPoint3d, MissionItem, MissionPlan, NavWaypoint, Vehicle,
    normalize_for_compare, plans_equivalent,
};

fn waypoint(lat: f64, lon: f64, alt: f32) -> MissionItem {
    NavWaypoint::from_point(GeoPoint3d::rel_home(lat, lon, f64::from(alt))).into()
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let bind_addr =
        std::env::var("MAVKIT_EXAMPLE_UDP_BIND").unwrap_or_else(|_| "0.0.0.0:14550".to_string());

    let vehicle = Vehicle::connect_udp(&bind_addr).await?;
    let mission = vehicle.mission();

    let plan = MissionPlan::new(vec![
        waypoint(47.397742, 8.545594, 25.0),
        waypoint(47.398100, 8.546100, 30.0),
    ]);

    mission.upload(plan.clone())?.wait().await?;
    let downloaded = mission.download()?.wait().await?;

    let expected = normalize_for_compare(&plan);
    let got = normalize_for_compare(&downloaded);

    println!(
        "roundtrip equivalent: {}",
        plans_equivalent(&expected, &got, CompareTolerance::default())
    );

    vehicle.disconnect().await?;
    Ok(())
}
