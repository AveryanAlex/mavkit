use mavkit::{
    CompareTolerance, GeoPoint3d, GeoPoint3dRelHome, MissionItem, MissionPlan, MissionType,
    NavWaypoint, Vehicle, normalize_for_compare, plans_equivalent,
};

fn waypoint(lat: f64, lon: f64, alt: f32) -> MissionItem {
    MissionItem {
        command: NavWaypoint {
            position: GeoPoint3d::RelHome(GeoPoint3dRelHome {
                latitude_deg: lat,
                longitude_deg: lon,
                relative_alt_m: f64::from(alt),
            }),
            hold_time_s: 0.0,
            acceptance_radius_m: 0.0,
            pass_radius_m: 0.0,
            yaw_deg: 0.0,
        }
        .into(),
        current: false,
        autocontinue: true,
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let bind_addr =
        std::env::var("MAVKIT_EXAMPLE_UDP_BIND").unwrap_or_else(|_| "0.0.0.0:14550".to_string());

    let vehicle = Vehicle::connect_udp(&bind_addr).await?;
    let mission = vehicle.mission();

    let mut items = vec![
        waypoint(47.397742, 8.545594, 25.0),
        waypoint(47.398100, 8.546100, 30.0),
    ];
    items[0].current = true;
    let plan = MissionPlan {
        mission_type: MissionType::Mission,
        items,
    };

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
