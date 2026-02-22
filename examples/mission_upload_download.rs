use mavkit::{
    normalize_for_compare, plans_equivalent, CompareTolerance, HomePosition, MissionFrame,
    MissionItem, MissionPlan, MissionType, Vehicle,
};

fn waypoint(seq: u16, lat: f64, lon: f64, alt: f32) -> MissionItem {
    MissionItem {
        seq,
        command: 16,
        frame: MissionFrame::GlobalRelativeAltInt,
        current: seq == 0,
        autocontinue: true,
        param1: 0.0,
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        x: (lat * 1e7) as i32,
        y: (lon * 1e7) as i32,
        z: alt,
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let bind_addr =
        std::env::var("MAVKIT_EXAMPLE_UDP_BIND").unwrap_or_else(|_| "0.0.0.0:14550".to_string());

    let vehicle = Vehicle::connect_udp(&bind_addr).await?;

    let plan = MissionPlan {
        mission_type: MissionType::Mission,
        home: Some(HomePosition {
            latitude_deg: 47.397742,
            longitude_deg: 8.545594,
            altitude_m: 0.0,
        }),
        items: vec![
            waypoint(0, 47.397742, 8.545594, 25.0),
            waypoint(1, 47.398100, 8.546100, 30.0),
        ],
    };

    vehicle.mission().upload(plan.clone()).await?;
    let downloaded = vehicle.mission().download(MissionType::Mission).await?;

    let mut expected = normalize_for_compare(&plan);
    let mut got = normalize_for_compare(&downloaded);
    expected.home = None;
    got.home = None;

    println!(
        "roundtrip equivalent: {}",
        plans_equivalent(&expected, &got, CompareTolerance::default())
    );

    vehicle.disconnect().await?;
    Ok(())
}
