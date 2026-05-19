use mavkit::mission::commands::NavWaypoint;
use mavkit::{GeoPoint3d, MissionItem, MissionPlan};

pub fn waypoint(lat: f64, lon: f64, alt: f32) -> MissionItem {
    NavWaypoint::from_point(GeoPoint3d::rel_home(lat, lon, f64::from(alt))).into()
}

pub fn sample_plan_mission(item_count: usize) -> MissionPlan {
    let base_lat = 47.397_742;
    let base_lon = 8.545_594;
    let mut items = Vec::with_capacity(item_count);
    for i in 0..item_count {
        let lat = base_lat + (i as f64) * 0.000_2;
        let lon = base_lon + (i as f64) * 0.000_2;
        let alt = 25.0 + (i as f32);
        items.push(waypoint(lat, lon, alt));
    }

    MissionPlan { items }
}
