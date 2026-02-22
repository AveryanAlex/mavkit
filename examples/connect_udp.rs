use mavkit::Vehicle;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let bind_addr =
        std::env::var("MAVKIT_EXAMPLE_UDP_BIND").unwrap_or_else(|_| "0.0.0.0:14550".to_string());

    let vehicle = Vehicle::connect_udp(&bind_addr).await?;

    let mut state_rx = vehicle.state();
    state_rx.changed().await?;
    let state = state_rx.borrow().clone();

    println!(
        "connected: mode={} armed={} autopilot={:?} vehicle_type={:?}",
        state.mode_name, state.armed, state.autopilot, state.vehicle_type
    );

    let mut telemetry_rx = vehicle.telemetry();
    telemetry_rx.changed().await?;
    let telemetry = telemetry_rx.borrow().clone();
    println!(
        "telemetry: lat={:?} lon={:?} alt={:?}",
        telemetry.latitude_deg, telemetry.longitude_deg, telemetry.altitude_m
    );

    vehicle.disconnect().await?;
    Ok(())
}
