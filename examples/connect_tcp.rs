use mavkit::Vehicle;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let addr =
        std::env::var("MAVKIT_EXAMPLE_TCP_ADDR").unwrap_or_else(|_| "127.0.0.1:5760".to_string());

    let vehicle = Vehicle::connect_tcp(&addr).await?;

    let state = vehicle.state().borrow().clone();
    println!(
        "connected: mode={} armed={} autopilot={:?} vehicle_type={:?}",
        state.mode_name, state.armed, state.autopilot, state.vehicle_type
    );

    vehicle.disconnect().await?;
    Ok(())
}
