use mavkit::Vehicle;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let addr =
        std::env::var("MAVKIT_EXAMPLE_TCP_ADDR").unwrap_or_else(|_| "127.0.0.1:5760".to_string());

    let vehicle = Vehicle::connect_tcp(&addr).await?;
    let identity = vehicle.identity();
    let current_mode = vehicle.current_mode().wait().await?;
    let armed = vehicle.telemetry().armed().wait().await?.value;

    println!(
        "connected: sys={} comp={} autopilot={:?} vehicle_type={:?} mode={} armed={}",
        identity.system_id,
        identity.component_id,
        identity.autopilot,
        identity.vehicle_type,
        current_mode.name,
        armed,
    );

    vehicle.disconnect().await?;
    Ok(())
}
