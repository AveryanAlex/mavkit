use mavkit::Vehicle;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let bind_addr =
        std::env::var("MAVKIT_EXAMPLE_UDP_BIND").unwrap_or_else(|_| "0.0.0.0:14550".to_string());

    let vehicle = Vehicle::connect_udp(&bind_addr).await?;
    let identity = vehicle.identity();
    let current_mode = vehicle.current_mode().wait().await?;
    let armed = vehicle.telemetry().armed().wait().await?.value;
    let position = vehicle.telemetry().position().global().wait().await?.value;

    println!(
        "connected: sys={} comp={} autopilot={:?} vehicle_type={:?} mode={} armed={}",
        identity.system_id,
        identity.component_id,
        identity.autopilot,
        identity.vehicle_type,
        current_mode.name,
        armed,
    );
    println!(
        "telemetry: lat={:.7} lon={:.7} alt_msl={:.1} rel_alt={:.1}",
        position.latitude_deg,
        position.longitude_deg,
        position.altitude_msl_m,
        position.relative_alt_m
    );

    vehicle.disconnect().await?;
    Ok(())
}
