use mavkit::Vehicle;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let bind_addr =
        std::env::var("MAVKIT_EXAMPLE_UDP_BIND").unwrap_or_else(|_| "0.0.0.0:14550".to_string());
    let mode = std::env::var("MAVKIT_EXAMPLE_MODE").unwrap_or_else(|_| "GUIDED".to_string());

    let vehicle = Vehicle::connect_udp(&bind_addr).await?;
    let current_mode = vehicle.available_modes().current();
    let armed = vehicle.telemetry().armed();

    let before_mode = current_mode.wait().await?.name;
    let before_armed = armed.wait().await?.value;
    println!("before: mode={} armed={}", before_mode, before_armed);

    println!("setting mode to {mode}...");
    vehicle.set_mode_by_name(&mode, true).await?;

    println!("arming...");
    vehicle.arm(false).await?;

    let after_mode = current_mode.wait().await?;
    let after_armed = armed.wait().await?.value;
    println!("after:  mode={} armed={}", after_mode.name, after_armed);

    vehicle.disconnect().await?;
    Ok(())
}
