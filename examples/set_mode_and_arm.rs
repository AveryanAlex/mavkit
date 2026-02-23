use mavkit::Vehicle;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let bind_addr =
        std::env::var("MAVKIT_EXAMPLE_UDP_BIND").unwrap_or_else(|_| "0.0.0.0:14550".to_string());
    let mode = std::env::var("MAVKIT_EXAMPLE_MODE").unwrap_or_else(|_| "GUIDED".to_string());

    let vehicle = Vehicle::connect_udp(&bind_addr).await?;

    let state = vehicle.state().borrow().clone();
    println!("before: mode={} armed={}", state.mode_name, state.armed);

    println!("setting mode to {mode}...");
    vehicle.set_mode_by_name(&mode).await?;

    println!("arming...");
    vehicle.arm(false).await?;

    // Wait for state to reflect the changes
    let mut state_rx = vehicle.state();
    state_rx.changed().await?;
    let state = state_rx.borrow().clone();
    println!("after:  mode={} armed={}", state.mode_name, state.armed);

    vehicle.disconnect().await?;
    Ok(())
}
