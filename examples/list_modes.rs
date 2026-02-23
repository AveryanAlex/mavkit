use mavkit::Vehicle;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let bind_addr =
        std::env::var("MAVKIT_EXAMPLE_UDP_BIND").unwrap_or_else(|_| "0.0.0.0:14550".to_string());

    let vehicle = Vehicle::connect_udp(&bind_addr).await?;

    let state = vehicle.state().borrow().clone();
    println!(
        "current mode: {} (id={})\n",
        state.mode_name, state.custom_mode
    );

    let modes = vehicle.available_modes();
    if modes.is_empty() {
        println!("no mode list available for autopilot {:?}", state.autopilot);
    } else {
        println!("available flight modes:");
        for mode in &modes {
            let marker = if mode.custom_mode == state.custom_mode {
                " <-- active"
            } else {
                ""
            };
            println!("  {:>3}  {}{}", mode.custom_mode, mode.name, marker);
        }
    }

    vehicle.disconnect().await?;
    Ok(())
}
