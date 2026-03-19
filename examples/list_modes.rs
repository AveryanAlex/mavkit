use mavkit::Vehicle;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let bind_addr =
        std::env::var("MAVKIT_EXAMPLE_UDP_BIND").unwrap_or_else(|_| "0.0.0.0:14550".to_string());

    let vehicle = Vehicle::connect_udp(&bind_addr).await?;
    let modes = vehicle.available_modes();
    let current_handle = modes.current();
    let catalog_handle = modes.catalog();
    let identity = vehicle.identity();

    let current = current_handle.wait().await?;

    println!(
        "current mode: {} (id={})\n",
        current.name, current.custom_mode
    );

    let catalog = catalog_handle.wait().await?;

    if catalog.is_empty() {
        println!(
            "no mode list available for autopilot {:?}",
            identity.autopilot
        );
    } else {
        println!("available flight modes:");
        for mode in catalog {
            let marker = if mode.custom_mode == current.custom_mode {
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
