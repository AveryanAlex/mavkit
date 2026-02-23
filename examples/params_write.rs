use mavkit::Vehicle;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let bind_addr =
        std::env::var("MAVKIT_EXAMPLE_UDP_BIND").unwrap_or_else(|_| "0.0.0.0:14550".to_string());

    let vehicle = Vehicle::connect_udp(&bind_addr).await?;

    // --- Single parameter write ---
    let param = vehicle
        .params()
        .write("WPNAV_SPEED".to_string(), 500.0)
        .await?;
    println!(
        "wrote {}: value={} type={:?}",
        param.name, param.value, param.param_type
    );

    // --- Batch write ---
    let results = vehicle
        .params()
        .write_batch(vec![
            ("WPNAV_SPEED".to_string(), 500.0),
            ("WPNAV_SPEED_DN".to_string(), 150.0),
            ("WPNAV_SPEED_UP".to_string(), 250.0),
        ])
        .await?;

    println!("\nbatch results:");
    for r in &results {
        let status = if r.success { "ok" } else { "MISMATCH" };
        println!(
            "  {} = {} (requested {}) [{}]",
            r.name, r.confirmed_value, r.requested_value, status
        );
    }

    vehicle.disconnect().await?;
    Ok(())
}
