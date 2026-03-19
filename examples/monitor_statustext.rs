use mavkit::Vehicle;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let bind_addr =
        std::env::var("MAVKIT_EXAMPLE_UDP_BIND").unwrap_or_else(|_| "0.0.0.0:14550".to_string());

    let vehicle = Vehicle::connect_udp(&bind_addr).await?;
    println!("listening for status messages (Ctrl+C to stop)...\n");

    let status_text = vehicle.telemetry().messages().status_text();
    let mut subscription = status_text.subscribe();

    while let Some(sample) = subscription.recv().await {
        let msg = sample.value;
        println!("[{:?}] {}", msg.severity, msg.text);
    }

    Ok(())
}
