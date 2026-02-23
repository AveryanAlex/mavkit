use mavkit::Vehicle;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let bind_addr =
        std::env::var("MAVKIT_EXAMPLE_UDP_BIND").unwrap_or_else(|_| "0.0.0.0:14550".to_string());

    let vehicle = Vehicle::connect_udp(&bind_addr).await?;
    println!("listening for status messages (Ctrl+C to stop)...\n");

    let mut st_rx = vehicle.statustext();

    loop {
        st_rx.changed().await?;
        if let Some(msg) = st_rx.borrow().clone() {
            let label = match msg.severity {
                0 => "EMERGENCY",
                1 => "ALERT",
                2 => "CRITICAL",
                3 => "ERROR",
                4 => "WARNING",
                5 => "NOTICE",
                6 => "INFO",
                7 => "DEBUG",
                _ => "UNKNOWN",
            };
            println!("[{label}] {}", msg.text);
        }
    }
}
