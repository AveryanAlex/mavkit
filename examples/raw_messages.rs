use mavkit::Vehicle;
use tokio_stream::StreamExt;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let bind_addr =
        std::env::var("MAVKIT_EXAMPLE_UDP_BIND").unwrap_or_else(|_| "0.0.0.0:14550".to_string());
    let message_count: usize = std::env::var("MAVKIT_EXAMPLE_RAW_COUNT")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(5);

    let vehicle = Vehicle::connect_udp(&bind_addr).await?;
    let identity = vehicle.identity();
    println!("connected: sys={} comp={}", identity.system_id, identity.component_id);
    println!("listening for {message_count} raw MAVLink messages...");

    let mut stream = vehicle.raw().subscribe();
    for _ in 0..message_count {
        if let Some(message) = stream.next().await {
            println!(
                "message_id={} sys={} comp={} payload_len={}",
                message.message_id, message.system_id, message.component_id, message.payload.len(),
            );
        }
    }

    vehicle.disconnect().await?;
    Ok(())
}
