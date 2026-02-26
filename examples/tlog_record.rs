use mavkit::Vehicle;
use mavkit::tlog::TlogWriter;
use mavlink::MavlinkVersion;
use std::io::BufWriter;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let bind_addr = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "0.0.0.0:14550".to_string());
    let path = std::env::args()
        .nth(2)
        .unwrap_or_else(|| "recording.tlog".to_string());

    println!("connecting to {bind_addr}...");
    let vehicle = Vehicle::connect_udp(&bind_addr).await?;
    println!("connected, recording to {path}");

    let mut rx = vehicle.raw_messages();
    let file = std::fs::File::create(&path)?;
    let mut writer = TlogWriter::new(BufWriter::new(file), MavlinkVersion::V2);

    let mut count = 0u64;
    loop {
        match rx.recv().await {
            Ok((header, msg)) => {
                writer.write_now(&header, &msg)?;
                count += 1;
                if count.is_multiple_of(100) {
                    writer.flush()?;
                    println!("{count} messages recorded");
                }
            }
            Err(tokio::sync::broadcast::error::RecvError::Lagged(n)) => {
                eprintln!("warning: skipped {n} messages (receiver lagged)");
            }
            Err(tokio::sync::broadcast::error::RecvError::Closed) => {
                break;
            }
        }
    }

    writer.flush()?;
    println!("{count} messages total");
    Ok(())
}
