use mavkit::tlog::TlogFile;
use mavlink::Message;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = std::env::args()
        .nth(1)
        .expect("usage: tlog_parse <path.tlog>");

    let tlog = TlogFile::open(&path).await?;

    // Print time range
    if let Some((first, last)) = tlog.time_range().await? {
        println!("time range: {first} .. {last} usec");
    } else {
        println!("empty file");
        return Ok(());
    }

    // Print all entries
    let mut reader = tlog.entries().await?;
    let mut count = 0u64;
    while let Some(entry) = reader.next().await? {
        println!(
            "[{}] {} (id={})",
            entry.timestamp_usec,
            entry.message.message_name(),
            entry.message.message_id(),
        );
        count += 1;
    }
    println!("{count} entries total");

    Ok(())
}
