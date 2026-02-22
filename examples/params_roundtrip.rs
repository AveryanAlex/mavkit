use mavkit::{Vehicle, format_param_file, parse_param_file};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let bind_addr =
        std::env::var("MAVKIT_EXAMPLE_UDP_BIND").unwrap_or_else(|_| "0.0.0.0:14550".to_string());

    let vehicle = Vehicle::connect_udp(&bind_addr).await?;

    let store = vehicle.params().download_all().await?;
    println!("downloaded {} params", store.params.len());

    let text = format_param_file(&store);
    let parsed = parse_param_file(&text)?;
    println!("formatted+parsed {} params", parsed.len());

    vehicle.disconnect().await?;
    Ok(())
}
