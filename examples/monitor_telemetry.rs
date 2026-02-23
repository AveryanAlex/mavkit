use mavkit::Vehicle;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let bind_addr =
        std::env::var("MAVKIT_EXAMPLE_UDP_BIND").unwrap_or_else(|_| "0.0.0.0:14550".to_string());

    let vehicle = Vehicle::connect_udp(&bind_addr).await?;
    println!("connected, streaming telemetry (Ctrl+C to stop)...\n");

    let mut telem_rx = vehicle.telemetry();

    loop {
        telem_rx.changed().await?;
        let t = telem_rx.borrow().clone();

        print!("\x1b[2J\x1b[H"); // clear terminal

        println!("=== Position ===");
        println!("  lat: {:>12.7}°", t.latitude_deg.unwrap_or_default());
        println!("  lon: {:>12.7}°", t.longitude_deg.unwrap_or_default());
        println!("  alt: {:>8.1} m", t.altitude_m.unwrap_or_default());

        println!("\n=== Movement ===");
        println!("  speed:    {:>6.1} m/s", t.speed_mps.unwrap_or_default());
        println!(
            "  airspeed: {:>6.1} m/s",
            t.airspeed_mps.unwrap_or_default()
        );
        println!(
            "  climb:    {:>6.1} m/s",
            t.climb_rate_mps.unwrap_or_default()
        );
        println!("  heading:  {:>6.1}°", t.heading_deg.unwrap_or_default());

        println!("\n=== Attitude ===");
        println!("  roll:  {:>7.2}°", t.roll_deg.unwrap_or_default());
        println!("  pitch: {:>7.2}°", t.pitch_deg.unwrap_or_default());
        println!("  yaw:   {:>7.2}°", t.yaw_deg.unwrap_or_default());

        println!("\n=== Battery ===");
        println!(
            "  voltage: {:>6.2} V",
            t.battery_voltage_v.unwrap_or_default()
        );
        println!(
            "  current: {:>6.2} A",
            t.battery_current_a.unwrap_or_default()
        );
        println!("  remaining: {:>3.0}%", t.battery_pct.unwrap_or_default());

        println!("\n=== GPS ===");
        println!("  fix:        {:?}", t.gps_fix_type.unwrap_or_default());
        println!(
            "  satellites: {}",
            t.gps_satellites.map_or("-".to_string(), |s| s.to_string())
        );
        println!("  hdop:       {:.2}", t.gps_hdop.unwrap_or_default());
    }
}
