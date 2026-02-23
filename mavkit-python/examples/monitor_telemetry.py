"""Stream live telemetry to the terminal (Ctrl+C to stop)."""

import asyncio
import os

import mavkit

CLEAR = "\x1b[2J\x1b[H"


async def main():
    bind_addr = os.environ.get("MAVKIT_EXAMPLE_UDP_BIND", "0.0.0.0:14550")

    vehicle = await mavkit.Vehicle.connect_udp(bind_addr)
    print("connected, streaming telemetry (Ctrl+C to stop)...\n")

    while True:
        t = await vehicle.wait_telemetry()

        print(CLEAR, end="")

        print("=== Position ===")
        print(f"  lat: {t.latitude_deg or 0:>12.7f}°")
        print(f"  lon: {t.longitude_deg or 0:>12.7f}°")
        print(f"  alt: {t.altitude_m or 0:>8.1f} m")

        print("\n=== Movement ===")
        print(f"  speed:    {t.speed_mps or 0:>6.1f} m/s")
        print(f"  airspeed: {t.airspeed_mps or 0:>6.1f} m/s")
        print(f"  climb:    {t.climb_rate_mps or 0:>6.1f} m/s")
        print(f"  heading:  {t.heading_deg or 0:>6.1f}°")

        print("\n=== Attitude ===")
        print(f"  roll:  {t.roll_deg or 0:>7.2f}°")
        print(f"  pitch: {t.pitch_deg or 0:>7.2f}°")
        print(f"  yaw:   {t.yaw_deg or 0:>7.2f}°")

        print("\n=== Battery ===")
        print(f"  voltage:   {t.battery_voltage_v or 0:>6.2f} V")
        print(f"  current:   {t.battery_current_a or 0:>6.2f} A")
        print(f"  remaining: {t.battery_pct or 0:>3.0f}%")

        print("\n=== GPS ===")
        print(f"  fix:        {t.gps_fix_type}")
        sats = str(t.gps_satellites) if t.gps_satellites is not None else "-"
        print(f"  satellites: {sats}")
        print(f"  hdop:       {t.gps_hdop or 0:.2f}")


asyncio.run(main())
