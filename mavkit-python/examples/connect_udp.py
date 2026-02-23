"""Connect over UDP, print vehicle state and telemetry, then disconnect."""

import asyncio
import os

import mavkit


async def main():
    bind_addr = os.environ.get("MAVKIT_EXAMPLE_UDP_BIND", "0.0.0.0:14550")

    vehicle = await mavkit.Vehicle.connect_udp(bind_addr)

    state = await vehicle.wait_state()
    print(
        f"connected: mode={state.mode_name} armed={state.armed}"
        f" autopilot={state.autopilot} vehicle_type={state.vehicle_type}"
    )

    telemetry = await vehicle.wait_telemetry()
    print(
        f"telemetry: lat={telemetry.latitude_deg}"
        f" lon={telemetry.longitude_deg} alt={telemetry.altitude_m}"
    )

    await vehicle.disconnect()


asyncio.run(main())
