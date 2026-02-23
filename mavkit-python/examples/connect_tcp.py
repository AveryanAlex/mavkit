"""Connect over TCP, print vehicle state, then disconnect."""

import asyncio
import os

import mavkit


async def main():
    addr = os.environ.get("MAVKIT_EXAMPLE_TCP_ADDR", "127.0.0.1:5760")

    vehicle = await mavkit.Vehicle.connect_tcp(addr)

    state = vehicle.state
    print(
        f"connected: mode={state.mode_name} armed={state.armed}"
        f" autopilot={state.autopilot} vehicle_type={state.vehicle_type}"
    )

    await vehicle.disconnect()


asyncio.run(main())
