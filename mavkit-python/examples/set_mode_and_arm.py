"""Set flight mode and arm the vehicle."""

import asyncio
import os

import mavkit


async def main():
    bind_addr = os.environ.get("MAVKIT_EXAMPLE_UDP_BIND", "0.0.0.0:14550")
    mode = os.environ.get("MAVKIT_EXAMPLE_MODE", "GUIDED")

    vehicle = await mavkit.Vehicle.connect_udp(bind_addr)

    state = vehicle.state
    print(f"before: mode={state.mode_name} armed={state.armed}")

    print(f"setting mode to {mode}...")
    await vehicle.set_mode_by_name(mode)

    print("arming...")
    await vehicle.arm(False)

    state = await vehicle.wait_state()
    print(f"after:  mode={state.mode_name} armed={state.armed}")

    await vehicle.disconnect()


asyncio.run(main())
