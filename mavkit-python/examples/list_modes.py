"""List available flight modes for the connected vehicle."""

import asyncio
import os

import mavkit


async def main():
    bind_addr = os.environ.get("MAVKIT_EXAMPLE_UDP_BIND", "0.0.0.0:14550")

    vehicle = await mavkit.Vehicle.connect_udp(bind_addr)

    state = vehicle.state
    print(f"current mode: {state.mode_name} (id={state.custom_mode})\n")

    modes = vehicle.available_modes()
    if not modes:
        print(f"no mode list available for autopilot {state.autopilot}")
    else:
        print("available flight modes:")
        for mode in modes:
            marker = " <-- active" if mode.custom_mode == state.custom_mode else ""
            print(f"  {mode.custom_mode:>3}  {mode.name}{marker}")

    await vehicle.disconnect()


asyncio.run(main())
