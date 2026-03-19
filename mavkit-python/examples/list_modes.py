"""List available flight modes for the connected vehicle."""

import asyncio
import os

import mavkit


async def main():
    bind_addr = os.environ.get("MAVKIT_EXAMPLE_UDP_BIND", "0.0.0.0:14550")

    async with await mavkit.Vehicle.connect_udp(bind_addr) as vehicle:
        modes = vehicle.available_modes()
        current_handle = modes.current()
        catalog_handle = modes.catalog()
        identity = vehicle.identity()

        current = await current_handle.wait()

        print(f"current mode: {current.name} (id={current.custom_mode})\n")

        catalog = await catalog_handle.wait()

        if not catalog:
            print(f"no mode list available for autopilot {identity.autopilot}")
        else:
            print("available flight modes:")
            for mode in catalog:
                marker = (
                    " <-- active" if mode.custom_mode == current.custom_mode else ""
                )
                print(f"  {mode.custom_mode:>3}  {mode.name}{marker}")


asyncio.run(main())
