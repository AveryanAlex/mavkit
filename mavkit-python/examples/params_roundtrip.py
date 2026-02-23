"""Download all parameters and round-trip through the param file format."""

import asyncio
import os

import mavkit


async def main():
    bind_addr = os.environ.get("MAVKIT_EXAMPLE_UDP_BIND", "0.0.0.0:14550")

    vehicle = await mavkit.Vehicle.connect_udp(bind_addr)

    store = await vehicle.download_params()
    print(f"downloaded {len(store)} params")

    text = mavkit.format_param_file(store)
    parsed = mavkit.parse_param_file(text)
    print(f"formatted+parsed {len(parsed)} params")

    await vehicle.disconnect()


asyncio.run(main())
