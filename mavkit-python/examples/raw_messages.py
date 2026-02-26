"""Subscribe to raw MAVLink messages and print each one."""

import asyncio
import os

import mavkit


async def main():
    bind_addr = os.environ.get("MAVKIT_EXAMPLE_UDP_BIND", "0.0.0.0:14550")

    vehicle = await mavkit.Vehicle.connect_udp(bind_addr)
    stream = vehicle.subscribe_raw_messages()

    count = 0
    while True:
        msg = await stream.recv()
        print(
            f"[{msg.message_name}] sys={msg.system_id}"
            f" comp={msg.component_id} id={msg.message_id}"
        )
        count += 1
        if count >= 100:
            break

    await vehicle.disconnect()
    print(f"{count} messages received")


asyncio.run(main())
