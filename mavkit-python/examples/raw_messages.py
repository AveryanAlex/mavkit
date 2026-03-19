import asyncio
import os

import mavkit


async def main():
    bind_addr = os.environ.get("MAVKIT_EXAMPLE_UDP_BIND", "0.0.0.0:14550")
    message_count = int(os.environ.get("MAVKIT_EXAMPLE_RAW_COUNT", "5"))

    async with await mavkit.Vehicle.connect_udp(bind_addr) as vehicle:
        identity = vehicle.identity()
        raw = vehicle.raw()
        stream = raw.subscribe()

        print(f"connected: sys={identity.system_id} comp={identity.component_id}")
        print(f"raw handle: {raw}")
        print(f"listening for {message_count} raw MAVLink messages...")

        for _ in range(message_count):
            message = await stream.recv()
            print(
                f"message_id={message.message_id} name={message.message_name} "
                f"sys={message.system_id} comp={message.component_id} payload_len={len(message.payload)}"
            )


asyncio.run(main())
