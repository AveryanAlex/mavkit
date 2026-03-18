"""Record raw MAVLink messages to a TLOG file."""

import asyncio
import os
import sys

import mavkit


async def main():
    bind_addr = os.environ.get("MAVKIT_EXAMPLE_UDP_BIND", "0.0.0.0:14550")
    path = sys.argv[1] if len(sys.argv) > 1 else "recording.tlog"

    vehicle = await mavkit.Vehicle.connect_udp(bind_addr)
    try:
        raw = vehicle.raw()
        stream = raw.subscribe()

        with mavkit.TlogWriter(path) as writer:
            print(f"connected, recording to {path}")
            count = 0
            async for message in stream:
                _ = writer.write(message)
                count += 1
                if count % 100 == 0:
                    writer.flush()
                    print(f"{count} messages recorded")
    finally:
        await vehicle.disconnect()


asyncio.run(main())
