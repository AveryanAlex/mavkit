"""Print STATUSTEXT messages from the vehicle (Ctrl+C to stop)."""

import asyncio
import os

import mavkit

SEVERITY_LABELS = {
    0: "EMERGENCY",
    1: "ALERT",
    2: "CRITICAL",
    3: "ERROR",
    4: "WARNING",
    5: "NOTICE",
    6: "INFO",
    7: "DEBUG",
}


async def main():
    bind_addr = os.environ.get("MAVKIT_EXAMPLE_UDP_BIND", "0.0.0.0:14550")

    vehicle = await mavkit.Vehicle.connect_udp(bind_addr)
    print("listening for status messages (Ctrl+C to stop)...\n")

    while True:
        msg = await vehicle.wait_statustext()
        if msg is not None:
            label = SEVERITY_LABELS.get(msg.severity, "UNKNOWN")
            print(f"[{label}] {msg.text}")


asyncio.run(main())
