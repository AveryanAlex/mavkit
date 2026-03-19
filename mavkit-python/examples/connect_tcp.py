import asyncio
import os

import mavkit


async def main():
    addr = os.environ.get("MAVKIT_EXAMPLE_TCP_ADDR", "127.0.0.1:5760")

    async with await mavkit.Vehicle.connect_tcp(addr) as vehicle:
        identity = vehicle.identity()
        current_mode = vehicle.available_modes().current().latest()
        mode_name = current_mode.name if current_mode is not None else "unknown"

        print(
            f"connected: sys={identity.system_id} comp={identity.component_id} "
            f"autopilot={identity.autopilot} vehicle_type={identity.vehicle_type} "
            f"mode={mode_name}"
        )


asyncio.run(main())
