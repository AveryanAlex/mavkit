"""Set flight mode and arm the vehicle."""

import asyncio
import os

import mavkit


async def main():
    bind_addr = os.environ.get("MAVKIT_EXAMPLE_UDP_BIND", "0.0.0.0:14550")
    mode = os.environ.get("MAVKIT_EXAMPLE_MODE", "GUIDED")

    async with await mavkit.Vehicle.connect_udp(bind_addr) as vehicle:
        current_mode_handle = vehicle.available_modes().current()
        armed_handle = vehicle.telemetry().armed()

        before_mode = current_mode_handle.latest()
        before_armed = armed_handle.latest()
        before_mode_name = before_mode.name if before_mode is not None else "unknown"
        before_armed_value = before_armed.value if before_armed is not None else None
        print(f"before: mode={before_mode_name} armed={before_armed_value}")

        print(f"setting mode to {mode}...")
        await vehicle.set_mode_by_name(mode)

        print("arming...")
        await vehicle.arm(force=False)

        after_mode = current_mode_handle.latest()
        after_armed = armed_handle.latest()
        after_mode_name = after_mode.name if after_mode is not None else "unknown"
        after_armed_value = after_armed.value if after_armed is not None else None
        print(f"after:  mode={after_mode_name} armed={after_armed_value}")


asyncio.run(main())
