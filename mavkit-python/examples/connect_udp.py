import asyncio
import os

import mavkit


async def main():
    bind_addr = os.environ.get("MAVKIT_EXAMPLE_UDP_BIND", "0.0.0.0:14550")

    async with await mavkit.Vehicle.connect_udp(bind_addr) as vehicle:
        identity = vehicle.identity()
        current_mode = vehicle.available_modes().current().latest()
        armed_sample = vehicle.telemetry().armed().latest()
        position_sample = await vehicle.telemetry().position().global_pos().wait()
        position = position_sample.value
        mode_name = current_mode.name if current_mode is not None else "unknown"
        armed = armed_sample.value if armed_sample is not None else None

        print(
            f"connected: sys={identity.system_id} comp={identity.component_id} "
            f"autopilot={identity.autopilot} vehicle_type={identity.vehicle_type} "
            f"mode={mode_name} armed={armed}"
        )

        print(
            f"telemetry: lat={position.latitude_deg:.7f} lon={position.longitude_deg:.7f}"
            f" alt_msl={position.altitude_msl_m:.1f} rel_alt={position.relative_alt_m:.1f}"
        )


asyncio.run(main())
