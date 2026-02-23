"""Write individual and batch parameters to the vehicle."""

import asyncio
import os

import mavkit


async def main():
    bind_addr = os.environ.get("MAVKIT_EXAMPLE_UDP_BIND", "0.0.0.0:14550")

    vehicle = await mavkit.Vehicle.connect_udp(bind_addr)

    # --- Single parameter write ---
    param = await vehicle.write_param("WPNAV_SPEED", 500.0)
    print(f"wrote {param.name}: value={param.value} type={param.param_type}")

    # --- Batch write ---
    results = await vehicle.write_params_batch(
        [
            ("WPNAV_SPEED", 500.0),
            ("WPNAV_SPEED_DN", 150.0),
            ("WPNAV_SPEED_UP", 250.0),
        ]
    )

    print("\nbatch results:")
    for r in results:
        status = "ok" if r.success else "MISMATCH"
        print(
            f"  {r.name} = {r.confirmed_value} (requested {r.requested_value}) [{status}]"
        )

    await vehicle.disconnect()


asyncio.run(main())
