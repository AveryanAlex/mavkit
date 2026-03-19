"""Write individual and batch parameters to the vehicle."""

import asyncio
import os

import mavkit


async def main():
    bind_addr = os.environ.get("MAVKIT_EXAMPLE_UDP_BIND", "0.0.0.0:14550")

    async with await mavkit.Vehicle.connect_udp(bind_addr) as vehicle:
        params = vehicle.params()

        # --- Single parameter write ---
        result = await params.write("WPNAV_SPEED", 500.0)
        print(
            f"wrote {result.name}: requested={result.requested_value}"
            f" confirmed={result.confirmed_value} success={result.success}"
        )

        # --- Batch write ---
        batch = params.write_batch(
            [
                ("WPNAV_SPEED", 500.0),
                ("WPNAV_SPEED_DN", 150.0),
                ("WPNAV_SPEED_UP", 250.0),
            ]
        )
        results = await batch.wait()

        print("\nbatch results:")
        for r in results:
            status = "ok" if r.success else "MISMATCH"
            print(
                f"  {r.name} = {r.confirmed_value} (requested {r.requested_value}) [{status}]"
            )


asyncio.run(main())
