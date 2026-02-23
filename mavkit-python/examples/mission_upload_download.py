"""Upload a mission, download it back, and verify the round-trip."""

import asyncio
import os

import mavkit


def waypoint(seq: int, lat: float, lon: float, alt: float) -> mavkit.MissionItem:
    return mavkit.MissionItem(
        seq=seq,
        command=16,
        frame=mavkit.MissionFrame.GlobalRelativeAltInt,
        x=int(lat * 1e7),
        y=int(lon * 1e7),
        z=alt,
        current=seq == 0,
    )


async def main():
    bind_addr = os.environ.get("MAVKIT_EXAMPLE_UDP_BIND", "0.0.0.0:14550")

    vehicle = await mavkit.Vehicle.connect_udp(bind_addr)

    plan = mavkit.MissionPlan(
        mission_type=mavkit.MissionType.Mission,
        home=mavkit.HomePosition(
            latitude_deg=47.397742,
            longitude_deg=8.545594,
            altitude_m=0.0,
        ),
        items=[
            waypoint(0, 47.397742, 8.545594, 25.0),
            waypoint(1, 47.398100, 8.546100, 30.0),
        ],
    )

    await vehicle.upload_mission(plan)
    downloaded = await vehicle.download_mission(mavkit.MissionType.Mission)

    equivalent = mavkit.plans_equivalent(
        mavkit.normalize_for_compare(plan),
        mavkit.normalize_for_compare(downloaded),
        mavkit.CompareTolerance(),
    )
    print(f"roundtrip equivalent: {equivalent}")

    await vehicle.disconnect()


asyncio.run(main())
