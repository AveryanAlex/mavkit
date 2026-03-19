import asyncio
import os

import mavkit


def waypoint(lat: float, lon: float, alt: float) -> mavkit.MissionItem:
    return mavkit.MissionItem(
        command=mavkit.NavWaypoint(
            latitude_deg=lat,
            longitude_deg=lon,
            altitude_m=alt,
            frame=mavkit.MissionFrame.GlobalRelativeAltInt,
        ),
    )


async def main():
    bind_addr = os.environ.get("MAVKIT_EXAMPLE_UDP_BIND", "0.0.0.0:14550")

    vehicle = await mavkit.Vehicle.connect_udp(bind_addr)
    try:
        mission = vehicle.mission()

        items = [
            waypoint(47.397742, 8.545594, 25.0),
            waypoint(47.398100, 8.546100, 30.0),
        ]
        items[0] = mavkit.MissionItem(
            command=mavkit.NavWaypoint(
                latitude_deg=47.397742,
                longitude_deg=8.545594,
                altitude_m=25.0,
                frame=mavkit.MissionFrame.GlobalRelativeAltInt,
            ),
            current=True,
        )
        plan = mavkit.MissionPlan(
            mission_type=mavkit.MissionType.Mission,
            items=items,
        )

        issues = mavkit.validate_plan(plan)
        if issues:
            raise RuntimeError(f"mission plan has {len(issues)} validation issue(s)")

        upload_op = mission.upload(plan)
        await upload_op.wait()

        download_op = mission.download()
        downloaded = await download_op.wait()

        equivalent = mavkit.plans_equivalent(
            mavkit.normalize_for_compare(plan),
            mavkit.normalize_for_compare(downloaded),
            mavkit.CompareTolerance(),
        )

        upload_progress = upload_op.latest()
        download_progress = download_op.latest()

        print(f"mission handle: {mission}")
        print(
            f"upload phase: {upload_progress.phase if upload_progress else 'unknown'}"
        )
        print(
            f"download phase: {download_progress.phase if download_progress else 'unknown'}"
        )
        print(f"uploaded items: {len(plan.items)}")
        print(f"downloaded items: {len(downloaded.items)}")
        print(f"roundtrip equivalent: {equivalent}")
    finally:
        await vehicle.disconnect()


asyncio.run(main())
