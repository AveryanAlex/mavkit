from __future__ import annotations

from collections.abc import Callable, Coroutine
from typing import Any, Protocol

import mavkit


class _CopterGoto(Protocol):
    def __call__(
        self,
        *,
        latitude_deg: float,
        longitude_deg: float,
        relative_alt_m: float,
    ) -> Coroutine[Any, Any, None]: ...


class _PlaneReposition(Protocol):
    def __call__(
        self,
        *,
        latitude_deg: float,
        longitude_deg: float,
        altitude_msl_m: float,
    ) -> Coroutine[Any, Any, None]: ...


class _RoverDriveTo(Protocol):
    def __call__(
        self,
        *,
        latitude_deg: float,
        longitude_deg: float,
    ) -> Coroutine[Any, Any, None]: ...


class _SubGotoDepth(Protocol):
    def __call__(
        self,
        *,
        latitude_deg: float,
        longitude_deg: float,
        depth_m: float,
    ) -> Coroutine[Any, Any, None]: ...


async def check_vehicle_api(bind_addr: str) -> None:
    vehicle = await mavkit.Vehicle.connect_udp(bind_addr)

    link_handle: mavkit.LinkHandle = vehicle.link()
    state_handle: mavkit.LinkStateHandle = link_handle.state()
    current_state: mavkit.LinkState | None = state_handle.latest()
    if current_state is not None:
        _stable_state: mavkit.LinkState = current_state

    subscription: mavkit.LinkStateSubscription = state_handle.subscribe()
    last_error_message: str | None = subscription.last_error_message
    _ = last_error_message
    awaited_state: mavkit.LinkState = await subscription.recv()
    _ = awaited_state

    mission_handle: mavkit.MissionHandle = vehicle.mission()
    mission_plan = mavkit.MissionPlan(items=[])
    upload_op: mavkit.MissionUploadOp = mission_handle.upload(mission_plan)
    latest_progress: mavkit.MissionOperationProgress | None = upload_op.latest()
    if latest_progress is not None:
        _phase: str = latest_progress.phase

    arm_no_wait: Callable[[], Coroutine[Any, Any, None]] = vehicle.arm_no_wait
    force_arm_no_wait: Callable[[], Coroutine[Any, Any, None]] = (
        vehicle.force_arm_no_wait
    )
    disarm_no_wait: Callable[[], Coroutine[Any, Any, None]] = vehicle.disarm_no_wait
    force_disarm_no_wait: Callable[[], Coroutine[Any, Any, None]] = (
        vehicle.force_disarm_no_wait
    )
    set_mode_no_wait: Callable[[int], Coroutine[Any, Any, None]] = (
        vehicle.set_mode_no_wait
    )
    set_mode_by_name_no_wait: Callable[[str], Coroutine[Any, Any, None]] = (
        vehicle.set_mode_by_name_no_wait
    )
    _ = (
        arm_no_wait,
        force_arm_no_wait,
        disarm_no_wait,
        force_disarm_no_wait,
        set_mode_no_wait,
        set_mode_by_name_no_wait,
    )

    ardupilot_handle: mavkit.ArduPilotHandle = vehicle.ardupilot()
    guided_entrypoint: Callable[[], Coroutine[Any, Any, mavkit.ArduGuidedSession]] = (
        ardupilot_handle.guided
    )
    guided_session: mavkit.ArduGuidedSession = await guided_entrypoint()
    close_guided: Callable[[], Coroutine[Any, Any, None]] = guided_session.close
    _ = close_guided

    copter_guided: mavkit.ArduCopterGuidedHandle | None = guided_session.copter()
    plane_guided: mavkit.ArduPlaneGuidedHandle | None = guided_session.plane()
    rover_guided: mavkit.ArduRoverGuidedHandle | None = guided_session.rover()
    sub_guided: mavkit.ArduSubGuidedHandle | None = guided_session.sub()

    if copter_guided is not None:
        copter_takeoff: Callable[[float], Coroutine[Any, Any, None]] = (
            copter_guided.takeoff
        )
        copter_goto: _CopterGoto = copter_guided.goto
        _ = (copter_takeoff, copter_goto)

    if plane_guided is not None:
        plane_reposition: _PlaneReposition = plane_guided.reposition
        plane_vtol_guided: mavkit.ArduPlaneVtolGuidedHandle | None = plane_guided.vtol()
        _ = plane_reposition
        if plane_vtol_guided is not None:
            plane_vtol_takeoff: Callable[[float], Coroutine[Any, Any, None]] = (
                plane_vtol_guided.takeoff
            )
            _ = plane_vtol_takeoff

    if rover_guided is not None:
        rover_drive_to: _RoverDriveTo = rover_guided.drive_to
        _ = rover_drive_to

    if sub_guided is not None:
        sub_goto_depth: _SubGotoDepth = sub_guided.goto_depth
        _ = sub_goto_depth

    await vehicle.disconnect()


async def check_context_manager(bind_addr: str) -> None:
    async with await mavkit.Vehicle.connect_udp(bind_addr) as vehicle:
        modes_handle: mavkit.ModesHandle = vehicle.available_modes()
        current_mode: mavkit.CurrentMode | None = modes_handle.current().latest()
        if current_mode is not None:
            _mode_name: str = current_mode.name
