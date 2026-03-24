import mavkit


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

    await vehicle.disconnect()


async def check_context_manager(bind_addr: str) -> None:
    async with await mavkit.Vehicle.connect_udp(bind_addr) as vehicle:
        modes_handle: mavkit.ModesHandle = vehicle.available_modes()
        current_mode: mavkit.CurrentMode | None = modes_handle.current().latest()
        if current_mode is not None:
            _mode_name: str = current_mode.name
