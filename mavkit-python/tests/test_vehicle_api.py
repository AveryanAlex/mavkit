import asyncio
import importlib.metadata
import inspect
import queue
import socket
import struct
import threading
import time
from collections.abc import Callable
from typing import cast

import pytest

import mavkit


def _x25_crc(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        tmp = byte ^ (crc & 0xFF)
        tmp ^= (tmp << 4) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ ((tmp & 0xFF) >> 4)) & 0xFFFF
    return crc


def _pack_mavlink_v2(
    *,
    sequence: int,
    system_id: int,
    component_id: int,
    message_id: int,
    payload: bytes,
    crc_extra: int,
) -> bytes:
    header = bytes(
        [
            len(payload),
            0,
            0,
            sequence & 0xFF,
            system_id & 0xFF,
            component_id & 0xFF,
            message_id & 0xFF,
            (message_id >> 8) & 0xFF,
            (message_id >> 16) & 0xFF,
        ]
    )
    crc = _x25_crc(header + payload + bytes([crc_extra & 0xFF]))
    return bytes([0xFD]) + header + payload + struct.pack("<H", crc)


def _heartbeat_packet(
    sequence: int = 0, *, system_id: int = 1, component_id: int = 1
) -> bytes:
    payload = bytes([0x05, 0x00, 0x00, 0x00, 0x02, 0x03, 0x59, 0x03, 0x03])
    return _pack_mavlink_v2(
        sequence=sequence,
        system_id=system_id,
        component_id=component_id,
        message_id=0,
        payload=payload,
        crc_extra=50,
    )


def _command_ack_packet(
    command: int,
    result: int,
    sequence: int = 0,
    *,
    system_id: int = 1,
    component_id: int = 1,
) -> bytes:
    payload = struct.pack("<HBBiBB", command, result, 0, 0, 0, 0)
    return _pack_mavlink_v2(
        sequence=sequence,
        system_id=system_id,
        component_id=component_id,
        message_id=77,
        payload=payload,
        crc_extra=143,
    )


def _global_position_int_packet(
    sequence: int = 0, *, system_id: int = 1, component_id: int = 1
) -> bytes:
    payload = struct.pack(
        "<IiiiihhhH",
        42,
        473_977_420,
        85_455_940,
        500_000,
        15_000,
        0,
        0,
        0,
        9_000,
    )
    return _pack_mavlink_v2(
        sequence=sequence,
        system_id=system_id,
        component_id=component_id,
        message_id=33,
        payload=payload,
        crc_extra=104,
    )


def _mission_waypoint(lat: float, lon: float, alt: float) -> mavkit.MissionItem:
    return mavkit.MissionItem(
        command=mavkit.NavWaypoint(
            latitude_deg=lat,
            longitude_deg=lon,
            altitude_m=alt,
            frame=mavkit.MissionFrame.GlobalRelativeAltInt,
        ),
    )


def _free_udp_port() -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    try:
        return int(sock.getsockname()[1])
    finally:
        sock.close()


class _FakeMavlinkPeer:
    def __init__(self, vehicle_port: int):
        self.vehicle_addr: tuple[str, int] = ("127.0.0.1", vehicle_port)
        self.sock: socket.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.02)
        self._stop: threading.Event = threading.Event()
        self._scheduled: queue.Queue[tuple[float, bytes]] = queue.Queue()
        self.received_packets: list[bytes] = []
        self._heartbeat_seq: int = 0
        self._thread: threading.Thread = threading.Thread(target=self._run, daemon=True)

    @property
    def bind_addr(self) -> str:
        return f"127.0.0.1:{self.vehicle_addr[1]}"

    def __enter__(self) -> "_FakeMavlinkPeer":
        self._thread.start()
        return self

    def __exit__(self, exc_type: object, exc: object, tb: object) -> None:
        self._stop.set()
        self._thread.join(timeout=1)
        self.sock.close()

    def send_later(self, packet: bytes, delay_s: float = 0.0) -> None:
        self._scheduled.put((time.monotonic() + delay_s, packet))

    def _run(self) -> None:
        next_heartbeat = 0.0
        pending: list[tuple[float, bytes]] = []
        while not self._stop.is_set():
            now = time.monotonic()
            if now >= next_heartbeat:
                self.sock.sendto(
                    _heartbeat_packet(self._heartbeat_seq),
                    self.vehicle_addr,
                )
                self._heartbeat_seq = (self._heartbeat_seq + 1) % 256
                next_heartbeat = now + 0.05

            try:
                pending.append(self._scheduled.get_nowait())
            except queue.Empty:
                pass

            ready = [item for item in pending if item[0] <= now]
            pending = [item for item in pending if item[0] > now]
            for _, packet in ready:
                self.sock.sendto(packet, self.vehicle_addr)

            try:
                data, _addr = self.sock.recvfrom(4096)
            except TimeoutError:
                continue
            self.received_packets.append(data)


class TestVehicleApi:
    def test_vehicle_exposes_domain_handle_accessors(self):
        for name in (
            "telemetry",
            "available_modes",
            "info",
            "support",
            "link",
            "mission",
            "fence",
            "rally",
            "params",
            "raw",
            "ardupilot",
        ):
            assert hasattr(mavkit.Vehicle, name)
            assert callable(getattr(mavkit.Vehicle, name))

    def test_handle_classes_are_exported(self):
        for name in (
            "TelemetryHandle",
            "ModesHandle",
            "InfoHandle",
            "SupportHandle",
            "LinkHandle",
            "MissionHandle",
            "FenceHandle",
            "RallyHandle",
            "ParamsHandle",
            "RawHandle",
            "ArduPilotHandle",
        ):
            assert hasattr(mavkit, name)

    def test_vehicle_root_commands_match_domain_split(self):
        for name in (
            "arm",
            "arm_no_wait",
            "force_arm",
            "force_arm_no_wait",
            "disarm",
            "disarm_no_wait",
            "force_disarm",
            "force_disarm_no_wait",
            "set_mode",
            "set_mode_no_wait",
            "set_mode_by_name",
            "set_mode_by_name_no_wait",
            "set_home",
            "set_home_current",
            "set_origin",
            "disconnect",
            "identity",
        ):
            assert hasattr(mavkit.Vehicle, name)
            assert callable(getattr(mavkit.Vehicle, name))

    def test_vehicle_root_no_wait_signatures_match_runtime_surface(self):
        expected_signatures = {
            "arm_no_wait": "(self, /)",
            "force_arm_no_wait": "(self, /)",
            "disarm_no_wait": "(self, /)",
            "force_disarm_no_wait": "(self, /)",
            "set_mode_no_wait": "(self, /, custom_mode)",
            "set_mode_by_name_no_wait": "(self, /, name)",
        }

        for name, expected_signature in expected_signatures.items():
            method = getattr(mavkit.Vehicle, name)
            assert str(inspect.signature(method)) == expected_signature

    def test_package_root_version_matches_installed_metadata(self):
        assert mavkit.__version__ == importlib.metadata.version("mavkit")

    def test_domain_handles_expose_runtime_methods(self):
        async def scenario() -> None:
            with _FakeMavlinkPeer(_free_udp_port()) as peer:
                vehicle = await mavkit.Vehicle.connect_udp(peer.bind_addr)
                try:
                    mission = vehicle.mission()
                    mission_plan = mavkit.MissionPlan(
                        items=[],
                    )
                    mission_state = mission.latest()
                    assert mission_state is not None
                    assert mission_state.plan is None
                    assert mission_state.current_index is None
                    assert mission_state.sync == mavkit.SyncState.Unknown
                    assert mission_state.active_op is None
                    assert (await mission.wait()).sync == mavkit.SyncState.Unknown
                    assert (
                        await mission.wait_timeout(0.05)
                    ).sync == mavkit.SyncState.Unknown
                    assert (
                        await mission.subscribe().recv()
                    ).sync == mavkit.SyncState.Unknown

                    mission_upload = mission.upload(mission_plan)
                    assert (
                        await mission_upload.subscribe().recv()
                    ).phase == "request_count"
                    mission_upload.cancel()
                    with pytest.raises(mavkit.MavkitError):
                        await mission_upload.wait()

                    mission_download = mission.download()
                    assert mission_download.latest() is not None
                    mission_download.cancel()
                    with pytest.raises(mavkit.MavkitError):
                        await mission_download.wait()

                    mission_verify = mission.verify(mission_plan)
                    assert mission_verify.latest() is not None
                    mission_verify.cancel()
                    with pytest.raises(mavkit.MavkitError):
                        await mission_verify.wait()

                    mission_clear = mission.clear()
                    assert mission_clear.latest() is not None
                    mission_clear.cancel()
                    with pytest.raises(mavkit.MavkitError):
                        await mission_clear.wait()

                    fence = vehicle.fence()
                    assert fence.support().latest() is not None
                    fence_plan = mavkit.FencePlan(return_point=None, regions=[])
                    fence_state = fence.latest()
                    assert fence_state is not None
                    assert fence_state.plan is None
                    assert fence_state.sync == mavkit.SyncState.Unknown
                    assert fence_state.active_op is None
                    assert (await fence.wait()).sync == mavkit.SyncState.Unknown
                    assert (
                        await fence.subscribe().recv()
                    ).sync == mavkit.SyncState.Unknown

                    fence_upload = fence.upload(fence_plan)
                    assert (
                        await fence_upload.subscribe().recv()
                    ).phase == "request_count"
                    fence_upload.cancel()
                    with pytest.raises(mavkit.MavkitError):
                        await fence_upload.wait()

                    fence_download = fence.download()
                    assert fence_download.latest() is not None
                    fence_download.cancel()
                    with pytest.raises(mavkit.MavkitError):
                        await fence_download.wait()

                    fence_clear = fence.clear()
                    assert fence_clear.latest() is not None
                    fence_clear.cancel()
                    with pytest.raises(mavkit.MavkitError):
                        await fence_clear.wait()

                    rally = vehicle.rally()
                    assert rally.support().latest() is not None
                    rally_plan = mavkit.RallyPlan(points=[])
                    rally_state = rally.latest()
                    assert rally_state is not None
                    assert rally_state.plan is None
                    assert rally_state.sync == mavkit.SyncState.Unknown
                    assert rally_state.active_op is None
                    assert (await rally.wait()).sync == mavkit.SyncState.Unknown
                    assert (
                        await rally.subscribe().recv()
                    ).sync == mavkit.SyncState.Unknown

                    rally_upload = rally.upload(rally_plan)
                    assert (
                        await rally_upload.subscribe().recv()
                    ).phase == "request_count"
                    rally_upload.cancel()
                    with pytest.raises(mavkit.MavkitError):
                        await rally_upload.wait()

                    rally_download = rally.download()
                    assert rally_download.latest() is not None
                    rally_download.cancel()
                    with pytest.raises(mavkit.MavkitError):
                        await rally_download.wait()

                    rally_clear = rally.clear()
                    assert rally_clear.latest() is not None
                    rally_clear.cancel()
                    with pytest.raises(mavkit.MavkitError):
                        await rally_clear.wait()
                finally:
                    await vehicle.disconnect()

        asyncio.run(scenario())

    def test_mission_runtime_success_paths_with_test_harness(self):
        async def scenario() -> None:
            vehicle, harness = await mavkit._connect_test_vehicle()
            try:
                mission = vehicle.mission()
                plan = mavkit.MissionPlan(
                    items=[
                        _mission_waypoint(47.397742, 8.545594, 25.0),
                        _mission_waypoint(47.398100, 8.546100, 30.0),
                    ],
                )

                upload_op = mission.upload(plan)
                await harness.push_mission_upload_success(plan)
                await upload_op.wait()
                upload_progress = upload_op.latest()
                assert upload_progress is not None
                assert upload_progress.phase == "completed"

                download_op = mission.download()
                await harness.push_mission_download_success(plan)
                downloaded = await download_op.wait()

                download_progress = download_op.latest()
                assert download_progress is not None
                assert download_progress.phase == "completed"
                assert len(downloaded) == len(plan)
            finally:
                await vehicle.disconnect()

        asyncio.run(scenario())

    def test_raw_handle_runtime_methods(self):
        async def scenario() -> None:
            with _FakeMavlinkPeer(_free_udp_port()) as peer:
                vehicle = await mavkit.Vehicle.connect_udp(peer.bind_addr)
                try:
                    raw = vehicle.raw()

                    ack_task = asyncio.ensure_future(
                        raw.command_long(400, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                    )
                    await asyncio.sleep(0.01)
                    peer.send_later(_command_ack_packet(400, 0))
                    ack = await ack_task
                    assert ack.command == 400
                    assert ack.result == 0

                    with pytest.raises(mavkit.MavkitError):
                        await raw.command_int(
                            400, 255, 0, 0, [1.0, 0.0, 0.0, 0.0], 0, 0, 0.0
                        )

                    subscribe = cast(
                        Callable[[], mavkit.RawMessageStream], getattr(raw, "subscribe")
                    )
                    subscription = subscribe()
                    peer.send_later(_global_position_int_packet(), delay_s=0.01)
                    streamed = await asyncio.wait_for(subscription.recv(), timeout=0.25)
                    assert streamed.message_id == 33
                    assert streamed.message_name == "GLOBAL_POSITION_INT"

                    message_task = asyncio.ensure_future(raw.request_message(33, 0.25))
                    await asyncio.sleep(0.01)
                    peer.send_later(_command_ack_packet(512, 0))
                    peer.send_later(_global_position_int_packet(), delay_s=0.05)
                    message = await message_task
                    assert message.message_id == 33
                    assert message.message_name == "GLOBAL_POSITION_INT"

                    interval_task = asyncio.ensure_future(
                        raw.set_message_interval(33, 250_000)
                    )
                    await asyncio.sleep(0.01)
                    peer.send_later(_command_ack_packet(511, 0))
                    await interval_task
                    assert peer.received_packets

                    sendable = mavkit.RawMessage(
                        message_id=33,
                        payload=message.payload,
                        system_id=message.system_id,
                        component_id=message.component_id,
                    )
                    await raw.send(sendable)
                    assert len(peer.received_packets) >= 2
                finally:
                    await vehicle.disconnect()

        asyncio.run(scenario())

    def test_link_handle_runtime_methods(self):
        async def scenario() -> None:
            with _FakeMavlinkPeer(_free_udp_port()) as peer:
                vehicle = await mavkit.Vehicle.connect_udp(peer.bind_addr)
                try:
                    link = vehicle.link()
                    state = link.state()
                    assert state.latest() == mavkit.LinkState.Connected
                    assert await state.wait_timeout(0.1) == mavkit.LinkState.Connected
                finally:
                    await vehicle.disconnect()

        asyncio.run(scenario())
