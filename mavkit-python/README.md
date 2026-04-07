# MAVKit

Async Python SDK for MAVLink vehicle control, built on a fast Rust core via PyO3.

MAVKit provides a high-level async API for connecting to MAVLink vehicles (ArduPilot, PX4)
over UDP, TCP, or serial, with support for telemetry, commands, missions, parameters,
and ArduPilot-guided sessions.

## Installation

```bash
pip install mavkit
```

Requires Python 3.10+. Pre-built wheels are available for Linux (x86_64, aarch64), macOS (x86_64, ARM), and Windows (x86_64).

## Quick Start

```python
import asyncio
import mavkit


async def main():
    # Connect to a vehicle over UDP
    vehicle = await mavkit.Vehicle.connect_udp("0.0.0.0:14550")

    # Read current mode and armed state
    mode = vehicle.available_modes().current().latest()
    armed = vehicle.telemetry().armed().latest()
    print(
        f"mode={mode.name if mode else 'unknown'} "
        f"armed={armed.value if armed else None}"
    )

    # Wait for a position sample (returns immediately if one is already cached)
    position = (await vehicle.telemetry().position().global_pos().wait()).value
    print(f"lat={position.latitude_deg:.7f} alt={position.relative_alt_m:.1f}m")

    # set_mode_by_name() waits for ACK plus matching mode telemetry
    await vehicle.set_mode_by_name("GUIDED")

    # arm() returns on command ACK only; subscribe before arming if you need
    # a fresh armed-state update as well.
    await vehicle.arm()

    await vehicle.disconnect()


asyncio.run(main())
```

## Features

- **Connections** -- UDP, TCP, serial, custom byte streams
- **Telemetry** -- sync property getters + async `wait_*` methods for reactive updates
- **Commands** -- root `Vehicle` action methods plus `*_no_wait` variants for ACK-only flows
- **ArduPilot guided** -- family-specific guided actions via `await vehicle.ardupilot().guided()`
- **Missions** -- upload, download, clear, set current item, verify roundtrip
- **Parameters** -- download all, write single/batch, `.param` file I/O
- **Validation** -- plan validation, normalization, tolerance-based comparison

## API Overview

### Connecting

```python
# UDP (most common for SITL)
vehicle = await mavkit.Vehicle.connect_udp("0.0.0.0:14550")

# TCP
vehicle = await mavkit.Vehicle.connect_tcp("127.0.0.1:5760")

# Serial
vehicle = await mavkit.Vehicle.connect_serial("/dev/ttyUSB0", 57600)

# With custom config
config = mavkit.VehicleConfig(connect_timeout_secs=60.0)
vehicle = await mavkit.Vehicle.connect_with_config("udpin:0.0.0.0:14550", config)
```

### State and Telemetry

```python
# Sync access (latest snapshot, returns None if no data yet)
position = vehicle.telemetry().position().global_pos().latest()
armed = vehicle.telemetry().armed().latest()
mode = vehicle.available_modes().current().latest()

# Async wait (returns immediately if a value is already cached,
# otherwise waits for the first update)
position = await vehicle.telemetry().position().global_pos().wait()
battery = await vehicle.telemetry().battery().voltage_v().wait()
```

### ArduPilot Guided Actions

Guided actions stay off the root `Vehicle` API. Acquire a guided session first via
`vehicle.ardupilot().guided()`, then narrow to the detected family.

```python
async with await vehicle.ardupilot().guided() as guided:
    copter = guided.copter()
    if copter is None:
        raise RuntimeError("connected vehicle is not an ArduCopter")

    await copter.takeoff(10.0)
    await copter.goto(
        latitude_deg=47.397742,
        longitude_deg=8.545594,
        relative_alt_m=25.0,
    )
```

### Missions

```python
def waypoint(lat: float, lon: float, alt: float) -> mavkit.MissionItem:
    return mavkit.MissionItem(
        command=mavkit.NavWaypoint.from_point(
            position=mavkit.GeoPoint3d.rel_home(
                latitude_deg=lat,
                longitude_deg=lon,
                relative_alt_m=alt,
            ),
        ),
    )


plan = mavkit.MissionPlan(
    items=[
        waypoint(47.397742, 8.545594, 25.0),
        waypoint(47.398100, 8.546100, 30.0),
    ],
)
mission = vehicle.mission()
await mission.upload(plan).wait()
downloaded = await mission.download().wait()
```

### Parameters

```python
params = vehicle.params()
download_op = params.download_all()
store = await download_op.wait()
result = await params.write("BATT_MONITOR", 4.0)
results = await params.write_batch(
    [("BATT_MONITOR", 4.0), ("BATT_CAPACITY", 5000.0)]
).wait()
```

## Examples

The `examples/` directory contains runnable scripts demonstrating each feature:

| Script | Description |
|--------|-------------|
| `connect_tcp.py` | Connect over TCP and print vehicle state |
| `connect_udp.py` | Connect over UDP and print vehicle state and telemetry |
| `list_modes.py` | List available flight modes for the connected vehicle |
| `mission_upload_download.py` | Upload a mission, download it back, and verify the round-trip |
| `monitor_link_state.py` | Watch link state transitions until disconnect or error |
| `monitor_statustext.py` | Print STATUSTEXT messages from the vehicle |
| `monitor_telemetry.py` | Stream live telemetry to the terminal |
| `params_roundtrip.py` | Download all parameters and round-trip through the param file format |
| `params_write.py` | Write individual and batch parameters to the vehicle |
| `raw_messages.py` | Subscribe to raw MAVLink messages |
| `set_mode_and_arm.py` | Set flight mode and arm the vehicle |
| `tlog_parse.py` | Parse a TLOG file and print all entries |
| `tlog_record.py` | Record raw messages to a TLOG file |

## Links

- [GitHub Repository](https://github.com/AveryanAlex/mavkit)
- [Rust API Documentation](https://docs.rs/mavkit)
- [Rust Crate](https://crates.io/crates/mavkit)

## License

MIT
