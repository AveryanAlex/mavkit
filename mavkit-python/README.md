# MAVKit

Async Python SDK for MAVLink vehicle control, built on a fast Rust core via PyO3.

MAVKit provides a high-level async API for connecting to MAVLink vehicles (ArduPilot, PX4)
over UDP, TCP, or serial, with support for telemetry, commands, missions, and parameters.

## Installation

```bash
pip install mavkit
```

Requires Python 3.9+. Pre-built wheels are available for Linux (x86_64, aarch64), macOS (x86_64, ARM), and Windows (x86_64).

## Quick Start

```python
import asyncio
import mavkit

async def main():
    # Connect to a vehicle over UDP
    vehicle = await mavkit.Vehicle.connect_udp("0.0.0.0:14550")

    # Read current state
    state = vehicle.state
    print(f"mode={state.mode_name} armed={state.armed}")

    # Wait for next telemetry update
    telem = await vehicle.wait_telemetry()
    print(f"alt={telem.altitude_m}m  battery={telem.battery_pct}%")

    # Arm, take off, then disarm
    await vehicle.set_mode_by_name("GUIDED")
    await vehicle.arm(force=False)
    await vehicle.takeoff(altitude_m=10.0)

    await vehicle.disconnect()

asyncio.run(main())
```

## Features

- **Connections** -- UDP, TCP, serial, custom byte streams
- **Telemetry** -- sync property getters + async `wait_*` methods for reactive updates
- **Commands** -- arm, disarm, set mode (by name or number), takeoff, guided goto
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
# Sync access (latest snapshot)
state = vehicle.state
telem = vehicle.telemetry

# Async wait (blocks until next update)
state = await vehicle.wait_state()
telem = await vehicle.wait_telemetry()
home = await vehicle.wait_home_position()
```

### Missions

```python
plan = mavkit.MissionPlan(
    mission_type=mavkit.MissionType.Mission,
    items=[
        mavkit.MissionItem(
            seq=0, command=16,
            frame=mavkit.MissionFrame.GlobalRelativeAltInt,
            x=int(47.397742 * 1e7), y=int(8.545594 * 1e7), z=50.0,
        ),
    ],
)
await vehicle.upload_mission(plan)
downloaded = await vehicle.download_mission(mavkit.MissionType.Mission)
```

### Parameters

```python
store = await vehicle.download_params()
param = await vehicle.write_param("BATT_MONITOR", 4.0)
results = await vehicle.write_params_batch([("BATT_MONITOR", 4.0), ("BATT_CAPACITY", 5000.0)])
```

## Examples

The `examples/` directory contains runnable scripts demonstrating each feature:

| Script | Description |
|--------|-------------|
| `connect_udp.py` | Connect over UDP, print vehicle state and telemetry |
| `connect_tcp.py` | Connect over TCP, print vehicle state |
| `monitor_telemetry.py` | Stream live telemetry to the terminal |
| `monitor_statustext.py` | Print STATUSTEXT messages from the vehicle |
| `list_modes.py` | List available flight modes for the connected vehicle |
| `set_mode_and_arm.py` | Set flight mode and arm the vehicle |
| `mission_upload_download.py` | Upload a mission, download it back, and verify the round-trip |
| `params_roundtrip.py` | Download all parameters and round-trip through the param file format |
| `params_write.py` | Write individual and batch parameters to the vehicle |
| `tlog_parse.py` | Parse a TLOG file and print all entries |

## Links

- [GitHub Repository](https://github.com/averyanalex/mavkit)
- [Rust API Documentation](https://docs.rs/mavkit)
- [Rust Crate](https://crates.io/crates/mavkit)

## License

MIT
