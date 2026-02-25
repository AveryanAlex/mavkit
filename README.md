# MAVKit

[![Crates.io](https://img.shields.io/crates/v/mavkit)](https://crates.io/crates/mavkit)
[![PyPI](https://img.shields.io/pypi/v/mavkit)](https://pypi.org/project/mavkit/)
[![docs.rs](https://img.shields.io/docsrs/mavkit)](https://docs.rs/mavkit)
[![CI](https://github.com/averyanalex/mavkit/actions/workflows/ci.yml/badge.svg)](https://github.com/averyanalex/mavkit/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](https://github.com/averyanalex/mavkit/blob/main/LICENSE)

Async MAVLink SDK for vehicle control, telemetry, missions, and parameters.
Available as a Rust crate and a Python package with full async support.

## Install

**Python**
```bash
pip install mavkit
```

**Rust**
```toml
[dependencies]
mavkit = "0.3"
tokio = { version = "1", features = ["macros", "rt-multi-thread"] }
```

## Quick Start

### Python

```python
import asyncio
import mavkit

async def main():
    vehicle = await mavkit.Vehicle.connect_udp("0.0.0.0:14550")

    state = await vehicle.wait_state()
    print(f"mode={state.mode_name} armed={state.armed}")

    telem = await vehicle.wait_telemetry()
    print(f"alt={telem.altitude_m} bat={telem.battery_pct}")

    await vehicle.disconnect()

asyncio.run(main())
```

### Rust

```rust,no_run
use mavkit::Vehicle;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let vehicle = Vehicle::connect_udp("0.0.0.0:14550").await?;

    let mut state_rx = vehicle.state();
    state_rx.changed().await?;
    let state = state_rx.borrow().clone();
    println!("mode={} armed={}", state.mode_name, state.armed);

    vehicle.disconnect().await?;
    Ok(())
}
```

## Features

- **Connections** -- UDP, TCP, serial, BLE/SPP via byte-stream adapters
- **Telemetry** -- reactive watch channels (Rust) / sync properties + async waiters (Python)
- **Commands** -- arm, disarm, set mode, takeoff, guided goto, arbitrary COMMAND_LONG
- **Missions** -- upload, download, clear, set current, verify roundtrip
- **Parameters** -- download all, write single, batch write, `.param` file I/O
- **Validation** -- plan validation, normalization, tolerance-based comparison

## Feature Flags (Rust)

| Flag | Default | Description |
|---|---|---|
| `udp` | Yes | MAVLink UDP transport |
| `tcp` | No | MAVLink TCP transport |
| `serial` | Yes | MAVLink direct serial transport |
| `ardupilot` | Yes | ArduPilot mode-name mapping |
| `stream` | No | Byte-stream adapters for BLE/SPP/custom links |
| `tlog` | No | TLOG file parser for timestamped MAVLink logs |

## Mission Wire Semantics

For `MissionType::Mission`, MAVLink wire transfer is normalized:
- **Upload**: a home item is prepended at `seq=0` (or a zero placeholder if missing)
- **Download**: wire `seq=0` is extracted as home; remaining items are resequenced from 0

For `Fence` and `Rally` types, items pass through unchanged.

## Examples

**Rust** -- see [`examples/`](examples/):
```bash
cargo run --example connect_udp
```

**Python** -- see [`mavkit-python/examples/`](mavkit-python/examples/):
```bash
cd mavkit-python && uv run python examples/connect_udp.py
```

## Development

### Rust

```bash
cargo check
cargo clippy --all-targets --all-features -- -D warnings
cargo test
cargo fmt --all -- --check
```

### Python

```bash
cd mavkit-python
uv sync
uv run maturin develop
uv run ruff format --check .
uv run ruff check .
```

### SITL Integration Tests

Requires Docker + ArduPilot SITL:

```bash
make bridge-up        # start SITL + MAVProxy bridge
make test-sitl        # run integration tests
make bridge-down      # cleanup
```

Environment: `MAVKIT_SITL_UDP_BIND` (default `0.0.0.0:14550`), `MAVKIT_SITL_STRICT` (set `1` for strict mode).

## License

MIT
