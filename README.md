# MAVKit

[![Crates.io](https://img.shields.io/crates/v/mavkit)](https://crates.io/crates/mavkit)
[![PyPI](https://img.shields.io/pypi/v/mavkit)](https://pypi.org/project/mavkit/)
[![docs.rs](https://img.shields.io/docsrs/mavkit)](https://docs.rs/mavkit)
[![CI](https://github.com/AveryanAlex/mavkit/actions/workflows/ci.yml/badge.svg)](https://github.com/AveryanAlex/mavkit/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](https://github.com/AveryanAlex/mavkit/blob/main/LICENSE)

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
mavkit = "0.5"
tokio = { version = "1", features = ["macros", "rt-multi-thread"] }
```

For browser/wasm builds, disable default transports and enable the wasm entry point:

```toml
[dependencies]
mavkit = { version = "0.5", default-features = false, features = ["wasm"] }
```

## Quick Start

### Python

```python
import asyncio
import mavkit

async def main():
    vehicle = await mavkit.Vehicle.connect_udp("0.0.0.0:14550")

    mode = vehicle.available_modes().current().latest()
    armed = vehicle.telemetry().armed().latest()
    print(f"mode={mode.name if mode else 'unknown'} armed={armed.value if armed else None}")

    position = (await vehicle.telemetry().position().global_pos().wait()).value
    print(f"lat={position.latitude_deg:.7f} lon={position.longitude_deg:.7f}")

    await vehicle.disconnect()

asyncio.run(main())
```

### Rust

```rust,no_run
use mavkit::Vehicle;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let vehicle = Vehicle::connect_udp("0.0.0.0:14550").await?;

    let mode = vehicle.available_modes().current().wait().await?;
    let armed = vehicle.telemetry().armed().wait().await?.value;
    println!("mode={} armed={armed}", mode.name);

    vehicle.disconnect().await?;
    Ok(())
}
```

### Rust: custom byte transports / wasm

`Vehicle::connect*` remains the easiest path when MAVKit owns the socket or serial transport.
Use `Vehicle::from_connection` when you already have an `AsyncMavConnection`, and use
`Vehicle::from_byte_connection` when the host environment owns raw MAVLink bytes (for example a
browser WebSocket/WebSerial bridge or another callback-driven transport).

With the `wasm` feature enabled, `from_byte_connection` returns a `ByteBridge` immediately plus a
vehicle future. Feed inbound/outbound frames through the bridge, then await the future once the
first heartbeat can flow through your transport.

On the current stable `wasm32-unknown-unknown` toolchain, spawned task panics are not recoverable
through MAVKit's internal task join handles; browser transports should treat such panics as fatal
and restart the owning task/session.

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
| `stream` | No | Generic async stream helpers for BLE/SPP/custom links |
| `byte-connection` | No | Callback-style raw byte bridge for custom transports |
| `wasm` | No | Browser-friendly runtime + `from_byte_connection` entry point (`byte-connection` implied) |
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
make test-sitl        # start fresh TCP SITL, run integration tests, cleanup
make test-sitl-strict # same with MAVKIT_SITL_STRICT=1
make test-wasm-sitl   # start SITL + WebSocket bridge, run browser wasm integration test
```

Each `make test-sitl` invocation gets its own Docker container and random localhost TCP port, so parallel invocations do not contend on UDP `14550`. Keep in-process SITL test threads at the default `1` for stateful tests; parallelize by running separate `make test-sitl` invocations. For a specific test binary, run `MAVKIT_SITL_CARGO_TEST_ARGS="--test sitl_roundtrip" make test-sitl`.

Wasm SITL tests also require `wasm-bindgen-test-runner` plus a browser WebDriver such as ChromeDriver or GeckoDriver on `PATH`. The wasm harness connects browser WebSocket bytes to SITL TCP through `scripts/ws_tcp_proxy.py` and compiles the proxy URL into the test with `MAVKIT_WASM_SITL_WS_URL`.

## License

MIT
