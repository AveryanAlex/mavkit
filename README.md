# MAVKit

MAVKit (crate name: `mavkit`) is an async Rust SDK for MAVLink vehicles.

It provides a transport-agnostic `Vehicle` API for:
- connection and link state
- telemetry and status watches
- vehicle actions (arm/disarm, mode, takeoff, guided goto)
- mission upload/download/clear/set-current
- parameter download and write operations

The crate is designed to be embedded in desktop/mobile apps, CLIs, or backend services.
Python bindings are available via the `mavkit-python` sub-crate (built with PyO3 + maturin).

## Features

- `udp` (default): enable MAVLink UDP transport strings (via `mavlink`)
- `tcp`: enable MAVLink TCP transport strings
- `serial` (default): enable MAVLink direct serial transport strings
- `ardupilot` (default): ArduPilot mode-name mapping helpers
- `stream`: byte-stream transport adapters for BLE/SPP/custom links

Default features: `udp`, `serial`, `ardupilot`.

## Install

```toml
[dependencies]
mavkit = "0.1"
tokio = { version = "1", features = ["macros", "rt-multi-thread"] }
```

## Quick start

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

## Mission wire semantics

For `MissionType::Mission`, MAVLink wire transfer is normalized as:
- upload: a home item is prepended at `seq=0` (or placeholder if missing)
- download: wire `seq=0` is extracted as home and remaining items are resequenced from `0`

For `MissionType::Fence` and `MissionType::Rally`, items pass through unchanged.

These semantics are implemented in:
- `items_for_wire_upload`
- `plan_from_wire_download`

## Python bindings

The `mavkit-python/` sub-crate provides an async-first Python API via PyO3.

### Install

```bash
cd mavkit-python
uv sync            # install dev deps (ruff, etc.)
uv run maturin develop   # build + install into venv
```

### Quick start

```python
import asyncio
import mavkit

async def main():
    vehicle = await mavkit.Vehicle.connect_udp("0.0.0.0:14550")

    state = await vehicle.wait_state()
    print(f"mode={state.mode_name} armed={state.armed}")

    await vehicle.disconnect()

asyncio.run(main())
```

### Python examples

See `mavkit-python/examples/` for full examples mirroring the Rust ones:

```bash
cd mavkit-python
source .venv/bin/activate
python examples/connect_udp.py
```

## Rust examples

- `examples/connect_udp.rs`
- `examples/mission_upload_download.rs`
- `examples/params_roundtrip.rs`

Run an example:

```bash
cargo run --example connect_udp
```

## Development

### Rust

```bash
cargo check
cargo clippy --all-targets --all-features -- -D warnings
cargo test
```

SITL helper targets:

```bash
make bridge-up
make test-sitl
make test-sitl-strict
make bridge-down
```

### Python

```bash
cd mavkit-python
uv run ruff format .
uv run ruff check .
```

## SITL integration testing

Ignored integration tests in `tests/sitl_*.rs` can be run against ArduPilot SITL.

Environment variables:
- `MAVKIT_SITL_UDP_BIND` (default: `0.0.0.0:14550`)
- `MAVKIT_SITL_STRICT` (`1` to fail on strict timeout/unsupported behavior)

Run:

```bash
MAVKIT_SITL_UDP_BIND=0.0.0.0:14550 \
cargo test --tests -- --ignored --nocapture --test-threads=1
```

## Notes

- `Vehicle::from_connection(...)` is the transport-agnostic entry point for custom links.
- `Vehicle::identity()` currently reflects autopilot and vehicle type; system/component ids are not yet exposed from the current state channels.
