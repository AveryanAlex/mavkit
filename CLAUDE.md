# CLAUDE.md

This file provides guidance to Claude Code when working in `crates/mavkit`.

## Build and Test Commands (from repo root)

```bash
# Compile mavkit
cargo check -p mavkit

# Lint strictly
cargo clippy -p mavkit --all-targets --all-features -- -D warnings

# Unit/integration tests (non-SITL)
cargo test -p mavkit

# Single test
cargo test -p mavkit wire_upload_prepends_home

# SITL integration (requires bridge)
make bridge-up
make test-sitl
make test-sitl-strict
make bridge-down
```

## Architecture

`mavkit` is an async MAVLink SDK.

- `Vehicle` is `Clone + Send + Sync` and wraps shared state/command channels
- Event loop (`event_loop.rs`) owns MAVLink recv/send flow and state updates
- Reactive state is exposed through `tokio::sync::watch` channels
- Mission APIs are under `mission` and use wire-boundary conversions
- Param APIs are under `params` and support download + single/batch writes
- Transport-agnostic entry point: `Vehicle::from_connection()`
- Optional stream adapter: `StreamConnection` (feature `stream`)

## Wire Boundary Convention

Mission wire format carries home at seq 0 for `Mission` type.

- Upload: `items_for_wire_upload()` prepends home and resequences mission items
- Download: `plan_from_wire_download()` extracts seq 0 home and resequences items
- Fence/Rally: pass through without home handling

## Key Patterns

- Keep serde enums shared across boundaries in `snake_case`
- Keep `MissionItem.x/y` as `degE7` (`i32`)
- Preserve transport-agnostic behavior in event loop and `from_connection`
- Prefer targeted changes; avoid reshaping public API unless requested

## Testing Notes

- SITL tests are `#[ignore]` and run with explicit commands
- SITL runs should be single-threaded (`--test-threads=1`)
- For optional mission-type support on target firmware, keep graceful skip behavior
