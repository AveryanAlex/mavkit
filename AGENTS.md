# AGENTS.md

Operational guide for coding agents working in `crates/mavkit`.
Scope: `crates/mavkit/**`.

## Priority Instructions

1. Follow explicit user instructions first.
2. Preserve transport-agnostic behavior (`Vehicle::from_connection`).
3. Preserve mission wire-boundary semantics.
4. Keep changes minimal and targeted.

## Build, Lint, and Test Commands (from repo root)

- `cargo check -p mavkit`
- `cargo clippy -p mavkit --all-targets --all-features -- -D warnings`
- `cargo test -p mavkit`
- `cargo test -p mavkit wire_upload_prepends_home`
- SITL:
  - `make bridge-up`
  - `make test-sitl`
  - `make test-sitl-strict`
  - `make bridge-down`

## Architecture Constraints to Preserve

- `Vehicle` remains `Clone + Send + Sync` via shared inner state.
- Event loop owns recv/send processing and state updates.
- Reactive state is exposed via `tokio::sync::watch` channels.
- Mission APIs stay under `mission`; param APIs stay under `params`.
- Keep optional stream integration behind the `stream` feature.

## Mission Wire Boundary Rules

- For `Mission` type:
  - Upload prepends home at sequence 0.
  - Download extracts sequence 0 into `home` and resequences remaining items.
- For `Fence`/`Rally`:
  - No home insertion/extraction.
  - Items pass through unchanged.

## Rust Style Guidelines

- Use standard `rustfmt` style.
- Keep serde enums shared across boundaries in `snake_case`.
- Keep `MissionItem.x`/`y` in `degE7` (`i32`).
- Prefer `Result<T, VehicleError>` and contextual error messages.
- Avoid broad API reshaping unless requested.

## Testing Expectations

- Run `cargo clippy -p mavkit --all-targets --all-features -- -D warnings` for code changes.
- Run targeted tests for touched mission/params/event-loop behavior.
- Run SITL for mission-transfer/transport changes when environment is available.
- Keep optional firmware-capability behavior graceful (skip/partial where intended).
