# Agent Instructions

This project uses **bd** (beads) for issue tracking. Run `bd onboard` to get started.

## Quick Reference

```bash
bd ready              # Find available work
bd show <id>          # View issue details
bd update <id> --status in_progress  # Claim work
bd close <id>         # Complete work
bd sync               # Sync with git
```

## Project Overview

MAVKit is an async Rust SDK for MAVLink vehicle control. It provides a transport-agnostic `Vehicle` API for telemetry, vehicle actions, missions, and parameters. Python bindings are available via the `mavkit-python` sub-crate (PyO3 + maturin).

## Build & Test Commands

### Rust (workspace root)

```bash
cargo check                                                  # Type check
cargo clippy --all-targets --all-features -- -D warnings     # Lint (must pass)
cargo test                                                   # Unit tests
cargo fmt --all -- --check                                   # Format check
cargo test wire_upload_prepends_home -- --exact --nocapture  # Single test
```

### SITL Integration Tests (requires Docker + ArduPilot SITL)

```bash
make bridge-up                # Start SITL Docker container + MAVProxy bridge
make test-sitl                # Run ignored SITL tests
make test-sitl-strict         # Run with MAVKIT_SITL_STRICT=1
make bridge-down              # Cleanup
```

Run a specific SITL test file:
```bash
cargo test --test sitl_roundtrip -- --ignored --nocapture --test-threads=1
```

Environment: `MAVKIT_SITL_UDP_BIND` (default `0.0.0.0:14550`), `MAVKIT_SITL_STRICT` (set `1` for strict mode).

### Python Bindings (`mavkit-python/`)

```bash
cd mavkit-python && uv sync                        # Install dev deps
cd mavkit-python && uv run maturin develop          # Build extension into venv
cd mavkit-python && uv run ruff format --check .    # Format check
cd mavkit-python && uv run ruff check .             # Lint
cd mavkit-python && uv run python -c "import mavkit" # Smoke test
```

## Architecture

### Command Flow

1. User calls public method on `Vehicle` (e.g., `vehicle.arm()`)
2. Method sends a `Command` to the event loop via `mpsc` channel
3. Event loop (`event_loop.rs`) processes the command, sends MAVLink messages
4. Responses update shared state via `tokio::sync::watch` channels
5. Callers observe changes through watch receivers

### Key Source Modules

- **`src/vehicle.rs`** — Public `Vehicle` API and connection constructors
- **`src/event_loop.rs`** — Core async loop: MAVLink recv/send, command dispatch, state updates
- **`src/state.rs`** — Shared reactive state types (`VehicleState`, `Telemetry`, etc.)
- **`src/mission/`** — Mission types, wire format (home normalization), validation, transfer state machine
- **`src/params/`** — Parameter types and param-file parsing
- **`src/modes.rs`** — ArduPilot mode-name mapping
- **`src/tlog/`** — TLOG file parser: timestamped MAVLink message reader (behind `tlog` feature)
- **`src/ble_transport.rs`**, **`src/stream_connection.rs`** — Stream adapters (behind `stream` feature)

### Python Bindings (`mavkit-python/`)

Uses maturin mixed Rust+Python layout (`python-source = "."` in pyproject.toml):

- **`src/`** — PyO3 wrapper files: one per domain (vehicle, state, mission, params, enums, config, error). Async Rust methods become Python awaitables via `pyo3-async-runtimes`. Lifetime-bound handles (`MissionHandle`, `ParamsHandle`) are flattened onto `PyVehicle`. Watch channels are exposed as sync property getters + async `wait_*` methods.
- **`mavkit/__init__.py`** — Re-exports from the native `.so` module
- **`mavkit/__init__.pyi`** — Hand-written type stubs for IDE support
- **`mavkit/py.typed`** — PEP 561 marker for typed package

## Architecture Constraints

- `Vehicle` must remain `Clone + Send + Sync` (Arc-wrapped inner state)
- Event loop owns all recv/send processing — no direct socket access elsewhere
- State exposure only through `tokio::sync::watch` channels
- Optional transports behind feature flags: `udp`, `tcp`, `serial`, `stream`
- Keep Python bindings in sync when public Rust API changes (update both `mavkit-python/src/*.rs` and `mavkit/__init__.pyi` type stubs)
- Keep examples in sync: each Rust example in `examples/` should have a matching Python example in `mavkit-python/examples/` demonstrating the same functionality

## Mission Wire Boundary Rules

- **Mission type**: upload prepends home at `seq=0`; download extracts `seq=0` as home and resequences remaining items from 0
- **Fence/Rally types**: items pass through unchanged, no home insertion/extraction
- Implemented in `items_for_wire_upload()` and `plan_from_wire_download()`

## Code Style

- Standard `rustfmt`; serde enums in `snake_case`
- `MissionItem.x`/`y` in `degE7` (i32)
- Prefer `Result<T, VehicleError>` with contextual error messages
- Conventional Commits: `<type>(optional-scope): <short summary>` (types: `feat`, `fix`, `refactor`, `docs`, `ci`)
- Python: ruff for formatting and linting

## Versioning

Version must be kept in sync across three manifests:
- `Cargo.toml` → `package.version`
- `mavkit-python/Cargo.toml` → `package.version`
- `mavkit-python/pyproject.toml` → `project.version`

Validate with: `GIT_TAG=v<version> python3 scripts/check_tag_version.py`

## CI & Publishing

- **CI** (`.github/workflows/ci.yml`): `rust` (fmt + clippy + tests with coverage) → `python` and `sitl` run after rust passes
- **Crate publish** (`.github/workflows/publish.yml`): triggered on `v*.*.*` tags, publishes to crates.io
- **PyPI publish** (`.github/workflows/publish-python.yml`): triggered on `v*.*.*` tags, builds wheels for Linux/macOS/Windows and publishes via trusted OIDC

## Landing the Plane (Session Completion)

**When ending a work session**, you MUST complete ALL steps below. Work is NOT complete until `git push` succeeds.

**MANDATORY WORKFLOW:**

1. **File issues for remaining work** - Create issues for anything that needs follow-up
2. **Run quality gates** (if code changed) - Tests, linters, builds
3. **Update issue status** - Close finished work, update in-progress items
4. **PUSH TO REMOTE** - This is MANDATORY:
   ```bash
   git pull --rebase
   bd sync
   git push
   git status  # MUST show "up to date with origin"
   ```
5. **Clean up** - Clear stashes, prune remote branches
6. **Verify** - All changes committed AND pushed
7. **Hand off** - Provide context for next session

**CRITICAL RULES:**
- Work is NOT complete until `git push` succeeds
- NEVER stop before pushing - that leaves work stranded locally
- NEVER say "ready to push when you are" - YOU must push
- If push fails, resolve and retry until it succeeds
