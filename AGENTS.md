# AGENTS.md

Operational guide for coding agents working in this repository.
Scope: repository root (`**`).

## Priority Instructions

1. Follow explicit user instructions first.
2. Preserve transport-agnostic behavior (`Vehicle::from_connection`).
3. Preserve mission wire-boundary semantics.
4. Keep changes minimal and targeted.

## Build, Lint, and Test Commands

- `cargo check`
- `cargo clippy --all-targets --all-features -- -D warnings`
- `cargo test`
- `cargo test wire_upload_prepends_home`
- SITL:
  - `make bridge-up`
  - `make test-sitl`
  - `make test-sitl-strict`
  - `make bridge-down`

Useful one-off test commands:

- `cargo test wire_upload_prepends_home -- --exact --nocapture`
- `cargo test --test sitl_roundtrip -- --ignored --nocapture --test-threads=1`
- `MAVKIT_SITL_UDP_BIND=0.0.0.0:14550 cargo test --test sitl_roundtrip -- --ignored --nocapture --test-threads=1`
- `MAVKIT_SITL_STRICT=1 cargo test --test sitl_roundtrip -- --ignored --nocapture --test-threads=1`

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

- Run `cargo clippy --all-targets --all-features -- -D warnings` for code changes.
- Run targeted tests for touched mission/params/event-loop behavior.
- Run SITL for mission-transfer/transport changes when environment is available.
- Keep optional firmware-capability behavior graceful (skip/partial where intended).

## Commit Message Conventions

- Use Conventional Commit style in subject: `<type>: <short summary>`.
- Common types in this repo: `feat`, `fix`, `refactor`, `docs`, `ci`.
- Keep subject concise and imperative; optional body should explain why/value.
- Preferred commit format:
  - Line 1: `<type>(optional-scope): <short imperative summary>`
  - Line 2: blank
  - Line 3+: body paragraphs explaining motivation/impact (not just file list)
  - Final trailer (when agent-assisted): `Co-Authored-By: Name <email>`
- Wrap body lines for readability (roughly ~72-100 chars).
- Match recent history style: short Conventional subject + 1-2 explanatory body paragraphs.
- Include co-author trailer when work is co-developed with an agent:
  - `Co-Authored-By: Name <email>`
  - Examples:
    - `Co-Authored-By: GPT-5.3 Codex <noreply@openai.com>`
    - `Co-Authored-By: Claude Opus 4.6 <noreply@anthropic.com>`
- Match existing trailer capitalization exactly (`Co-Authored-By`).

## Practical Change Guidance

- Prefer additive and targeted edits over broad rewrites.
- Reuse existing mission/params helpers before introducing new abstractions.
- Preserve wire contracts and event-loop ownership boundaries.
- Keep platform/transport optionality behind feature flags (`stream`, transport features).
- Update docs when behavior, feature flags, or developer commands change.
