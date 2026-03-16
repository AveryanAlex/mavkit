# MAVKit Design

## 1. Scope

MAVKit is an async SDK for **MAVLink** vehicle communication.

It should support MAVLink vehicles broadly, but it is intentionally **ArduPilot-first**:

- ArduPilot gets the deepest typed workflows, the strongest SITL coverage, and the most deliberate API shaping.
- PX4 and other MAVLink autopilots should work through the shared MAVLink core where semantics truly match.
- MSP, INAV-over-MSP, Betaflight, and other non-MAVLink protocols are explicitly out of scope.

MAVKit is not a protocol-agnostic flight-controller SDK. It should not be stretched into one.

## 2. Product Goals

The new MAVKit should optimize for five things:

1. **Typed workflows over packet assembly**
   - Users should ask for intent (`arm`, `set_mode`, `upload mission`, `start mag cal`) rather than construct MAVLink packets by hand.

2. **Honest semantics**
   - The API should reflect what MAVLink and ArduPilot actually mean, not a flattened fiction that hides important differences.

3. **Small, domain-oriented root API**
   - `Vehicle` should expose a small number of coherent capability families, not dozens of top-level signal methods.

4. **ArduPilot quirks hidden behind normalization**
   - Downstream users should not think about ArduPilot wire-level oddities like mission slot 0 being home.

5. **Rust and Python parity**
   - Python is not an afterthought. Conceptual parity is required even when syntax differs.

## 3. Non-Goals

MAVKit should not:

- expose raw MAVLink packet structure as the primary API
- become a protocol-neutral SDK spanning MAVLink and non-MAVLink stacks
- force downstream users to understand ArduPilot wire quirks
- promise atomic telemetry snapshots where MAVLink only provides overlapping asynchronous streams
- over-normalize firmware behaviors that are meaningfully different

## 4. Design Principles

### 4.1 Vehicle root stays small

`Vehicle` is the root object and should remain `Clone + Send + Sync`, but the root should expose **domain families**, not every detail directly.

The root should own:

- connection lifecycle
- live identity
- rich vehicle info
- support/capability discovery
- telemetry observation
- missions
- parameters
- raw MAVLink
- command-style state changes that are broadly meaningful across MAVLink autopilots
- firmware-scoped ArduPilot extensions

### 4.2 Semantic APIs above raw protocol APIs

Public domain APIs should be organized by meaning:

- `telemetry().battery().voltage_v()` means the latest operator-facing battery voltage metric
- `telemetry().messages().battery_status(instance)` means coherent `BATTERY_STATUS` samples
- `mission()` means executable mission content
- `raw()` means literal MAVLink operations

The semantic layer is the product. The raw layer is the escape hatch.

### 4.3 Firmware-neutral core, firmware-scoped divergence

The shared MAVLink core should cover domains whose semantics can be expressed honestly across multiple autopilots.

When behavior diverges meaningfully, MAVKit should use firmware scopes instead of pretending everything is universal.

ArduPilot gets first-class typed scopes now. Other MAVLink autopilots use the shared core and raw access until enough stable semantics justify a dedicated scope.

### 4.4 Observation and control stay separate

Telemetry domains observe vehicle state.
Commands change vehicle state.

That boundary should remain explicit.

For example:

- `vehicle.telemetry().home()` observes live reported home
- `vehicle.set_home(...)` changes home

The command should not be smuggled into the telemetry domain.

### 4.5 One generic observation handle, not many bespoke ones

MAVKit exposes many observation handles: scalar metrics, grouped values, message handles, domain state handles, and mode observers.

The implementation should use **one shared generic infrastructure** for all of them rather than hand-rolling each one:

```rust
struct ObservationHandle<T> { /* shared backing state */ }

impl<T: Clone + Send + Sync> ObservationHandle<T> {
    fn latest(&self) -> Option<T>;
    async fn wait(&self) -> T;
    async fn wait_timeout(&self, timeout: Duration) -> Result<T, VehicleError>;
    fn subscribe(&self) -> impl Stream<Item = T>;
}
```

`wait()` blocks until a value is available with no caller-specified timeout. It relies on the vehicle's connection lifecycle: if the vehicle disconnects, the wait resolves with `VehicleError::Disconnected` rather than hanging forever.
`wait_timeout(duration)` adds an explicit caller-controlled deadline and returns `VehicleError::Timeout` if the value does not arrive in time.

For the first rewrite, `wait()` should be the common-path default. Callers who need bounded waits for specific domains (e.g., waiting for GPS fix before proceeding) should use `wait_timeout()`.
Operation handles use the same pattern: `wait()` blocks until terminal completion or link loss, `wait_timeout()` adds a deadline.

`MetricHandle<T>` wraps `ObservationHandle<MetricSample<T>>` and adds `support()`.
`MessageHandle<M>` wraps `ObservationHandle<MessageSample<M>>` and adds `support()`, `request(timeout)`, and optionally `set_rate(hz)`.
Domain state handles such as `MissionState` or `FenceState` use `ObservationHandle<T>` directly.
Operation handles use a parallel generic for progress/completion state.

Each backing store is one shared latest-value slot with watch-style notification fanout.
Calling the same accessor twice should return equivalent handles observing the same underlying store, not independent caches.

This means the implementation cost of adding a new observed quantity is small: define the value type, register the backing store, and wire the inbound message path. The observation surface is free.

### 4.6 Connection configuration

Connection and initialization behavior should be configurable for the first rewrite.

MAVKit should expose a configuration object that can be passed to `connect*()` calls:

```rust
struct VehicleConfig {
    /// Overall connect timeout (waiting for first heartbeat / target selection).
    connect_timeout: Duration,

    /// Default timeout for ACK-driven commands.
    command_timeout: Duration,

    /// Default completion timeout for ACK-plus-observation commands (e.g. set_mode).
    command_completion_timeout: Duration,

    /// Default timeout for mission/param transfer operations.
    transfer_timeout: Duration,

    /// Per-domain initialization policies (override defaults from §5.4.1).
    init_policy: InitPolicyConfig,
}
```

`InitPolicyConfig` should allow overriding the default bounded retry/backoff budgets per initialization domain (autopilot version, available modes, home, origin).

Reasonable defaults should be provided so that `Vehicle::connect_udp(addr)` works without any explicit configuration.
Advanced users may tune timeouts for slow links, disable specific init domains, or adjust retry budgets.

The exact field set may evolve, but the principle is: one config object at connect time, reasonable defaults, and per-domain overrides where needed.

## 5. Public API Shape

### 5.1 Vehicle root

The root should look conceptually like this:

```rust
vehicle.identity()
vehicle.info()
vehicle.support()
vehicle.available_modes()

vehicle.telemetry()
vehicle.mission()
vehicle.fence()
vehicle.rally()
vehicle.params()
vehicle.raw()

vehicle.arm(force)
vehicle.disarm(force)
vehicle.set_mode(custom_mode)
vehicle.set_mode_by_name(name)

vehicle.set_home(...)
vehicle.set_home_current()
vehicle.set_origin(...)
vehicle.disconnect()

vehicle.ardupilot()

vehicle.camera()
vehicle.gimbal()
vehicle.logs()
vehicle.ftp()
```

Not every domain above needs to ship in the first slice, but this is the intended organization.

### 5.2 Domain handles

Each major domain should be represented by a small handle that borrows from the vehicle and exposes a focused API.

Examples:

```rust
vehicle.telemetry()  -> TelemetryHandle
vehicle.available_modes() -> ModesHandle
vehicle.info()       -> InfoHandle
vehicle.support()    -> SupportHandle
vehicle.mission()    -> MissionHandle
vehicle.fence()      -> FenceHandle
vehicle.rally()      -> RallyHandle
vehicle.params()     -> ParamsHandle
vehicle.raw()        -> RawHandle
vehicle.ardupilot()  -> ArduPilotHandle
```

This keeps the root readable and lets each domain evolve without bloating `Vehicle`.

### 5.3 Firmware scopes

ArduPilot-specific behavior should live under `vehicle.ardupilot()`.

Guided control is the main case where the first rewrite uses one explicit session entrypoint:

```rust
vehicle.ardupilot()
vehicle.ardupilot().guided()
```

For non-guided workflows that truly diverge by ArduPilot vehicle family, MAVKit may still expose class-scoped handles under that firmware namespace:

```rust
vehicle.ardupilot().copter()
vehicle.ardupilot().plane()
vehicle.ardupilot().plane().vtol()
vehicle.ardupilot().rover()
vehicle.ardupilot().sub()
```

Examples of behavior that belongs there:

- guided/offboard-style control sessions via `vehicle.ardupilot().guided()`
- ArduPilot-specific calibration workflows
- motor test
- vehicle-class-specific takeoff / land / RTL semantics when they are not modeled as guided sessions
- ArduPilot-specific quirks and helper methods

If MAVKit later gains enough stable PX4-specific semantics, those should appear under a parallel firmware scope, not by weakening the root.

ArduPilot VTOL Plane (`QuadPlane`) should be modeled as an optional extension of the normal plane family handle, not as a separate top-level firmware family.
It is still ArduPlane with additional VTOL-specific semantics, not a sibling of Copter/Plane/Rover/Sub.

The important boundary is this:

- guided access remains `vehicle.ardupilot().guided()` with explicit runtime narrowing
- direct class-scoped handles are reserved for other ArduPilot-first workflows that truly need family-specific API surface
- any plane-family VTOL surface remains `plane.vtol() -> Option<_>`, not a sibling branch

### 5.4 Connection-ready vs initialization-ready

Connection and initialization should be distinct lifecycle concepts.

`Vehicle::connect*()` should return a usable `Vehicle` when MAVKit is **connection-ready**:

- transport/session is open
- the event loop is running
- a target system/component has been selected
- the first `HEARTBEAT` or equivalent target confirmation has been received

This is the right default contract because it gives downstream code a real vehicle handle quickly without tying startup latency to optional metadata round trips.

At that point, MAVKit should guarantee only the heartbeat-level facts that are genuinely available from the live link:

- `system_id`
- `component_id`
- `autopilot`
- `vehicle_type`
- heartbeat-derived mode state sufficient to seed the active-mode observer

Everything richer should be treated as **initialization-time enrichment**, not as a connection precondition.

That includes:

- `AUTOPILOT_VERSION`
- `AVAILABLE_MODES` / `AVAILABLE_MODES_MONITOR`
- `HOME_POSITION`
- `GPS_GLOBAL_ORIGIN`
- `COMPONENT_METADATA`
- other firmware-specific metadata and support discovery

MAVKit should start this initialization work eagerly in the background right after connect, with bounded retry/backoff where appropriate.

Failure to fetch richer metadata must not fail the connection.
Instead, the relevant domain should remain `Unknown` or become `Unavailable` based on actual evidence.

The design should keep **one `Vehicle` type**, not separate `Connection` and `InitializedVehicle` object types.
Readiness is per-domain, not global.

If a downstream workflow truly needs a metadata subset before proceeding, MAVKit may offer explicit wait helpers on the relevant domains and, if needed later, an optional composite helper such as `vehicle.wait_initialized(profile, timeout)`, where `profile` is an SDK-defined named set of domain-readiness requirements.
That would be a convenience wrapper over domain readiness, not the default meaning of `connect*()`.

#### 5.4.1 Initialization state machine

Initialization-backed domains should follow one explicit state machine:

- `Unknown` — no terminal conclusion yet
- `Requesting { attempt }` — MAVKit has an active or scheduled request in flight
- `Available(T)` — domain data is known
- `Unavailable { reason }` — MAVKit has bounded evidence that this domain is not available right now

For the first rewrite, MAVKit should use a bounded default eager-init policy per domain:

1. request immediately after connection-ready
2. retry on silence with bounded backoff
3. use a bounded budget appropriate to the domain rather than one global timeout
4. transition to `Unavailable` only after explicit negative evidence or exhausting that domain's bounded policy

Initial default policy table:

| Domain | First action | Default bounded policy | Terminal interpretation |
|---|---|---|---|
| `AUTOPILOT_VERSION` / richer info | eager request | 3 attempts over roughly the first 5 seconds | silence after budget → `Unavailable` |
| `AVAILABLE_MODES` | eager request | 2 attempts over roughly the first 5 seconds | silence after budget → keep static fallback mode table |
| `HOME_POSITION` | eager bootstrap request | request during init, then remain event-driven | silence does **not** mean unsupported; home may simply not be set yet |
| `GPS_GLOBAL_ORIGIN` | eager bootstrap request | request during init, then remain event-driven | silence does **not** mean unsupported; EKF origin may not exist yet |

The public API does not need to expose `Requesting` everywhere if that would be noisy, but the implementation must behave as if this state exists.
`Unknown` must mean “not yet concluded,” not “unsupported.”

Explicit user-triggered requests later may transition a domain from `Unavailable` back to `Requesting`.

#### 5.4.2 Link loss, reconnect, and target topology

For the first rewrite, one `Vehicle` instance should represent **one selected primary remote target**.

- `connect*()` binds to one `(system_id, component_id)` target chosen during connection setup
- multiple autopilot systems on one transport are out of scope for one `Vehicle` handle in the first rewrite
- typed command APIs are scoped to that selected target component
- other component traffic on the same link may still be consumed by typed domains only when the semantics clearly belong to the same vehicle; otherwise it belongs in `raw()` or future dedicated component APIs

If more than one plausible primary autopilot target is visible on the link during connection setup, the first rewrite should prefer an explicit selection API or fail the connection rather than guessing silently.

Link-loss behavior should be explicit:

- no implicit auto-reconnect in the first rewrite
- a dropped link transitions the vehicle into a disconnected/error lifecycle state
- last known observed values remain queryable as last-known state until a new `Vehicle` is created
- in-flight mission/param operations terminate with connection-related terminal errors
- in-flight ACK-driven commands terminate as **outcome unknown** if the command may already have been sent before the link was lost

Explicit reconnect should create a new `Vehicle` instance rather than mutating an existing disconnected one back into a connected handle.

## 6. Identity, Info, and Support

### 6.1 Identity

Identity is the set of live, common facts available from normal routing and heartbeat traffic.

`VehicleIdentity` should include:

- `system_id`
- `component_id`
- `autopilot`
- `vehicle_type`

This is live and lightweight. It belongs on the root and in live state.
It should be populated by the connection-ready handshake and available immediately when `connect*()` returns.
`vehicle.identity()` should be a synchronous current-value accessor returning `VehicleIdentity`, not a long-lived observation handle in the first rewrite.
For one selected target, these fields are expected to remain stable for the lifetime of one `Vehicle`; reconnect creates a new `Vehicle` instead of mutating identity in place.

### 6.2 Info

`vehicle.info()` should expose richer, slower, often sparse metadata.

This should be grouped, not flattened into many unrelated getters.

`info()` is intentionally allowed to be partial at first.
MAVKit should begin enriching it eagerly after connect, but the absence of rich metadata at construction time is normal and should not be treated as a connect failure.

Concrete shape for the first rewrite:

```rust
impl InfoHandle<'_> {
    fn firmware(&self) -> ObservationHandle<FirmwareInfo>;
    fn hardware(&self) -> ObservationHandle<HardwareInfo>;
    fn unique_ids(&self) -> ObservationHandle<UniqueIds>;

    /// Best hardware-derived unique identifier, or None.
    fn best_unique_id(&self) -> Option<String>;

    /// Best available identifier string with source prefix for display/logging.
    fn best_display_id(&self) -> String;

    /// Storage-grade persistent identity.
    fn persistent_identity(&self) -> ObservationHandle<PersistentIdentity>;
}

struct FirmwareInfo {
    /// Semantic version if parseable, otherwise raw version number.
    version: Option<String>,
    /// Custom firmware version field (e.g. ArduPilot custom_version).
    custom_version: Option<[u8; 8]>,
    /// Git hash of the firmware build, if available.
    git_hash: Option<String>,
    /// OS/board software version string, if available.
    os_version: Option<String>,
}

struct HardwareInfo {
    /// Board vendor ID.
    board_vendor_id: Option<u16>,
    /// Board product ID.
    board_product_id: Option<u16>,
    /// USB vendor ID.
    usb_vendor_id: Option<u16>,
    /// USB product ID.
    usb_product_id: Option<u16>,
    /// Board version / revision.
    board_version: Option<u32>,
}

struct UniqueIds {
    /// Primary hardware UID (uid2 if available, else uid).
    hardware_uid: Option<Vec<u8>>,
    /// Legacy 64-bit UID.
    uid: Option<u64>,
    /// Regulatory UAS / Remote ID identifier.
    remote_id: Option<String>,
    /// Board/model identifier (not per-unit unique).
    board_id: Option<String>,
}

enum PersistentIdentity {
    /// Strong identity not yet discovered. Session locator available for provisional use.
    Pending {
        system_id: u8,
        component_id: u8,
    },
    /// Strong durable identifier discovered.
    Ready {
        /// Strongest available identifier with source prefix (e.g. "mcu:00112233...").
        canonical_id: String,
        /// All discovered strong identifiers with source prefixes.
        aliases: Vec<String>,
    },
}
```

All info sub-handles use the generic `ObservationHandle<T>` infrastructure.
Each starts as `None` and populates asynchronously as `AUTOPILOT_VERSION` and other metadata arrives.
`best_unique_id()` and `best_display_id()` are synchronous convenience helpers that read from the current info state.

Backends should populate it from:

- `AUTOPILOT_VERSION`
- `COMPONENT_METADATA` when available
- `OPEN_DRONE_ID_*` when present

Examples of data that belong here:

- firmware version/build/hash
- board version
- USB vendor/product IDs
- `uid2`
- component metadata
- regulatory UAS ID / Remote ID

This is best-effort metadata, not always-present identity.
It is also not connection-gating metadata.

#### 6.2.1 Unique identifier policy

Unique identifiers need a stricter contract than general metadata.

MAVKit should expose them under:

- `info().unique_ids()`

This group should contain distinct identifier families rather than flattening everything into one string:

- `hardware_uid` — the best hardware-derived identifier available from MAVLink metadata
- `remote_id` — the regulatory UAS identifier when Open Drone ID is present
- `board_id` — board/model identity (not per-unit unique)
- `usb_ids` — vendor/product IDs (not per-unit unique)

For MAVLink/ArduPilot, `hardware_uid` should prefer:

1. `AUTOPILOT_VERSION.uid2` when present and non-zero
2. legacy `AUTOPILOT_VERSION.uid` when present and non-zero

`system_id` and `component_id` should never be treated as unique IDs. They belong to live routing identity only.

The group should distinguish:

- `Unknown` — metadata has not been fetched yet, or no explicit attempt has been made
- `Unavailable` — MAVKit explicitly requested or checked, and the vehicle does not provide this identifier family
- `Available(UniqueIds)` — the group is available, even if some leaf fields are still `None`

Inside `UniqueIds`, individual fields may still be optional because many vehicles only provide part of the full set.

MAVKit should provide two helper access patterns:

1. **Structured access** — callers inspect concrete fields and sources directly
2. **Best-effort helper** — callers ask for the best available identifier for display or caching

The best-effort helper should not be named `unique_id()` unless it can guarantee uniqueness. A safer shape is:

- `best_unique_id()` — returns only truly unique-style identifiers, or `None`
- `best_display_id()` — returns the best available identifier string with an explicit source prefix

Suggested precedence for `best_unique_id()`:

1. `uid2`
2. `uid`
3. regulatory UAS ID / Remote ID

Suggested precedence for `best_display_id()`:

1. `uid2`
2. `uid`
3. regulatory UAS ID / Remote ID
4. board ID
5. USB vendor/product tuple
6. `sysid` / `compid` as a last-resort display fallback only

The string form should always include a source prefix so users and logs do not confuse different identifier classes.

Examples:

- `mcu:00112233445566778899aabb`
- `uid64:0123456789abcdef`
- `rid:ABC123DEF456...`
- `board:apj-140`
- `usb:1209:5741`
- `sys:1/1`

This keeps the API honest: callers who need precision can use structured access, while callers who just need a stable label can use the helper without forgetting where the identifier came from.

#### 6.2.2 Persistence and long-term storage keys

The best currently known identifier is not automatically a safe long-term storage key.

MAVKit should distinguish two identity layers for downstream persistence:

1. **Session locator** — immediately available, connection-scoped, useful for live routing and provisional matching
2. **Persistent identity** — storage-grade, only available when a strong identifier has actually been discovered

The session locator should be built from:

- connection/discovery scope
- `system_id`
- `component_id`

This is useful for live session bookkeeping and provisional in-memory maps, but it must not be treated as globally durable.

Persistent identity should be derived only from strong identifiers, with this precedence:

1. `uid2`
2. `uid`
3. Remote ID / UAS ID only when MAVKit can document it as stable enough for that vehicle family

MAVKit should therefore expose an explicit storage-oriented helper, distinct from display helpers.

Suggested shape:

- `info().persistent_identity()`
- `info().wait_for_persistent_identity(timeout)`

The state model should be explicit:

- `Pending { session_locator }`
- `Ready { canonical_id, aliases }`

Where:

- `canonical_id` is the strongest available durable identifier, namespaced with its source
- `aliases` contains all strong identifiers discovered so far, also namespaced

Callers should not store long-lived records under `best_display_id()`.
They also should not store long-lived records under the current return value of `best_unique_id()` without accepting that it may improve later.

The recommended downstream storage pattern is:

1. application creates its own immutable local record ID
2. MAVKit provides session locator immediately
3. caller optionally waits for persistent identity for a bounded time
4. when a persistent identity appears, caller attaches it as the canonical alias for that local record
5. if a stronger ID appears later, caller upgrades/merges aliases rather than changing the local record key

This prevents key churn and duplicate-device records while still allowing early use of the connection before rich metadata arrives.
Persistent identity is therefore an initialization concern, not a connect precondition.

### 6.3 Support

`vehicle.support()` should answer whether a capability family is:

- `Unknown`
- `Supported`
- `Unsupported`

Concrete shape for the first rewrite:

```rust
enum SupportState {
    Unknown,
    Supported,
    Unsupported,
}

impl SupportHandle<'_> {
    fn command_int(&self) -> ObservationHandle<SupportState>;
    fn ftp(&self) -> ObservationHandle<SupportState>;
    fn terrain(&self) -> ObservationHandle<SupportState>;
    fn mission_fence(&self) -> ObservationHandle<SupportState>;
    fn mission_rally(&self) -> ObservationHandle<SupportState>;
    fn ardupilot(&self) -> ObservationHandle<SupportState>;
}
```

This should be used for:

- protocol capability families (`COMMAND_INT`, FTP, terrain, mission fence, mission rally)
- firmware-scoped availability
- major telemetry and control domains where the answer matters before waiting forever

Each child accessor is a lightweight `ObservationHandle<SupportState>` with the usual `latest()`, `wait()`, `wait_timeout()`, and `subscribe()` surface.
`SupportHandle` is the root capability-family registry.

Support state should not be silently inferred from channel existence.
It also should not collapse `Unknown` into `Unsupported` just because initialization has not finished yet.

Support discovery should run eagerly after connect, but a quiet or partial vehicle must still produce a usable `Vehicle` handle with support domains left `Unknown` until evidence arrives.

Support conclusions should be evidence-based:

- `Supported` from explicit capability bits, explicit positive protocol evidence, or observed successful use
- `Unsupported` from explicit negative capability bits, explicit negative protocol responses such as command rejection for unsupported behavior, or exhausting a bounded init policy for a capability-specific probe
- `Unknown` while there is still no terminal evidence either way

For data-bearing handles, capability support and current data availability are distinct.
A handle may be `Supported` while `latest()` is still `None` because the vehicle has not yet reported a current value, or because that value does not presently exist.
Silence alone must not turn data absence into `Unsupported` unless that specific capability probe truly produced negative evidence.

Default observation-handle rule:

- `handle.support()` answers whether MAVKit has evidence that this concept/message family is meaningful for the current vehicle/session
- `Supported` may come from family-level capability knowledge, vehicle-class semantics, or successful observation of the backing message family
- `Unsupported` requires explicit negative evidence or an explicit vehicle-class/protocol exclusion, not just missing current data
- `Unknown` remains the correct state while MAVKit still lacks terminal evidence either way

Nested observers under one family handle may inherit that family support rather than carrying their own separate support states.
This is distinct from the root `SupportHandle`: `vehicle.support()` answers coarse protocol/capability-family questions, while `some_handle.support()` is a convenience view on whether that specific observation concept is meaningful for the current vehicle/session.

## 7. Modes

Flight modes should be treated as a dedicated capability family, not as a rootless string lookup hack.

`vehicle.available_modes()` should return a dedicated modes handle, not a bare list.

Concrete shape for the first rewrite:

```rust
impl ModesHandle<'_> {
    fn support(&self) -> ObservationHandle<SupportState>;
    fn catalog(&self) -> ObservationHandle<Vec<ModeDescriptor>>;
    fn current(&self) -> ObservationHandle<CurrentMode>;
}
```

Usage:

```rust
let modes = vehicle.available_modes(); // ModesHandle

modes.support().latest()

modes.catalog().latest()
modes.catalog().wait()
modes.catalog().subscribe()

modes.current().latest()
modes.current().wait()
modes.current().subscribe()
```

Value types:

```rust
enum ModeCatalogSource {
    AvailableModes,
    StaticArduPilotTable,
}

struct ModeDescriptor {
    custom_mode: u32,
    name: String,
    user_selectable: bool,
    source: ModeCatalogSource,
}

enum CurrentModeSource {
    Heartbeat,
    CurrentModeMessage,
}

struct CurrentMode {
    custom_mode: u32,
    name: String,
    intended_custom_mode: Option<u32>,
    source: CurrentModeSource,
}
```

`catalog()` should be an observation handle over `Vec<ModeDescriptor>`.
`current()` should be an observation handle over `CurrentMode`.
In the first rewrite, these observers may return their value types directly rather than wrapping them in `MetricSample<T>`.
That means the minimal contract is:

```rust
catalog().latest() -> Option<Vec<ModeDescriptor>>
catalog().wait() -> Vec<ModeDescriptor>
catalog().subscribe() -> impl Stream<Item = Vec<ModeDescriptor>>

current().latest() -> Option<CurrentMode>
current().wait() -> CurrentMode
current().subscribe() -> impl Stream<Item = CurrentMode>
```

These are not telemetry metrics, so they do not need to reuse `MetricSample<T>`.
`modes.support()` is the family-level support state for the modes domain; `catalog()` and `current()` do not need separate independent support states in the first rewrite.

This keeps two distinct observation surfaces explicit:

- the **catalog** of selectable modes known for the current vehicle
- the **current mode** the vehicle is presently reporting

The current mode is not part of `identity()` and does not need to live under `telemetry()` in the first rewrite.
It belongs to the dedicated modes family because users usually want the current mode together with the catalog and its source metadata.
If the current `custom_mode` is not present in the current catalog, `CurrentMode.name` should fall back to a deterministic display string such as `MODE(<custom_mode>)` rather than becoming empty.

MAVKit should:

1. Prefer `AVAILABLE_MODES` for mode enumeration when the vehicle supports it.
2. Use `AVAILABLE_MODES_MONITOR` to detect dynamic mode-list changes and trigger re-enumeration.
3. Use `HEARTBEAT.custom_mode` as the canonical current-mode source for ArduPilot-first operation.
4. Treat `CURRENT_MODE` as auxiliary metadata when it is present, not as a connect prerequisite.
5. Fall back to known ArduPilot mode tables when `AVAILABLE_MODES` is unavailable.
6. Represent modes semantically with:
   - numeric custom mode
   - display name
   - user-selectable flag
   - optional source metadata

Mode discovery should not block `connect*()`.

For ArduPilot, MAVKit should be able to provide a useful catalog immediately from known firmware/type mode tables once heartbeat-level identity is known, then replace or refine that catalog if `AVAILABLE_MODES` arrives.
For ArduPilot-first operation, that means `catalog().latest()` should be populated immediately after a heartbeat-identified connection is returned, using the static table first and refining it later if `AVAILABLE_MODES` arrives.

Concrete ArduPilot-first rule:

- request `AVAILABLE_MODES` during initialization
- if it succeeds, cache that enumerated list
- if it times out or is rejected, keep the static ArduPilot mode table
- if `AVAILABLE_MODES_MONITOR` later signals a sequence change, re-request the list
- do not wait for a `CURRENT_MODE` message because ArduPilot mode tracking is still driven by `HEARTBEAT.custom_mode`
- if `CURRENT_MODE` is present later, MAVKit may record it as auxiliary/intended-mode metadata, but it does not replace the ArduPilot-first active-mode source

`vehicle.available_modes()` can remain a convenient root method because mode choice is central enough to justify it, but it should still return a dedicated handle rather than a one-shot list.

## 8. Telemetry Model

### 8.1 telemetry() is the canonical observation family

All observed flight state should hang off `vehicle.telemetry()`.

It should expose **two observation surfaces**:

1. **Metric namespaces** — the default, operator-facing API
2. **`messages()`** — source-faithful message-level telemetry

The public split should be explicit:

- use `telemetry().battery().voltage_v()` when you want the latest available value of a quantity
- use `telemetry().messages().battery_status(instance)` when you care about exact message coherence

The important boundary is this:

- **metrics are per-quantity and may merge multiple message families**
- **messages are coherent decoded MAVLink message samples**

### 8.2 Metric namespaces are the primary telemetry API

`vehicle.telemetry()` should default to **metric-oriented access**.

Metric namespaces are just namespaces. They are not atomic domain snapshots and they are not alternate packet-shaped views.
Each terminal scalar metric or grouped metric entry is an observation handle for one operator-facing quantity or one operator-facing grouped concept.
Intermediates such as `battery()`, `position()`, `gps()`, `navigation()`, `terrain()`, `rc()`, and `actuators()` are lightweight namespace handles only.
They do not need their own independent `support()` or observation state in the first rewrite; leaf/grouped handles carry the actual observation contract.

Suggested public shape:

```rust
vehicle.telemetry().messages()

vehicle.telemetry().position().global()
vehicle.telemetry().position().groundspeed_mps()
vehicle.telemetry().position().airspeed_mps()
vehicle.telemetry().position().climb_rate_mps()
vehicle.telemetry().position().heading_deg()
vehicle.telemetry().position().throttle_pct()

vehicle.telemetry().attitude().euler()
vehicle.telemetry().armed()

vehicle.telemetry().battery().remaining_pct()
vehicle.telemetry().battery().voltage_v()
vehicle.telemetry().battery().current_a()
vehicle.telemetry().battery().energy_consumed_wh()
vehicle.telemetry().battery().time_remaining_s()
vehicle.telemetry().battery().cells()

vehicle.telemetry().gps().quality()
vehicle.telemetry().gps().position_msl()

vehicle.telemetry().navigation().waypoint()
vehicle.telemetry().navigation().guidance()

vehicle.telemetry().terrain().clearance()

vehicle.telemetry().rc().channel_pwm_us(index)
vehicle.telemetry().rc().rssi_pct()

vehicle.telemetry().actuators().servo_pwm_us(index)

vehicle.telemetry().sensor_health()
vehicle.telemetry().home()
vehicle.telemetry().origin()
```

This list is the full initial telemetry surface inventory for the rewrite: metric namespaces, concept handles, and the advanced `messages()` branch.
If a future quantity matters to users, it should be added as another explicit metric handle rather than reviving a second family of merged per-domain sample objects.

### Exact grouped value objects

The metric API should use real typed grouped value objects only where the operator concept is inherently composite and awkward or misleading when exposed only as unrelated scalars.

Recommended grouped value objects:

| Path | Value type | Fields on grouped value | Why it is grouped |
|---|---|---|---|
| `telemetry().position().global()` | `GlobalPosition` | `latitude_deg`, `longitude_deg`, `altitude_msl_m`, `relative_alt_m` | one fused global position sample with both useful altitude references |
| `telemetry().attitude().euler()` | `EulerAttitude` | `roll_deg`, `pitch_deg`, `yaw_deg` | attitude is naturally a 3-axis orientation |
| `telemetry().gps().quality()` | `GpsQuality` | `fix_type`, `satellites`, `hdop` | these three together define whether GPS is usable |
| `telemetry().gps().position_msl()` | `GeoPoint3dMsl` | `latitude_deg`, `longitude_deg`, `altitude_msl_m` | raw GPS position is naturally one 3D MSL point |
| `telemetry().battery().cells()` | `CellVoltages` | `voltages_v` | per-cell voltages are one coherent cell bank |
| `telemetry().navigation().waypoint()` | `WaypointProgress` | `distance_m`, `bearing_deg` | one operator concept: where the waypoint is relative to the vehicle |
| `telemetry().navigation().guidance()` | `GuidanceState` | `bearing_deg`, `cross_track_error_m` | one operator concept: how the controller is guiding back onto path |
| `telemetry().terrain().clearance()` | `TerrainClearance` | `terrain_height_m`, `height_above_terrain_m` | terrain and clearance are only meaningful together |
| `telemetry().home()` | `GeoPoint3dMsl` | `latitude_deg`, `longitude_deg`, `altitude_msl_m` | home is one MSL-based reference point |
| `telemetry().origin()` | `GeoPoint3dMsl` | `latitude_deg`, `longitude_deg`, `altitude_msl_m` | EKF origin is one MSL-based reference point |
| `telemetry().sensor_health()` | `SensorHealthSummary` | `gyro`, `accel`, `mag`, `baro`, `gps`, `airspeed`, `rc_receiver`, `battery`, `terrain`, `geofence` (each `SensorHealthState`) | health only makes sense as a grouped summary |

Everything else should remain a scalar/enum metric leaf under the appropriate namespace.
Notable scalar leaf examples include `armed() -> bool`, `voltage_v() -> f32`, and `rssi_pct() -> u8`.

For sensor health, the first rewrite should use:

```rust
enum SensorHealthState {
    NotPresent,
    Disabled,
    Unhealthy,
    Healthy,
}

struct SensorHealthSummary {
    gyro: SensorHealthState,
    accel: SensorHealthState,
    mag: SensorHealthState,
    baro: SensorHealthState,
    gps: SensorHealthState,
    airspeed: SensorHealthState,
    rc_receiver: SensorHealthState,
    battery: SensorHealthState,
    terrain: SensorHealthState,
    geofence: SensorHealthState,
}
```

Important distinction:

- the **grouped handle** exposes grouped observation methods only
- the **grouped value type** is plain data only

For example, `GlobalPosition` is just:

```rust
struct GlobalPosition {
    latitude_deg: f64,
    longitude_deg: f64,
    altitude_msl_m: f64,
    relative_alt_m: f64,
}
```

It should not have methods like `latitude_deg()` or `subscribe()` on the value itself.
And the grouped handle should not expose per-field metric accessors either.
If a caller wants one field from `position().global()`, they read it from the returned `GlobalPosition` sample value.

For 2D global points, the first rewrite should use:

```rust
struct GeoPoint2d {
    latitude_deg: f64,
    longitude_deg: f64,
}
```

For MSL-based 3D reference points, the first rewrite should use an explicitly named type:

```rust
struct GeoPoint3dMsl {
    latitude_deg: f64,
    longitude_deg: f64,
    altitude_msl_m: f64,
}
```

This is appropriate for telemetry concepts like `home()` and `origin()`.
It can also be reused when a guided command really is an MSL-based 3D point target, such as plane-family reposition.

For relative-home 3D targets, the first rewrite should use:

```rust
struct GeoPoint3dRelHome {
    latitude_deg: f64,
    longitude_deg: f64,
    relative_alt_m: f64,
}
```

For terrain-relative 3D targets, the first rewrite should use:

```rust
struct GeoPoint3dTerrain {
    latitude_deg: f64,
    longitude_deg: f64,
    altitude_terrain_m: f64,
}
```

This is appropriate for rally points and mission items that use `MAV_FRAME_GLOBAL_TERRAIN_ALT` or similar terrain-relative altitude references.

For contexts where any altitude reference is valid (rally points, mission waypoints, guided targets), the first rewrite should use a shared altitude-polymorphic 3D point:

```rust
enum GeoPoint3d {
    /// MAV_FRAME_GLOBAL — altitude above mean sea level.
    Msl(GeoPoint3dMsl),
    /// MAV_FRAME_GLOBAL_RELATIVE_ALT — altitude relative to home.
    RelHome(GeoPoint3dRelHome),
    /// MAV_FRAME_GLOBAL_TERRAIN_ALT — altitude above terrain.
    Terrain(GeoPoint3dTerrain),
}
```

`GeoPoint3d` is the single shared type used wherever a 3D geographic point with variable altitude reference appears in the SDK: rally points, mission item positions, and any other context that accepts all three altitude frames.
Specific concrete types (`GeoPoint3dMsl`, `GeoPoint3dRelHome`, `GeoPoint3dTerrain`) remain available for contexts where only one altitude frame is semantically valid (e.g. telemetry `home()` is always MSL, copter guided `goto()` is always relative-home).

For fused global position ergonomics, the design should support direct conversion from `GlobalPosition`:

```rust
impl From<GlobalPosition> for GeoPoint2d
impl From<GlobalPosition> for GeoPoint3dMsl
impl From<GlobalPosition> for GeoPoint3dRelHome
impl From<GeoPoint3dMsl> for GeoPoint2d
impl From<GeoPoint3dRelHome> for GeoPoint2d
impl From<GeoPoint3dTerrain> for GeoPoint2d
impl From<GeoPoint3d> for GeoPoint2d
```

These conversions are projections of the same fused sample, not separate live telemetry handles.

### Metric handle shape

For a scalar metric leaf handle, the preferred shape is:

```rust
let voltage = vehicle.telemetry().battery().voltage_v();

voltage.support()
voltage.latest()
voltage.wait()
voltage.subscribe()
```

The returned sample should carry provenance explicitly:

```rust
struct MetricSample<T> {
    value: T,
    source: TelemetryMessageKind,
    vehicle_time: Option<VehicleTimestamp>,
    received_at: std::time::Instant,
}
```

For grouped value objects, that provenance belongs to the grouped sample as a whole.
The first rewrite does **not** expose per-field provenance or per-field timestamps inside grouped values.
That means a grouped handle should only publish samples that can honestly report one `source` and one `vehicle_time` for the whole grouped value.

`TelemetryMessageKind` should identify the telemetry-relevant message family behind a metric refresh, grouped sample, or message-layer event/handle, such as `SysStatus`, `BatteryStatus`, `GlobalPositionInt`, `GpsRawInt`, `VfrHud`, `Attitude`, or `StatusText`.

`TelemetryMessageKind` should be a real public SDK enum with one variant per telemetry-relevant message family used by the metric layer, the message layer, or both.
Initial inventory:

- `Heartbeat`
- `VfrHud`
- `GlobalPositionInt`
- `LocalPositionNed`
- `GpsRawInt`
- `Attitude`
- `SysStatus`
- `BatteryStatus`
- `NavControllerOutput`
- `TerrainReport`
- `RcChannels`
- `ServoOutputRaw`
- `HomePosition`
- `GpsGlobalOrigin`
- `StatusText`

### Vehicle timestamps

`VehicleTimestamp` should be an explicit SDK type rather than an undocumented alias.

Suggested shape:

```rust
enum VehicleTimestamp {
    BootTime(std::time::Duration),
    UnixEpochMicros(u64),
}
```

Rules:

- use `BootTime` when the source message carries time since boot
- use `UnixEpochMicros` only when the source message really carries an absolute epoch-style timestamp
- use `None` when the message family has no trustworthy vehicle-origin timestamp

`received_at` remains the local monotonic receive time inside MAVKit and is always distinct from `VehicleTimestamp`.

Initial mapping for the first rewrite:

| `TelemetryMessageKind` | Vehicle timestamp mapping |
|---|---|
| `Heartbeat` | `None` |
| `VfrHud` | `None` |
| `GlobalPositionInt` | `BootTime(Duration::from_millis(time_boot_ms))` |
| `LocalPositionNed` | `BootTime(Duration::from_millis(time_boot_ms))` |
| `GpsRawInt` | infer from `time_usec` magnitude; on ArduPilot this is usually Unix-epoch time once GPS time is available |
| `Attitude` | `BootTime(Duration::from_millis(time_boot_ms))` |
| `SysStatus` | `None` |
| `BatteryStatus` | `None` |
| `NavControllerOutput` | `None` |
| `TerrainReport` | `None` |
| `RcChannels` | `None` |
| `ServoOutputRaw` | `None` |
| `HomePosition` | infer from `time_usec` magnitude; `0` or placeholder values → `None` |
| `GpsGlobalOrigin` | infer from `time_usec` magnitude; `0` or placeholder values → `None` |
| `StatusText` | `None` |

When `time_usec` is used, MAVKit should infer epoch vs boot-relative time by magnitude exactly as MAVLink specifies.
`0` and other obviously placeholder values should be treated as `None`, not as `BootTime(0)`.
If the value is ambiguous or obviously unreliable for the active firmware family, the SDK should prefer `None` over pretending to know.

`received_at` is the only universally comparable ordering key across message families.
Source timestamps are only comparable when they clearly share the same time basis.

### Metric handle identity and backing stores

The design should define one canonical backing store per exposed observation handle.

- each scalar metric handle has one shared latest-value slot plus notification fanout
- each grouped value object has one shared latest-value slot for the grouped object itself
- each message handle has one shared latest-value slot for that decoded message family

`MetricHandle<T>`, grouped value object handles, message handles, and operation handles should all be cheap cloneable wrappers over shared backing state.
Calling the same accessor twice should return equivalent handles observing the same underlying store, not independent caches.

For grouped value objects such as `position().global()` or `gps().quality()`:

- the grouped store is first-class because the concept is first-class
- there are no separate per-field metric handles under the grouped object in the first rewrite
- subfield access is done by reading fields from the grouped value returned by `latest()`, `wait()`, or `subscribe()`

This means `position().global()` is the only telemetry handle for that grouped concept.

For a grouped value object handle, the preferred shape is:

```rust
let global_handle = vehicle.telemetry().position().global();

global_handle.support()
global_handle.latest()      // Option<MetricSample<GlobalPosition>>
global_handle.wait()
global_handle.subscribe()

let sample = global_handle.latest();
let lat = sample.as_ref().map(|s| s.value.latitude_deg);
let lon = sample.as_ref().map(|s| s.value.longitude_deg);
let rel = sample.as_ref().map(|s| s.value.relative_alt_m);
```

Grouped value objects should support grouped observation only at the handle level:

- grouped observation for widgets that truly want the pair/triple together
- field access on returned values for plots, alerts, and other per-field consumers

### Merge policy is per metric, not per domain

MAVKit should merge at the level of a **single user-facing quantity**, not at the level of a whole `battery()` or `position()` object.

For example:

- `battery().voltage_v()` means the latest available vehicle battery voltage
- that metric may refresh from `SYS_STATUS` or `BATTERY_STATUS`
- every accepted sample still reports which message family produced it
- `battery().cells()` is a different grouped metric and will naturally update at a different cadence

This gives the UI the simple behavior it wants while remaining honest about provenance.

The grouped value objects above do not change that rule. They are small semantic bundles over leaves that already belong together, not permission to reintroduce broad mixed-rate domain snapshots.

For multi-source metrics, the implementation must define an explicit acceptance policy per metric rather than inventing a hidden global rule.
Each such metric must declare:

- which message families may refresh it
- whether one family is primary and another fallback
- whether lower-priority sources may overwrite a previously seen higher-priority source
- whether `received_at` or a same-basis source timestamp is the tie-breaker when both are valid

The design rule is that every multi-source metric should be explainable in one sentence in user docs.
If a metric cannot be explained clearly, it should not be merged at the operator-facing metric layer.

### Initial metric source map

The first rewrite should lock the most ambiguous metric rules explicitly:

| Metric / handle | Primary source | Fallback | Concrete rule |
|---|---|---|---|
| `armed()` | `HEARTBEAT.base_mode` | none | armed state is a direct heartbeat-derived operator status |
| `position().global()` | `GLOBAL_POSITION_INT.lat/lon/relative_alt/alt` | none | fused global position is refreshed only from `GLOBAL_POSITION_INT` so one grouped sample always has one truthful message provenance |
| `position().groundspeed_mps()` | `VFR_HUD.groundspeed` | `GLOBAL_POSITION_INT` velocity norm | operator-facing groundspeed uses HUD semantics first |
| `position().heading_deg()` | `VFR_HUD.heading` | `GLOBAL_POSITION_INT.hdg` | HUD heading is primary; both are earth-frame heading in ArduPilot |
| `position().climb_rate_mps()` | `VFR_HUD.climb` | `-GLOBAL_POSITION_INT.vz` | positive-up climb metric |
| `position().airspeed_mps()` | `VFR_HUD.airspeed` | none | on ArduPilot this may degrade to GPS groundspeed when no airspeed sensor exists |
| `position().throttle_pct()` | `VFR_HUD.throttle` | none | HUD-only throttle metric |
| `attitude().euler()` | `ATTITUDE` | none | canonical attitude triplet |
| `gps().quality()` | `GPS_RAW_INT.fix_type/satellites/eph` | none | GPS-quality metrics stay raw-GPS only; `eph` is converted from centi-HDOP wire units into the public `hdop` field |
| `gps().position_msl()` | `GPS_RAW_INT.lat/lon/alt` | none | raw GPS position is a 3D MSL point, not a fused vehicle estimate |
| `battery().remaining_pct()` | `BATTERY_STATUS(id=0).battery_remaining` | `SYS_STATUS.battery_remaining` | primary/operator battery summary prefers battery instance 0 |
| `battery().voltage_v()` | summed valid cells from `BATTERY_STATUS(id=0)` | `SYS_STATUS.voltage_battery` | use primary battery instance 0 when available; only fall back to SYS_STATUS if BATTERY_STATUS has not been observed |
| `battery().current_a()` | `BATTERY_STATUS(id=0).current_battery` | `SYS_STATUS.current_battery` | same primary-battery rule |
| `battery().energy_consumed_wh()` | `BATTERY_STATUS(id=0).energy_consumed` | none | convert hJ to Wh |
| `battery().time_remaining_s()` | `BATTERY_STATUS(id=0).time_remaining` | none | `0` means unknown, not zero seconds remaining |
| `battery().cells()` | `BATTERY_STATUS(id=0)` | none | grouped cell-bank view for the operator battery only |
| `navigation().waypoint()` | `NAV_CONTROLLER_OUTPUT` | none | guidance-to-waypoint pair |
| `navigation().guidance()` | `NAV_CONTROLLER_OUTPUT` | none | controller guidance pair |
| `terrain().clearance()` | `TERRAIN_REPORT` | none | terrain and clearance pair |
| `rc().channel_pwm_us(index)` | `RC_CHANNELS.chan*_raw` | none | per-channel RC PWM is a direct `RC_CHANNELS` leaf metric |
| `rc().rssi_pct()` | `RC_CHANNELS.rssi` | none | operator-facing RC RSSI comes directly from `RC_CHANNELS` |
| `actuators().servo_pwm_us(index)` | `SERVO_OUTPUT_RAW.servo*_raw` | none | per-servo PWM is a direct `SERVO_OUTPUT_RAW` leaf metric |
| `sensor_health()` | `SYS_STATUS` | none | grouped sensor health is derived from `SYS_STATUS` presence/enabled/health bitmasks |
| `home()` | `HOME_POSITION` | none | live home is the latest decoded `HOME_POSITION` grouped sample |
| `origin()` | `GPS_GLOBAL_ORIGIN` | none | live EKF origin is the latest decoded `GPS_GLOBAL_ORIGIN` grouped sample |

This means the first rewrite treats `battery()` as the **primary/operator battery summary**, not a full multi-battery aggregation API.
Advanced per-instance battery access remains under `telemetry().messages().battery_status(instance)`.

`armed()` is a boolean metric in the first rewrite.

`LOCAL_POSITION_NED` remains message-level only in the first rewrite.
It is useful and coherent, but it does not get an operator-facing metric namespace until a downstream need is proven.

### Per-metric plotting semantics

Plots should subscribe to the specific scalar metric they are graphing.

Examples:

```rust
vehicle.telemetry().battery().voltage_v().subscribe()
```

For grouped values, callers should subscribe to the grouped handle and project the field client-side.

Example:

```rust
let cells = vehicle.telemetry().battery().cells();
let stream = cells.subscribe(); // caller projects sample.value.voltages_v[0]
```

```rust
let global = vehicle.telemetry().position().global();
let stream = global.subscribe(); // caller projects sample.value.relative_alt_m or sample.value.altitude_msl_m
```

These streams are expected to update at different rates, and that is correct.
`battery().voltage_v()` may refresh faster than `battery().cells()` because they are distinct metrics with distinct source availability and merge rules.

### Why `position().global()` is a fused global position sample

`position().global()` should be a grouped fused position sample, not just a 2D point.

- `GLOBAL_POSITION_INT` already provides one coherent global fix with `lat`, `lon`, `alt` (MSL), and `relative_alt`
- exposing those together avoids forcing users to manually re-pair the same sample into a 3D position
- this does **not** mean altitude is universally simple; it means the grouped value carries both altitude references explicitly
- users who need wire-faithful semantics still use `messages().global_position_int()`

### Altitude semantics are not universal

The public API should not pretend there is one generic altitude meaning across ArduPilot vehicle classes.

For the first rewrite:

| Context | Canonical vertical reference |
|---|---|
| `position().global().relative_alt_m` | above HOME |
| `position().global().altitude_msl_m` | above mean sea level |
| Copter guided `goto` / `takeoff` | relative altitude above HOME |
| Plane guided `reposition` | MSL / absolute altitude |
| Plane VTOL guided `reposition` | MSL / absolute altitude |
| Plane VTOL guided `takeoff` | relative climb target |
| Sub guided depth target | depth below surface/origin, not generic altitude |

That is why telemetry uses one grouped `GlobalPosition` sample with both altitude references, and why guided command targets still use distinct target types rather than one overloaded generic 3D point type.

### 8.4 `messages()` is the advanced telemetry branch

`vehicle.telemetry().messages()` should expose **typed, coherent, message-level** telemetry handles for users who care about exact MAVLink message boundaries.

Not every message handle has the same control surface:

- **periodic/requestable telemetry** may expose both `request(timeout)` and `set_rate(hz)`
- **event-driven/request-only telemetry** may expose `request(timeout)` without `set_rate(hz)`
- **push/event streams** may expose neither control method

Suggested initial message handle set:

```rust
vehicle.telemetry().messages().vfr_hud()
vehicle.telemetry().messages().global_position_int()
vehicle.telemetry().messages().local_position_ned()
vehicle.telemetry().messages().gps_raw_int()
vehicle.telemetry().messages().attitude()
vehicle.telemetry().messages().sys_status()
vehicle.telemetry().messages().battery_status(instance)
vehicle.telemetry().messages().nav_controller_output()
vehicle.telemetry().messages().terrain_report()
vehicle.telemetry().messages().rc_channels()
vehicle.telemetry().messages().servo_output_raw(port)
vehicle.telemetry().messages().home_position()
vehicle.telemetry().messages().gps_global_origin()
vehicle.telemetry().messages().status_text()
```

These handles should expose the usual observation methods plus typed `request(timeout)` and `set_rate(hz)` when the operation can honestly target one concrete message family:

```rust
let battery_status = vehicle.telemetry().messages().battery_status(0);

battery_status.support()
battery_status.latest()
battery_status.wait()
battery_status.subscribe()
battery_status.request(timeout)
battery_status.set_rate(2.0)
```

The sample shape here is message-coherent rather than metric-coherent:

```rust
struct MessageSample<M> {
    value: M,
    received_at: std::time::Instant,
    vehicle_time: Option<VehicleTimestamp>,
}
```

Message handle names should follow MAVLink message family names in `snake_case`.

`status_text()` deserves special treatment:

- it is an event/history-style handle, not a latest-state metric in disguise
- its primary observation surface is `subscribe()`, which should behave as a push stream rather than a coalescing latest-value stream for this handle
- `latest()` may return the most recently assembled status text for convenience, but callers must not rely on it as lossless history
- when `id == 0` (or MAVLink 2 extension fields are absent), the message is complete and should be emitted immediately
- when `id != 0`, MAVKit should reassemble by `(source_system, source_component, id)` before exposing the event to users
- chunk reassembly is best-effort: there is no retransmit mechanism or total-chunk-count field, so incomplete sequences should be flushed after a bounded timeout
- `id` is an opaque correlation key, not a receiver-visible sequence number
- repeated identical text lines should still be surfaced as separate events by default

Because `status_text()` is event-oriented rather than latest-state, its backing store should use bounded broadcast fanout rather than watch-style coalescing, so that messages are not lost when the consumer is slightly behind.

Concrete control classification for the first rewrite:

- `vfr_hud()`, `global_position_int()`, `local_position_ned()`, `gps_raw_int()`, `attitude()`, `sys_status()`, `battery_status(instance)`, `nav_controller_output()`, `terrain_report()`, `rc_channels()`, `servo_output_raw(port)` → observation + `request(timeout)` + `set_rate(hz)`
- `home_position()`, `gps_global_origin()` → observation + `request(timeout)`, but **no** `set_rate(hz)` in the first rewrite because ArduPilot treats them as event-driven/request-only rather than periodic streams
- `status_text()` → observation only; no `request()` or `set_rate(hz)`

### 8.5 Grouped value objects and scalar metrics are both intentional

MAVKit should deliberately expose both:

- scalar metric handles such as `voltage_v()`, `groundspeed_mps()`, or `channel_pwm_us(index)`
- grouped value objects such as `global()`, `euler()`, `quality()`, or `waypoint()`

The important rule is not “never expose one quantity at a time.”
The important rule is:

- do not expose a naked scalar without provenance
- do not pretend a set of independently refreshed metrics is an atomic domain sample
- do keep the advanced coherent message surface available for users who need it

And also:

- do use a real grouped value object when a concept is naturally a pair/triple/reference point
- do not use grouped objects as an excuse to merge broad mixed-rate domains

That is why metric handles return `MetricSample<T>` rather than bare values when callers use the observation APIs.

### 8.6 `subscribe()` is the stream API

`subscribe()` is the only stream API for the first rewrite.

Concrete channel model:

- `subscribe()` is watch-like latest-state fanout
- it emits the current known sample immediately when one exists
- then emits future accepted metric refreshes
- intermediate states may be coalesced if the receiver is slower than the producer

This is the right model for:

- dashboards
- gauges
- most UI logic
- most plots that only care about the latest accepted point per metric

For event/history-style handles such as `status_text()`, `subscribe()` should behave as a push stream rather than a coalescing latest-value stream, so that repeated messages are not silently dropped.

If a consumer falls behind, `subscribe()` may coalesce or drop intermediate states for latest-value handles. For event/push handles such as `status_text()`, the backing bounded broadcast buffer may drop oldest items when full. This is acceptable — callers who need the latest state can always use `latest()`.

### 8.7 `request()` and `set_rate(hz)` belong only on message-level handles

MAVKit may add typed `request()` and `set_rate(hz)` APIs, but only on **message-level** handles under `telemetry().messages()`.

Examples:

```rust
vehicle.telemetry().messages().gps_raw_int().request(timeout)
vehicle.telemetry().messages().global_position_int().request(timeout)
vehicle.telemetry().messages().gps_raw_int().set_rate(5.0)
vehicle.telemetry().messages().global_position_int().set_rate(2.0)
```

These should **not** exist on metric handles such as `battery().voltage_v()`, because those views do not correspond honestly to one concrete MAVLink message family.

`request()` should be defined as typed sugar over raw MAVLink request machinery:

1. request the backing message from the vehicle
2. wait for the next **fresh** message sample on that handle
3. return the typed `MessageSample<M>`

This is a one-shot request. It does not start a periodic stream.

Freshness should be based on receipt generation or receive timestamp, not on value equality.

`set_rate(hz)` should be defined as typed sugar over `raw().set_message_interval(...)` for that message family.

It should:

1. configure the desired emit interval for that message family
2. remain best-effort because autopilot support varies
3. affect the underlying message stream, not invent a new buffering layer in telemetry
4. be treated as session-scoped rather than durable configuration

`set_rate(hz)` is only valid for message families that are meaningfully interval-controlled.
Event-driven/request-only families such as `HOME_POSITION`, `GPS_GLOBAL_ORIGIN`, and push-only event streams such as `STATUSTEXT` should not pretend to support it just because the generic message handle API exists.
ArduPilot may explicitly reject too-high message rates rather than silently capping them, and a successful request still does not promise exact long-term persistence across reconnects.
Protocol parameters that some autopilots ignore, such as response-target details, should remain best-effort and not be over-promoted in the typed API.

MAVKit should not manage aggregate stream bandwidth or rate budgets on behalf of the autopilot.
`set_rate(hz)` is a pass-through to the autopilot's own `SET_MESSAGE_INTERVAL` implementation.
ArduPilot maintains its own per-message and total stream-rate budgets internally; MAVKit should not duplicate or second-guess that logic.
If the autopilot rejects or silently caps a rate request, that is the autopilot's decision.
MAVKit's job is to provide the typed API to issue the request honestly, not to invent a client-side rate governor.

### 8.8 Telemetry is observe-only

Metric surfaces should not expose generic setter-style APIs.

Observation belongs under `telemetry()`.
State changes belong elsewhere.

If the SDK wants explicit request/rate control, the general-purpose protocol tools still belong in `raw()`.
Typed `request(timeout)` and `set_rate(hz)` on eligible `telemetry().messages().*` handles are only narrow conveniences over one concrete MAVLink message family.

### 8.9 Older merged domain objects should not define the model

The API shape above replaces the older idea of a general family of merged per-domain sample objects such as `telemetry().position()` or `telemetry().battery()`.

Those names may still exist as **metric namespaces**, but they should no longer mean “here is one coherent merged domain sample.”
They are now just the path to grouped value handles and scalar metric handles such as `position().global()` or `battery().voltage_v()`.

## 9. Raw MAVLink

Raw MAVLink access is a core surface, not optional future work.

`vehicle.raw()` should return a `RawHandle` that owns the literal protocol tools.

Concrete shape for the first rewrite:

```rust
impl RawHandle<'_> {
    /// Send an arbitrary COMMAND_LONG and wait for the keyed COMMAND_ACK.
    async fn command_long(
        &self,
        command: u16,
        params: [f32; 7],
    ) -> Result<CommandAck, VehicleError>;

    /// Send an arbitrary COMMAND_INT and wait for the keyed COMMAND_ACK.
    async fn command_int(
        &self,
        command: u16,
        frame: u8,
        current: u8,
        autocontinue: u8,
        params: [f32; 4],
        x: i32,
        y: i32,
        z: f32,
    ) -> Result<CommandAck, VehicleError>;

    /// Request a single message by message ID. Returns when the message arrives or timeout.
    async fn request_message(
        &self,
        message_id: u32,
        timeout: Duration,
    ) -> Result<RawMessage, VehicleError>;

    /// Set the emit interval for a message family on the vehicle.
    async fn set_message_interval(
        &self,
        message_id: u32,
        interval_us: i32,
    ) -> Result<(), VehicleError>;

    /// Send an arbitrary raw MAVLink message. No ACK correlation.
    /// This is caller-managed protocol territory.
    async fn send(&self, message: RawMessage) -> Result<(), VehicleError>;

    /// Subscribe to all inbound raw MAVLink messages.
    fn subscribe(&self) -> impl Stream<Item = RawMessage>;

    /// Subscribe to inbound raw messages filtered by message ID.
    fn subscribe_filtered(&self, message_id: u32) -> impl Stream<Item = RawMessage>;
}

struct CommandAck {
    command: u16,
    result: u8,
    progress: Option<u8>,
    result_param2: Option<i32>,
}

struct RawMessage {
    message_id: u32,
    system_id: u8,
    component_id: u8,
    payload: Vec<u8>,
    received_at: std::time::Instant,
}
```

`command_long()` and `command_int()` participate in the same ACK-correlation registry as typed command APIs (see §16).
They share the per-key `(target_system, target_component, MAV_CMD)` conflict scope, so a raw `command_long(MAV_CMD_COMPONENT_ARM_DISARM, ...)` and a typed `vehicle.arm(true)` cannot overlap on the same target.

`send()` bypasses the ACK registry entirely. It is the true escape hatch for protocol-shaped sends that MAVKit does not model.
The docs should mark it clearly as caller-managed.

`subscribe()` and `subscribe_filtered()` provide raw inbound message access.
These are separate from telemetry message handles: raw streams carry undecoded payloads, while `telemetry().messages()` carries typed decoded samples.

`telemetry().messages()` may offer curated typed handles for telemetry-relevant decoded messages, but `raw()` remains the canonical place for arbitrary message requests, rate control, and protocol-shaped behavior across the whole protocol surface.

## 10. Home and Origin

Home and EKF origin are not the same thing and should not be merged.

They should be observed under telemetry:

```rust
vehicle.telemetry().home()
vehicle.telemetry().origin()
```

These handles should support:

- `support()`
- `latest()`
- `wait()`
- `subscribe()`

They should remain observe-only at the metric layer.
Like other grouped telemetry concepts, `home()` and `origin()` expose only grouped observation methods in the first rewrite; there are no per-field subhandles under them.
If a caller wants explicit request/rate control for the underlying wire messages, the typed path is:

```rust
vehicle.telemetry().messages().home_position().request(timeout)

vehicle.telemetry().messages().gps_global_origin().request(timeout)
```

Connect-time behavior should be concrete:

- MAVKit should issue an eager home/origin fetch during initialization
- for ArduPilot-first behavior, `MAV_CMD_GET_HOME_POSITION` is the preferred bootstrap request for home itself, but `GPS_GLOBAL_ORIGIN` remains a separate event/request path and must not be assumed to arrive just because home did
- callers may still use the message-level request helpers later for explicit refreshes

Availability should also be concrete:

- `HOME_POSITION` may remain absent until the vehicle actually has a home
- `GPS_GLOBAL_ORIGIN` may remain absent until EKF origin is established
- lack of these messages during early connection must not be interpreted as protocol `Unsupported`

But state-changing home/origin operations should remain command-style APIs on the vehicle root:

```rust
vehicle.set_home(...)
vehicle.set_home_current()
vehicle.set_origin(...)
```

Exact target signatures for the first rewrite:

```rust
async fn set_home(&self, position: GeoPoint3dMsl) -> Result<(), VehicleError>;
async fn set_home_current(&self) -> Result<(), VehicleError>;
async fn set_origin(&self, origin: GeoPoint3dMsl) -> Result<(), VehicleError>;
```

`set_home*()` remains an ACK-driven command family.
`set_origin(...)` is different: it sends `SET_GPS_GLOBAL_ORIGIN`, so its confirmation contract is observation-based rather than `COMMAND_ACK`-based.
The first rewrite should define success for `set_origin(...)` as “the request was sent and the matching origin change was later observed” or timeout/failure if that confirmation never arrives.
Matching should use the request values after MAVLink wire normalization/quantization, not raw float equality.
A pre-existing origin value only counts as confirmation if it is observed after the request is sent.
An observed but non-matching origin value does not count as success or rejection by itself; MAVKit should continue waiting for a matching observation until timeout or terminal link failure.

That keeps the semantics clean:

- telemetry observes
- commands change state

## 11. Commands and Control

### 11.1 Shared root commands

Commands that are broadly meaningful across MAVLink autopilots should live on `Vehicle`:

- `arm(force)`
- `disarm(force)`
- `set_mode(custom_mode)`
- `set_mode_by_name(name)`
- `set_home(...)`
- `set_home_current()`
- `set_origin(...)`
- `disconnect()`

Exact target signatures for the first rewrite:

```rust
async fn arm(&self, force: bool) -> Result<(), VehicleError>;
async fn disarm(&self, force: bool) -> Result<(), VehicleError>;
async fn set_mode(&self, custom_mode: u32, wait_for_observation: bool) -> Result<(), VehicleError>;
async fn set_mode_by_name(&self, name: &str, wait_for_observation: bool) -> Result<(), VehicleError>;
```

ACK-driven shared commands should default to ACK-level success unless explicitly documented otherwise.
Root commands whose protocol basis is not ACK-driven, such as `set_origin(...)`, must document their own confirmation semantics explicitly.

For the first rewrite, `set_mode()` and `set_mode_by_name()` should support two confirmation levels controlled by `wait_for_observation`:

- **`wait_for_observation: true`** (recommended default for most callers) — ACK-plus-observation contract. After a positive ACK, MAVKit waits for `vehicle.available_modes().current()` to report the requested `custom_mode` before resolving success. This adds up to one heartbeat interval of latency (~1 second at ArduPilot's default 1 Hz heartbeat rate) but confirms the mode actually changed.
- **`wait_for_observation: false`** — ACK-only contract. Resolves immediately on positive ACK without waiting for heartbeat confirmation. Useful for latency-sensitive callers who can tolerate the small risk that the ACK was positive but the mode change did not actually take effect.

In both modes:
- Negative ACK fails immediately.
- If `wait_for_observation` is true and the mode never matches before the completion timeout, the call fails with a mode-change confirmation timeout.
- `set_mode_by_name(name)` resolves `name` against the current catalog, including the static ArduPilot fallback table, and unknown names fail as immediate validation errors.

### 11.2 Continuous control

Continuous control is not a one-shot command. It should use managed sessions.

This belongs in firmware/vehicle-specific scopes with an explicit session boundary.

For ArduPilot, guided control should use **one session entrypoint** with explicit runtime narrowing to the actual vehicle-class-specific API.

```rust
let guided = vehicle.ardupilot().guided().await?; // ArduGuidedSession

guided.kind()

match guided.specific() {
    GuidedSpecific::Copter(c) => { /* copter guided API */ }
    GuidedSpecific::Plane(p) => { /* plane API, with optional VTOL extension */ }
    GuidedSpecific::Rover(r) => { /* rover guided API */ }
    GuidedSpecific::Sub(s) => { /* sub guided API */ }
}

guided.close().await?
```

This keeps the session/lease model explicit while avoiding fake always-available methods like `guided.plane()` on a Sub.
VTOL Plane remains first-class as an extension of the Plane guided handle, not as a sibling top-level variant.

### Why guided needs a handle/session

`guided()` should not be just a namespacing trick.
It needs to be a real handle because MAVKit must be able to own:

- exclusivity: only one active MAVKit-guided control lease for that vehicle at a time
- the lifetime of guided-mode-scoped actions
- capability availability under the current guided context
- session validity if the vehicle leaves guided mode or the link drops
- explicit best-effort shutdown via `close()`

A stateless namespace cannot honestly own any of that.
It would make conflicting `goto()` / velocity / hold requests from different tasks or clones look safe when they are not.

And the narrowing should not be plain always-present methods like `guided.plane()` / `guided.sub()` either.
Those make the wrong branch look callable everywhere and turn a category error into a later runtime check.
The API should force the caller to narrow explicitly once.

### Exact ArduPilot guided API

The first rewrite should use this shape (see §11.3 for the full `ArduPilotHandle` surface):

```rust
impl ArduGuidedSession {
    fn kind(&self) -> ArduGuidedKind;
    fn specific(&self) -> GuidedSpecific<'_>;

    async fn close(self) -> Result<(), VehicleError>;
}

enum ArduGuidedKind {
    Copter,
    Plane,
    Rover,
    Sub,
}

enum GuidedSpecific<'a> {
    Copter(ArduCopterGuidedHandle<'a>),
    Plane(ArduPlaneGuidedHandle<'a>),
    Rover(ArduRoverGuidedHandle<'a>),
    Sub(ArduSubGuidedHandle<'a>),
}

struct RelativeClimbTarget {
    relative_climb_m: f32,
}

struct SubGotoDepthTarget {
    point: GeoPoint2d,
    depth_m: f32,
}

impl ArduCopterGuidedHandle {
    async fn takeoff(&self, target: RelativeClimbTarget) -> Result<(), VehicleError>;
    async fn goto(&self, target: GeoPoint3dRelHome) -> Result<(), VehicleError>;
    async fn goto_msl(&self, target: GeoPoint3dMsl) -> Result<(), VehicleError>;
    async fn set_velocity_ned(&self, north_mps: f32, east_mps: f32, down_mps: f32) -> Result<(), VehicleError>;
    async fn hold(&self) -> Result<(), VehicleError>;
}

impl ArduPlaneGuidedHandle {
    fn kind(&self) -> ArduPlaneKind;
    fn vtol(&self) -> Option<ArduPlaneVtolGuidedHandle<'_>>;

    async fn reposition(&self, target: GeoPoint3dMsl) -> Result<(), VehicleError>;
    async fn reposition_rel_home(&self, target: GeoPoint3dRelHome) -> Result<(), VehicleError>;
}

enum ArduPlaneKind {
    FixedWing,
    Vtol,
}

impl ArduPlaneVtolGuidedHandle {
    async fn takeoff(&self, target: RelativeClimbTarget) -> Result<(), VehicleError>;
    async fn hold(&self) -> Result<(), VehicleError>;
}

impl ArduRoverGuidedHandle {
    async fn drive_to(&self, target: GeoPoint2d) -> Result<(), VehicleError>;
    async fn drive(&self, forward_mps: f32, turn_rate_dps: f32) -> Result<(), VehicleError>;
    async fn hold(&self) -> Result<(), VehicleError>;
}

impl ArduSubGuidedHandle {
    async fn goto_depth(&self, target: SubGotoDepthTarget) -> Result<(), VehicleError>;
    async fn set_velocity_body(&self, forward_mps: f32, lateral_mps: f32, vertical_mps: f32, yaw_rate_dps: f32) -> Result<(), VehicleError>;
    async fn hold(&self) -> Result<(), VehicleError>;
}
```

The VTOL split is deliberate:

- `GuidedSpecific::Plane` is the single plane-family branch
- `ArduPlaneGuidedHandle` carries the shared plane-family reposition behavior
- `plane.vtol()` adds QuadPlane-only guided behavior without duplicating the base plane API
- MAVKit should not expose `plane.takeoff()` for all planes because ArduPilot rejects that on non-QuadPlane aircraft

### Session semantics

- `guided().await?` acquires the MAVKit-guided control lease and is mode-asserting: if the vehicle is not already in the appropriate guided-capable mode, MAVKit should issue the required mode change and wait for the same mode-confirmation contract used by `set_mode*()` before returning the session
- the session does **not** imply arming, takeoff completion, or arrival at a destination unless a specific method documents that stronger contract
- `close().await` is explicit best-effort shutdown of MAVKit-managed guided control
- dropping the session only releases MAVKit's local lease and stops local resend loops; it must not silently restore the previous flight mode
- if the link drops or the vehicle leaves guided mode, the session becomes terminal and later calls fail

`close().await` should release MAVKit's local guided lease and stop SDK-managed guided activity, but it should not silently send a vehicle mode change or restore a previous mode.
If callers want a vehicle-side exit action such as hold, loiter, or RTL, they should issue it explicitly before closing the session.

Guided actions live **only** on guided sessions in the first rewrite.
There should be no one-off guided wrappers on root or firmware-scope handles such as `vehicle.takeoff(...)`, `vehicle.ardupilot().copter().takeoff(...)`, or `vehicle.ardupilot().plane().reposition(...)` that bypass the session boundary.

Method naming should stay honest about semantics:

- Copter keeps `goto(...)` because that name matches common multicopter guided usage well
- fixed-wing Plane uses `reposition(...)`, not `goto(...)`, because ArduPlane guided navigation semantics are loiter/reposition-like rather than copter-style point-and-hold
- Rover uses `drive_to(...)`, not `goto(...)`, because it is a ground-vehicle navigation intent
- Sub uses `goto_depth(...)`, not generic `goto(...)`, because depth is a first-class part of the command contract

Target types should stay honest too:

- plain geo point types are reused where the command contract is exactly a point target
- wrapper structs are reserved for commands with extra semantic payload such as relative climb or depth
- Copter guided targets use `GeoPoint3dRelHome`
- Plane guided reposition targets use `GeoPoint3dMsl`
- VTOL Plane takeoff uses `relative_climb_m`
- Rover drive-to targets use `GeoPoint2d`
- Sub guided targets use `GeoPoint2d + depth_m`

Default methods should follow the natural ArduPilot-first path, while alternate explicit methods remain available when firmware supports another altitude reference:

- `ArduCopterGuidedHandle::goto(...)` uses `GeoPoint3dRelHome` as the default high-level path
- `ArduCopterGuidedHandle::goto_msl(...)` is the explicit MSL variant
- `ArduPlaneGuidedHandle::reposition(...)` uses `GeoPoint3dMsl` as the default high-level path
- `ArduPlaneGuidedHandle::reposition_rel_home(...)` is the explicit relative-home variant
- `ArduPlaneVtolGuidedHandle` adds VTOL-only methods such as `takeoff(...)` and `hold()` without duplicating the base plane reposition API

This keeps the common path short and honest while still exposing the alternate reference frame explicitly when users really need it.

The narrowing result proves that the session belongs to the plane family.
VTOL support is then determined inside that plane branch via `plane.kind()` / `plane.vtol()`.

If a caller wants a single guided action, it should still acquire a session explicitly, issue the action, and then close the session.
The extra step is intentional because it keeps the lease, exclusivity, and lifecycle semantics visible instead of hiding them behind misleading one-off helpers.

### 11.3 ArduPilot-specific commands

ArduPilot-specific behavior should stay under `vehicle.ardupilot()`.

For the first rewrite, `ArduPilotHandle` should expose:

```rust
impl ArduPilotHandle<'_> {
    /// Acquire the guided control session (see §11.2).
    async fn guided(&self) -> Result<ArduGuidedSession, VehicleError>;

    /// Request pre-arm check status. Returns pre-arm pass/fail plus
    /// sensor health details via telemetry observation.
    async fn request_prearm_checks(&self) -> Result<(), VehicleError>;

    /// Start magnetometer calibration for the given compass mask.
    async fn start_mag_cal(&self, compass_mask: u8) -> Result<(), VehicleError>;

    /// Accept the current magnetometer calibration results.
    async fn accept_mag_cal(&self) -> Result<(), VehicleError>;

    /// Cancel an in-progress magnetometer calibration.
    async fn cancel_mag_cal(&self) -> Result<(), VehicleError>;

    /// Run a motor test on one motor.
    /// `instance` is 1-based (motor 1..12).
    /// `throttle_pct` is 0..100.
    /// `duration_s` is 1..30.
    async fn motor_test(
        &self,
        instance: u8,
        throttle_pct: f32,
        duration_s: u16,
    ) -> Result<(), VehicleError>;

    /// Trigger a preflight calibration. Exactly one axis flag should be set.
    async fn preflight_calibration(
        &self,
        gyro: bool,
        accel: bool,
        baro: bool,
        accel_trim: bool,
    ) -> Result<(), VehicleError>;

    /// Command a normal reboot.
    async fn reboot(&self) -> Result<(), VehicleError>;

    /// Command reboot into bootloader mode.
    async fn reboot_to_bootloader(&self) -> Result<(), VehicleError>;

    /// Mag cal progress observation (per-compass).
    fn mag_cal_progress(&self) -> ObservationHandle<Vec<MagCalProgress>>;

    /// Mag cal completion reports (per-compass).
    fn mag_cal_report(&self) -> ObservationHandle<Vec<MagCalReport>>;
}
```

Supporting types:

```rust
enum MagCalStatus {
    NotStarted,
    WaitingToStart,
    RunningStepOne,
    RunningStepTwo,
    Success,
    Failed,
    BadOrientation,
    BadRadius,
}

struct MagCalProgress {
    compass_id: u8,
    completion_pct: u8,
    status: MagCalStatus,
    attempt: u8,
}

struct MagCalReport {
    compass_id: u8,
    status: MagCalStatus,
    fitness: f32,
    ofs_x: f32,
    ofs_y: f32,
    ofs_z: f32,
    autosaved: bool,
}
```

Mag cal progress and report are observation handles because they update asynchronously as calibration proceeds.
`motor_test` validates `instance`, `throttle_pct`, and `duration_s` at call site and returns `InvalidParameter` for out-of-range values.

The following are deliberately deferred from the first rewrite:

- manual control / RC override
- ArduPilot-specific telemetry extensions

## 12. Mission Model

### 12.0 Mission data types

The first rewrite should model mission items as **typed commands with named fields**, not raw `param1..7` / `x`/`y`/`z` tuples.

Users should be able to construct a mission plan without consulting MAVLink or ArduPilot wire documentation.
Wire-level details (degE7 coordinates, frame integers, raw parameter slots) are hidden behind internal encode/decode conversions.

#### 12.0.1 Plan and item shape

```rust
/// A complete mission plan: ordered executable items only, no home slot.
/// Item index in the Vec is the item's identity — there is no separate sequence number.
struct MissionPlan {
    items: Vec<MissionItem>,
}

/// A single executable mission item with a typed command.
struct MissionItem {
    command: MissionCommand,
    /// Whether the vehicle should automatically advance to the next item after completing this one.
    /// Defaults to `true`. ArduPilot ignores this field entirely (always auto-continues regardless;
    /// hold behavior is controlled by command parameters such as `NavWaypoint.hold_time_s`).
    /// Preserved for PX4 and other autopilots that respect the MAVLink `autocontinue` field.
    autocontinue: bool, // default: true
}
```

`MissionItem.command` is a typed `MissionCommand` enum that absorbs the MAVLink frame, `param1..4`, `x`, `y`, and `z` fields into named, domain-appropriate fields per command variant.
Position-bearing commands embed `GeoPoint3d` (see §8.2), which carries the altitude reference (MSL / relative-home / terrain) as part of the point type, eliminating a separate `frame` field from the public API.

`MissionItem` should implement `Default` for `autocontinue` (defaulting to `true`), and any command struct should convert directly into a `MissionItem` via `From`:

```rust
impl<T: Into<MissionCommand>> From<T> for MissionItem {
    fn from(cmd: T) -> Self {
        MissionItem { command: cmd.into(), autocontinue: true }
    }
}
```

This means `NavWaypoint { ... }.into()` produces a complete `MissionItem` with `autocontinue: true` — the common path requires no boilerplate.
Callers who need `autocontinue: false` (PX4 workflows) construct `MissionItem` explicitly.

Design rules:

- `MissionPlan` does not contain `home`. Home belongs to the telemetry domain (see §10, §12.4).
- `MissionPlan` does not contain `mission_type`. Mission plans are always executable-sequence content; fence and rally have their own typed plan models (`FencePlan`, `RallyPlan`).
- `MissionItem` does not contain `current`. Current-item state is observed via `MissionState.current_index`, not stored per-item.
- `MissionItem` does not contain `seq`. The item's index in `MissionPlan.items` is its identity. Wire sequence numbers (`MISSION_ITEM_INT.seq`) are assigned internally during upload and stripped during download. Users never see or manage sequence numbers.
- `MissionItem.autocontinue` defaults to `true`. ArduPilot ignores this wire field entirely — it never stores it, never reads it during execution, and hardcodes `1` on download. The field is preserved for forward compatibility with PX4 and other autopilots that respect the MAVLink `autocontinue` semantics.

These types should derive `Clone`, `Debug`, `PartialEq`, `Serialize`, `Deserialize`.

#### 12.0.2 Typed mission command

`MissionCommand` is a categorized enum mirroring ArduPilot's own NAV / DO / CONDITION classification.
This distinction is semantically meaningful: navigation commands advance the waypoint counter during mission execution, while do-commands and conditions do not.

Every data-carrying enum variant wraps a **named struct** so that each command type is independently addressable as a Rust type — passable to functions, usable in generics, and impl-able.
Fieldless variants (e.g. `ReturnToLaunch`, `SetRoiNone`) remain unit variants.

```rust
enum MissionCommand {
    Nav(NavCommand),
    Do(DoCommand),
    Condition(ConditionCommand),
    /// Escape hatch for unknown or future commands.
    Other(RawMissionCommand),
}

/// Raw wire representation of an unrecognized mission command.
/// Roundtrips any MAVLink mission item that MAVKit does not model with a typed variant.
struct RawMissionCommand {
    command: u16,
    frame: MissionFrame,
    param1: f32,
    param2: f32,
    param3: f32,
    param4: f32,
    x: i32,
    y: i32,
    z: f32,
}

enum MissionFrame {
    Global,
    GlobalRelativeAlt,
    GlobalTerrainAlt,
    Mission,
    Other(u8),
}
```

`MissionCommand::Other` preserves the raw wire representation for any command MAVKit does not yet model.
If ArduPilot adds a new `MAV_CMD` in a future release, it roundtrips losslessly through `Other` until MAVKit adds a typed variant.
`MissionFrame` remains available only inside `Other` and in the internal wire conversion layer; typed command variants use `GeoPoint3d` instead.
`MissionFrame::Other(u8)` preserves unrecognized wire frame values for roundtrip fidelity.

Every command struct should implement `From` conversions through the full chain — struct → sub-enum → `MissionCommand` → `MissionItem` — so that `.into()` works at every level:

```rust
// Struct → sub-enum
impl From<NavWaypoint> for NavCommand
impl From<DoChangeSpeed> for DoCommand
impl From<CondYaw> for ConditionCommand
// ... for every command struct

// Sub-enum → MissionCommand
impl From<NavCommand> for MissionCommand
impl From<DoCommand> for MissionCommand
impl From<ConditionCommand> for MissionCommand
impl From<RawMissionCommand> for MissionCommand

// Struct → MissionCommand (explicit, since Rust has no transitive From)
impl From<NavWaypoint> for MissionCommand
impl From<DoChangeSpeed> for MissionCommand
// ... for every command struct

// Any MissionCommand-convertible → MissionItem (blanket)
impl<T: Into<MissionCommand>> From<T> for MissionItem
```

This means `NavWaypoint { ... }.into()` produces a complete `MissionItem` with `autocontinue: true` — the most common path is the shortest.

These `From` impls are mechanical and should be generated by a `macro_rules!` macro to avoid maintaining ~120 hand-written impls:

```rust
mission_commands! {
    Nav {
        Waypoint(NavWaypoint),
        SplineWaypoint(NavSplineWaypoint),
        Takeoff(NavTakeoff),
        // ...
    }
    Do {
        ChangeSpeed(DoChangeSpeed),
        SetServo(DoSetServo),
        // ...
    }
    Condition {
        Delay(CondDelay),
        Distance(CondDistance),
        Yaw(CondYaw),
    }
}
// Expands to: enum definitions + all From impls + to_wire/from_wire match arms
```

#### 12.0.3 Navigation commands

Navigation commands create waypoints and advance the mission sequence.
Based on the full set of commands accepted by ArduPilot's `AP_Mission::mavlink_int_to_mission_cmd`:

```rust
struct NavWaypoint {
    position: GeoPoint3d,
    hold_time_s: f32,
    acceptance_radius_m: f32,
    pass_radius_m: f32,
    yaw_deg: f32,
}

struct NavSplineWaypoint {
    position: GeoPoint3d,
    hold_time_s: f32,
}

struct NavArcWaypoint {
    position: GeoPoint3d,
    arc_angle_deg: f32,
    direction: LoiterDirection,
}

struct NavTakeoff {
    position: GeoPoint3d,
    pitch_deg: f32,
}

struct NavLand {
    position: GeoPoint3d,
    abort_alt_m: f32,
}

struct NavLoiterUnlimited {
    position: GeoPoint3d,
    radius_m: f32,
    direction: LoiterDirection,
}

struct NavLoiterTurns {
    position: GeoPoint3d,
    turns: f32,
    radius_m: f32,
    direction: LoiterDirection,
    exit_xtrack: bool,
}

struct NavLoiterTime {
    position: GeoPoint3d,
    time_s: f32,
    direction: LoiterDirection,
    exit_xtrack: bool,
}

struct NavLoiterToAlt {
    position: GeoPoint3d,
    radius_m: f32,
    direction: LoiterDirection,
    exit_xtrack: bool,
}

struct NavContinueAndChangeAlt {
    position: GeoPoint3d,
    action: AltChangeAction,
}

struct NavDelay {
    seconds: f32,
    hour_utc: f32,
    min_utc: f32,
    sec_utc: f32,
}

struct NavGuidedEnable {
    enabled: bool,
}

struct NavAltitudeWait {
    altitude_m: f32,
    descent_rate_mps: f32,
    wiggle_time_s: f32,
}

struct NavVtolTakeoff {
    position: GeoPoint3d,
}

struct NavVtolLand {
    position: GeoPoint3d,
    options: u8,
}

struct NavPayloadPlace {
    position: GeoPoint3d,
    max_descent_m: f32,
}

struct NavSetYawSpeed {
    angle_deg: f32,
    speed_mps: f32,
    relative: bool,
}

struct NavScriptTime {
    command: u16,
    timeout_s: f32,
    arg1: f32,
    arg2: f32,
    arg3: i16,
    arg4: i16,
}

struct NavAttitudeTime {
    time_s: f32,
    roll_deg: f32,
    pitch_deg: f32,
    yaw_deg: f32,
    climb_rate_mps: f32,
}

enum NavCommand {
    Waypoint(NavWaypoint),
    SplineWaypoint(NavSplineWaypoint),
    ArcWaypoint(NavArcWaypoint),
    Takeoff(NavTakeoff),
    Land(NavLand),
    ReturnToLaunch,
    LoiterUnlimited(NavLoiterUnlimited),
    LoiterTurns(NavLoiterTurns),
    LoiterTime(NavLoiterTime),
    LoiterToAlt(NavLoiterToAlt),
    ContinueAndChangeAlt(NavContinueAndChangeAlt),
    Delay(NavDelay),
    GuidedEnable(NavGuidedEnable),
    AltitudeWait(NavAltitudeWait),
    VtolTakeoff(NavVtolTakeoff),
    VtolLand(NavVtolLand),
    PayloadPlace(NavPayloadPlace),
    SetYawSpeed(NavSetYawSpeed),
    ScriptTime(NavScriptTime),
    AttitudeTime(NavAttitudeTime),
}
```

#### 12.0.4 Condition commands

Condition commands gate mission progress without creating waypoints.

```rust
struct CondDelay {
    seconds: f32,
}

struct CondDistance {
    meters: f32,
}

struct CondYaw {
    angle_deg: f32,
    turn_rate_dps: f32,
    direction: YawDirection,
    relative: bool,
}

enum ConditionCommand {
    Delay(CondDelay),
    Distance(CondDistance),
    Yaw(CondYaw),
}
```

#### 12.0.5 Do commands

Do-commands trigger actions at the current mission item without advancing the waypoint counter.
This includes hardware control, camera, gimbal, flow control, and vehicle system commands.

```rust
// --- Flow control ---

struct DoJump {
    target_index: u16,
    repeat_count: u16,
}

struct DoJumpTag {
    tag: u16,
    repeat_count: u16,
}

struct DoTag {
    tag: u16,
}

struct DoPauseContinue {
    pause: bool,
}

// --- Speed / navigation ---

struct DoChangeSpeed {
    speed_type: SpeedType,
    speed_mps: f32,
    throttle_pct: f32,
}

struct DoSetReverse {
    reverse: bool,
}

// --- Home / landing markers ---

struct DoSetHome {
    position: GeoPoint3d,
    use_current: bool,
}

struct DoLandStart {
    position: GeoPoint3d,
}

struct DoReturnPathStart {
    position: GeoPoint3d,
}

struct DoGoAround {
    position: GeoPoint3d,
}

// --- ROI / gimbal ---

struct DoSetRoiLocation {
    position: GeoPoint3d,
}

struct DoSetRoi {
    mode: u8,
    position: GeoPoint3d,
}

struct DoMountControl {
    pitch_deg: f32,
    roll_deg: f32,
    yaw_deg: f32,
}

struct DoGimbalManagerPitchYaw {
    pitch_deg: f32,
    yaw_deg: f32,
    pitch_rate_dps: f32,
    yaw_rate_dps: f32,
    flags: u32,
    gimbal_id: u8,
}

// --- Camera ---

struct DoCamTriggerDistance {
    meters: f32,
    trigger_now: bool,
}

struct DoImageStartCapture {
    instance: u8,
    interval_s: f32,
    total_images: u32,
    start_number: u32,
}

struct DoImageStopCapture {
    instance: u8,
}

struct DoVideoStartCapture {
    stream_id: u8,
}

struct DoVideoStopCapture {
    stream_id: u8,
}

struct DoSetCameraZoom {
    zoom_type: u8,
    zoom_value: f32,
}

struct DoSetCameraFocus {
    focus_type: u8,
    focus_value: f32,
}

struct DoSetCameraSource {
    instance: u8,
    primary: u8,
    secondary: u8,
}

struct DoDigicamConfigure {
    shooting_mode: u8,
    shutter_speed: u16,
    aperture: f32,
    iso: u16,
    exposure_type: u8,
    cmd_id: u8,
    cutoff_time: f32,
}

struct DoDigicamControl {
    session: u8,
    zoom_pos: u8,
    zoom_step: i8,
    focus_lock: u8,
    shooting_cmd: u8,
    cmd_id: u8,
}

// --- Hardware ---

struct DoSetServo {
    channel: u16,
    pwm: u16,
}

struct DoSetRelay {
    number: u8,
    state: bool,
}

struct DoRepeatServo {
    channel: u16,
    pwm: u16,
    count: u16,
    cycle_time_s: f32,
}

struct DoRepeatRelay {
    number: u8,
    count: u16,
    cycle_time_s: f32,
}

// --- Vehicle systems ---

struct DoFenceEnable {
    action: FenceAction,
}

struct DoParachute {
    action: ParachuteAction,
}

struct DoGripper {
    number: u8,
    action: GripperAction,
}

struct DoSprayer {
    enabled: bool,
}

struct DoWinch {
    number: u8,
    action: WinchAction,
    release_length_m: f32,
    release_rate_mps: f32,
}

struct DoEngineControl {
    start: bool,
    cold_start: bool,
    height_delay_m: f32,
    allow_disarmed: bool,
}

struct DoInvertedFlight {
    inverted: bool,
}

struct DoAutotuneEnable {
    enabled: bool,
}

struct DoVtolTransition {
    target_state: u8,
}

struct DoGuidedLimits {
    max_time_s: f32,
    min_alt_m: f32,
    max_alt_m: f32,
    max_horiz_m: f32,
}

struct DoSetResumeRepeatDist {
    distance_m: f32,
}

struct DoAuxFunction {
    function: u16,
    switch_pos: u8,
}

struct DoSendScriptMessage {
    id: u16,
    p1: f32,
    p2: f32,
    p3: f32,
}

enum DoCommand {
    // Flow control
    Jump(DoJump),
    JumpTag(DoJumpTag),
    Tag(DoTag),
    PauseContinue(DoPauseContinue),

    // Speed / navigation
    ChangeSpeed(DoChangeSpeed),
    SetReverse(DoSetReverse),

    // Home / landing markers
    SetHome(DoSetHome),
    LandStart(DoLandStart),
    ReturnPathStart(DoReturnPathStart),
    GoAround(DoGoAround),

    // ROI / gimbal
    SetRoiLocation(DoSetRoiLocation),
    SetRoiNone,
    SetRoi(DoSetRoi),
    MountControl(DoMountControl),
    GimbalManagerPitchYaw(DoGimbalManagerPitchYaw),

    // Camera
    CamTriggerDistance(DoCamTriggerDistance),
    ImageStartCapture(DoImageStartCapture),
    ImageStopCapture(DoImageStopCapture),
    VideoStartCapture(DoVideoStartCapture),
    VideoStopCapture(DoVideoStopCapture),
    SetCameraZoom(DoSetCameraZoom),
    SetCameraFocus(DoSetCameraFocus),
    SetCameraSource(DoSetCameraSource),
    DigicamConfigure(DoDigicamConfigure),
    DigicamControl(DoDigicamControl),

    // Hardware
    SetServo(DoSetServo),
    SetRelay(DoSetRelay),
    RepeatServo(DoRepeatServo),
    RepeatRelay(DoRepeatRelay),

    // Vehicle systems
    FenceEnable(DoFenceEnable),
    Parachute(DoParachute),
    Gripper(DoGripper),
    Sprayer(DoSprayer),
    Winch(DoWinch),
    EngineControl(DoEngineControl),
    InvertedFlight(DoInvertedFlight),
    AutotuneEnable(DoAutotuneEnable),
    VtolTransition(DoVtolTransition),
    GuidedLimits(DoGuidedLimits),
    SetResumeRepeatDist(DoSetResumeRepeatDist),
    AuxFunction(DoAuxFunction),
    SendScriptMessage(DoSendScriptMessage),
}
```

#### 12.0.6 Supporting enums

```rust
enum LoiterDirection {
    Clockwise,
    CounterClockwise,
}

enum YawDirection {
    Clockwise,
    CounterClockwise,
}

enum SpeedType {
    Airspeed,
    Groundspeed,
}

enum AltChangeAction {
    Neutral,
    Climb,
    Descend,
}

enum FenceAction {
    Disable,
    Enable,
    DisableFloor,
}

enum ParachuteAction {
    Disable,
    Enable,
    Release,
}

enum GripperAction {
    Release,
    Grab,
}

enum WinchAction {
    Relax,
    LengthControl,
    RateControl,
}
```

#### 12.0.7 Wire conversion boundary

`MissionCommand` is the public API type. The wire representation (`MISSION_ITEM_INT` with `command: u16`, `frame`, `param1..4`, `x`, `y`, `z`) is internal.

Bidirectional conversion should be implemented inside the wire normalization layer:

```rust
impl MissionCommand {
    /// Decode a wire mission item into a typed command.
    /// Unknown commands become `MissionCommand::Other`.
    pub(crate) fn from_wire(
        command: u16,
        frame: MissionFrame,
        params: [f32; 4],
        x: i32,
        y: i32,
        z: f32,
    ) -> Self;

    /// Encode a typed command back to wire representation.
    pub(crate) fn to_wire(&self) -> (u16, MissionFrame, [f32; 4], i32, i32, f32);
}
```

This conversion layer replaces the current `u16 → MavCmd` validation in the event loop.
It lives alongside the existing `items_for_wire_upload` / `plan_from_wire_download` normalization.

Round-trip fidelity rules:
- typed commands encode to exactly the wire values that ArduPilot expects
- `Other` preserves raw wire values without loss
- download → typed decode → re-encode should produce identical wire items (within MAVLink float tolerance)
- position fields use `f64` in the public API but convert to/from degE7 `i32` at the wire boundary (lossless: every `i32` value is exactly representable as `f64`)

Wire frame assignment rules for `to_wire()`:
- position-bearing typed commands derive the wire frame from the `GeoPoint3d` variant: `Msl` → `MAV_FRAME_GLOBAL`, `RelHome` → `MAV_FRAME_GLOBAL_RELATIVE_ALT`, `Terrain` → `MAV_FRAME_GLOBAL_TERRAIN_ALT`
- non-position typed commands (e.g. `NavDelay`, `NavReturnToLaunch`, `CondDelay`, `DoJump`) emit `MAV_FRAME_MISSION` (frame 2) on the wire
- `Other(RawMissionCommand)` preserves its stored `frame` value verbatim

#### 12.0.8 Ergonomic comparison

```rust
// Before (requires firmware docs, manual seq tracking, useless autocontinue):
MissionItem {
    seq: 0,
    command: 16,
    frame: MissionFrame::GlobalRelativeAlt,
    autocontinue: true,
    param1: 5.0, param2: 0.0, param3: 0.0, param4: 0.0,
    x: (47.397606 * 1e7) as i32,
    y: (8.545594 * 1e7) as i32,
    z: 50.0,
}

// After — .into() yields a complete MissionItem:
let item: MissionItem = NavWaypoint {
    position: GeoPoint3d::RelHome(GeoPoint3dRelHome {
        latitude_deg: 47.397606,
        longitude_deg: 8.545594,
        relative_alt_m: 50.0,
    }),
    hold_time_s: 5.0,
    acceptance_radius_m: 0.0,
    pass_radius_m: 0.0,
    yaw_deg: 0.0,
}.into(); // autocontinue defaults to true
```

#### 12.0.9 Design rules

- Every data-carrying enum variant wraps a named struct (e.g. `NavCommand::Waypoint(NavWaypoint)`). This makes each command type independently usable as a Rust type for function parameters, generics, trait impls, and builder patterns. Fieldless variants (`ReturnToLaunch`, `SetRoiNone`) remain unit variants.
- Struct names follow the pattern `{Category}{Command}` (e.g. `NavWaypoint`, `DoChangeSpeed`, `CondYaw`). The category prefix avoids name collisions across the NAV/DO/CONDITION namespaces.
- Every command struct implements `From` through the full chain up to `MissionItem`, enabling `.into()` at every level. `NavWaypoint { ... }.into()` produces a complete `MissionItem` with `autocontinue: true`. This is the **preferred ergonomic path** — users should not need to manually nest `MissionCommand::Nav(NavCommand::Waypoint(...))` in common cases. All `From` impls should be generated by a `macro_rules!` macro.
- The typed variant inventory covers all commands accepted by ArduPilot's `AP_Mission::mavlink_int_to_mission_cmd` as of the first rewrite baseline.
- New ArduPilot commands added in future firmware releases roundtrip losslessly through `MissionCommand::Other(RawMissionCommand)` until MAVKit adds a typed variant.
- `MissionFrame` exists only inside `RawMissionCommand` and in the internal wire layer. Typed commands use `GeoPoint3d` for position, which carries the altitude frame implicitly.
- `MissionFrame::Other(u8)` preserves unrecognized wire frame values for roundtrip fidelity.
- All command structs should derive `Clone`, `Debug`, `PartialEq`, `Serialize`, `Deserialize`.
- All supporting enums (`LoiterDirection`, `SpeedType`, etc.) should derive `Clone`, `Copy`, `Debug`, `PartialEq`, `Eq`, `Serialize`, `Deserialize`.
- Wire conversion must be lossless within MAVLink float/degE7 tolerance for all typed variants and exactly lossless for `Other`.

#### 12.0.10 Python ergonomics

Python is dynamically typed, so the NAV/DO/CONDITION enum nesting that helps Rust's type system would be pure friction in Python.

The Python bindings should accept **any command struct directly** wherever a `MissionCommand` is expected.
The PyO3 layer should perform the wrapping automatically using `FromPyObject` / union-style extraction.

```python
# Python — just commands in order, autocontinue defaults to True:
plan = MissionPlan(items=[
    MissionItem(NavWaypoint(
        position=GeoPoint3d.rel_home(lat=47.397606, lon=8.545594, alt=50.0),
        hold_time_s=5.0,
        acceptance_radius_m=0.0,
        pass_radius_m=0.0,
        yaw_deg=0.0,
    )),
    MissionItem(NavTakeoff(
        position=GeoPoint3d.rel_home(lat=0.0, lon=0.0, alt=20.0),
        pitch_deg=15.0,
    )),
    MissionItem(DoChangeSpeed(
        speed_type=SpeedType.GROUNDSPEED,
        speed_mps=5.0,
        throttle_pct=0.0,
    )),
    MissionItem(NavReturnToLaunch()),
    # PX4 workflow — explicit autocontinue=False:
    MissionItem(NavWaypoint(...), autocontinue=False),
])
```

Python rules:
- All command structs (`NavWaypoint`, `DoChangeSpeed`, `CondYaw`, etc.) are directly importable from `mavkit`.
- `MissionItem(command, autocontinue=True)` takes a positional command argument and an optional `autocontinue` keyword argument. The PyO3 layer wraps the command into `MissionCommand` internally.
- Fieldless Rust variants map to zero-argument Python classes (e.g. `NavReturnToLaunch()`, `DoSetRoiNone()`).
- `GeoPoint3d` should expose class methods for convenient construction: `GeoPoint3d.msl(lat, lon, alt)`, `GeoPoint3d.rel_home(lat, lon, alt)`, `GeoPoint3d.terrain(lat, lon, alt)`.
- The `.pyi` stub should type the `MissionItem` constructor's first argument as a union of all command types for IDE support.

### 12.1 Public mission semantics

Public mission APIs should expose only **executable mission items**.

`MissionPlan` should contain:

- ordered executable items
- no hidden home slot
- no sequence numbers — item identity is its index in `items`

Public mission indexing should always be:

- zero-based
- by Vec index

So:

- `set_current(0)` = first real mission item
- counts = `plan.items.len()`
- `DoJump.target_index` refers to a Vec index

Wire sequence numbers (`MISSION_ITEM_INT.seq`) are assigned internally:
- on upload, MAVKit assigns wire seq from the item's Vec index (plus ArduPilot home-slot offset)
- on download, MAVKit strips wire seq and produces a plain `Vec<MissionItem>` in order

Users never see, assign, or validate sequence numbers.

`set_current(index)` remains a short mission command using this Vec-index addressing.
It is not a long-running transfer operation.

#### Upload-time validation

Validation should run at upload time (in the upload path), not during item construction or `to_wire()`:

- `DoJump.target_index` must refer to a valid index within the plan (`< items.len()`). Out-of-range jump targets fail with `MissionValidation`.
- `DoJumpTag.tag` and `DoTag.tag` do not require index validation — they use tag-based addressing resolved by ArduPilot at runtime.
- Empty plans (`items.len() == 0`) are valid: the upload path sends only the home placeholder (wire count=1). Download of an empty vehicle mission returns an empty `Vec<MissionItem>`.
- Position-bearing commands should validate latitude in `[-90, 90]` and longitude in `[-180, 180]` before upload.

### 12.2 ArduPilot slot-0 home quirk must be fully hidden

ArduPilot reserves wire mission item 0 for home.

That slot is:

- synthesized from AHRS home
- excluded from user mission totals
- never executed as a normal mission item
- silently ignored on write/replace
- remapped away during set-current handling

Downstream users should not know or care.

MAVKit should hide this through internal normalization only.

### 12.3 Upload / download normalization

Internal helpers such as `items_for_wire_upload()` and `plan_from_wire_download()` should remain the ArduPilot wire boundary.

They should:

- insert the reserved home slot on upload for ArduPilot mission wire format
- extract and discard it on download
- assign wire sequence numbers from Vec indices (with ArduPilot home-slot offset)
- decode typed `MissionCommand` from wire fields on download
- encode typed `MissionCommand` back to wire fields on upload

Concrete upload rule for the first rewrite:

- MAVKit converts each `MissionItem.command` to wire representation via `MissionCommand::to_wire()`
- for `MissionType::Mission`, MAVKit inserts a zero-placeholder wire item at seq 0 (ArduPilot home slot)
- remaining items are assigned wire seq starting at 1, in Vec order
- ArduPilot treats wire seq 0 as the reserved home slot and replaces it from AHRS home internally

Concrete download rule for the first rewrite:

- for `MissionType::Mission`, MAVKit strips wire seq 0 (the home slot) and discards it
- remaining wire items are decoded via `MissionCommand::from_wire()` into typed `MissionItem` values
- the resulting `Vec<MissionItem>` is in execution order; the item's Vec index is its identity
- wire sequence numbers are not preserved — they are an internal protocol detail

Fence and Rally do **not** use this home-slot normalization path; they have their own typed plan models and separate wire mappings.

This normalization is internal. The public mission API never exposes wire sequence numbers or the home slot.
If callers need home alongside a mission download, they should use the telemetry home domain or keep caller-owned metadata separately.

### 12.4 Home is not part of mission identity

`MissionPlan` should not contain `home` as a first-class field.

If a workflow needs home, it should use the telemetry/home domain or separate offline metadata, not core executable mission semantics.

Mission roundtrip should mean:

- executable items preserved
- order preserved
- params preserved

not byte-for-byte wire identity.

Public mission download therefore returns only executable mission content.
The reserved home slot is a hidden wire quirk, not part of the returned `MissionPlan`.

### 12.5 Mission family domains

The cleanest target shape is to expose mission families as dedicated typed domains rather than forcing all workflows through one generic mission-type parameter.

Conceptually:

```rust
vehicle.mission()
vehicle.fence()
vehicle.rally()
```

Each can still use shared internal transfer machinery, but the user-facing domains should reflect the semantic difference.

`vehicle.mission()` remains the executable-sequence domain.
`vehicle.fence()` and `vehicle.rally()` should instead be modeled as **stored-plan domains**.
They do not expose `set_current`, `current_index`, mission-execution progress, or any other execution-only concepts because ArduPilot does not have an active-item concept for fence or rally.

For the first rewrite, the shared freshness vocabulary for mission/fence/rally/params should be:

```rust
enum SyncState {
    Unknown,
    Current,
    PossiblyStale,
}

enum StoredPlanOperationKind {
    Upload,
    Download,
    Clear,
}

struct FenceState {
    plan: Option<FencePlan>,
    sync: SyncState,
    active_op: Option<StoredPlanOperationKind>,
}

struct RallyState {
    plan: Option<RallyPlan>,
    sync: SyncState,
    active_op: Option<StoredPlanOperationKind>,
}
```

Transition rules for `SyncState`:

- `Unknown` is the correct initial state for a new `Vehicle` before MAVKit has successfully synchronized that domain
- a successful `download()` transitions the domain to `Current` and refreshes the cached plan/state from the remote vehicle
- a successful `upload(plan)` transitions the domain to `Current` and updates the cached plan/state to the uploaded normalized content
- a successful `clear()` transitions the domain to `Current` with an explicitly empty cached plan/state
- `PossiblyStale` means MAVKit still has cached state from this `Vehicle` session but no longer has confidence that it still matches the remote vehicle, for example after an ambiguous transfer outcome or other loss of certainty
- `verify(plan)` may transition mission sync back to `Current` because it refreshes the cached remote mission snapshot as part of verification

#### 12.5.1 Fence domain

Fence should be modeled as a logical geofence plan, not as a raw flat list of MAVLink fence items.

```rust
struct FencePlan {
    return_point: Option<GeoPoint2d>,
    regions: Vec<FenceRegion>,
}

struct FenceInclusionPolygon {
    vertices: Vec<GeoPoint2d>,
    inclusion_group: u8,
}

struct FenceExclusionPolygon {
    vertices: Vec<GeoPoint2d>,
}

struct FenceInclusionCircle {
    center: GeoPoint2d,
    radius_m: f32,
    inclusion_group: u8,
}

struct FenceExclusionCircle {
    center: GeoPoint2d,
    radius_m: f32,
}

enum FenceRegion {
    InclusionPolygon(FenceInclusionPolygon),
    ExclusionPolygon(FenceExclusionPolygon),
    InclusionCircle(FenceInclusionCircle),
    ExclusionCircle(FenceExclusionCircle),
}

impl From<FenceInclusionPolygon> for FenceRegion
impl From<FenceExclusionPolygon> for FenceRegion
impl From<FenceInclusionCircle> for FenceRegion
impl From<FenceExclusionCircle> for FenceRegion

impl FenceHandle {
    fn support(&self) -> SupportStateHandle;
    fn latest(&self) -> Option<FenceState>;
    fn wait(&self) -> FenceState;
    fn subscribe(&self) -> impl Stream<Item = FenceState>;

    fn upload(&self, plan: FencePlan) -> Result<FenceUploadOp, VehicleError>;
    fn download(&self) -> Result<FenceDownloadOp, VehicleError>;
    fn clear(&self) -> Result<FenceClearOp, VehicleError>;
}
```

Fence wire quirks should remain internal:

- MAVKit owns flattening logical regions into ordered `MAV_CMD_NAV_FENCE_*` items on upload
- MAVKit owns regrouping flat downloaded items back into `FenceRegion`s on download
- polygon vertex ordering/count constraints remain wire-level details, not public API requirements
- wire item count is a flat storage detail, not the public `FencePlan` size metric
- `inclusion_group` preserves MAVLink inclusion-group metadata for inclusion regions; exclusion regions have no corresponding field and therefore do not expose one

Fence runtime behavior is a separate concern from fence-plan storage.
Fence enable/disable commands and breach/status telemetry such as `MAV_CMD_DO_FENCE_ENABLE` and `FENCE_STATUS` should not be smuggled into `FenceHandle` in the first rewrite; they belong under telemetry and/or firmware-scoped command APIs later.

**Wire conflict scope**: `FenceHandle` operations share the MAVLink mission microservice wire protocol with `MissionHandle` and `RallyHandle`.
A fence upload, mission download, and rally clear cannot run concurrently on the same vehicle even though they are separate public domains.
Starting any mission-protocol operation while another is active on any of the three domains returns `OperationConflict`.
The error message should explicitly name the conflicting domain and operation kind so callers understand why their fence download was rejected.

#### 12.5.2 Rally domain

Rally should be modeled as a stored list of safe return points, not as executable mission content.

```rust
struct RallyPlan {
    points: Vec<GeoPoint3d>,
}

impl RallyHandle {
    fn support(&self) -> SupportStateHandle;
    fn latest(&self) -> Option<RallyState>;
    fn wait(&self) -> RallyState;
    fn subscribe(&self) -> impl Stream<Item = RallyState>;

    fn upload(&self, plan: RallyPlan) -> Result<RallyUploadOp, VehicleError>;
    fn download(&self) -> Result<RallyDownloadOp, VehicleError>;
    fn clear(&self) -> Result<RallyClearOp, VehicleError>;
}
```

Rally should expose only the MAVLink-roundtrippable subset of rally data.
ArduPilot has additional rally storage fields that are not represented in the standard `MAV_CMD_NAV_RALLY_POINT` mission-item wire format; MAVKit should not pretend those are part of the public MAVLink API.
Rally points use the shared `GeoPoint3d` enum (see §8.2), whose variants map directly to the corresponding MAVLink global altitude frames (`MAV_FRAME_GLOBAL_RELATIVE_ALT`, `MAV_FRAME_GLOBAL`, `MAV_FRAME_GLOBAL_TERRAIN_ALT`) used for `MISSION_ITEM_INT` rally-point items.

Rally selection at RTL time is internal vehicle behavior, not rally-plan state.
The first rewrite should not expose a "current rally point" or similar runtime concept because MAVLink does not provide one.

**Wire conflict scope**: `RallyHandle` operations share the same MAVLink mission microservice conflict scope as `MissionHandle` and `FenceHandle`. See fence domain (§12.5.1) for details.

#### 12.5.3 Fence/rally omissions and shared rules

For the first rewrite, fence and rally should deliberately omit:

- `set_current`
- `current_index`
- mission-execution state
- item-level remote editing APIs
- a public `mission_type` parameter on the fence/rally handles
- `verify(plan)` until fence/rally-specific equivalence rules are worth standardizing

Both domains should use whole-plan operations only: `upload(plan)`, `download()`, and `clear()`.
Those operation handles should follow the same lifecycle rules as mission and parameter operations, including explicit `cancel()`, conflict scopes, and typed `wait()` results.
For the first rewrite, `wait()` should resolve to `FencePlan` / `RallyPlan` for download operations and `()` for upload/clear operations.
Fence/rally support also depends on MAVLink 2 mission-type support; if the vehicle or link cannot support mission types `FENCE` / `RALLY`, the corresponding domain support should remain `Unsupported`.

### 12.6 Mission state versus mission operations

Mission content and mission workflows should not be represented by the same object.

Conceptually:

```rust
enum MissionOperationKind {
    Upload,
    Download,
    Clear,
    Verify,
}

struct MissionState {
    plan: Option<MissionPlan>,
    current_index: Option<u16>,
    sync: SyncState,
    active_op: Option<MissionOperationKind>,
}
```

`vehicle.mission()` should return a `MissionHandle` that exposes both durable state observation and workflow operations.

Concrete observation surface:

```rust
impl MissionHandle<'_> {
    fn latest(&self) -> Option<MissionState>;
    async fn wait(&self) -> MissionState;
    async fn wait_timeout(&self, timeout: Duration) -> Result<MissionState, VehicleError>;
    fn subscribe(&self) -> impl Stream<Item = MissionState>;

    fn upload(&self, plan: MissionPlan) -> Result<MissionUploadOp, VehicleError>;
    fn download(&self) -> Result<MissionDownloadOp, VehicleError>;
    fn clear(&self) -> Result<MissionClearOp, VehicleError>;
    fn verify(&self, plan: MissionPlan) -> Result<MissionVerifyOp, VehicleError>;
    async fn set_current(&self, index: u16) -> Result<(), VehicleError>;
}
```

`MissionState` contains:

- last known/cached mission content
- current executable mission item index normalized from `MISSION_CURRENT`
- mission sync/freshness status (`SyncState`)
- optionally a coarse `active_op` summary for UI convenience

For the first rewrite, `current_index` should be `Option<u16>`.
The hidden wire home slot (`MISSION_CURRENT.seq == 0` for ArduPilot mission type) normalizes to `None` rather than a fake item index.
For ArduPilot mission type, observed wire sequence `seq > 0` normalizes to `Some(seq - 1)`, which is the zero-based Vec index.
No current executable mission item yet also normalizes to `None`.
Transfer workflows do not synthesize this field; it is updated from observed mission-current state.

But long-running workflows should return operation-scoped handles rather than forcing detailed progress and errors into global mission state.

Examples:

```rust
let op = vehicle.mission().upload(plan)?;
let op = vehicle.mission().download()?;
let op = vehicle.mission().clear()?;
let op = vehicle.mission().verify(plan)?;
```

`verify(plan)` is a read-only workflow: MAVKit downloads the current remote mission, applies the same public normalization rules, and compares it against `plan` using the SDK's mission-equivalence rules.
It does not modify the vehicle.
Its progress naturally includes download/receive phases followed by a verification/comparison phase.
For the first rewrite, `wait()` on `verify(plan)` should resolve to `bool`, where `true` means the normalized remote mission is equivalent to the supplied plan.
`false` means the download/normalization succeeded but the compared mission content did not match the supplied plan.
Transfer/protocol/timeout failures still resolve as terminal operation errors, not as `false`.
Successful `verify(plan)` refreshes the cached remote mission snapshot and leaves mission sync state at `Current`, regardless of whether the returned comparison result is `true` or `false`.

Mission equivalence rules for `verify(plan)` should be explicit:

- both sides are compared as public `MissionPlan` values after the normal mission download/upload normalization rules have already removed the hidden home slot and stripped wire sequence numbers
- executable item count and order must match
- each `MissionItem.command` must be structurally equal: same `MissionCommand` variant with matching fields under the SDK's documented numeric tolerance policy
- position fields are compared after normal wire quantization/normalization (degE7 round-trip), not as byte-for-byte original inputs
- hidden-home identity and wire sequence numbers are not part of mission equivalence

Short command-style actions such as `set_current(index)` may remain plain `Result`-returning mission methods rather than operation handles because they do not represent transfer workflows.
The first rewrite should treat that as:

```rust
async fn set_current(&self, index: u16) -> Result<(), VehicleError>;
```

`index` is the zero-based Vec index of the target item in `MissionPlan.items`, not a wire sequence number.
For ArduPilot mission type, MAVKit should translate it to wire sequence `index + 1` (accounting for the hidden home slot), send `MAV_CMD_DO_SET_MISSION_CURRENT`, require a positive command ACK, and then wait for normalized mission state to report `current_index == Some(index)` before resolving success.
Negative ACK fails immediately, and stale/unrelated `MISSION_CURRENT` observations do not satisfy the confirmation contract.

Those operation handles should expose:

- `latest()`
- `subscribe()`
- `wait()`
- `cancel()`

Operation handles are generic over their natural terminal output.
For the first rewrite, `wait()` should resolve to that workflow's terminal result or a terminal operation error.
Examples: mission `download()` returns a `MissionPlan`, mission `verify(plan)` returns `bool`, and mutating workflows such as `upload()` / `clear()` / `write_batch()` return `()` on success.

Progress should be modeled as **phase plus optional counts/context**, not just percent.

Examples:

- `RequestCount`
- `SendingItem { current, total }`
- `ReceivingItem { current, total }`
- `AwaitingAck`
- `Verifying`
- `Completed`
- `Failed`
- `Cancelled`

Detailed runtime errors also belong on the operation handle, not in durable mission state.

Examples:

- `Timeout { phase, item_index? }`
- `AckRejected { code, item_index? }`
- `ProtocolError { code, message }`
- `Cancelled`

Starting a second mission workflow while one is already active should return an immediate `OperationConflict`-style error.

### 12.7 Shared operation-handle lifecycle

Mission and parameter operation handles should follow one shared lifecycle contract:

- the real operation lives in the vehicle event loop, not inside one caller-owned handle object
- all `Vehicle` clones observe the same underlying operation and share the same conflict scope
- dropping an operation handle does **not** cancel the underlying operation automatically
- `cancel()` is explicit and best-effort
- if the link drops, the operation resolves with a connection-related terminal error

`active_op` on the domain handle is only a coarse discoverability aid.
It does not imply that a fully detailed operation handle can always be reconstructed later if every original operation handle has been dropped.

This means the conflict scope for `OperationConflict` is the physical vehicle/domain pair, not one Rust handle instance.
For whole-plan mission-protocol workflows, that scope is shared across `mission()`, `fence()`, and `rally()` on one vehicle because they all use the same MAVLink mission microservice on the wire.
In other words, MAVKit should not run a mission upload concurrently with a fence download or a rally clear on the same vehicle, even though they are separate public domains.

`OperationConflict` should carry enough information for callers to understand the cross-domain conflict:

```rust
struct OperationConflict {
    conflicting_domain: &'static str,  // e.g. "mission", "fence", "rally"
    conflicting_op: &'static str,      // e.g. "upload", "download", "clear"
}
```

This prevents confusing error experiences where a fence download fails with no apparent reason because a mission upload is running.

Operation observation channels should be distinct from telemetry metric/message handles.
They may use the same observation method names (`latest()`, `subscribe()`, `wait()`), but they are backed by operation-status state, not telemetry state.

## 13. Parameters

Parameters remain a dedicated handle:

```rust
vehicle.params()
```

Core operations:

- download all
- write one
- write batch
- param-file import/export helpers

This is a good example of a stable domain handle that MAVKit should continue to emulate elsewhere.

### 13.1 Parameter data types

Concrete parameter types for the first rewrite:

```rust
/// MAVLink parameter value type tag.
enum ParamType {
    Uint8,
    Int8,
    Uint16,
    Int16,
    Uint32,
    Int32,
    Real32,
}

/// A single vehicle parameter.
struct Param {
    name: String,
    value: f32,
    param_type: ParamType,
    index: u16,
}

/// In-memory store of all downloaded vehicle parameters.
struct ParamStore {
    params: HashMap<String, Param>,
    expected_count: u16,
}

/// Result of a single parameter write.
struct ParamWriteResult {
    name: String,
    requested_value: f32,
    confirmed_value: f32,
    success: bool,
}
```

`ParamStore` is the full parameter cache. `Param.value` is always `f32` on the wire regardless of `param_type`; the type tag records the declared semantic type for display and validation. `ParamWriteResult.success` is `true` when `confirmed_value` matches `requested_value` within MAVLink float tolerance.

These types should derive `Clone`, `Debug`, `PartialEq`, `Serialize`, `Deserialize`.

### 13.2 Parameter state versus parameter operations

Like missions, parameters have both durable state and workflow state.

```rust
enum ParamOperationKind {
    DownloadAll,
    WriteBatch,
}

struct ParamState {
    store: Option<ParamStore>,
    sync: SyncState,
    active_op: Option<ParamOperationKind>,
}
```

`vehicle.params()` should return a `ParamsHandle` that exposes both durable state observation and workflow operations.

Concrete observation surface:

```rust
impl ParamsHandle<'_> {
    fn latest(&self) -> Option<ParamState>;
    async fn wait(&self) -> ParamState;
    async fn wait_timeout(&self, timeout: Duration) -> Result<ParamState, VehicleError>;
    fn subscribe(&self) -> impl Stream<Item = ParamState>;

    fn download_all(&self) -> Result<ParamDownloadOp, VehicleError>;
    fn write_batch(&self, batch: Vec<(String, f32)>) -> Result<ParamWriteBatchOp, VehicleError>;
    async fn write(&self, name: &str, value: f32) -> Result<ParamWriteResult, VehicleError>;

    /// Parse a `.param` file into a list of (name, value) pairs.
    fn parse_param_file(content: &str) -> Result<Vec<(String, f32)>, VehicleError>;

    /// Format the current cached store as `.param` file content.
    fn format_param_file(store: &ParamStore) -> String;
}
```

`ParamState` contains:

- the current cached `ParamStore`
- sync/freshness status (`SyncState`)
- optionally a coarse `active_op` summary

Single writes (`write()`) remain plain `Result`-returning convenience methods because they are short ACK-driven exchanges, not transfer workflows.
Batch downloads and batch writes return operation handles with progress/cancel semantics.

Parameter operation handles should expose:

- `latest()`
- `subscribe()`
- `wait()`
- `cancel()`

Progress should again be phase-oriented.

Examples:

- `Downloading { received, expected: Option<u16> }`
- `Writing { index, total, name }`
- `Completed`
- `Failed`
- `Cancelled`

Terminal errors should distinguish start-time errors from runtime completion errors.

Examples:

- immediate start-time: validation error, disconnected, operation conflict
- terminal runtime: timeout, write mismatch, incomplete download, cancelled

MAVKit should not overwrite last-known-good parameter state with partial failed downloads unless the partial state is explicitly marked and isolated.

Parameter operations follow the shared lifecycle rules from Section 12.7.

## 14. Additional MAVLink Domains

The design should leave clean homes for additional MAVLink capability families:

- `vehicle.camera()`
- `vehicle.gimbal()`
- `vehicle.logs()`
- `vehicle.ftp()`

These should follow the same rule set:

- semantic handle first
- raw protocol details hidden by default
- firmware-specific divergence scoped when needed

Not every MAVLink-adjacent utility belongs under the `Vehicle` root.
Feature-gated connection-layer helpers such as byte-stream adapters and BLE / SPP bridges remain transport utilities, not new root domains.
Likewise, offline TLOG readers/writers remain standalone utilities rather than live `Vehicle` capabilities.

## 15. Target Feature Matrix

The table below summarizes the intended target API and the primary MAVLink basis for each feature family.

| Category | Feature family | Primary MAVLink basis | Target API surface |
|---|---|---|---|
| Connection | Link establishment and lifecycle | `HEARTBEAT` + transport lifecycle | `Vehicle::connect*()` returns a connection-ready `Vehicle` / `disconnect()` |
| Initialization | Best-effort metadata enrichment after connect | `AUTOPILOT_VERSION`<br>`AVAILABLE_MODES*`<br>`HOME_POSITION`<br>`GPS_GLOBAL_ORIGIN`<br>`COMPONENT_METADATA` | background after connect + explicit wait helpers where needed |
| Identity | Live routing identity | packet `sysid` / `compid` + `HEARTBEAT.type` / `autopilot` | `vehicle.identity()` |
| Info | Firmware / hardware / unique IDs | `AUTOPILOT_VERSION`<br>`COMPONENT_METADATA`<br>`OPEN_DRONE_ID_*` | `vehicle.info()` |
| Support | Protocol capabilities | `AUTOPILOT_VERSION.capabilities` | `vehicle.support()` |
| Modes | Available / active mode metadata | `AVAILABLE_MODES`<br>`AVAILABLE_MODES_MONITOR`<br>`HEARTBEAT.custom_mode` | `vehicle.available_modes().catalog()`<br>`vehicle.available_modes().current()` |
| Telemetry | Metric namespaces | operator-facing scalar metrics and small grouped value objects over one or more message families | `vehicle.telemetry().armed()`<br>`vehicle.telemetry().battery().voltage_v()`<br>`vehicle.telemetry().position().global()`<br>`vehicle.telemetry().gps().quality()` etc. |
| Telemetry | Message handles | coherent decoded MAVLink telemetry messages | `vehicle.telemetry().messages().global_position_int()`<br>`vehicle.telemetry().messages().gps_raw_int()`<br>`vehicle.telemetry().messages().battery_status(instance)` etc. + eligible handles may offer typed `request(timeout)` and `set_rate(hz)` |
| Telemetry | Status / health concepts | `STATUSTEXT` + `SYS_STATUS` health bitmasks | `vehicle.telemetry().messages().status_text()`<br>`vehicle.telemetry().sensor_health()` |
| Raw MAVLink | Packet/message-level operations | all MAVLink messages and commands | `vehicle.raw()` |
| Home / origin | Observe live home and origin | `HOME_POSITION`<br>`GPS_GLOBAL_ORIGIN` | `vehicle.telemetry().home()`<br>`vehicle.telemetry().origin()` |
| Home / origin | Change home | `MAV_CMD_DO_SET_HOME` | `vehicle.set_home*()` |
| Home / origin | Change origin | `SET_GPS_GLOBAL_ORIGIN` | `vehicle.set_origin()` (observation-confirmed, not `COMMAND_ACK`-confirmed) |
| Shared commands | Broad MAVLink actions | `COMMAND_LONG` / `SET_MODE` / `HEARTBEAT.custom_mode` / `COMMAND_ACK` | `vehicle.arm()`<br>`disarm()`<br>`set_mode()` |
| ArduPilot workflows | Firmware-specific typed behavior | ArduPilot-specific command and state semantics | `vehicle.ardupilot()` |
| Missions | Executable-item mission state | normalized executable mission content + normalized `MISSION_CURRENT` state + sync status | `vehicle.mission().latest()`<br>`subscribe()` |
| Missions | Long-running mission workflows | `MISSION_*` + ArduPilot normalization boundary | `vehicle.mission().upload()` / `download()` / `clear()` / `verify()` -> operation handle |
| Missions | Short mission commands | `MAV_CMD_DO_SET_MISSION_CURRENT` + `MISSION_CURRENT` | `vehicle.mission().set_current(index)` |
| Fence | Stored geofence plan domain | `MISSION_*` with `MAV_MISSION_TYPE_FENCE` + `MAV_CMD_NAV_FENCE_*` | `vehicle.fence().latest()`<br>`subscribe()`<br>`upload(plan)`<br>`download()`<br>`clear()` |
| Rally | Stored safe-point plan domain | `MISSION_*` with `MAV_MISSION_TYPE_RALLY` + `MAV_CMD_NAV_RALLY_POINT` | `vehicle.rally().latest()`<br>`subscribe()`<br>`upload(plan)`<br>`download()`<br>`clear()` |
| Params | Durable parameter state | `PARAM_*` cached store | `vehicle.params().latest()`<br>`subscribe()` |
| Params | Long-running parameter workflows | `PARAM_*` | `vehicle.params().download_all()` / `write_batch()` -> operation handle |
| Payload / data | Camera, gimbal, logs, FTP | relevant MAVLink microservices | `vehicle.camera()`<br>`gimbal()`<br>`logs()`<br>`ftp()` |

## 16. Error Model

Errors should distinguish clear categories that help callers decide whether to retry, resync, or surface a user-facing problem.

Concrete `VehicleError` for the first rewrite:

```rust
enum VehicleError {
    // --- Connection lifecycle ---
    /// Transport/session could not be established.
    ConnectionFailed(String),

    /// Vehicle was connected but the link has been lost or closed.
    Disconnected,

    // --- Command results ---
    /// ACK-driven command was explicitly rejected by the vehicle.
    CommandRejected {
        command: u16,
        result: CommandResult,
    },

    /// Command was sent but the expected outcome was never observed.
    /// The vehicle may or may not have executed it.
    OutcomeUnknown {
        command: u16,
    },

    // --- Timeouts ---
    /// A bounded wait expired. Includes command ACK, observation waits,
    /// transfer phases, and handle `wait()` calls.
    Timeout,

    // --- Capability ---
    /// The requested capability is not supported by this vehicle/session.
    Unsupported(String),

    // --- Validation ---
    /// A caller-provided value failed validation before being sent.
    InvalidParameter(String),

    /// Mode name not found in the current catalog.
    ModeNotAvailable(String),

    // --- Transfer operations (mission, fence, rally, params) ---
    /// Transfer-level protocol failure during a mission/param workflow.
    TransferFailed {
        domain: &'static str,
        phase: String,
        detail: String,
    },

    /// Another operation is already active on the same wire-protocol scope.
    OperationConflict {
        conflicting_domain: &'static str,
        conflicting_op: &'static str,
    },

    /// Operation was explicitly cancelled by the caller.
    Cancelled,

    // --- Validation (mission-specific) ---
    /// Mission plan failed structural validation before transfer.
    MissionValidation(String),

    // --- Identity ---
    /// Operation requires heartbeat-level identity that is not yet available.
    IdentityUnknown,

    // --- I/O ---
    /// Low-level transport I/O error.
    Io(std::io::Error),
}

enum CommandResult {
    Denied,
    Failed,
    Unsupported,
    TemporarilyRejected,
    Other(u8),
}
```

Design rules:

- `CommandRejected` carries the wire `MAV_CMD` and the typed rejection reason. It means the vehicle explicitly said no.
- `OutcomeUnknown` means the command may have been sent before the link dropped or a timeout fired. The caller cannot assume success or failure.
- `Timeout` is a single variant covering all bounded waits. The call site provides context.
- `TransferFailed` covers mission, fence, rally, and param protocol-level failures during active transfers. `domain` identifies which domain ("mission", "fence", "rally", "params"). `phase` and `detail` provide protocol-level context.
- `OperationConflict` carries the domain and operation kind that blocked the new request (see §12.7).
- `InvalidParameter` covers all caller-input validation: out-of-range motor test values, bad throttle, bad duration, etc.
- `Io` is the low-level transport escape hatch.

Start-time errors (returned from operation-creating methods) may be any applicable variant.
Terminal operation errors (resolved from operation handles) are the subset that can occur during an active workflow: `TransferFailed`, `Timeout`, `Disconnected`, `Cancelled`.

Not every root command is ACK-driven.
Commands built on raw state-setting messages rather than `COMMAND_*`/`COMMAND_ACK`, such as `set_origin(...)`, need observation-based confirmation and should report timeout/failure in those terms rather than pretending to have ACK semantics.

For long-running mission and parameter workflows, MAVKit should distinguish **start-time errors** from **terminal operation errors**:

- start-time errors are returned directly when creating the operation handle
- terminal errors are reported by the operation handle when the workflow completes or fails

This keeps global domain state clean and lets downstream apps model uploads/downloads/batch writes as explicit tasks rather than scraping stale shared progress channels.

For ACK-driven command APIs, MAVKit should allow **bounded parallelism across different command kinds**, while never overlapping the same command kind on the same remote target component.

The correlation key is:

- `(target_system, target_component, MAV_CMD)`

`COMMAND_ACK.command` is sufficient only under that rule and only when ACKs are also filtered by the expected remote sender endpoint.
In other words, MAVKit may safely run `arm()` and `set_mode()` in parallel, but it must not run two concurrent `MAV_CMD_COMPONENT_ARM_DISARM` operations to the same target component.

Internally, the dispatcher should keep a small registry keyed by `(target_system, target_component, MAV_CMD)` with one active request per key and a small bounded queue for same-kind followers.
If that per-key queue is full, the SDK should surface a busy/conflict error rather than inventing ambiguous overlap.

This means:

- different ACK-driven command kinds may run in parallel, bounded per target component
- the same `MAV_CMD` never overlaps on the same target component, regardless of parameters or whether it used `COMMAND_LONG` or `COMMAND_INT`
- `COMMAND_ACK` is matched by expected command kind **and** sender endpoint, not treated as a general-purpose request ID
- when `COMMAND_ACK.target_system` / `target_component` are populated, they should also match MAVKit's local endpoint before the ACK is accepted
- terminal keyed responses include `ACCEPTED`, `DENIED`, `FAILED`, `UNSUPPORTED`, `CANCELLED`, and `TEMPORARILY_REJECTED`
- `IN_PROGRESS` is non-terminal: MAVKit should stop retransmitting and continue waiting for the final response
- retries are only for **missing** responses, not for explicit negative responses like `UNSUPPORTED` or `FAILED`
- for `COMMAND_LONG`, retry retransmissions should increment the wire `confirmation` field
- if a public command also requires secondary confirmation beyond ACK (for example mode change confirmed by later state), the same key should remain occupied until the full command contract resolves
- cancellation after send is local-only; the vehicle may still execute the command, so timeout/cancel-after-send means **outcome unknown**, not guaranteed failure

This captures the maximum safe concurrency that MAVLink's ACK model allows without pretending the protocol has request IDs that it does not have.

### 16.1 ACK-driven dispatcher architecture

The command dispatcher should no longer be modeled as one blocking command wait loop over the shared connection.
Instead it should have three explicit pieces:

1. **send/queue layer** — accepts typed or raw ACK-driven commands and places them into the per-key registry
2. **message router** — reads inbound MAVLink traffic once, updates state, and routes ACK-relevant messages to interested waiters
3. **per-key command registry** — owns one active request plus a small bounded FIFO for same-kind followers

This prevents ACK routing from depending on one blocking global waiter and avoids accidental consumption of non-matching ACKs.

The router should:

- inspect every inbound `COMMAND_ACK`
- match it to at most one active key by `(target_system, target_component, MAV_CMD)` and sender endpoint
- ignore/log unmatched late ACKs rather than accidentally satisfying another command
- keep `IN_PROGRESS` entries alive until terminal completion or a bounded completion timeout

Secondary confirmation waiters (for example a mode-change API that also waits for later state) should also register filtered predicates with the dispatcher.
Those predicates must be endpoint-filtered and command-scoped; an unfiltered global heartbeat fallback is not acceptable.

### 16.2 Timeout and retry phases

ACK-driven commands should have two distinct timing phases:

1. **initial-response timeout** — waiting for the first keyed response
2. **completion timeout** — waiting for the terminal outcome after `IN_PROGRESS` or after ACK-plus-secondary-confirmation commands enter their completion phase

Retries apply only during the initial-response phase.
Once the command reaches `IN_PROGRESS` or a public API enters an ACK-plus-secondary-confirmation phase, the dispatcher should stop retransmitting and wait for completion under the bounded completion timeout.

### 16.3 Raw command participation

`raw().command_long()`, `raw().command_int()`, `raw().request_message()`, and `raw().set_message_interval()` all participate in this same ACK-driven registry when they expect keyed responses.

This is important for safety:

- typed commands and raw commands must not be able to overlap unsafely on the same `(target_system, target_component, MAV_CMD)` key
- the escape hatch is about protocol shape, not about bypassing correlation rules

Non-ACK-driven raw packet sends may remain outside the registry, but the docs should mark them clearly as caller-managed protocol operations.

## 17. Python

Rust and Python are one product.

That means:

- conceptual parity is required
- domain grouping should remain recognizable in Python
- observation semantics should match even if Python uses different mechanics
- examples and stubs should evolve with the Rust API

The Python surface does not need to mirror Rust’s internal primitives, but it should mirror the same domain model.

That means, concretely:

- the same top-level split should exist: `telemetry()`, `telemetry().messages()`, grouped value objects, operation handles, `raw()`
- `vehicle.available_modes()` should have a clear Python equivalent with explicit catalog/current observers rather than collapsing back into one ad-hoc list getter
- guided control should remain an explicit session object in Python too; enum-style narrowing may use Pythonic classes/methods, but the session boundary and VTOL-as-plane-extension semantics must remain visible
- repeated handle access should return lightweight wrappers over shared underlying state, not copies of cached values
- grouped value objects should map to normal Python value classes or dataclasses
- `latest()` may be exposed as either a method or a property-like accessor, but `wait()`, `subscribe()`, `request()`, and `set_rate()` should remain clearly asynchronous where the corresponding Rust handle actually exposes them
- operation handles should remain first-class in Python too; they should not collapse back into ad-hoc callback APIs just because Python syntax differs

Python does not have Rust move semantics, so guided-session `close()` cannot literally consume the object.
The Python surface should therefore make `close()` idempotent and mark the session as closed; any later control call on that session should fail with a session-closed style error.
An async context-manager form is acceptable sugar as long as it preserves the same explicit session boundary.

The design should not assume Rust-specific zero-cost wrapper ergonomics.
If a handle chain becomes too awkward in Python, the Python layer may add light sugar, but it must not erase the distinction between metrics, messages, and operation handles.
Transport-specific helpers and offline utilities may remain language-specific until they are deliberately designed on both sides; conceptual parity applies first to the main vehicle/control/telemetry domains.

## 18. Verification

The design assumes two kinds of verification:

- **unit tests** for normalization, merge policy, parsing, and API logic
- **SITL tests** for ArduPilot behavior and wire quirks

ArduPilot-specific behavior should not be trusted without real-firmware validation.

## 19. Rewrite Guidance

This document is intended to be detailed enough to guide a full MAVKit rewrite without further MAVLink or ArduPilot research.

Current repository structures such as flat watch-channel telemetry domains, fully serialized ACK-driven commands, and ad-hoc readiness gaps are implementation legacy, not part of the target public design unless this document says otherwise.

The same applies to current packaging details.
Feature flags such as today's `stream`, `tlog`, or empty marker flags do not by themselves define the target public API shape.
The rewrite may keep, remove, or repurpose those packaging boundaries as needed, as long as the semantic surface described here remains coherent.

In scope for the rewrite are both live-vehicle APIs and adjacent MAVLink utilities when they already belong to the product, but they do not all need to become `Vehicle` root families.
Connection-layer byte-stream adapters remain transport concerns, and offline TLOG parsing/writing remains a standalone utility family rather than live telemetry state.

If new evidence later contradicts a point here, the API should change deliberately. But absent new evidence, this is the design baseline.
