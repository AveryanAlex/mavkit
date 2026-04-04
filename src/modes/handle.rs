use crate::modes::{CurrentMode, FlightMode, ModeDescriptor};
use crate::observation::ObservationHandle;
use crate::types::SupportState;
use crate::vehicle::VehicleInner;

/// Accessor for mode support, catalog, and current mode observations.
///
/// Obtained from [`Vehicle::available_modes`](crate::Vehicle::available_modes).
///
/// The mode catalog is built from the first available source:
/// 1. `AVAILABLE_MODES` messages from the vehicle (MAVLink 2.0 extension, preferred)
/// 2. A static ArduPilot look-up table keyed on vehicle type (copter/plane/rover)
///
/// The catalog is refreshed whenever an `AVAILABLE_MODES_MONITOR` sequence-number change is
/// detected. The current mode is updated from `CURRENT_MODE` messages (which carry the optional
/// `intended_custom_mode`) or from the heartbeat `custom_mode` field as a fallback.
///
/// All observation handles returned here have their initial value seeded from the latest known
/// vehicle state, so callers get a non-stale result even before the async loop pushes an update.
#[derive(Clone)]
pub struct ModesHandle<'a> {
    inner: &'a VehicleInner,
}

impl<'a> ModesHandle<'a> {
    pub(crate) fn new(inner: &'a VehicleInner) -> Self {
        Self { inner }
    }

    /// Returns a capability-support observation for the modes domain.
    ///
    /// Becomes [`SupportState::Supported`](crate::types::SupportState::Supported) once the first
    /// heartbeat is received.
    pub fn support(&self) -> ObservationHandle<SupportState> {
        self.seed_from_vehicle_state();
        self.inner.modes.support()
    }

    /// Returns a live observation of the current mode catalog.
    ///
    /// The catalog is replaced wholesale when a new `AVAILABLE_MODES` set arrives or when the
    /// vehicle identity changes (e.g. reconnect with different autopilot type).
    pub fn catalog(&self) -> ObservationHandle<Vec<ModeDescriptor>> {
        self.seed_from_vehicle_state();
        self.inner.modes.catalog()
    }

    /// Returns a live observation of the currently active mode.
    ///
    /// Updated from `CURRENT_MODE` messages when available (includes `intended_custom_mode`),
    /// otherwise from the heartbeat `custom_mode` field.
    pub fn current(&self) -> ObservationHandle<CurrentMode> {
        self.seed_from_vehicle_state();
        self.inner.modes.current()
    }

    fn seed_from_vehicle_state(&self) {
        let state = self.inner.stores.vehicle_state.borrow().clone();
        self.inner.modes.seed_from_vehicle_state(&state);
    }

    fn snapshot(&self) -> Vec<FlightMode> {
        self.catalog()
            .latest()
            .unwrap_or_default()
            .into_iter()
            .map(|mode| FlightMode {
                custom_mode: mode.custom_mode,
                name: mode.name,
            })
            .collect()
    }

    /// Returns the number of modes in the current catalog snapshot.
    pub fn len(&self) -> usize {
        self.snapshot().len()
    }

    /// Returns `true` if the catalog snapshot is empty (no modes known yet).
    pub fn is_empty(&self) -> bool {
        self.snapshot().is_empty()
    }

    /// Returns an iterator over a snapshot of the current mode catalog.
    ///
    /// Each item is a [`FlightMode`] with `custom_mode` and `name`. The snapshot is taken at
    /// call time; changes to the catalog after this call are not reflected in the iterator.
    pub fn iter(&self) -> std::vec::IntoIter<FlightMode> {
        self.snapshot().into_iter()
    }
}

impl IntoIterator for ModesHandle<'_> {
    type Item = FlightMode;
    type IntoIter = std::vec::IntoIter<FlightMode>;

    fn into_iter(self) -> Self::IntoIter {
        self.snapshot().into_iter()
    }
}

impl IntoIterator for &ModesHandle<'_> {
    type Item = FlightMode;
    type IntoIter = std::vec::IntoIter<FlightMode>;

    fn into_iter(self) -> Self::IntoIter {
        self.snapshot().into_iter()
    }
}
