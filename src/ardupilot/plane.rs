use crate::vehicle::VehicleInner;

/// ArduPlane-specific capability accessor.
pub struct ArduPlaneHandle<'a> {
    inner: &'a VehicleInner,
}

impl<'a> ArduPlaneHandle<'a> {
    pub(crate) fn new(inner: &'a VehicleInner) -> Self {
        Self { inner }
    }

    pub fn vtol(&self) -> Option<ArduPlaneVtolHandle<'a>> {
        matches!(
            super::vehicle_family::plane_kind(
                self.inner.stores.vehicle_state.borrow().vehicle_type
            ),
            Some(super::ArduPlaneKind::Vtol)
        )
        .then_some(ArduPlaneVtolHandle::new(self.inner))
    }
}

/// VTOL-only extension handle for ArduPlane vehicles.
pub struct ArduPlaneVtolHandle<'a> {
    _inner: &'a VehicleInner,
}

impl<'a> ArduPlaneVtolHandle<'a> {
    pub(crate) fn new(inner: &'a VehicleInner) -> Self {
        Self { _inner: inner }
    }
}
