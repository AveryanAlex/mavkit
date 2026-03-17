use crate::vehicle::VehicleInner;

/// ArduCopter-specific capability accessor.
pub struct ArduCopterHandle<'a> {
    _inner: &'a VehicleInner,
}

impl<'a> ArduCopterHandle<'a> {
    pub(crate) fn new(inner: &'a VehicleInner) -> Self {
        Self { _inner: inner }
    }
}
