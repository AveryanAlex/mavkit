use crate::vehicle::VehicleInner;

/// ArduRover-specific capability accessor.
pub struct ArduRoverHandle<'a> {
    _inner: &'a VehicleInner,
}

impl<'a> ArduRoverHandle<'a> {
    pub(crate) fn new(inner: &'a VehicleInner) -> Self {
        Self { _inner: inner }
    }
}
