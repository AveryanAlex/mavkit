use crate::vehicle::VehicleInner;

/// ArduSub-specific capability accessor.
pub struct ArduSubHandle<'a> {
    _inner: &'a VehicleInner,
}

impl<'a> ArduSubHandle<'a> {
    pub(crate) fn new(inner: &'a VehicleInner) -> Self {
        Self { _inner: inner }
    }
}
