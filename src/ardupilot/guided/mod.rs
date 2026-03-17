mod copter;
mod plane;
mod rover;
mod session;
mod sub;

pub use plane::{ArduPlaneGuidedHandle, ArduPlaneKind, ArduPlaneVtolGuidedHandle};
pub use session::ArduGuidedSession;
pub(crate) use session::GuidedLeaseScope;
pub(crate) use session::GuidedSessionInit;

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// Vehicle-family discriminator for an active guided session.
pub enum ArduGuidedKind {
    Copter,
    Plane,
    Rover,
    Sub,
}

#[derive(Debug)]
/// Family-specific guided control handle for a session.
pub enum GuidedSpecific<'a> {
    Copter(ArduCopterGuidedHandle<'a>),
    Plane(ArduPlaneGuidedHandle<'a>),
    Rover(ArduRoverGuidedHandle<'a>),
    Sub(ArduSubGuidedHandle<'a>),
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
/// Relative climb target used by VTOL and copter takeoff helpers.
pub struct RelativeClimbTarget {
    pub relative_climb_m: f32,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
/// Horizontal point and depth target for submarine guided goto.
pub struct SubGotoDepthTarget {
    pub point: crate::GeoPoint2d,
    pub depth_m: f32,
}

#[derive(Debug)]
/// Guided control surface for ArduCopter sessions.
pub struct ArduCopterGuidedHandle<'a> {
    pub(crate) _session: &'a ArduGuidedSession,
}

#[derive(Debug)]
/// Guided control surface for ArduRover sessions.
pub struct ArduRoverGuidedHandle<'a> {
    pub(crate) _session: &'a ArduGuidedSession,
}

impl<'a> ArduRoverGuidedHandle<'a> {
    pub(crate) fn new(session: &'a ArduGuidedSession) -> Self {
        Self { _session: session }
    }
}

#[derive(Debug)]
/// Guided control surface for ArduSub sessions.
pub struct ArduSubGuidedHandle<'a> {
    pub(crate) _session: &'a ArduGuidedSession,
}

impl<'a> ArduSubGuidedHandle<'a> {
    pub(crate) fn new(session: &'a ArduGuidedSession) -> Self {
        Self { _session: session }
    }
}
