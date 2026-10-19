//! ArduPilot-specific domain state, capability handles, and regression seams.
//!
//! Public access remains rooted at [`ArduPilotHandle`] and the family-specific handles while
//! domain state, handle behavior, and exact-path regressions live in smaller files.

pub mod parameters;

mod calibration;
pub mod copter;
mod domain;
pub mod guided;
mod handle;
mod mag_cal;
pub mod plane;
pub mod rover;
pub mod sub;
#[cfg(test)]
mod tests;
pub mod types;
mod vehicle_family;

pub use copter::ArduCopterHandle;
pub(crate) use domain::ArduPilotDomain;
pub use guided::{
    ArduCopterGuidedHandle, ArduGuidedKind, ArduGuidedSession, ArduPlaneGuidedHandle,
    ArduPlaneKind, ArduPlaneVtolGuidedHandle, ArduRoverGuidedHandle, ArduSubGuidedHandle,
    GuidedSpecific, RelativeClimbTarget, SubGotoDepthTarget,
};
pub use handle::ArduPilotHandle;
pub use plane::{ArduPlaneHandle, ArduPlaneVtolHandle};
pub use rover::ArduRoverHandle;
pub use sub::ArduSubHandle;
pub use types::{MagCalProgress, MagCalReport, MagCalStatus};
