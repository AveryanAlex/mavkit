mod catalog;
mod domain;
mod handle;
#[cfg(test)]
mod tests;
mod types;

pub use crate::state::FlightMode;
pub use handle::ModesHandle;
pub use types::{CurrentMode, CurrentModeSource, ModeCatalogSource, ModeDescriptor};

pub(crate) use catalog::{mode_name, mode_number};
pub(crate) use domain::ModeDomain;
