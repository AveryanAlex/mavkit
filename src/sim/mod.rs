//! Feature-gated in-process demo MAVLink simulator.

mod api;
mod dynamics;
mod mission;
mod params;
mod power;
mod protocol;
mod runtime;
mod state;
mod transport;

pub use api::{
    DemoClock, DemoProfile, DemoVehicle, DemoVehicleBuilder, DemoVehicleHandle, DemoVehicleSnapshot,
};
