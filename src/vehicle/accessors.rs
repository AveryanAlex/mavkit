use crate::ardupilot::ArduPilotHandle;
use crate::info::InfoHandle;
use crate::link::LinkHandle;
use crate::mission::MissionHandle;
use crate::modes::ModesHandle;
use crate::params::ParamsHandle;
use crate::rally::RallyHandle;
use crate::raw::RawHandle;
use crate::state::VehicleState;
use crate::support::SupportHandle;
use crate::telemetry::TelemetryHandle;
use crate::vehicle::{FenceHandle, Vehicle, VehicleIdentity, VehicleInner};

trait DomainHandle<'a>: Sized {
    fn from_inner(inner: &'a VehicleInner) -> Self;
}

impl<'a> DomainHandle<'a> for MissionHandle<'a> {
    fn from_inner(inner: &'a VehicleInner) -> Self {
        MissionHandle::new(inner)
    }
}

impl<'a> DomainHandle<'a> for FenceHandle<'a> {
    fn from_inner(inner: &'a VehicleInner) -> Self {
        FenceHandle::new(inner)
    }
}

impl<'a> DomainHandle<'a> for RallyHandle<'a> {
    fn from_inner(inner: &'a VehicleInner) -> Self {
        RallyHandle::new(inner)
    }
}

impl<'a> DomainHandle<'a> for ArduPilotHandle<'a> {
    fn from_inner(inner: &'a VehicleInner) -> Self {
        ArduPilotHandle::new(inner)
    }
}

impl<'a> DomainHandle<'a> for ModesHandle<'a> {
    fn from_inner(inner: &'a VehicleInner) -> Self {
        Self::new(inner)
    }
}

impl<'a> DomainHandle<'a> for InfoHandle<'a> {
    fn from_inner(inner: &'a VehicleInner) -> Self {
        Self::new(inner)
    }
}

impl<'a> DomainHandle<'a> for SupportHandle<'a> {
    fn from_inner(inner: &'a VehicleInner) -> Self {
        SupportHandle::new(inner)
    }
}

impl<'a> DomainHandle<'a> for LinkHandle<'a> {
    fn from_inner(inner: &'a VehicleInner) -> Self {
        LinkHandle::new(inner)
    }
}

impl<'a> DomainHandle<'a> for ParamsHandle<'a> {
    fn from_inner(inner: &'a VehicleInner) -> Self {
        ParamsHandle::new(inner)
    }
}

impl Vehicle {
    fn handle<'a, H>(&'a self) -> H
    where
        H: DomainHandle<'a>,
    {
        H::from_inner(self.inner.as_ref())
    }

    /// Return the vehicle's identity snapshot from the first heartbeat.
    ///
    /// This is always populated after a successful `connect` because the connect path waits for
    /// the first heartbeat. If called on a handle constructed outside the connect path before any
    /// heartbeat arrives, the returned fields reflect zero-value defaults.
    pub fn identity(&self) -> VehicleIdentity {
        let state: VehicleState = self.inner.stores.vehicle_state.borrow().clone();

        VehicleIdentity {
            system_id: state.system_id,
            component_id: state.component_id,
            autopilot: state.autopilot,
            vehicle_type: state.vehicle_type,
        }
    }

    /// Access autopilot firmware info (version, capabilities).
    pub fn info(&self) -> InfoHandle<'_> {
        self.handle()
    }

    /// Access vehicle support / capability probing.
    pub fn support(&self) -> SupportHandle<'_> {
        self.handle()
    }

    /// Access link state and statistics.
    pub fn link(&self) -> LinkHandle<'_> {
        self.handle()
    }

    /// Access the available flight-mode catalog reported by the vehicle.
    pub fn available_modes(&self) -> ModesHandle<'_> {
        self.handle()
    }

    /// Access telemetry streams (attitude, position, battery, etc.).
    pub fn telemetry(&self) -> TelemetryHandle<'_> {
        TelemetryHandle::with_command_tx(
            &self.inner.stores.telemetry_handles,
            &self.inner.command_tx,
        )
    }

    /// Access mission upload, download, and monitoring operations.
    pub fn mission(&self) -> MissionHandle<'_> {
        self.handle()
    }

    /// Access geofence upload and download operations.
    pub fn fence(&self) -> FenceHandle<'_> {
        self.handle()
    }

    /// Access rally-point upload and download operations.
    pub fn rally(&self) -> RallyHandle<'_> {
        self.handle()
    }

    /// Access parameter fetch, upload, and typed read operations.
    pub fn params(&self) -> ParamsHandle<'_> {
        self.handle()
    }

    /// Access raw MAVLink send/receive for operations not covered by the typed API.
    pub fn raw(&self) -> RawHandle<'_> {
        RawHandle::new(self)
    }

    /// Access ArduPilot-specific extensions (scripting, EKF, etc.).
    pub fn ardupilot(&self) -> ArduPilotHandle<'_> {
        self.handle()
    }
}
