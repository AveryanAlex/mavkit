use mavkit::Vehicle;
#[cfg(feature = "sim")]
use mavkit::sim::DemoVehicleHandle;

pub struct BackendVehicle {
    pub vehicle: Vehicle,
    #[cfg(feature = "sim")]
    pub(crate) demo_handle: Option<DemoVehicleHandle>,
}

pub async fn disconnect(backend: BackendVehicle) {
    let _ = backend.vehicle.disconnect().await;
    #[cfg(feature = "sim")]
    if let Some(demo_handle) = backend.demo_handle.as_ref() {
        let _ = demo_handle.shutdown().await;
    }
}
