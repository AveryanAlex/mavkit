use std::sync::Arc;

use pyo3::prelude::*;

use crate::macros::py_async_unit;

use super::{geo_point_2d, guided_family_unavailable_error, session::PyGuidedSessionShared};

fn rover_handle(
    session: &mavkit::ArduGuidedSession,
) -> Result<mavkit::ArduRoverGuidedHandle<'_>, mavkit::VehicleError> {
    let mavkit::GuidedSpecific::Rover(rover) = session.specific() else {
        return Err(guided_family_unavailable_error("rover"));
    };

    Ok(rover)
}

#[pyclass(name = "ArduRoverGuidedHandle", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyArduRoverGuidedHandle {
    shared: Arc<PyGuidedSessionShared>,
}

impl PyArduRoverGuidedHandle {
    pub(super) fn new(shared: Arc<PyGuidedSessionShared>) -> Self {
        Self { shared }
    }

    pub(crate) async fn drive_to_impl(
        &self,
        latitude_deg: f64,
        longitude_deg: f64,
    ) -> Result<(), mavkit::VehicleError> {
        let session = self.shared.session_clone().await?;
        rover_handle(&session)?
            .drive_to(geo_point_2d(latitude_deg, longitude_deg))
            .await
    }

    pub(crate) async fn drive_impl(
        &self,
        forward_mps: f32,
        turn_rate_dps: f32,
    ) -> Result<(), mavkit::VehicleError> {
        let session = self.shared.session_clone().await?;
        rover_handle(&session)?
            .drive(forward_mps, turn_rate_dps)
            .await
    }

    async fn hold_impl(&self) -> Result<(), mavkit::VehicleError> {
        let session = self.shared.session_clone().await?;
        rover_handle(&session)?.hold().await
    }
}

#[pymethods]
impl PyArduRoverGuidedHandle {
    #[pyo3(signature = (*, latitude_deg, longitude_deg))]
    fn drive_to<'py>(
        &self,
        py: Python<'py>,
        latitude_deg: f64,
        longitude_deg: f64,
    ) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(
            py,
            handle = self.clone();
            handle.drive_to_impl(latitude_deg, longitude_deg)
        )
    }

    fn drive<'py>(
        &self,
        py: Python<'py>,
        forward_mps: f32,
        turn_rate_dps: f32,
    ) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(
            py,
            handle = self.clone();
            handle.drive_impl(forward_mps, turn_rate_dps)
        )
    }

    fn hold<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(py, handle = self.clone(); handle.hold_impl())
    }

    fn __repr__(&self) -> String {
        format!("ArduRoverGuidedHandle({})", self.shared.label())
    }
}
