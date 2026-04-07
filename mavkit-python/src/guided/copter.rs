use std::sync::Arc;

use pyo3::prelude::*;

use crate::macros::py_async_unit;

use super::{
    geo_point_msl, geo_point_rel_home, guided_family_unavailable_error, relative_climb_target,
    session::PyGuidedSessionShared,
};

fn copter_handle(
    session: &mavkit::ArduGuidedSession,
) -> Result<mavkit::ArduCopterGuidedHandle<'_>, mavkit::VehicleError> {
    let mavkit::GuidedSpecific::Copter(copter) = session.specific() else {
        return Err(guided_family_unavailable_error("copter"));
    };

    Ok(copter)
}

#[pyclass(name = "ArduCopterGuidedHandle", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyArduCopterGuidedHandle {
    shared: Arc<PyGuidedSessionShared>,
}

impl PyArduCopterGuidedHandle {
    pub(super) fn new(shared: Arc<PyGuidedSessionShared>) -> Self {
        Self { shared }
    }

    async fn takeoff_impl(&self, relative_climb_m: f32) -> Result<(), mavkit::VehicleError> {
        let session = self.shared.session_clone().await?;
        copter_handle(&session)?
            .takeoff(relative_climb_target(relative_climb_m))
            .await
    }

    async fn goto_impl(
        &self,
        latitude_deg: f64,
        longitude_deg: f64,
        relative_alt_m: f64,
    ) -> Result<(), mavkit::VehicleError> {
        let session = self.shared.session_clone().await?;
        copter_handle(&session)?
            .goto(geo_point_rel_home(
                latitude_deg,
                longitude_deg,
                relative_alt_m,
            ))
            .await
    }

    async fn goto_msl_impl(
        &self,
        latitude_deg: f64,
        longitude_deg: f64,
        altitude_msl_m: f64,
    ) -> Result<(), mavkit::VehicleError> {
        let session = self.shared.session_clone().await?;
        copter_handle(&session)?
            .goto_msl(geo_point_msl(latitude_deg, longitude_deg, altitude_msl_m))
            .await
    }

    async fn set_velocity_ned_impl(
        &self,
        north_mps: f32,
        east_mps: f32,
        down_mps: f32,
    ) -> Result<(), mavkit::VehicleError> {
        let session = self.shared.session_clone().await?;
        copter_handle(&session)?
            .set_velocity_ned(north_mps, east_mps, down_mps)
            .await
    }

    async fn hold_impl(&self) -> Result<(), mavkit::VehicleError> {
        let session = self.shared.session_clone().await?;
        copter_handle(&session)?.hold().await
    }
}

#[pymethods]
impl PyArduCopterGuidedHandle {
    fn takeoff<'py>(&self, py: Python<'py>, relative_climb_m: f32) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(py, handle = self.clone(); handle.takeoff_impl(relative_climb_m))
    }

    #[pyo3(signature = (*, latitude_deg, longitude_deg, relative_alt_m))]
    fn goto<'py>(
        &self,
        py: Python<'py>,
        latitude_deg: f64,
        longitude_deg: f64,
        relative_alt_m: f64,
    ) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(
            py,
            handle = self.clone();
            handle.goto_impl(latitude_deg, longitude_deg, relative_alt_m)
        )
    }

    #[pyo3(signature = (*, latitude_deg, longitude_deg, altitude_msl_m))]
    fn goto_msl<'py>(
        &self,
        py: Python<'py>,
        latitude_deg: f64,
        longitude_deg: f64,
        altitude_msl_m: f64,
    ) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(
            py,
            handle = self.clone();
            handle.goto_msl_impl(latitude_deg, longitude_deg, altitude_msl_m)
        )
    }

    fn set_velocity_ned<'py>(
        &self,
        py: Python<'py>,
        north_mps: f32,
        east_mps: f32,
        down_mps: f32,
    ) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(
            py,
            handle = self.clone();
            handle.set_velocity_ned_impl(north_mps, east_mps, down_mps)
        )
    }

    fn hold<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(py, handle = self.clone(); handle.hold_impl())
    }

    fn __repr__(&self) -> String {
        format!("ArduCopterGuidedHandle({})", self.shared.label())
    }
}
