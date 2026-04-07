use std::sync::Arc;

use pyo3::prelude::*;

use crate::macros::py_async_unit;

use super::{
    guided_family_unavailable_error, session::PyGuidedSessionShared, sub_goto_depth_target,
};

fn sub_handle(
    session: &mavkit::ArduGuidedSession,
) -> Result<mavkit::ArduSubGuidedHandle<'_>, mavkit::VehicleError> {
    let mavkit::GuidedSpecific::Sub(sub) = session.specific() else {
        return Err(guided_family_unavailable_error("sub"));
    };

    Ok(sub)
}

#[pyclass(name = "ArduSubGuidedHandle", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyArduSubGuidedHandle {
    shared: Arc<PyGuidedSessionShared>,
}

impl PyArduSubGuidedHandle {
    pub(super) fn new(shared: Arc<PyGuidedSessionShared>) -> Self {
        Self { shared }
    }

    async fn goto_depth_impl(
        &self,
        latitude_deg: f64,
        longitude_deg: f64,
        depth_m: f32,
    ) -> Result<(), mavkit::VehicleError> {
        let session = self.shared.session_clone().await?;
        sub_handle(&session)?
            .goto_depth(sub_goto_depth_target(latitude_deg, longitude_deg, depth_m))
            .await
    }

    async fn set_velocity_body_impl(
        &self,
        forward_mps: f32,
        lateral_mps: f32,
        vertical_mps: f32,
        yaw_rate_dps: f32,
    ) -> Result<(), mavkit::VehicleError> {
        let session = self.shared.session_clone().await?;
        sub_handle(&session)?
            .set_velocity_body(forward_mps, lateral_mps, vertical_mps, yaw_rate_dps)
            .await
    }

    async fn hold_impl(&self) -> Result<(), mavkit::VehicleError> {
        let session = self.shared.session_clone().await?;
        sub_handle(&session)?.hold().await
    }
}

#[pymethods]
impl PyArduSubGuidedHandle {
    #[pyo3(signature = (*, latitude_deg, longitude_deg, depth_m))]
    fn goto_depth<'py>(
        &self,
        py: Python<'py>,
        latitude_deg: f64,
        longitude_deg: f64,
        depth_m: f32,
    ) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(
            py,
            handle = self.clone();
            handle.goto_depth_impl(latitude_deg, longitude_deg, depth_m)
        )
    }

    fn set_velocity_body<'py>(
        &self,
        py: Python<'py>,
        forward_mps: f32,
        lateral_mps: f32,
        vertical_mps: f32,
        yaw_rate_dps: f32,
    ) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(
            py,
            handle = self.clone();
            handle.set_velocity_body_impl(forward_mps, lateral_mps, vertical_mps, yaw_rate_dps)
        )
    }

    fn hold<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(py, handle = self.clone(); handle.hold_impl())
    }

    fn __repr__(&self) -> String {
        format!("ArduSubGuidedHandle({})", self.shared.label())
    }
}
