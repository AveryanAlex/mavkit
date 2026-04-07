use std::sync::Arc;

use pyo3::prelude::*;

use crate::error::to_py_err;
use crate::macros::py_async_unit;

use super::{
    geo_point_msl, geo_point_rel_home, guided_family_unavailable_error, relative_climb_target,
    session::PyGuidedSessionShared,
};

fn plane_handle(
    session: &mavkit::ArduGuidedSession,
) -> Result<mavkit::ArduPlaneGuidedHandle<'_>, mavkit::VehicleError> {
    let mavkit::GuidedSpecific::Plane(plane) = session.specific() else {
        return Err(guided_family_unavailable_error("plane"));
    };

    Ok(plane)
}

#[pyclass(name = "ArduPlaneGuidedHandle", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyArduPlaneGuidedHandle {
    shared: Arc<PyGuidedSessionShared>,
}

impl PyArduPlaneGuidedHandle {
    pub(super) fn new(shared: Arc<PyGuidedSessionShared>) -> Self {
        Self { shared }
    }

    async fn reposition_impl(
        &self,
        latitude_deg: f64,
        longitude_deg: f64,
        altitude_msl_m: f64,
    ) -> Result<(), mavkit::VehicleError> {
        let session = self.shared.session_clone().await?;
        plane_handle(&session)?
            .reposition(geo_point_msl(latitude_deg, longitude_deg, altitude_msl_m))
            .await
    }

    async fn reposition_rel_home_impl(
        &self,
        latitude_deg: f64,
        longitude_deg: f64,
        relative_alt_m: f64,
    ) -> Result<(), mavkit::VehicleError> {
        let session = self.shared.session_clone().await?;
        plane_handle(&session)?
            .reposition_rel_home(geo_point_rel_home(
                latitude_deg,
                longitude_deg,
                relative_alt_m,
            ))
            .await
    }

    #[cfg_attr(not(test), allow(dead_code))]
    pub(crate) fn vtol_impl(
        &self,
    ) -> Result<Option<PyArduPlaneVtolGuidedHandle>, mavkit::VehicleError> {
        self.shared.ensure_python_open()?;
        Ok(
            (self.shared.plane_kind() == Some(mavkit::ArduPlaneKind::Vtol))
                .then_some(PyArduPlaneVtolGuidedHandle::new(self.shared.clone())),
        )
    }
}

#[pymethods]
impl PyArduPlaneGuidedHandle {
    #[pyo3(signature = (*, latitude_deg, longitude_deg, altitude_msl_m))]
    fn reposition<'py>(
        &self,
        py: Python<'py>,
        latitude_deg: f64,
        longitude_deg: f64,
        altitude_msl_m: f64,
    ) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(
            py,
            handle = self.clone();
            handle.reposition_impl(latitude_deg, longitude_deg, altitude_msl_m)
        )
    }

    #[pyo3(signature = (*, latitude_deg, longitude_deg, relative_alt_m))]
    fn reposition_rel_home<'py>(
        &self,
        py: Python<'py>,
        latitude_deg: f64,
        longitude_deg: f64,
        relative_alt_m: f64,
    ) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(
            py,
            handle = self.clone();
            handle.reposition_rel_home_impl(latitude_deg, longitude_deg, relative_alt_m)
        )
    }

    fn vtol(&self) -> PyResult<Option<PyArduPlaneVtolGuidedHandle>> {
        self.vtol_impl().map_err(to_py_err)
    }

    fn __repr__(&self) -> String {
        format!("ArduPlaneGuidedHandle({})", self.shared.label())
    }
}

#[pyclass(name = "ArduPlaneVtolGuidedHandle", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyArduPlaneVtolGuidedHandle {
    shared: Arc<PyGuidedSessionShared>,
}

impl PyArduPlaneVtolGuidedHandle {
    pub(super) fn new(shared: Arc<PyGuidedSessionShared>) -> Self {
        Self { shared }
    }

    async fn takeoff_impl(&self, relative_climb_m: f32) -> Result<(), mavkit::VehicleError> {
        let session = self.shared.session_clone().await?;
        let mavkit::GuidedSpecific::Plane(plane) = session.specific() else {
            return Err(guided_family_unavailable_error("plane"));
        };
        let vtol = plane
            .vtol()
            .ok_or_else(|| guided_family_unavailable_error("plane VTOL"))?;

        vtol.takeoff(relative_climb_target(relative_climb_m)).await
    }

    async fn hold_impl(&self) -> Result<(), mavkit::VehicleError> {
        let session = self.shared.session_clone().await?;
        let mavkit::GuidedSpecific::Plane(plane) = session.specific() else {
            return Err(guided_family_unavailable_error("plane"));
        };
        let vtol = plane
            .vtol()
            .ok_or_else(|| guided_family_unavailable_error("plane VTOL"))?;

        vtol.hold().await
    }
}

#[pymethods]
impl PyArduPlaneVtolGuidedHandle {
    fn takeoff<'py>(&self, py: Python<'py>, relative_climb_m: f32) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(py, handle = self.clone(); handle.takeoff_impl(relative_climb_m))
    }

    fn hold<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(py, handle = self.clone(); handle.hold_impl())
    }

    fn __repr__(&self) -> String {
        format!("ArduPlaneVtolGuidedHandle({})", self.shared.label())
    }
}
