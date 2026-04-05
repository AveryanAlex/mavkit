use pyo3::prelude::*;

use crate::ardupilot::PyArduPilotHandle;
use crate::config::PyVehicleConfig;
use crate::enums::{PyAutopilotType, PyVehicleType};
use crate::error::to_py_err;
use crate::info::PyInfoHandle;
use crate::link::PyLinkHandle;
use crate::macros::py_async_unit;
use crate::modes::PyModesHandle;
use crate::params::PyParamsHandle;
use crate::support::PySupportHandle;
use crate::telemetry::PyTelemetryHandle;
use crate::vehicle::plan_handles::{PyFenceHandle, PyMissionHandle, PyRallyHandle};
use crate::vehicle::raw::PyRawHandle;

fn geo_point_msl(
    latitude_deg: f64,
    longitude_deg: f64,
    altitude_msl_m: f64,
) -> mavkit::GeoPoint3dMsl {
    mavkit::GeoPoint3dMsl {
        latitude_deg,
        longitude_deg,
        altitude_msl_m,
    }
}

#[pyclass(name = "VehicleIdentity", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyVehicleIdentity {
    pub(crate) inner: mavkit::VehicleIdentity,
}

#[pymethods]
impl PyVehicleIdentity {
    #[getter]
    fn system_id(&self) -> u8 {
        self.inner.system_id
    }

    #[getter]
    fn component_id(&self) -> u8 {
        self.inner.component_id
    }

    #[getter]
    fn autopilot(&self) -> PyAutopilotType {
        self.inner.autopilot.into()
    }

    #[getter]
    fn vehicle_type(&self) -> PyVehicleType {
        self.inner.vehicle_type.into()
    }

    fn __repr__(&self) -> String {
        format!(
            "VehicleIdentity(sys={}, comp={}, autopilot={:?}, type={:?})",
            self.inner.system_id,
            self.inner.component_id,
            self.inner.autopilot,
            self.inner.vehicle_type
        )
    }
}

#[pyclass(name = "Vehicle", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyVehicle {
    pub(crate) inner: mavkit::Vehicle,
}

impl PyVehicle {
    #[cfg(any(test, feature = "test-support"))]
    pub(crate) fn from_inner(inner: mavkit::Vehicle) -> Self {
        Self { inner }
    }
}

#[pymethods]
impl PyVehicle {
    #[staticmethod]
    fn connect<'py>(py: Python<'py>, address: &str) -> PyResult<Bound<'py, PyAny>> {
        let addr = address.to_string();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let vehicle = mavkit::Vehicle::connect(&addr).await.map_err(to_py_err)?;
            Ok(Self { inner: vehicle })
        })
    }

    #[staticmethod]
    fn connect_udp<'py>(py: Python<'py>, bind_addr: &str) -> PyResult<Bound<'py, PyAny>> {
        let addr = bind_addr.to_string();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let vehicle = mavkit::Vehicle::connect_udp(&addr)
                .await
                .map_err(to_py_err)?;
            Ok(Self { inner: vehicle })
        })
    }

    #[staticmethod]
    fn connect_tcp<'py>(py: Python<'py>, addr: &str) -> PyResult<Bound<'py, PyAny>> {
        let address = addr.to_string();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let vehicle = mavkit::Vehicle::connect_tcp(&address)
                .await
                .map_err(to_py_err)?;
            Ok(Self { inner: vehicle })
        })
    }

    #[staticmethod]
    fn connect_serial<'py>(py: Python<'py>, port: &str, baud: u32) -> PyResult<Bound<'py, PyAny>> {
        let serial_port = port.to_string();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let vehicle = mavkit::Vehicle::connect_serial(&serial_port, baud)
                .await
                .map_err(to_py_err)?;
            Ok(Self { inner: vehicle })
        })
    }

    #[staticmethod]
    fn connect_with_config<'py>(
        py: Python<'py>,
        address: &str,
        config: PyVehicleConfig,
    ) -> PyResult<Bound<'py, PyAny>> {
        let addr = address.to_string();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let vehicle = mavkit::Vehicle::connect_with_config(&addr, config.into_inner())
                .await
                .map_err(to_py_err)?;
            Ok(Self { inner: vehicle })
        })
    }

    fn telemetry(&self) -> PyTelemetryHandle {
        PyTelemetryHandle::new(self.inner.clone())
    }

    fn available_modes(&self) -> PyModesHandle {
        PyModesHandle::new(self.inner.clone())
    }

    fn info(&self) -> PyInfoHandle {
        PyInfoHandle::new(self.inner.clone())
    }

    fn support(&self) -> PySupportHandle {
        PySupportHandle::new(self.inner.clone())
    }

    fn link(&self) -> PyLinkHandle {
        PyLinkHandle::new(self.inner.clone())
    }

    fn mission(&self) -> PyMissionHandle {
        PyMissionHandle::new(self.inner.clone())
    }

    fn fence(&self) -> PyFenceHandle {
        PyFenceHandle::new(self.inner.clone())
    }

    fn rally(&self) -> PyRallyHandle {
        PyRallyHandle::new(self.inner.clone())
    }

    fn params(&self) -> PyParamsHandle {
        PyParamsHandle::new(self.inner.clone())
    }

    fn raw(&self) -> PyRawHandle {
        PyRawHandle::new(self.inner.clone())
    }

    fn ardupilot(&self) -> PyArduPilotHandle {
        PyArduPilotHandle::new(self.inner.clone())
    }

    fn identity(&self) -> PyVehicleIdentity {
        PyVehicleIdentity {
            inner: self.inner.identity(),
        }
    }

    fn arm<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(py, vehicle = self.inner.clone(); vehicle.arm())
    }

    fn arm_no_wait<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(py, vehicle = self.inner.clone(); vehicle.arm_no_wait())
    }

    fn force_arm<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(py, vehicle = self.inner.clone(); vehicle.force_arm())
    }

    fn force_arm_no_wait<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(py, vehicle = self.inner.clone(); vehicle.force_arm_no_wait())
    }

    fn disarm<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(py, vehicle = self.inner.clone(); vehicle.disarm())
    }

    fn disarm_no_wait<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(py, vehicle = self.inner.clone(); vehicle.disarm_no_wait())
    }

    fn force_disarm<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(py, vehicle = self.inner.clone(); vehicle.force_disarm())
    }

    fn force_disarm_no_wait<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(py, vehicle = self.inner.clone(); vehicle.force_disarm_no_wait())
    }

    fn set_mode<'py>(&self, py: Python<'py>, custom_mode: u32) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(py, vehicle = self.inner.clone(); vehicle.set_mode(custom_mode))
    }

    fn set_mode_no_wait<'py>(
        &self,
        py: Python<'py>,
        custom_mode: u32,
    ) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(py, vehicle = self.inner.clone(); vehicle.set_mode_no_wait(custom_mode))
    }

    fn set_mode_by_name<'py>(&self, py: Python<'py>, name: &str) -> PyResult<Bound<'py, PyAny>> {
        let mode_name = name.to_string();
        py_async_unit!(py, vehicle = self.inner.clone(); vehicle.set_mode_by_name(&mode_name))
    }

    fn set_mode_by_name_no_wait<'py>(
        &self,
        py: Python<'py>,
        name: &str,
    ) -> PyResult<Bound<'py, PyAny>> {
        let mode_name = name.to_string();
        py_async_unit!(
            py,
            vehicle = self.inner.clone();
            vehicle.set_mode_by_name_no_wait(&mode_name)
        )
    }

    #[pyo3(signature = (*, latitude_deg, longitude_deg, altitude_msl_m))]
    fn set_home<'py>(
        &self,
        py: Python<'py>,
        latitude_deg: f64,
        longitude_deg: f64,
        altitude_msl_m: f64,
    ) -> PyResult<Bound<'py, PyAny>> {
        let point = geo_point_msl(latitude_deg, longitude_deg, altitude_msl_m);
        py_async_unit!(py, vehicle = self.inner.clone(); vehicle.set_home(point))
    }

    fn set_home_current<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(py, vehicle = self.inner.clone(); vehicle.set_home_current())
    }

    #[pyo3(signature = (*, latitude_deg, longitude_deg, altitude_msl_m))]
    fn set_origin<'py>(
        &self,
        py: Python<'py>,
        latitude_deg: f64,
        longitude_deg: f64,
        altitude_msl_m: f64,
    ) -> PyResult<Bound<'py, PyAny>> {
        let point = geo_point_msl(latitude_deg, longitude_deg, altitude_msl_m);
        py_async_unit!(py, vehicle = self.inner.clone(); vehicle.set_origin(point))
    }

    fn disconnect<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(py, vehicle = self.inner.clone(); vehicle.disconnect())
    }

    fn __aenter__<'py>(slf: &Bound<'py, Self>, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let obj = slf.clone().into_any().unbind();
        pyo3_async_runtimes::tokio::future_into_py(py, async move { Ok(obj) })
    }

    #[pyo3(signature = (_exc_type=None, _exc_val=None, _exc_tb=None))]
    fn __aexit__<'py>(
        &self,
        py: Python<'py>,
        _exc_type: Option<&Bound<'py, PyAny>>,
        _exc_val: Option<&Bound<'py, PyAny>>,
        _exc_tb: Option<&Bound<'py, PyAny>>,
    ) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            vehicle.disconnect().await.map_err(to_py_err)?;
            Ok(false)
        })
    }

    fn __repr__(&self) -> String {
        let identity = self.inner.identity();
        format!(
            "Vehicle(sys={}, comp={}, autopilot={:?}, type={:?})",
            identity.system_id, identity.component_id, identity.autopilot, identity.vehicle_type,
        )
    }
}
