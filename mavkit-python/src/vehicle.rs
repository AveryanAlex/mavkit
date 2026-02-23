use num_traits::FromPrimitive;
use pyo3::prelude::*;

use crate::config::PyVehicleConfig;
use crate::enums::*;
use crate::error::to_py_err;
use crate::mission::*;
use crate::params::*;
use crate::state::*;

#[pyclass(name = "Vehicle", frozen, skip_from_py_object)]
pub struct PyVehicle {
    inner: mavkit::Vehicle,
}

#[pymethods]
impl PyVehicle {
    // --- Connection (static async constructors) ---

    #[staticmethod]
    fn connect<'py>(py: Python<'py>, address: &str) -> PyResult<Bound<'py, PyAny>> {
        let addr = address.to_string();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let v = mavkit::Vehicle::connect(&addr).await.map_err(to_py_err)?;
            Ok(PyVehicle { inner: v })
        })
    }

    #[staticmethod]
    fn connect_udp<'py>(py: Python<'py>, bind_addr: &str) -> PyResult<Bound<'py, PyAny>> {
        let addr = bind_addr.to_string();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let v = mavkit::Vehicle::connect_udp(&addr)
                .await
                .map_err(to_py_err)?;
            Ok(PyVehicle { inner: v })
        })
    }

    #[staticmethod]
    fn connect_tcp<'py>(py: Python<'py>, addr: &str) -> PyResult<Bound<'py, PyAny>> {
        let a = addr.to_string();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let v = mavkit::Vehicle::connect_tcp(&a).await.map_err(to_py_err)?;
            Ok(PyVehicle { inner: v })
        })
    }

    #[staticmethod]
    fn connect_serial<'py>(py: Python<'py>, port: &str, baud: u32) -> PyResult<Bound<'py, PyAny>> {
        let p = port.to_string();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let v = mavkit::Vehicle::connect_serial(&p, baud)
                .await
                .map_err(to_py_err)?;
            Ok(PyVehicle { inner: v })
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
            let v = mavkit::Vehicle::connect_with_config(&addr, config.into_inner())
                .await
                .map_err(to_py_err)?;
            Ok(PyVehicle { inner: v })
        })
    }

    // --- State snapshots (sync getters) ---

    #[getter]
    fn state(&self) -> PyVehicleState {
        PyVehicleState {
            inner: self.inner.state().borrow().clone(),
        }
    }

    #[getter]
    fn telemetry(&self) -> PyTelemetry {
        PyTelemetry {
            inner: self.inner.telemetry().borrow().clone(),
        }
    }

    #[getter]
    fn home_position(&self) -> Option<PyHomePosition> {
        self.inner
            .home_position()
            .borrow()
            .as_ref()
            .map(|h| PyHomePosition { inner: h.clone() })
    }

    #[getter]
    fn mission_state(&self) -> PyMissionState {
        PyMissionState {
            inner: self.inner.mission_state().borrow().clone(),
        }
    }

    #[getter]
    fn link_state(&self) -> PyLinkState {
        self.inner.link_state().borrow().clone().into()
    }

    #[getter]
    fn mission_progress(&self) -> Option<PyTransferProgress> {
        self.inner
            .mission_progress()
            .borrow()
            .as_ref()
            .map(|p| PyTransferProgress { inner: p.clone() })
    }

    #[getter]
    fn param_store(&self) -> PyParamStore {
        PyParamStore {
            inner: self.inner.param_store().borrow().clone(),
        }
    }

    #[getter]
    fn param_progress(&self) -> PyParamProgress {
        PyParamProgress {
            inner: self.inner.param_progress().borrow().clone(),
        }
    }

    #[getter]
    fn statustext(&self) -> Option<PyStatusMessage> {
        self.inner
            .statustext()
            .borrow()
            .as_ref()
            .map(|s| PyStatusMessage { inner: s.clone() })
    }

    // --- Async wait for state changes ---

    fn wait_state<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let mut rx = vehicle.state();
            rx.borrow_and_update();
            rx.changed()
                .await
                .map_err(|_| to_py_err(mavkit::VehicleError::Disconnected))?;
            Ok(PyVehicleState {
                inner: rx.borrow().clone(),
            })
        })
    }

    fn wait_telemetry<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let mut rx = vehicle.telemetry();
            rx.borrow_and_update();
            rx.changed()
                .await
                .map_err(|_| to_py_err(mavkit::VehicleError::Disconnected))?;
            Ok(PyTelemetry {
                inner: rx.borrow().clone(),
            })
        })
    }

    fn wait_home_position<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let mut rx = vehicle.home_position();
            rx.borrow_and_update();
            rx.changed()
                .await
                .map_err(|_| to_py_err(mavkit::VehicleError::Disconnected))?;
            Ok(rx
                .borrow()
                .as_ref()
                .map(|h| PyHomePosition { inner: h.clone() }))
        })
    }

    fn wait_mission_state<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let mut rx = vehicle.mission_state();
            rx.borrow_and_update();
            rx.changed()
                .await
                .map_err(|_| to_py_err(mavkit::VehicleError::Disconnected))?;
            Ok(PyMissionState {
                inner: rx.borrow().clone(),
            })
        })
    }

    fn wait_link_state<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let mut rx = vehicle.link_state();
            rx.borrow_and_update();
            rx.changed()
                .await
                .map_err(|_| to_py_err(mavkit::VehicleError::Disconnected))?;
            let state: PyLinkState = rx.borrow().clone().into();
            Ok(state)
        })
    }

    fn wait_mission_progress<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let mut rx = vehicle.mission_progress();
            rx.borrow_and_update();
            rx.changed()
                .await
                .map_err(|_| to_py_err(mavkit::VehicleError::Disconnected))?;
            Ok(rx
                .borrow()
                .as_ref()
                .map(|p| PyTransferProgress { inner: p.clone() }))
        })
    }

    fn wait_param_store<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let mut rx = vehicle.param_store();
            rx.borrow_and_update();
            rx.changed()
                .await
                .map_err(|_| to_py_err(mavkit::VehicleError::Disconnected))?;
            Ok(PyParamStore {
                inner: rx.borrow().clone(),
            })
        })
    }

    fn wait_param_progress<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let mut rx = vehicle.param_progress();
            rx.borrow_and_update();
            rx.changed()
                .await
                .map_err(|_| to_py_err(mavkit::VehicleError::Disconnected))?;
            Ok(PyParamProgress {
                inner: rx.borrow().clone(),
            })
        })
    }

    fn wait_statustext<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let mut rx = vehicle.statustext();
            rx.borrow_and_update();
            rx.changed()
                .await
                .map_err(|_| to_py_err(mavkit::VehicleError::Disconnected))?;
            Ok(rx
                .borrow()
                .as_ref()
                .map(|s| PyStatusMessage { inner: s.clone() }))
        })
    }

    // --- Vehicle commands ---

    fn arm<'py>(&self, py: Python<'py>, force: bool) -> PyResult<Bound<'py, PyAny>> {
        let v = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            v.arm(force).await.map_err(to_py_err)?;
            Ok(())
        })
    }

    fn disarm<'py>(&self, py: Python<'py>, force: bool) -> PyResult<Bound<'py, PyAny>> {
        let v = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            v.disarm(force).await.map_err(to_py_err)?;
            Ok(())
        })
    }

    fn set_mode<'py>(&self, py: Python<'py>, custom_mode: u32) -> PyResult<Bound<'py, PyAny>> {
        let v = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            v.set_mode(custom_mode).await.map_err(to_py_err)?;
            Ok(())
        })
    }

    fn set_mode_by_name<'py>(&self, py: Python<'py>, name: &str) -> PyResult<Bound<'py, PyAny>> {
        let v = self.inner.clone();
        let n = name.to_string();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            v.set_mode_by_name(&n).await.map_err(to_py_err)?;
            Ok(())
        })
    }

    fn takeoff<'py>(&self, py: Python<'py>, altitude_m: f32) -> PyResult<Bound<'py, PyAny>> {
        let v = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            v.takeoff(altitude_m).await.map_err(to_py_err)?;
            Ok(())
        })
    }

    fn goto<'py>(
        &self,
        py: Python<'py>,
        lat_deg: f64,
        lon_deg: f64,
        alt_m: f32,
    ) -> PyResult<Bound<'py, PyAny>> {
        let v = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            v.goto(lat_deg, lon_deg, alt_m).await.map_err(to_py_err)?;
            Ok(())
        })
    }

    #[pyo3(signature = (command_id, param1=0.0, param2=0.0, param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0))]
    #[allow(clippy::too_many_arguments)]
    fn command_long<'py>(
        &self,
        py: Python<'py>,
        command_id: u32,
        param1: f32,
        param2: f32,
        param3: f32,
        param4: f32,
        param5: f32,
        param6: f32,
        param7: f32,
    ) -> PyResult<Bound<'py, PyAny>> {
        let cmd = mavlink::common::MavCmd::from_u32(command_id).ok_or_else(|| {
            pyo3::exceptions::PyValueError::new_err(format!(
                "Unknown MAVLink command ID: {command_id}"
            ))
        })?;
        let v = self.inner.clone();
        let params = [param1, param2, param3, param4, param5, param6, param7];
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            v.command_long(cmd, params).await.map_err(to_py_err)?;
            Ok(())
        })
    }

    fn preflight_calibration<'py>(
        &self,
        py: Python<'py>,
        gyro: bool,
        accel: bool,
        radio_trim: bool,
    ) -> PyResult<Bound<'py, PyAny>> {
        let v = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            v.preflight_calibration(gyro, accel, radio_trim)
                .await
                .map_err(to_py_err)?;
            Ok(())
        })
    }

    fn available_modes(&self) -> Vec<PyFlightMode> {
        self.inner
            .available_modes()
            .into_iter()
            .map(|m| PyFlightMode { inner: m })
            .collect()
    }

    fn identity(&self) -> Option<PyVehicleIdentity> {
        self.inner
            .identity()
            .map(|id| PyVehicleIdentity { inner: id })
    }

    fn disconnect<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let v = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            v.disconnect().await.map_err(to_py_err)?;
            Ok(())
        })
    }

    // --- Mission operations (flattened from MissionHandle) ---

    fn upload_mission<'py>(
        &self,
        py: Python<'py>,
        plan: PyMissionPlan,
    ) -> PyResult<Bound<'py, PyAny>> {
        let v = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            v.mission().upload(plan.inner).await.map_err(to_py_err)?;
            Ok(())
        })
    }

    fn download_mission<'py>(
        &self,
        py: Python<'py>,
        mission_type: PyMissionType,
    ) -> PyResult<Bound<'py, PyAny>> {
        let v = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let plan = v
                .mission()
                .download(mission_type.into())
                .await
                .map_err(to_py_err)?;
            Ok(PyMissionPlan { inner: plan })
        })
    }

    fn clear_mission<'py>(
        &self,
        py: Python<'py>,
        mission_type: PyMissionType,
    ) -> PyResult<Bound<'py, PyAny>> {
        let v = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            v.mission()
                .clear(mission_type.into())
                .await
                .map_err(to_py_err)?;
            Ok(())
        })
    }

    fn verify_mission_roundtrip<'py>(
        &self,
        py: Python<'py>,
        plan: PyMissionPlan,
    ) -> PyResult<Bound<'py, PyAny>> {
        let v = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let ok = v
                .mission()
                .verify_roundtrip(plan.inner)
                .await
                .map_err(to_py_err)?;
            Ok(ok)
        })
    }

    fn set_current_mission_item<'py>(
        &self,
        py: Python<'py>,
        seq: u16,
    ) -> PyResult<Bound<'py, PyAny>> {
        let v = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            v.mission().set_current(seq).await.map_err(to_py_err)?;
            Ok(())
        })
    }

    fn cancel_mission_transfer(&self) {
        self.inner.mission().cancel_transfer();
    }

    // --- Param operations (flattened from ParamsHandle) ---

    fn download_params<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let v = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let store = v.params().download_all().await.map_err(to_py_err)?;
            Ok(PyParamStore { inner: store })
        })
    }

    fn write_param<'py>(
        &self,
        py: Python<'py>,
        name: &str,
        value: f32,
    ) -> PyResult<Bound<'py, PyAny>> {
        let v = self.inner.clone();
        let n = name.to_string();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let param = v.params().write(n, value).await.map_err(to_py_err)?;
            Ok(PyParam { inner: param })
        })
    }

    fn write_params_batch<'py>(
        &self,
        py: Python<'py>,
        params: Vec<(String, f32)>,
    ) -> PyResult<Bound<'py, PyAny>> {
        let v = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let results = v.params().write_batch(params).await.map_err(to_py_err)?;
            Ok(results
                .into_iter()
                .map(|r| PyParamWriteResult { inner: r })
                .collect::<Vec<_>>())
        })
    }

    fn __repr__(&self) -> String {
        let state = self.inner.state().borrow().clone();
        format!("Vehicle(mode='{}', armed={})", state.mode_name, state.armed)
    }
}
