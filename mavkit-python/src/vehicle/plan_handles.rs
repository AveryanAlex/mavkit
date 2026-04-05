use std::sync::Arc;

use pyo3::prelude::*;

use crate::error::to_py_err;
use crate::macros::define_vehicle_plan_handle;
use crate::mission::PyMissionPlan;
use crate::support::PySupportStateHandle;
use crate::vehicle::progress::{
    PyFenceClearOp, PyFenceDownloadOp, PyFenceUploadOp, PyMissionClearOp, PyMissionDownloadOp,
    PyMissionUploadOp, PyMissionVerifyOp, PyRallyClearOp, PyRallyDownloadOp, PyRallyUploadOp,
};
use crate::vehicle::states::{
    PyFenceState, PyFenceStateSubscription, PyMissionState, PyMissionStateSubscription,
    PyRallyState, PyRallyStateSubscription,
};
use crate::vehicle::stored_plans::{PyFencePlan, PyRallyPlan};

define_vehicle_plan_handle!(
    handle = PyMissionHandle,
    py_name = "MissionHandle",
    state = PyMissionState,
    subscription = PyMissionStateSubscription,
    plan = PyMissionPlan,
    upload_op = PyMissionUploadOp,
    download_op = PyMissionDownloadOp,
    clear_op = PyMissionClearOp,
    access = mission,
    extras = {
        fn verify(&self, plan: &PyMissionPlan) -> PyResult<PyMissionVerifyOp> {
            let vehicle = self.inner.clone();
            let plan = plan.inner.clone();
            let op = pyo3_async_runtimes::tokio::get_runtime()
                .block_on(async move { vehicle.mission().verify(plan) })
                .map_err(to_py_err)?;
            Ok(PyMissionVerifyOp {
                inner: Arc::new(op),
            })
        }

        fn set_current<'py>(&self, py: Python<'py>, index: u16) -> PyResult<Bound<'py, PyAny>> {
            let vehicle = self.inner.clone();
            pyo3_async_runtimes::tokio::future_into_py(py, async move {
                vehicle
                    .mission()
                    .set_current(index)
                    .await
                    .map_err(to_py_err)?;
                Ok(())
            })
        }
    }
);

define_vehicle_plan_handle!(
    handle = PyFenceHandle,
    py_name = "FenceHandle",
    state = PyFenceState,
    subscription = PyFenceStateSubscription,
    plan = PyFencePlan,
    upload_op = PyFenceUploadOp,
    download_op = PyFenceDownloadOp,
    clear_op = PyFenceClearOp,
    access = fence,
    extras = {
        fn support(&self) -> PySupportStateHandle {
            PySupportStateHandle::new(self.inner.fence().support())
        }
    }
);

define_vehicle_plan_handle!(
    handle = PyRallyHandle,
    py_name = "RallyHandle",
    state = PyRallyState,
    subscription = PyRallyStateSubscription,
    plan = PyRallyPlan,
    upload_op = PyRallyUploadOp,
    download_op = PyRallyDownloadOp,
    clear_op = PyRallyClearOp,
    access = rally,
    extras = {
        fn support(&self) -> PySupportStateHandle {
            PySupportStateHandle::new(self.inner.rally().support())
        }
    }
);
