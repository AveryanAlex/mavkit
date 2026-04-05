use pyo3::prelude::*;
use std::sync::Arc;

use crate::error::{duration_from_secs, to_py_err};
use crate::macros::py_subscription;
use crate::mission::PyMissionPlan;
use crate::vehicle::stored_plans::{PyFencePlan, PyRallyPlan};

fn mission_progress_phase_name(progress: &mavkit::MissionOperationProgress) -> &'static str {
    match progress {
        mavkit::MissionOperationProgress::RequestCount => "request_count",
        mavkit::MissionOperationProgress::SendingItem { .. } => "sending_item",
        mavkit::MissionOperationProgress::ReceivingItem { .. } => "receiving_item",
        mavkit::MissionOperationProgress::AwaitingAck => "awaiting_ack",
        mavkit::MissionOperationProgress::Verifying => "verifying",
        mavkit::MissionOperationProgress::Completed => "completed",
        mavkit::MissionOperationProgress::Failed => "failed",
        mavkit::MissionOperationProgress::Cancelled => "cancelled",
    }
}

#[pyclass(name = "MissionOperationProgress", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyMissionOperationProgress {
    inner: mavkit::MissionOperationProgress,
}

impl From<mavkit::MissionOperationProgress> for PyMissionOperationProgress {
    fn from(inner: mavkit::MissionOperationProgress) -> Self {
        Self { inner }
    }
}

#[pymethods]
impl PyMissionOperationProgress {
    #[getter]
    fn phase(&self) -> &str {
        mission_progress_phase_name(&self.inner)
    }

    #[getter]
    fn current(&self) -> u16 {
        match self.inner {
            mavkit::MissionOperationProgress::SendingItem { current, .. }
            | mavkit::MissionOperationProgress::ReceivingItem { current, .. } => current,
            _ => 0,
        }
    }

    #[getter]
    fn total(&self) -> u16 {
        match self.inner {
            mavkit::MissionOperationProgress::SendingItem { total, .. }
            | mavkit::MissionOperationProgress::ReceivingItem { total, .. } => total,
            _ => 0,
        }
    }
}

py_subscription!(
    PyMissionProgressSubscription,
    mavkit::MissionOperationProgress,
    PyMissionOperationProgress,
    "MissionProgressSubscription",
    "mission-progress subscription closed"
);

macro_rules! define_progress_op {
    ($rust_name:ident, $py_name:literal, $inner:ty, $map:expr) => {
        #[pyclass(name = $py_name, frozen, skip_from_py_object)]
        #[derive(Clone)]
        pub struct $rust_name {
            pub(crate) inner: Arc<$inner>,
        }

        #[pymethods]
        impl $rust_name {
            fn latest(&self) -> Option<PyMissionOperationProgress> {
                self.inner
                    .latest()
                    .map(|inner| PyMissionOperationProgress { inner })
            }

            fn subscribe(&self) -> PyMissionProgressSubscription {
                PyMissionProgressSubscription {
                    inner: Arc::new(tokio::sync::Mutex::new(self.inner.subscribe())),
                }
            }

            fn cancel(&self) {
                self.inner.cancel();
            }

            fn wait<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
                let inner = self.inner.clone();
                let map = $map;
                pyo3_async_runtimes::tokio::future_into_py(py, async move {
                    let value = inner.wait().await.map_err(to_py_err)?;
                    map(value)
                })
            }

            fn wait_timeout<'py>(
                &self,
                py: Python<'py>,
                timeout_secs: f64,
            ) -> PyResult<Bound<'py, PyAny>> {
                let inner = self.inner.clone();
                let timeout = duration_from_secs(timeout_secs)?;
                let map = $map;
                pyo3_async_runtimes::tokio::future_into_py(py, async move {
                    let value = inner.wait_timeout(timeout).await.map_err(to_py_err)?;
                    map(value)
                })
            }
        }
    };
}

define_progress_op!(
    PyMissionUploadOp,
    "MissionUploadOp",
    mavkit::MissionUploadOp,
    |_value| Ok(())
);
define_progress_op!(
    PyMissionDownloadOp,
    "MissionDownloadOp",
    mavkit::MissionDownloadOp,
    |value| Ok(PyMissionPlan { inner: value })
);
define_progress_op!(
    PyMissionClearOp,
    "MissionClearOp",
    mavkit::MissionClearOp,
    |_value| Ok(())
);
define_progress_op!(
    PyMissionVerifyOp,
    "MissionVerifyOp",
    mavkit::MissionVerifyOp,
    |value| Ok(value)
);
define_progress_op!(
    PyFenceUploadOp,
    "FenceUploadOp",
    mavkit::FenceUploadOp,
    |_value| Ok(())
);
define_progress_op!(
    PyFenceDownloadOp,
    "FenceDownloadOp",
    mavkit::FenceDownloadOp,
    |value| Ok(PyFencePlan::from_inner(value))
);
define_progress_op!(
    PyFenceClearOp,
    "FenceClearOp",
    mavkit::FenceClearOp,
    |_value| Ok(())
);
define_progress_op!(
    PyRallyUploadOp,
    "RallyUploadOp",
    mavkit::RallyUploadOp,
    |_value| Ok(())
);
define_progress_op!(
    PyRallyDownloadOp,
    "RallyDownloadOp",
    mavkit::RallyDownloadOp,
    |value| Ok(PyRallyPlan::from_inner(value))
);
define_progress_op!(
    PyRallyClearOp,
    "RallyClearOp",
    mavkit::RallyClearOp,
    |_value| Ok(())
);
