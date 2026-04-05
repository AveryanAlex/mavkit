use std::sync::Arc;
use std::time::Duration;

use pyo3::exceptions::PyStopAsyncIteration;
use pyo3::prelude::*;

use crate::error::{duration_from_secs, to_py_err};
use crate::geo::PyGeoPoint3dMsl;

use super::common::{monotonic_seconds, support_state_name, vehicle_timestamp_to_py};
use super::sensor_health::PySensorHealthSummary;
use super::values::{
    PyCellVoltages, PyEulerAttitude, PyGlobalPosition, PyGpsQuality, PyGuidanceState,
    PyTerrainClearance, PyWaypointProgress,
};

fn telemetry_message_kind_name(kind: mavkit::TelemetryMessageKind) -> &'static str {
    match kind {
        mavkit::TelemetryMessageKind::Heartbeat => "heartbeat",
        mavkit::TelemetryMessageKind::VfrHud => "vfr_hud",
        mavkit::TelemetryMessageKind::GlobalPositionInt => "global_position_int",
        mavkit::TelemetryMessageKind::LocalPositionNed => "local_position_ned",
        mavkit::TelemetryMessageKind::GpsRawInt => "gps_raw_int",
        mavkit::TelemetryMessageKind::Attitude => "attitude",
        mavkit::TelemetryMessageKind::SysStatus => "sys_status",
        mavkit::TelemetryMessageKind::BatteryStatus => "battery_status",
        mavkit::TelemetryMessageKind::NavControllerOutput => "nav_controller_output",
        mavkit::TelemetryMessageKind::TerrainReport => "terrain_report",
        mavkit::TelemetryMessageKind::RcChannels => "rc_channels",
        mavkit::TelemetryMessageKind::ServoOutputRaw => "servo_output_raw",
        mavkit::TelemetryMessageKind::HomePosition => "home_position",
        mavkit::TelemetryMessageKind::GpsGlobalOrigin => "gps_global_origin",
        mavkit::TelemetryMessageKind::StatusText => "status_text",
    }
}

#[pyclass(name = "MetricSample", frozen, skip_from_py_object)]
pub(crate) struct PyMetricSample {
    value: Py<PyAny>,
    source: String,
    vehicle_time: Option<Py<PyAny>>,
    received_at_monotonic_s: f64,
}

impl PyMetricSample {
    fn new(
        value: Py<PyAny>,
        source: String,
        vehicle_time: Option<Py<PyAny>>,
        received_at_monotonic_s: f64,
    ) -> Self {
        Self {
            value,
            source,
            vehicle_time,
            received_at_monotonic_s,
        }
    }
}

#[pymethods]
impl PyMetricSample {
    #[getter]
    fn value(&self, py: Python<'_>) -> Py<PyAny> {
        self.value.clone_ref(py)
    }

    #[getter]
    fn source(&self) -> &str {
        &self.source
    }

    #[getter]
    fn vehicle_time(&self, py: Python<'_>) -> Option<Py<PyAny>> {
        self.vehicle_time.as_ref().map(|value| value.clone_ref(py))
    }

    #[getter]
    fn received_at_monotonic_s(&self) -> f64 {
        self.received_at_monotonic_s
    }
}

macro_rules! metric_sample_py_value {
    ($py:expr, scalar, $value:expr) => {
        $value.into_pyobject($py)?.to_owned().into_any().unbind()
    };
    ($py:expr, pyclass, $py_type:ty, $value:expr) => {
        Py::new($py, <$py_type>::from($value))?.into_any()
    };
}

/// Rows are: enum variant, `PyMetricHandle` constructor name, sample type, conversion mode.
macro_rules! define_metric_wrapper_stack {
    (
        $(
            $variant:ident,
            $fn_name:tt,
            $sample_ty:ty,
            $py_conv_kind:ident $(, $py_conv_type:ty)?;
        )+
    ) => {
        enum MetricSampleValue {
            $($variant(mavkit::MetricSample<$sample_ty>),)+
        }

        impl MetricSampleValue {
            fn into_py(self, py: Python<'_>) -> PyResult<PyMetricSample> {
                match self {
                    $(Self::$variant(sample) => Ok(PyMetricSample::new(
                        metric_sample_py_value!(py, $py_conv_kind $(, $py_conv_type)?, sample.value),
                        telemetry_message_kind_name(sample.source).to_string(),
                        vehicle_timestamp_to_py(py, sample.vehicle_time)?,
                        monotonic_seconds(sample.received_at),
                    )),)+
                }
            }
        }

        #[derive(Clone)]
        enum MetricHandleKind {
            $($variant(mavkit::MetricHandle<$sample_ty>),)+
        }

        impl MetricHandleKind {
            fn latest_value(&self) -> Option<MetricSampleValue> {
                match self {
                    $(Self::$variant(handle) => handle.latest().map(MetricSampleValue::$variant),)+
                }
            }

            async fn wait_value(self) -> Result<MetricSampleValue, mavkit::VehicleError> {
                match self {
                    $(Self::$variant(handle) => handle.wait().await.map(MetricSampleValue::$variant),)+
                }
            }

            async fn wait_timeout_value(
                self,
                timeout: Duration,
            ) -> Result<MetricSampleValue, mavkit::VehicleError> {
                match self {
                    $(Self::$variant(handle) => handle
                        .wait_timeout(timeout)
                        .await
                        .map(MetricSampleValue::$variant),)+
                }
            }

            fn subscribe_value(&self) -> MetricSubscriptionKind {
                match self {
                    $(Self::$variant(handle) => MetricSubscriptionKind::$variant(handle.subscribe()),)+
                }
            }

            fn support_name(&self) -> Option<&'static str> {
                match self {
                    $(Self::$variant(handle) => support_state_name(handle.support().latest()),)+
                }
            }
        }

        enum MetricSubscriptionKind {
            $($variant(mavkit::ObservationSubscription<mavkit::MetricSample<$sample_ty>>),)+
        }

        impl MetricSubscriptionKind {
            async fn recv_value(&mut self) -> Option<MetricSampleValue> {
                match self {
                    $(Self::$variant(subscription) => {
                        subscription.recv().await.map(MetricSampleValue::$variant)
                    },)+
                }
            }
        }

        impl PyMetricHandle {
            $(pub(super) fn $fn_name(inner: mavkit::MetricHandle<$sample_ty>) -> Self {
                Self {
                    inner: MetricHandleKind::$variant(inner),
                }
            })+
        }
    };
}

define_metric_wrapper_stack! {
    Bool, bool, bool, scalar;
    F64, f64, f64, scalar;
    U8, u8, u8, scalar;
    I32, i32, i32, scalar;
    U16, u16, u16, scalar;
    GlobalPosition, global_position, mavkit::GlobalPosition, pyclass, PyGlobalPosition;
    EulerAttitude, euler_attitude, mavkit::EulerAttitude, pyclass, PyEulerAttitude;
    GpsQuality, gps_quality, mavkit::GpsQuality, pyclass, PyGpsQuality;
    GeoPoint3dMsl, geo_point_3d_msl, mavkit::GeoPoint3dMsl, pyclass, PyGeoPoint3dMsl;
    CellVoltages, cell_voltages, mavkit::CellVoltages, pyclass, PyCellVoltages;
    WaypointProgress, waypoint_progress, mavkit::WaypointProgress, pyclass, PyWaypointProgress;
    GuidanceState, guidance_state, mavkit::GuidanceState, pyclass, PyGuidanceState;
    TerrainClearance, terrain_clearance, mavkit::TerrainClearance, pyclass, PyTerrainClearance;
    SensorHealthSummary, sensor_health_summary, mavkit::SensorHealthSummary, pyclass, PySensorHealthSummary;
}

#[pyclass(name = "MetricHandle", frozen, skip_from_py_object)]
#[derive(Clone)]
pub(crate) struct PyMetricHandle {
    inner: MetricHandleKind,
}

#[pymethods]
impl PyMetricHandle {
    fn latest(&self, py: Python<'_>) -> PyResult<Option<PyMetricSample>> {
        self.inner
            .latest_value()
            .map(|sample| sample.into_py(py))
            .transpose()
    }

    fn support(&self) -> Option<String> {
        self.inner.support_name().map(str::to_string)
    }

    fn wait<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let inner = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let sample = inner.wait_value().await.map_err(to_py_err)?;
            Python::attach(|py| sample.into_py(py))
        })
    }

    fn wait_timeout<'py>(&self, py: Python<'py>, timeout_secs: f64) -> PyResult<Bound<'py, PyAny>> {
        let inner = self.inner.clone();
        let timeout = duration_from_secs(timeout_secs)?;
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let sample = inner.wait_timeout_value(timeout).await.map_err(to_py_err)?;
            Python::attach(|py| sample.into_py(py))
        })
    }

    fn subscribe(&self) -> PyMetricSubscription {
        PyMetricSubscription {
            inner: Arc::new(tokio::sync::Mutex::new(self.inner.subscribe_value())),
        }
    }
}

#[pyclass(name = "MetricSubscription", frozen, skip_from_py_object)]
pub(crate) struct PyMetricSubscription {
    inner: Arc<tokio::sync::Mutex<MetricSubscriptionKind>>,
}

#[pymethods]
impl PyMetricSubscription {
    fn recv<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let inner = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let mut guard = inner.lock().await;
            match guard.recv_value().await {
                Some(sample) => Python::attach(|py| sample.into_py(py)),
                None => Err(PyStopAsyncIteration::new_err("metric subscription closed")),
            }
        })
    }

    fn __aiter__(slf: PyRef<'_, Self>) -> PyRef<'_, Self> {
        slf
    }

    fn __anext__<'py>(slf: PyRef<'py, Self>, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        slf.recv(py)
    }
}
