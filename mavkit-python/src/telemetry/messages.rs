use std::sync::Arc;
use std::time::Duration;

use mavkit::dialect;
use pyo3::exceptions::{PyStopAsyncIteration, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList};
use serde_json::Value;

use crate::error::{duration_from_secs, to_py_err};

use super::common::{monotonic_seconds, support_state_name, vehicle_timestamp_to_py};
use super::status_text::PyStatusTextEvent;

fn json_value_to_py(py: Python<'_>, value: Value) -> PyResult<Py<PyAny>> {
    match value {
        Value::Null => Ok(py.None()),
        Value::Bool(value) => Ok(value.into_pyobject(py)?.to_owned().into_any().unbind()),
        Value::Number(value) => {
            if let Some(value) = value.as_i64() {
                Ok(value.into_pyobject(py)?.to_owned().into_any().unbind())
            } else if let Some(value) = value.as_u64() {
                Ok(value.into_pyobject(py)?.to_owned().into_any().unbind())
            } else if let Some(value) = value.as_f64() {
                Ok(value.into_pyobject(py)?.to_owned().into_any().unbind())
            } else {
                Ok(py.None())
            }
        }
        Value::String(value) => Ok(value.into_pyobject(py)?.to_owned().into_any().unbind()),
        Value::Array(values) => {
            let list = PyList::empty(py);
            for value in values {
                list.append(json_value_to_py(py, value)?)?;
            }
            Ok(list.into_any().unbind())
        }
        Value::Object(values) => {
            let dict = PyDict::new(py);
            for (key, value) in values {
                dict.set_item(key, json_value_to_py(py, value)?)?;
            }
            Ok(dict.into_any().unbind())
        }
    }
}

macro_rules! serialize_to_py {
    ($py:expr, $value:expr) => {{
        let value = serde_json::to_value(&$value).map_err(|err| {
            PyValueError::new_err(format!("telemetry serialization error: {err}"))
        })?;
        json_value_to_py($py, value)
    }};
}

#[pyclass(name = "MessageSample", frozen, skip_from_py_object)]
pub(crate) struct PyMessageSample {
    value: Py<PyAny>,
    vehicle_time: Option<Py<PyAny>>,
    received_at_monotonic_s: f64,
}

impl PyMessageSample {
    fn new(
        value: Py<PyAny>,
        vehicle_time: Option<Py<PyAny>>,
        received_at_monotonic_s: f64,
    ) -> Self {
        Self {
            value,
            vehicle_time,
            received_at_monotonic_s,
        }
    }
}

#[pymethods]
impl PyMessageSample {
    #[getter]
    fn value(&self, py: Python<'_>) -> Py<PyAny> {
        self.value.clone_ref(py)
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

macro_rules! message_sample_py_value {
    ($py:expr, serialize, $value:expr) => {
        serialize_to_py!($py, $value)?
    };
    ($py:expr, status_text, $value:expr) => {
        Py::new($py, PyStatusTextEvent::from($value))?.into_any()
    };
}

/// Rows are:
/// - periodic: enum variant, `PyPeriodicMessageHandle` constructor name, sample type.
/// - event: enum variant, `PyEventMessageHandle` constructor name, sample type.
/// - push: enum variant, `PyMessageHandle` constructor name, sample type, conversion mode.
///
/// "push" here means streamed message handles that support latest/wait/subscribe but not
/// request/set_rate. Keep this registry in sync with the public accessors on
/// `PyTelemetryMessagesHandle` below.
macro_rules! define_message_wrapper_stack {
    (
        periodic: {
            $($p_variant:ident, $p_fn_name:ident, $p_sample_ty:ty;)+
        }
        event: {
            $($e_variant:ident, $e_fn_name:ident, $e_sample_ty:ty;)+
        }
        push: {
            $($u_variant:ident, $u_fn_name:ident, $u_sample_ty:ty, $u_py_conv_kind:ident;)+
        }
    ) => {
        enum MessageSampleValue {
            $($p_variant(mavkit::MessageSample<$p_sample_ty>),)+
            $($e_variant(mavkit::MessageSample<$e_sample_ty>),)+
            $($u_variant(mavkit::MessageSample<$u_sample_ty>),)+
        }

        impl MessageSampleValue {
            fn into_py(self, py: Python<'_>) -> PyResult<PyMessageSample> {
                match self {
                    $(Self::$p_variant(sample) => Ok(PyMessageSample::new(
                        message_sample_py_value!(py, serialize, sample.value),
                        vehicle_timestamp_to_py(py, sample.vehicle_time)?,
                        monotonic_seconds(sample.received_at),
                    )),)+
                    $(Self::$e_variant(sample) => Ok(PyMessageSample::new(
                        message_sample_py_value!(py, serialize, sample.value),
                        vehicle_timestamp_to_py(py, sample.vehicle_time)?,
                        monotonic_seconds(sample.received_at),
                    )),)+
                    $(Self::$u_variant(sample) => Ok(PyMessageSample::new(
                        message_sample_py_value!(py, $u_py_conv_kind, sample.value),
                        vehicle_timestamp_to_py(py, sample.vehicle_time)?,
                        monotonic_seconds(sample.received_at),
                    )),)+
                }
            }
        }

        #[derive(Clone)]
        enum PeriodicMessageHandleKind {
            $($p_variant(mavkit::PeriodicMessageHandle<$p_sample_ty>),)+
        }

        impl PeriodicMessageHandleKind {
            fn latest_value(&self) -> Option<MessageSampleValue> {
                match self {
                    $(Self::$p_variant(handle) => handle.latest().map(MessageSampleValue::$p_variant),)+
                }
            }

            async fn wait_value(self) -> Result<MessageSampleValue, mavkit::VehicleError> {
                match self {
                    $(Self::$p_variant(handle) => handle.wait().await.map(MessageSampleValue::$p_variant),)+
                }
            }

            async fn wait_timeout_value(
                self,
                timeout: Duration,
            ) -> Result<MessageSampleValue, mavkit::VehicleError> {
                match self {
                    $(Self::$p_variant(handle) => handle
                        .wait_timeout(timeout)
                        .await
                        .map(MessageSampleValue::$p_variant),)+
                }
            }

            async fn request_value(
                self,
                timeout: Duration,
            ) -> Result<MessageSampleValue, mavkit::VehicleError> {
                match self {
                    $(Self::$p_variant(handle) => handle
                        .request(timeout)
                        .await
                        .map(MessageSampleValue::$p_variant),)+
                }
            }

            async fn set_rate(self, hz: f32) -> Result<(), mavkit::VehicleError> {
                match self {
                    $(Self::$p_variant(handle) => handle.set_rate(hz).await,)+
                }
            }

            fn subscribe_value(&self) -> MessageSubscriptionKind {
                match self {
                    $(Self::$p_variant(handle) => MessageSubscriptionKind::$p_variant(handle.subscribe()),)+
                }
            }

            fn support_name(&self) -> Option<&'static str> {
                match self {
                    $(Self::$p_variant(handle) => support_state_name(handle.support().latest()),)+
                }
            }
        }

        #[derive(Clone)]
        enum EventMessageHandleKind {
            $($e_variant(mavkit::EventMessageHandle<$e_sample_ty>),)+
        }

        impl EventMessageHandleKind {
            fn latest_value(&self) -> Option<MessageSampleValue> {
                match self {
                    $(Self::$e_variant(handle) => handle.latest().map(MessageSampleValue::$e_variant),)+
                }
            }

            async fn wait_value(self) -> Result<MessageSampleValue, mavkit::VehicleError> {
                match self {
                    $(Self::$e_variant(handle) => handle.wait().await.map(MessageSampleValue::$e_variant),)+
                }
            }

            async fn wait_timeout_value(
                self,
                timeout: Duration,
            ) -> Result<MessageSampleValue, mavkit::VehicleError> {
                match self {
                    $(Self::$e_variant(handle) => handle
                        .wait_timeout(timeout)
                        .await
                        .map(MessageSampleValue::$e_variant),)+
                }
            }

            async fn request_value(
                self,
                timeout: Duration,
            ) -> Result<MessageSampleValue, mavkit::VehicleError> {
                match self {
                    $(Self::$e_variant(handle) => handle
                        .request(timeout)
                        .await
                        .map(MessageSampleValue::$e_variant),)+
                }
            }

            fn subscribe_value(&self) -> MessageSubscriptionKind {
                match self {
                    $(Self::$e_variant(handle) => MessageSubscriptionKind::$e_variant(handle.subscribe()),)+
                }
            }

            fn support_name(&self) -> Option<&'static str> {
                match self {
                    $(Self::$e_variant(handle) => support_state_name(handle.support().latest()),)+
                }
            }
        }

        #[derive(Clone)]
        enum PushMessageHandleKind {
            $($u_variant(mavkit::MessageHandle<$u_sample_ty>),)+
        }

        impl PushMessageHandleKind {
            fn latest_value(&self) -> Option<MessageSampleValue> {
                match self {
                    $(Self::$u_variant(handle) => handle.latest().map(MessageSampleValue::$u_variant),)+
                }
            }

            async fn wait_value(self) -> Result<MessageSampleValue, mavkit::VehicleError> {
                match self {
                    $(Self::$u_variant(handle) => handle.wait().await.map(MessageSampleValue::$u_variant),)+
                }
            }

            async fn wait_timeout_value(
                self,
                timeout: Duration,
            ) -> Result<MessageSampleValue, mavkit::VehicleError> {
                match self {
                    $(Self::$u_variant(handle) => handle
                        .wait_timeout(timeout)
                        .await
                        .map(MessageSampleValue::$u_variant),)+
                }
            }

            fn subscribe_value(&self) -> MessageSubscriptionKind {
                match self {
                    $(Self::$u_variant(handle) => MessageSubscriptionKind::$u_variant(handle.subscribe()),)+
                }
            }

            fn support_name(&self) -> Option<&'static str> {
                match self {
                    $(Self::$u_variant(handle) => support_state_name(handle.support().latest()),)+
                }
            }
        }

        enum MessageSubscriptionKind {
            $($p_variant(mavkit::ObservationSubscription<mavkit::MessageSample<$p_sample_ty>>),)+
            $($e_variant(mavkit::ObservationSubscription<mavkit::MessageSample<$e_sample_ty>>),)+
            $($u_variant(mavkit::ObservationSubscription<mavkit::MessageSample<$u_sample_ty>>),)+
        }

        impl MessageSubscriptionKind {
            async fn recv_value(&mut self) -> Option<MessageSampleValue> {
                match self {
                    $(Self::$p_variant(subscription) => {
                        subscription.recv().await.map(MessageSampleValue::$p_variant)
                    },)+
                    $(Self::$e_variant(subscription) => {
                        subscription.recv().await.map(MessageSampleValue::$e_variant)
                    },)+
                    $(Self::$u_variant(subscription) => {
                        subscription.recv().await.map(MessageSampleValue::$u_variant)
                    },)+
                }
            }
        }

        impl PyPeriodicMessageHandle {
            $(pub(super) fn $p_fn_name(inner: mavkit::PeriodicMessageHandle<$p_sample_ty>) -> Self {
                Self {
                    inner: PeriodicMessageHandleKind::$p_variant(inner),
                }
            })+
        }

        impl PyEventMessageHandle {
            $(pub(super) fn $e_fn_name(inner: mavkit::EventMessageHandle<$e_sample_ty>) -> Self {
                Self {
                    inner: EventMessageHandleKind::$e_variant(inner),
                }
            })+
        }

        impl PyMessageHandle {
            $(pub(super) fn $u_fn_name(inner: mavkit::MessageHandle<$u_sample_ty>) -> Self {
                Self {
                    inner: PushMessageHandleKind::$u_variant(inner),
                }
            })+
        }
    }
}

define_message_wrapper_stack! {
    periodic: {
        VfrHud, vfr_hud, dialect::VFR_HUD_DATA;
        GlobalPositionInt, global_position_int, dialect::GLOBAL_POSITION_INT_DATA;
        LocalPositionNed, local_position_ned, dialect::LOCAL_POSITION_NED_DATA;
        GpsRawInt, gps_raw_int, dialect::GPS_RAW_INT_DATA;
        Attitude, attitude, dialect::ATTITUDE_DATA;
        SysStatus, sys_status, dialect::SYS_STATUS_DATA;
        BatteryStatus, battery_status, dialect::BATTERY_STATUS_DATA;
        NavControllerOutput, nav_controller_output, dialect::NAV_CONTROLLER_OUTPUT_DATA;
        TerrainReport, terrain_report, dialect::TERRAIN_REPORT_DATA;
        RcChannels, rc_channels, dialect::RC_CHANNELS_DATA;
        ServoOutputRaw, servo_output_raw, dialect::SERVO_OUTPUT_RAW_DATA;
    }
    event: {
        HomePosition, home_position, dialect::HOME_POSITION_DATA;
        GpsGlobalOrigin, gps_global_origin, dialect::GPS_GLOBAL_ORIGIN_DATA;
    }
    push: {
        StatusText, status_text, mavkit::StatusTextEvent, status_text;
    }
}

#[pyclass(name = "PeriodicMessageHandle", frozen, skip_from_py_object)]
#[derive(Clone)]
pub(crate) struct PyPeriodicMessageHandle {
    inner: PeriodicMessageHandleKind,
}

#[pyclass(name = "EventMessageHandle", frozen, skip_from_py_object)]
#[derive(Clone)]
pub(crate) struct PyEventMessageHandle {
    inner: EventMessageHandleKind,
}

#[pyclass(name = "MessageHandle", frozen, skip_from_py_object)]
#[derive(Clone)]
pub(crate) struct PyMessageHandle {
    inner: PushMessageHandleKind,
}

macro_rules! impl_message_handle_pymethods {
    ($py_type:ty, {$($extras:item)*}) => {
        #[pymethods]
        impl $py_type {
            fn latest(&self, py: Python<'_>) -> PyResult<Option<PyMessageSample>> {
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

            fn wait_timeout<'py>(
                &self,
                py: Python<'py>,
                timeout_secs: f64,
            ) -> PyResult<Bound<'py, PyAny>> {
                let inner = self.inner.clone();
                let timeout = duration_from_secs(timeout_secs)?;
                pyo3_async_runtimes::tokio::future_into_py(py, async move {
                    let sample = inner.wait_timeout_value(timeout).await.map_err(to_py_err)?;
                    Python::attach(|py| sample.into_py(py))
                })
            }

            fn subscribe(&self) -> PyMessageSubscription {
                PyMessageSubscription {
                    inner: Arc::new(tokio::sync::Mutex::new(self.inner.subscribe_value())),
                }
            }

            $($extras)*
        }
    };
}

impl_message_handle_pymethods!(PyPeriodicMessageHandle, {
    fn request<'py>(&self, py: Python<'py>, timeout_secs: f64) -> PyResult<Bound<'py, PyAny>> {
        let inner = self.inner.clone();
        let timeout = duration_from_secs(timeout_secs)?;
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let sample = inner.request_value(timeout).await.map_err(to_py_err)?;
            Python::attach(|py| sample.into_py(py))
        })
    }

    fn set_rate<'py>(&self, py: Python<'py>, hz: f32) -> PyResult<Bound<'py, PyAny>> {
        let inner = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            inner.set_rate(hz).await.map_err(to_py_err)?;
            Ok(())
        })
    }
});

impl_message_handle_pymethods!(PyEventMessageHandle, {
    fn request<'py>(&self, py: Python<'py>, timeout_secs: f64) -> PyResult<Bound<'py, PyAny>> {
        let inner = self.inner.clone();
        let timeout = duration_from_secs(timeout_secs)?;
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let sample = inner.request_value(timeout).await.map_err(to_py_err)?;
            Python::attach(|py| sample.into_py(py))
        })
    }
});

impl_message_handle_pymethods!(PyMessageHandle, {});

#[pyclass(name = "MessageSubscription", frozen, skip_from_py_object)]
pub(crate) struct PyMessageSubscription {
    inner: Arc<tokio::sync::Mutex<MessageSubscriptionKind>>,
}

#[pymethods]
impl PyMessageSubscription {
    fn recv<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let inner = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let mut guard = inner.lock().await;
            match guard.recv_value().await {
                Some(sample) => Python::attach(|py| sample.into_py(py)),
                None => Err(PyStopAsyncIteration::new_err("message subscription closed")),
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
