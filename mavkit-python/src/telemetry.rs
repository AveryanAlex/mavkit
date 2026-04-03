use std::sync::{Arc, OnceLock};
use std::time::{Duration, Instant};

use mavkit::dialect;
use pyo3::exceptions::{PyStopAsyncIteration, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList};
use serde_json::Value;

use crate::enums::{PyGpsFixType, PyMavSeverity};
use crate::error::{duration_from_secs, to_py_err};
use crate::geo::PyGeoPoint3dMsl;

static PY_INSTANT_EPOCH: OnceLock<Instant> = OnceLock::new();

fn monotonic_seconds(instant: Instant) -> f64 {
    let epoch = *PY_INSTANT_EPOCH.get_or_init(Instant::now);
    instant
        .checked_duration_since(epoch)
        .unwrap_or_default()
        .as_secs_f64()
}

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

fn convert_gps_fix_type(value: mavkit::GpsFixType) -> PyGpsFixType {
    match value {
        mavkit::GpsFixType::NoFix => PyGpsFixType::NoFix,
        mavkit::GpsFixType::Fix2d => PyGpsFixType::Fix2d,
        mavkit::GpsFixType::Fix3d => PyGpsFixType::Fix3d,
        mavkit::GpsFixType::Dgps => PyGpsFixType::Dgps,
        mavkit::GpsFixType::RtkFloat => PyGpsFixType::RtkFloat,
        mavkit::GpsFixType::RtkFixed => PyGpsFixType::RtkFixed,
    }
}

fn support_state_name(state: Option<mavkit::observation::SupportState>) -> Option<&'static str> {
    match state {
        Some(mavkit::observation::SupportState::Unknown) => Some("unknown"),
        Some(mavkit::observation::SupportState::Supported) => Some("supported"),
        Some(mavkit::observation::SupportState::Unsupported) => Some("unsupported"),
        None => None,
    }
}

fn vehicle_timestamp_to_py(
    py: Python<'_>,
    timestamp: Option<mavkit::VehicleTimestamp>,
) -> PyResult<Option<Py<PyAny>>> {
    match timestamp {
        Some(mavkit::VehicleTimestamp::BootTime(duration)) => Ok(Some(
            ("boot_time", duration.as_secs_f64())
                .into_pyobject(py)?
                .to_owned()
                .into_any()
                .unbind(),
        )),
        Some(mavkit::VehicleTimestamp::UnixEpochMicros(micros)) => Ok(Some(
            ("unix_epoch_micros", micros)
                .into_pyobject(py)?
                .to_owned()
                .into_any()
                .unbind(),
        )),
        None => Ok(None),
    }
}

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

#[pyclass(name = "SensorHealthState", eq, frozen, from_py_object)]
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub enum PySensorHealthState {
    NotPresent,
    Disabled,
    Unhealthy,
    Healthy,
}

impl From<mavkit::SensorHealthState> for PySensorHealthState {
    fn from(value: mavkit::SensorHealthState) -> Self {
        match value {
            mavkit::SensorHealthState::NotPresent => Self::NotPresent,
            mavkit::SensorHealthState::Disabled => Self::Disabled,
            mavkit::SensorHealthState::Unhealthy => Self::Unhealthy,
            mavkit::SensorHealthState::Healthy => Self::Healthy,
        }
    }
}

impl From<PySensorHealthState> for mavkit::SensorHealthState {
    fn from(value: PySensorHealthState) -> Self {
        match value {
            PySensorHealthState::NotPresent => Self::NotPresent,
            PySensorHealthState::Disabled => Self::Disabled,
            PySensorHealthState::Unhealthy => Self::Unhealthy,
            PySensorHealthState::Healthy => Self::Healthy,
        }
    }
}

#[pyclass(name = "GlobalPosition", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyGlobalPosition {
    latitude_deg: f64,
    longitude_deg: f64,
    altitude_msl_m: f64,
    relative_alt_m: f64,
}

impl From<mavkit::GlobalPosition> for PyGlobalPosition {
    fn from(value: mavkit::GlobalPosition) -> Self {
        Self {
            latitude_deg: value.latitude_deg,
            longitude_deg: value.longitude_deg,
            altitude_msl_m: value.altitude_msl_m,
            relative_alt_m: value.relative_alt_m,
        }
    }
}

#[pymethods]
impl PyGlobalPosition {
    #[new]
    #[pyo3(signature = (*, latitude_deg, longitude_deg, altitude_msl_m, relative_alt_m))]
    fn new(
        latitude_deg: f64,
        longitude_deg: f64,
        altitude_msl_m: f64,
        relative_alt_m: f64,
    ) -> Self {
        Self {
            latitude_deg,
            longitude_deg,
            altitude_msl_m,
            relative_alt_m,
        }
    }

    #[getter]
    fn latitude_deg(&self) -> f64 {
        self.latitude_deg
    }

    #[getter]
    fn longitude_deg(&self) -> f64 {
        self.longitude_deg
    }

    #[getter]
    fn altitude_msl_m(&self) -> f64 {
        self.altitude_msl_m
    }

    #[getter]
    fn relative_alt_m(&self) -> f64 {
        self.relative_alt_m
    }
}

#[pyclass(name = "EulerAttitude", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyEulerAttitude {
    roll_deg: f64,
    pitch_deg: f64,
    yaw_deg: f64,
}

impl From<mavkit::EulerAttitude> for PyEulerAttitude {
    fn from(value: mavkit::EulerAttitude) -> Self {
        Self {
            roll_deg: value.roll_deg,
            pitch_deg: value.pitch_deg,
            yaw_deg: value.yaw_deg,
        }
    }
}

#[pymethods]
impl PyEulerAttitude {
    #[new]
    fn new(roll_deg: f64, pitch_deg: f64, yaw_deg: f64) -> Self {
        Self {
            roll_deg,
            pitch_deg,
            yaw_deg,
        }
    }

    #[getter]
    fn roll_deg(&self) -> f64 {
        self.roll_deg
    }

    #[getter]
    fn pitch_deg(&self) -> f64 {
        self.pitch_deg
    }

    #[getter]
    fn yaw_deg(&self) -> f64 {
        self.yaw_deg
    }
}

#[pyclass(name = "GpsQuality", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyGpsQuality {
    fix_type: PyGpsFixType,
    satellites: Option<u8>,
    hdop: Option<f64>,
}

impl From<mavkit::GpsQuality> for PyGpsQuality {
    fn from(value: mavkit::GpsQuality) -> Self {
        let mavkit::GpsQuality {
            fix_type,
            satellites,
            hdop,
        } = value;
        Self {
            fix_type: convert_gps_fix_type(fix_type),
            satellites,
            hdop,
        }
    }
}

#[pymethods]
impl PyGpsQuality {
    #[new]
    fn new(fix_type: PyGpsFixType, satellites: Option<u8>, hdop: Option<f64>) -> Self {
        Self {
            fix_type,
            satellites,
            hdop,
        }
    }

    #[getter]
    fn fix_type(&self) -> PyGpsFixType {
        self.fix_type
    }

    #[getter]
    fn satellites(&self) -> Option<u8> {
        self.satellites
    }

    #[getter]
    fn hdop(&self) -> Option<f64> {
        self.hdop
    }
}

#[pyclass(name = "CellVoltages", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyCellVoltages {
    voltages_v: Vec<f64>,
}

impl From<mavkit::CellVoltages> for PyCellVoltages {
    fn from(value: mavkit::CellVoltages) -> Self {
        Self {
            voltages_v: value.voltages_v,
        }
    }
}

#[pymethods]
impl PyCellVoltages {
    #[new]
    fn new(voltages_v: Vec<f64>) -> Self {
        Self { voltages_v }
    }

    #[getter]
    fn voltages_v(&self) -> Vec<f64> {
        self.voltages_v.clone()
    }
}

#[pyclass(name = "WaypointProgress", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyWaypointProgress {
    distance_m: f64,
    bearing_deg: f64,
}

impl From<mavkit::WaypointProgress> for PyWaypointProgress {
    fn from(value: mavkit::WaypointProgress) -> Self {
        Self {
            distance_m: value.distance_m,
            bearing_deg: value.bearing_deg,
        }
    }
}

#[pymethods]
impl PyWaypointProgress {
    #[new]
    fn new(distance_m: f64, bearing_deg: f64) -> Self {
        Self {
            distance_m,
            bearing_deg,
        }
    }

    #[getter]
    fn distance_m(&self) -> f64 {
        self.distance_m
    }

    #[getter]
    fn bearing_deg(&self) -> f64 {
        self.bearing_deg
    }
}

#[pyclass(name = "GuidanceState", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyGuidanceState {
    bearing_deg: f64,
    cross_track_error_m: f64,
}

impl From<mavkit::GuidanceState> for PyGuidanceState {
    fn from(value: mavkit::GuidanceState) -> Self {
        Self {
            bearing_deg: value.bearing_deg,
            cross_track_error_m: value.cross_track_error_m,
        }
    }
}

#[pymethods]
impl PyGuidanceState {
    #[new]
    fn new(bearing_deg: f64, cross_track_error_m: f64) -> Self {
        Self {
            bearing_deg,
            cross_track_error_m,
        }
    }

    #[getter]
    fn bearing_deg(&self) -> f64 {
        self.bearing_deg
    }

    #[getter]
    fn cross_track_error_m(&self) -> f64 {
        self.cross_track_error_m
    }
}

#[pyclass(name = "TerrainClearance", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyTerrainClearance {
    terrain_height_m: f64,
    height_above_terrain_m: f64,
}

impl From<mavkit::TerrainClearance> for PyTerrainClearance {
    fn from(value: mavkit::TerrainClearance) -> Self {
        Self {
            terrain_height_m: value.terrain_height_m,
            height_above_terrain_m: value.height_above_terrain_m,
        }
    }
}

#[pymethods]
impl PyTerrainClearance {
    #[new]
    fn new(terrain_height_m: f64, height_above_terrain_m: f64) -> Self {
        Self {
            terrain_height_m,
            height_above_terrain_m,
        }
    }

    #[getter]
    fn terrain_height_m(&self) -> f64 {
        self.terrain_height_m
    }

    #[getter]
    fn height_above_terrain_m(&self) -> f64 {
        self.height_above_terrain_m
    }
}

#[pyclass(name = "SensorHealthSummary", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PySensorHealthSummary {
    gyro: PySensorHealthState,
    accel: PySensorHealthState,
    mag: PySensorHealthState,
    baro: PySensorHealthState,
    gps: PySensorHealthState,
    airspeed: PySensorHealthState,
    rc_receiver: PySensorHealthState,
    battery: PySensorHealthState,
    terrain: PySensorHealthState,
    geofence: PySensorHealthState,
}

impl From<mavkit::SensorHealthSummary> for PySensorHealthSummary {
    fn from(value: mavkit::SensorHealthSummary) -> Self {
        Self {
            gyro: value.gyro.into(),
            accel: value.accel.into(),
            mag: value.mag.into(),
            baro: value.baro.into(),
            gps: value.gps.into(),
            airspeed: value.airspeed.into(),
            rc_receiver: value.rc_receiver.into(),
            battery: value.battery.into(),
            terrain: value.terrain.into(),
            geofence: value.geofence.into(),
        }
    }
}

#[pymethods]
impl PySensorHealthSummary {
    #[new]
    #[pyo3(signature = (*, gyro, accel, mag, baro, gps, airspeed, rc_receiver, battery, terrain, geofence))]
    fn new(
        gyro: PySensorHealthState,
        accel: PySensorHealthState,
        mag: PySensorHealthState,
        baro: PySensorHealthState,
        gps: PySensorHealthState,
        airspeed: PySensorHealthState,
        rc_receiver: PySensorHealthState,
        battery: PySensorHealthState,
        terrain: PySensorHealthState,
        geofence: PySensorHealthState,
    ) -> Self {
        Self {
            gyro,
            accel,
            mag,
            baro,
            gps,
            airspeed,
            rc_receiver,
            battery,
            terrain,
            geofence,
        }
    }

    #[getter]
    fn gyro(&self) -> PySensorHealthState {
        self.gyro
    }

    #[getter]
    fn accel(&self) -> PySensorHealthState {
        self.accel
    }

    #[getter]
    fn mag(&self) -> PySensorHealthState {
        self.mag
    }

    #[getter]
    fn baro(&self) -> PySensorHealthState {
        self.baro
    }

    #[getter]
    fn gps(&self) -> PySensorHealthState {
        self.gps
    }

    #[getter]
    fn airspeed(&self) -> PySensorHealthState {
        self.airspeed
    }

    #[getter]
    fn rc_receiver(&self) -> PySensorHealthState {
        self.rc_receiver
    }

    #[getter]
    fn battery(&self) -> PySensorHealthState {
        self.battery
    }

    #[getter]
    fn terrain(&self) -> PySensorHealthState {
        self.terrain
    }

    #[getter]
    fn geofence(&self) -> PySensorHealthState {
        self.geofence
    }
}

#[pyclass(name = "StatusTextEvent", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyStatusTextEvent {
    text: String,
    severity: PyMavSeverity,
    id: u16,
    source_system: u8,
    source_component: u8,
}

impl From<mavkit::StatusTextEvent> for PyStatusTextEvent {
    fn from(value: mavkit::StatusTextEvent) -> Self {
        Self {
            text: value.text,
            severity: match value.severity {
                dialect::MavSeverity::MAV_SEVERITY_EMERGENCY => PyMavSeverity::Emergency,
                dialect::MavSeverity::MAV_SEVERITY_ALERT => PyMavSeverity::Alert,
                dialect::MavSeverity::MAV_SEVERITY_CRITICAL => PyMavSeverity::Critical,
                dialect::MavSeverity::MAV_SEVERITY_ERROR => PyMavSeverity::Error,
                dialect::MavSeverity::MAV_SEVERITY_WARNING => PyMavSeverity::Warning,
                dialect::MavSeverity::MAV_SEVERITY_NOTICE => PyMavSeverity::Notice,
                dialect::MavSeverity::MAV_SEVERITY_INFO => PyMavSeverity::Info,
                dialect::MavSeverity::MAV_SEVERITY_DEBUG => PyMavSeverity::Debug,
            },
            id: value.id,
            source_system: value.source_system,
            source_component: value.source_component,
        }
    }
}

#[pymethods]
impl PyStatusTextEvent {
    #[new]
    fn new(
        text: String,
        severity: PyMavSeverity,
        id: u16,
        source_system: u8,
        source_component: u8,
    ) -> Self {
        Self {
            text,
            severity,
            id,
            source_system,
            source_component,
        }
    }

    #[getter]
    fn text(&self) -> &str {
        &self.text
    }

    #[getter]
    fn severity(&self) -> PyMavSeverity {
        self.severity
    }

    #[getter]
    fn id(&self) -> u16 {
        self.id
    }

    #[getter]
    fn source_system(&self) -> u8 {
        self.source_system
    }

    #[getter]
    fn source_component(&self) -> u8 {
        self.source_component
    }
}

#[pyclass(name = "MetricSample", frozen, skip_from_py_object)]
pub struct PyMetricSample {
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

#[pyclass(name = "MessageSample", frozen, skip_from_py_object)]
pub struct PyMessageSample {
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
            $(fn $fn_name(inner: mavkit::MetricHandle<$sample_ty>) -> Self {
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
pub struct PyMetricHandle {
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
            $(fn $p_fn_name(inner: mavkit::PeriodicMessageHandle<$p_sample_ty>) -> Self {
                Self {
                    inner: PeriodicMessageHandleKind::$p_variant(inner),
                }
            })+
        }

        impl PyEventMessageHandle {
            $(fn $e_fn_name(inner: mavkit::EventMessageHandle<$e_sample_ty>) -> Self {
                Self {
                    inner: EventMessageHandleKind::$e_variant(inner),
                }
            })+
        }

        impl PyMessageHandle {
            $(fn $u_fn_name(inner: mavkit::MessageHandle<$u_sample_ty>) -> Self {
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
pub struct PyPeriodicMessageHandle {
    inner: PeriodicMessageHandleKind,
}

#[pymethods]
impl PyPeriodicMessageHandle {
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

    fn wait_timeout<'py>(&self, py: Python<'py>, timeout_secs: f64) -> PyResult<Bound<'py, PyAny>> {
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
}

#[pyclass(name = "EventMessageHandle", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyEventMessageHandle {
    inner: EventMessageHandleKind,
}

#[pymethods]
impl PyEventMessageHandle {
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

    fn wait_timeout<'py>(&self, py: Python<'py>, timeout_secs: f64) -> PyResult<Bound<'py, PyAny>> {
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

    fn request<'py>(&self, py: Python<'py>, timeout_secs: f64) -> PyResult<Bound<'py, PyAny>> {
        let inner = self.inner.clone();
        let timeout = duration_from_secs(timeout_secs)?;
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let sample = inner.request_value(timeout).await.map_err(to_py_err)?;
            Python::attach(|py| sample.into_py(py))
        })
    }
}

#[pyclass(name = "MessageHandle", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyMessageHandle {
    inner: PushMessageHandleKind,
}

#[pymethods]
impl PyMessageHandle {
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

    fn wait_timeout<'py>(&self, py: Python<'py>, timeout_secs: f64) -> PyResult<Bound<'py, PyAny>> {
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
}

#[pyclass(name = "MetricSubscription", frozen, skip_from_py_object)]
pub struct PyMetricSubscription {
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
        let inner = slf.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let mut guard = inner.lock().await;
            match guard.recv_value().await {
                Some(sample) => Python::attach(|py| sample.into_py(py)),
                None => Err(PyStopAsyncIteration::new_err("metric subscription closed")),
            }
        })
    }
}

#[pyclass(name = "MessageSubscription", frozen, skip_from_py_object)]
pub struct PyMessageSubscription {
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
        let inner = slf.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let mut guard = inner.lock().await;
            match guard.recv_value().await {
                Some(sample) => Python::attach(|py| sample.into_py(py)),
                None => Err(PyStopAsyncIteration::new_err("message subscription closed")),
            }
        })
    }
}

#[pyclass(name = "TelemetryHandle", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyTelemetryHandle {
    pub(crate) inner: mavkit::Vehicle,
}

macro_rules! define_vehicle_shell_wrappers {
    ($($namespace:ident => $py_name:literal;)+) => {
        $(
            #[pyclass(name = $py_name, frozen, skip_from_py_object)]
            #[derive(Clone)]
            pub struct $namespace {
                inner: mavkit::Vehicle,
            }

            impl $namespace {
                fn new(inner: mavkit::Vehicle) -> Self {
                    Self { inner }
                }
            }
        )+
    };
}

define_vehicle_shell_wrappers! {
    PyTelemetryPositionNamespace => "TelemetryPositionNamespace";
    PyTelemetryAttitudeNamespace => "TelemetryAttitudeNamespace";
    PyTelemetryBatteryNamespace => "TelemetryBatteryNamespace";
    PyTelemetryGpsNamespace => "TelemetryGpsNamespace";
    PyTelemetryNavigationNamespace => "TelemetryNavigationNamespace";
    PyTelemetryTerrainNamespace => "TelemetryTerrainNamespace";
    PyTelemetryRcNamespace => "TelemetryRcNamespace";
    PyTelemetryActuatorsNamespace => "TelemetryActuatorsNamespace";
    PyTelemetryMessagesHandle => "TelemetryMessagesHandle";
}

impl PyTelemetryHandle {
    pub(crate) fn new(inner: mavkit::Vehicle) -> Self {
        Self { inner }
    }

    fn wrap_vehicle<T>(&self, ctor: impl FnOnce(mavkit::Vehicle) -> T) -> T {
        ctor(self.inner.clone())
    }
}

#[pymethods]
impl PyTelemetryHandle {
    fn position(&self) -> PyTelemetryPositionNamespace {
        self.wrap_vehicle(PyTelemetryPositionNamespace::new)
    }

    fn attitude(&self) -> PyTelemetryAttitudeNamespace {
        self.wrap_vehicle(PyTelemetryAttitudeNamespace::new)
    }

    fn battery(&self) -> PyTelemetryBatteryNamespace {
        self.wrap_vehicle(PyTelemetryBatteryNamespace::new)
    }

    fn gps(&self) -> PyTelemetryGpsNamespace {
        self.wrap_vehicle(PyTelemetryGpsNamespace::new)
    }

    fn navigation(&self) -> PyTelemetryNavigationNamespace {
        self.wrap_vehicle(PyTelemetryNavigationNamespace::new)
    }

    fn terrain(&self) -> PyTelemetryTerrainNamespace {
        self.wrap_vehicle(PyTelemetryTerrainNamespace::new)
    }

    fn rc(&self) -> PyTelemetryRcNamespace {
        self.wrap_vehicle(PyTelemetryRcNamespace::new)
    }

    fn actuators(&self) -> PyTelemetryActuatorsNamespace {
        self.wrap_vehicle(PyTelemetryActuatorsNamespace::new)
    }

    fn messages(&self) -> PyTelemetryMessagesHandle {
        self.wrap_vehicle(PyTelemetryMessagesHandle::new)
    }

    fn armed(&self) -> PyMetricHandle {
        PyMetricHandle::bool(self.inner.telemetry().armed())
    }

    fn sensor_health(&self) -> PyMetricHandle {
        PyMetricHandle::sensor_health_summary(self.inner.telemetry().sensor_health())
    }

    fn home(&self) -> PyMetricHandle {
        PyMetricHandle::geo_point_3d_msl(self.inner.telemetry().home())
    }

    fn origin(&self) -> PyMetricHandle {
        PyMetricHandle::geo_point_3d_msl(self.inner.telemetry().origin())
    }

    fn __repr__(&self) -> String {
        let identity = self.inner.identity();
        format!(
            "TelemetryHandle(sys={}, comp={})",
            identity.system_id, identity.component_id
        )
    }
}

#[pymethods]
impl PyTelemetryPositionNamespace {
    fn global_pos(&self) -> PyMetricHandle {
        PyMetricHandle::global_position(self.inner.telemetry().position().global())
    }

    fn groundspeed_mps(&self) -> PyMetricHandle {
        PyMetricHandle::f64(self.inner.telemetry().position().groundspeed_mps())
    }

    fn airspeed_mps(&self) -> PyMetricHandle {
        PyMetricHandle::f64(self.inner.telemetry().position().airspeed_mps())
    }

    fn climb_rate_mps(&self) -> PyMetricHandle {
        PyMetricHandle::f64(self.inner.telemetry().position().climb_rate_mps())
    }

    fn heading_deg(&self) -> PyMetricHandle {
        PyMetricHandle::f64(self.inner.telemetry().position().heading_deg())
    }

    fn throttle_pct(&self) -> PyMetricHandle {
        PyMetricHandle::f64(self.inner.telemetry().position().throttle_pct())
    }
}

#[pymethods]
impl PyTelemetryAttitudeNamespace {
    fn euler(&self) -> PyMetricHandle {
        PyMetricHandle::euler_attitude(self.inner.telemetry().attitude().euler())
    }
}

#[pymethods]
impl PyTelemetryBatteryNamespace {
    fn remaining_pct(&self) -> PyMetricHandle {
        PyMetricHandle::f64(self.inner.telemetry().battery().remaining_pct())
    }

    fn voltage_v(&self) -> PyMetricHandle {
        PyMetricHandle::f64(self.inner.telemetry().battery().voltage_v())
    }

    fn current_a(&self) -> PyMetricHandle {
        PyMetricHandle::f64(self.inner.telemetry().battery().current_a())
    }

    fn energy_consumed_wh(&self) -> PyMetricHandle {
        PyMetricHandle::f64(self.inner.telemetry().battery().energy_consumed_wh())
    }

    fn time_remaining_s(&self) -> PyMetricHandle {
        PyMetricHandle::i32(self.inner.telemetry().battery().time_remaining_s())
    }

    fn cells(&self) -> PyMetricHandle {
        PyMetricHandle::cell_voltages(self.inner.telemetry().battery().cells())
    }
}

#[pymethods]
impl PyTelemetryGpsNamespace {
    fn quality(&self) -> PyMetricHandle {
        PyMetricHandle::gps_quality(self.inner.telemetry().gps().quality())
    }

    fn position_msl(&self) -> PyMetricHandle {
        PyMetricHandle::geo_point_3d_msl(self.inner.telemetry().gps().position_msl())
    }
}

#[pymethods]
impl PyTelemetryNavigationNamespace {
    fn waypoint(&self) -> PyMetricHandle {
        PyMetricHandle::waypoint_progress(self.inner.telemetry().navigation().waypoint())
    }

    fn guidance(&self) -> PyMetricHandle {
        PyMetricHandle::guidance_state(self.inner.telemetry().navigation().guidance())
    }
}

#[pymethods]
impl PyTelemetryTerrainNamespace {
    fn clearance(&self) -> PyMetricHandle {
        PyMetricHandle::terrain_clearance(self.inner.telemetry().terrain().clearance())
    }
}

#[pymethods]
impl PyTelemetryRcNamespace {
    fn channel_pwm_us(&self, index: usize) -> PyResult<PyMetricHandle> {
        self.inner
            .telemetry()
            .rc()
            .channel_pwm_us(index)
            .map(PyMetricHandle::u16)
            .ok_or_else(|| pyo3::exceptions::PyIndexError::new_err("rc channel index out of range"))
    }

    fn rssi_pct(&self) -> PyMetricHandle {
        PyMetricHandle::u8(self.inner.telemetry().rc().rssi_pct())
    }
}

#[pymethods]
impl PyTelemetryActuatorsNamespace {
    fn servo_pwm_us(&self, index: usize) -> PyResult<PyMetricHandle> {
        self.inner
            .telemetry()
            .actuators()
            .servo_pwm_us(index)
            .map(PyMetricHandle::u16)
            .ok_or_else(|| pyo3::exceptions::PyIndexError::new_err("servo index out of range"))
    }
}

#[pymethods]
impl PyTelemetryMessagesHandle {
    fn vfr_hud(&self) -> PyPeriodicMessageHandle {
        PyPeriodicMessageHandle::vfr_hud(self.inner.telemetry().messages().vfr_hud())
    }

    fn global_position_int(&self) -> PyPeriodicMessageHandle {
        PyPeriodicMessageHandle::global_position_int(
            self.inner.telemetry().messages().global_position_int(),
        )
    }

    fn local_position_ned(&self) -> PyPeriodicMessageHandle {
        PyPeriodicMessageHandle::local_position_ned(
            self.inner.telemetry().messages().local_position_ned(),
        )
    }

    fn gps_raw_int(&self) -> PyPeriodicMessageHandle {
        PyPeriodicMessageHandle::gps_raw_int(self.inner.telemetry().messages().gps_raw_int())
    }

    fn attitude(&self) -> PyPeriodicMessageHandle {
        PyPeriodicMessageHandle::attitude(self.inner.telemetry().messages().attitude())
    }

    fn sys_status(&self) -> PyPeriodicMessageHandle {
        PyPeriodicMessageHandle::sys_status(self.inner.telemetry().messages().sys_status())
    }

    fn battery_status(&self, instance: u8) -> PyPeriodicMessageHandle {
        PyPeriodicMessageHandle::battery_status(
            self.inner.telemetry().messages().battery_status(instance),
        )
    }

    fn nav_controller_output(&self) -> PyPeriodicMessageHandle {
        PyPeriodicMessageHandle::nav_controller_output(
            self.inner.telemetry().messages().nav_controller_output(),
        )
    }

    fn terrain_report(&self) -> PyPeriodicMessageHandle {
        PyPeriodicMessageHandle::terrain_report(self.inner.telemetry().messages().terrain_report())
    }

    fn rc_channels(&self) -> PyPeriodicMessageHandle {
        PyPeriodicMessageHandle::rc_channels(self.inner.telemetry().messages().rc_channels())
    }

    fn servo_output_raw(&self, port: u8) -> PyPeriodicMessageHandle {
        PyPeriodicMessageHandle::servo_output_raw(
            self.inner.telemetry().messages().servo_output_raw(port),
        )
    }

    fn home_position(&self) -> PyEventMessageHandle {
        PyEventMessageHandle::home_position(self.inner.telemetry().messages().home_position())
    }

    fn gps_global_origin(&self) -> PyEventMessageHandle {
        PyEventMessageHandle::gps_global_origin(
            self.inner.telemetry().messages().gps_global_origin(),
        )
    }

    fn status_text(&self) -> PyMessageHandle {
        PyMessageHandle::status_text(self.inner.telemetry().messages().status_text())
    }
}
