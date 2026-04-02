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

enum MetricSampleValue {
    Bool(mavkit::MetricSample<bool>),
    F64(mavkit::MetricSample<f64>),
    U8(mavkit::MetricSample<u8>),
    I32(mavkit::MetricSample<i32>),
    U16(mavkit::MetricSample<u16>),
    GlobalPosition(mavkit::MetricSample<mavkit::GlobalPosition>),
    EulerAttitude(mavkit::MetricSample<mavkit::EulerAttitude>),
    GpsQuality(mavkit::MetricSample<mavkit::GpsQuality>),
    GeoPoint3dMsl(mavkit::MetricSample<mavkit::GeoPoint3dMsl>),
    CellVoltages(mavkit::MetricSample<mavkit::CellVoltages>),
    WaypointProgress(mavkit::MetricSample<mavkit::WaypointProgress>),
    GuidanceState(mavkit::MetricSample<mavkit::GuidanceState>),
    TerrainClearance(mavkit::MetricSample<mavkit::TerrainClearance>),
    SensorHealthSummary(mavkit::MetricSample<mavkit::SensorHealthSummary>),
}

impl MetricSampleValue {
    fn into_py(self, py: Python<'_>) -> PyResult<PyMetricSample> {
        match self {
            Self::Bool(sample) => Ok(PyMetricSample::new(
                sample
                    .value
                    .into_pyobject(py)?
                    .to_owned()
                    .into_any()
                    .unbind(),
                telemetry_message_kind_name(sample.source).to_string(),
                vehicle_timestamp_to_py(py, sample.vehicle_time)?,
                monotonic_seconds(sample.received_at),
            )),
            Self::F64(sample) => Ok(PyMetricSample::new(
                sample
                    .value
                    .into_pyobject(py)?
                    .to_owned()
                    .into_any()
                    .unbind(),
                telemetry_message_kind_name(sample.source).to_string(),
                vehicle_timestamp_to_py(py, sample.vehicle_time)?,
                monotonic_seconds(sample.received_at),
            )),
            Self::U8(sample) => Ok(PyMetricSample::new(
                sample
                    .value
                    .into_pyobject(py)?
                    .to_owned()
                    .into_any()
                    .unbind(),
                telemetry_message_kind_name(sample.source).to_string(),
                vehicle_timestamp_to_py(py, sample.vehicle_time)?,
                monotonic_seconds(sample.received_at),
            )),
            Self::I32(sample) => Ok(PyMetricSample::new(
                sample
                    .value
                    .into_pyobject(py)?
                    .to_owned()
                    .into_any()
                    .unbind(),
                telemetry_message_kind_name(sample.source).to_string(),
                vehicle_timestamp_to_py(py, sample.vehicle_time)?,
                monotonic_seconds(sample.received_at),
            )),
            Self::U16(sample) => Ok(PyMetricSample::new(
                sample
                    .value
                    .into_pyobject(py)?
                    .to_owned()
                    .into_any()
                    .unbind(),
                telemetry_message_kind_name(sample.source).to_string(),
                vehicle_timestamp_to_py(py, sample.vehicle_time)?,
                monotonic_seconds(sample.received_at),
            )),
            Self::GlobalPosition(sample) => Ok(PyMetricSample::new(
                Py::new(py, PyGlobalPosition::from(sample.value))?.into_any(),
                telemetry_message_kind_name(sample.source).to_string(),
                vehicle_timestamp_to_py(py, sample.vehicle_time)?,
                monotonic_seconds(sample.received_at),
            )),
            Self::EulerAttitude(sample) => Ok(PyMetricSample::new(
                Py::new(py, PyEulerAttitude::from(sample.value))?.into_any(),
                telemetry_message_kind_name(sample.source).to_string(),
                vehicle_timestamp_to_py(py, sample.vehicle_time)?,
                monotonic_seconds(sample.received_at),
            )),
            Self::GpsQuality(sample) => Ok(PyMetricSample::new(
                Py::new(py, PyGpsQuality::from(sample.value))?.into_any(),
                telemetry_message_kind_name(sample.source).to_string(),
                vehicle_timestamp_to_py(py, sample.vehicle_time)?,
                monotonic_seconds(sample.received_at),
            )),
            Self::GeoPoint3dMsl(sample) => Ok(PyMetricSample::new(
                Py::new(py, PyGeoPoint3dMsl::from(sample.value))?.into_any(),
                telemetry_message_kind_name(sample.source).to_string(),
                vehicle_timestamp_to_py(py, sample.vehicle_time)?,
                monotonic_seconds(sample.received_at),
            )),
            Self::CellVoltages(sample) => Ok(PyMetricSample::new(
                Py::new(py, PyCellVoltages::from(sample.value))?.into_any(),
                telemetry_message_kind_name(sample.source).to_string(),
                vehicle_timestamp_to_py(py, sample.vehicle_time)?,
                monotonic_seconds(sample.received_at),
            )),
            Self::WaypointProgress(sample) => Ok(PyMetricSample::new(
                Py::new(py, PyWaypointProgress::from(sample.value))?.into_any(),
                telemetry_message_kind_name(sample.source).to_string(),
                vehicle_timestamp_to_py(py, sample.vehicle_time)?,
                monotonic_seconds(sample.received_at),
            )),
            Self::GuidanceState(sample) => Ok(PyMetricSample::new(
                Py::new(py, PyGuidanceState::from(sample.value))?.into_any(),
                telemetry_message_kind_name(sample.source).to_string(),
                vehicle_timestamp_to_py(py, sample.vehicle_time)?,
                monotonic_seconds(sample.received_at),
            )),
            Self::TerrainClearance(sample) => Ok(PyMetricSample::new(
                Py::new(py, PyTerrainClearance::from(sample.value))?.into_any(),
                telemetry_message_kind_name(sample.source).to_string(),
                vehicle_timestamp_to_py(py, sample.vehicle_time)?,
                monotonic_seconds(sample.received_at),
            )),
            Self::SensorHealthSummary(sample) => Ok(PyMetricSample::new(
                Py::new(py, PySensorHealthSummary::from(sample.value))?.into_any(),
                telemetry_message_kind_name(sample.source).to_string(),
                vehicle_timestamp_to_py(py, sample.vehicle_time)?,
                monotonic_seconds(sample.received_at),
            )),
        }
    }
}

#[derive(Clone)]
enum MetricHandleKind {
    Bool(mavkit::MetricHandle<bool>),
    F64(mavkit::MetricHandle<f64>),
    U8(mavkit::MetricHandle<u8>),
    I32(mavkit::MetricHandle<i32>),
    U16(mavkit::MetricHandle<u16>),
    GlobalPosition(mavkit::MetricHandle<mavkit::GlobalPosition>),
    EulerAttitude(mavkit::MetricHandle<mavkit::EulerAttitude>),
    GpsQuality(mavkit::MetricHandle<mavkit::GpsQuality>),
    GeoPoint3dMsl(mavkit::MetricHandle<mavkit::GeoPoint3dMsl>),
    CellVoltages(mavkit::MetricHandle<mavkit::CellVoltages>),
    WaypointProgress(mavkit::MetricHandle<mavkit::WaypointProgress>),
    GuidanceState(mavkit::MetricHandle<mavkit::GuidanceState>),
    TerrainClearance(mavkit::MetricHandle<mavkit::TerrainClearance>),
    SensorHealthSummary(mavkit::MetricHandle<mavkit::SensorHealthSummary>),
}

impl MetricHandleKind {
    fn latest_value(&self) -> Option<MetricSampleValue> {
        match self {
            Self::Bool(handle) => handle.latest().map(MetricSampleValue::Bool),
            Self::F64(handle) => handle.latest().map(MetricSampleValue::F64),
            Self::U8(handle) => handle.latest().map(MetricSampleValue::U8),
            Self::I32(handle) => handle.latest().map(MetricSampleValue::I32),
            Self::U16(handle) => handle.latest().map(MetricSampleValue::U16),
            Self::GlobalPosition(handle) => handle.latest().map(MetricSampleValue::GlobalPosition),
            Self::EulerAttitude(handle) => handle.latest().map(MetricSampleValue::EulerAttitude),
            Self::GpsQuality(handle) => handle.latest().map(MetricSampleValue::GpsQuality),
            Self::GeoPoint3dMsl(handle) => handle.latest().map(MetricSampleValue::GeoPoint3dMsl),
            Self::CellVoltages(handle) => handle.latest().map(MetricSampleValue::CellVoltages),
            Self::WaypointProgress(handle) => {
                handle.latest().map(MetricSampleValue::WaypointProgress)
            }
            Self::GuidanceState(handle) => handle.latest().map(MetricSampleValue::GuidanceState),
            Self::TerrainClearance(handle) => {
                handle.latest().map(MetricSampleValue::TerrainClearance)
            }
            Self::SensorHealthSummary(handle) => {
                handle.latest().map(MetricSampleValue::SensorHealthSummary)
            }
        }
    }

    async fn wait_value(self) -> Result<MetricSampleValue, mavkit::VehicleError> {
        match self {
            Self::Bool(handle) => handle.wait().await.map(MetricSampleValue::Bool),
            Self::F64(handle) => handle.wait().await.map(MetricSampleValue::F64),
            Self::U8(handle) => handle.wait().await.map(MetricSampleValue::U8),
            Self::I32(handle) => handle.wait().await.map(MetricSampleValue::I32),
            Self::U16(handle) => handle.wait().await.map(MetricSampleValue::U16),
            Self::GlobalPosition(handle) => {
                handle.wait().await.map(MetricSampleValue::GlobalPosition)
            }
            Self::EulerAttitude(handle) => {
                handle.wait().await.map(MetricSampleValue::EulerAttitude)
            }
            Self::GpsQuality(handle) => handle.wait().await.map(MetricSampleValue::GpsQuality),
            Self::GeoPoint3dMsl(handle) => {
                handle.wait().await.map(MetricSampleValue::GeoPoint3dMsl)
            }
            Self::CellVoltages(handle) => handle.wait().await.map(MetricSampleValue::CellVoltages),
            Self::WaypointProgress(handle) => {
                handle.wait().await.map(MetricSampleValue::WaypointProgress)
            }
            Self::GuidanceState(handle) => {
                handle.wait().await.map(MetricSampleValue::GuidanceState)
            }
            Self::TerrainClearance(handle) => {
                handle.wait().await.map(MetricSampleValue::TerrainClearance)
            }
            Self::SensorHealthSummary(handle) => handle
                .wait()
                .await
                .map(MetricSampleValue::SensorHealthSummary),
        }
    }

    async fn wait_timeout_value(
        self,
        timeout: Duration,
    ) -> Result<MetricSampleValue, mavkit::VehicleError> {
        match self {
            Self::Bool(handle) => handle
                .wait_timeout(timeout)
                .await
                .map(MetricSampleValue::Bool),
            Self::F64(handle) => handle
                .wait_timeout(timeout)
                .await
                .map(MetricSampleValue::F64),
            Self::U8(handle) => handle
                .wait_timeout(timeout)
                .await
                .map(MetricSampleValue::U8),
            Self::I32(handle) => handle
                .wait_timeout(timeout)
                .await
                .map(MetricSampleValue::I32),
            Self::U16(handle) => handle
                .wait_timeout(timeout)
                .await
                .map(MetricSampleValue::U16),
            Self::GlobalPosition(handle) => handle
                .wait_timeout(timeout)
                .await
                .map(MetricSampleValue::GlobalPosition),
            Self::EulerAttitude(handle) => handle
                .wait_timeout(timeout)
                .await
                .map(MetricSampleValue::EulerAttitude),
            Self::GpsQuality(handle) => handle
                .wait_timeout(timeout)
                .await
                .map(MetricSampleValue::GpsQuality),
            Self::GeoPoint3dMsl(handle) => handle
                .wait_timeout(timeout)
                .await
                .map(MetricSampleValue::GeoPoint3dMsl),
            Self::CellVoltages(handle) => handle
                .wait_timeout(timeout)
                .await
                .map(MetricSampleValue::CellVoltages),
            Self::WaypointProgress(handle) => handle
                .wait_timeout(timeout)
                .await
                .map(MetricSampleValue::WaypointProgress),
            Self::GuidanceState(handle) => handle
                .wait_timeout(timeout)
                .await
                .map(MetricSampleValue::GuidanceState),
            Self::TerrainClearance(handle) => handle
                .wait_timeout(timeout)
                .await
                .map(MetricSampleValue::TerrainClearance),
            Self::SensorHealthSummary(handle) => handle
                .wait_timeout(timeout)
                .await
                .map(MetricSampleValue::SensorHealthSummary),
        }
    }

    fn subscribe_value(&self) -> MetricSubscriptionKind {
        match self {
            Self::Bool(handle) => MetricSubscriptionKind::Bool(handle.subscribe()),
            Self::F64(handle) => MetricSubscriptionKind::F64(handle.subscribe()),
            Self::U8(handle) => MetricSubscriptionKind::U8(handle.subscribe()),
            Self::I32(handle) => MetricSubscriptionKind::I32(handle.subscribe()),
            Self::U16(handle) => MetricSubscriptionKind::U16(handle.subscribe()),
            Self::GlobalPosition(handle) => {
                MetricSubscriptionKind::GlobalPosition(handle.subscribe())
            }
            Self::EulerAttitude(handle) => {
                MetricSubscriptionKind::EulerAttitude(handle.subscribe())
            }
            Self::GpsQuality(handle) => MetricSubscriptionKind::GpsQuality(handle.subscribe()),
            Self::GeoPoint3dMsl(handle) => {
                MetricSubscriptionKind::GeoPoint3dMsl(handle.subscribe())
            }
            Self::CellVoltages(handle) => MetricSubscriptionKind::CellVoltages(handle.subscribe()),
            Self::WaypointProgress(handle) => {
                MetricSubscriptionKind::WaypointProgress(handle.subscribe())
            }
            Self::GuidanceState(handle) => {
                MetricSubscriptionKind::GuidanceState(handle.subscribe())
            }
            Self::TerrainClearance(handle) => {
                MetricSubscriptionKind::TerrainClearance(handle.subscribe())
            }
            Self::SensorHealthSummary(handle) => {
                MetricSubscriptionKind::SensorHealthSummary(handle.subscribe())
            }
        }
    }

    fn support_name(&self) -> Option<&'static str> {
        match self {
            Self::Bool(handle) => support_state_name(handle.support().latest()),
            Self::F64(handle) => support_state_name(handle.support().latest()),
            Self::U8(handle) => support_state_name(handle.support().latest()),
            Self::I32(handle) => support_state_name(handle.support().latest()),
            Self::U16(handle) => support_state_name(handle.support().latest()),
            Self::GlobalPosition(handle) => support_state_name(handle.support().latest()),
            Self::EulerAttitude(handle) => support_state_name(handle.support().latest()),
            Self::GpsQuality(handle) => support_state_name(handle.support().latest()),
            Self::GeoPoint3dMsl(handle) => support_state_name(handle.support().latest()),
            Self::CellVoltages(handle) => support_state_name(handle.support().latest()),
            Self::WaypointProgress(handle) => support_state_name(handle.support().latest()),
            Self::GuidanceState(handle) => support_state_name(handle.support().latest()),
            Self::TerrainClearance(handle) => support_state_name(handle.support().latest()),
            Self::SensorHealthSummary(handle) => support_state_name(handle.support().latest()),
        }
    }
}

enum MetricSubscriptionKind {
    Bool(mavkit::ObservationSubscription<mavkit::MetricSample<bool>>),
    F64(mavkit::ObservationSubscription<mavkit::MetricSample<f64>>),
    U8(mavkit::ObservationSubscription<mavkit::MetricSample<u8>>),
    I32(mavkit::ObservationSubscription<mavkit::MetricSample<i32>>),
    U16(mavkit::ObservationSubscription<mavkit::MetricSample<u16>>),
    GlobalPosition(mavkit::ObservationSubscription<mavkit::MetricSample<mavkit::GlobalPosition>>),
    EulerAttitude(mavkit::ObservationSubscription<mavkit::MetricSample<mavkit::EulerAttitude>>),
    GpsQuality(mavkit::ObservationSubscription<mavkit::MetricSample<mavkit::GpsQuality>>),
    GeoPoint3dMsl(mavkit::ObservationSubscription<mavkit::MetricSample<mavkit::GeoPoint3dMsl>>),
    CellVoltages(mavkit::ObservationSubscription<mavkit::MetricSample<mavkit::CellVoltages>>),
    WaypointProgress(
        mavkit::ObservationSubscription<mavkit::MetricSample<mavkit::WaypointProgress>>,
    ),
    GuidanceState(mavkit::ObservationSubscription<mavkit::MetricSample<mavkit::GuidanceState>>),
    TerrainClearance(
        mavkit::ObservationSubscription<mavkit::MetricSample<mavkit::TerrainClearance>>,
    ),
    SensorHealthSummary(
        mavkit::ObservationSubscription<mavkit::MetricSample<mavkit::SensorHealthSummary>>,
    ),
}

impl MetricSubscriptionKind {
    async fn recv_value(&mut self) -> Option<MetricSampleValue> {
        match self {
            Self::Bool(subscription) => subscription.recv().await.map(MetricSampleValue::Bool),
            Self::F64(subscription) => subscription.recv().await.map(MetricSampleValue::F64),
            Self::U8(subscription) => subscription.recv().await.map(MetricSampleValue::U8),
            Self::I32(subscription) => subscription.recv().await.map(MetricSampleValue::I32),
            Self::U16(subscription) => subscription.recv().await.map(MetricSampleValue::U16),
            Self::GlobalPosition(subscription) => subscription
                .recv()
                .await
                .map(MetricSampleValue::GlobalPosition),
            Self::EulerAttitude(subscription) => subscription
                .recv()
                .await
                .map(MetricSampleValue::EulerAttitude),
            Self::GpsQuality(subscription) => {
                subscription.recv().await.map(MetricSampleValue::GpsQuality)
            }
            Self::GeoPoint3dMsl(subscription) => subscription
                .recv()
                .await
                .map(MetricSampleValue::GeoPoint3dMsl),
            Self::CellVoltages(subscription) => subscription
                .recv()
                .await
                .map(MetricSampleValue::CellVoltages),
            Self::WaypointProgress(subscription) => subscription
                .recv()
                .await
                .map(MetricSampleValue::WaypointProgress),
            Self::GuidanceState(subscription) => subscription
                .recv()
                .await
                .map(MetricSampleValue::GuidanceState),
            Self::TerrainClearance(subscription) => subscription
                .recv()
                .await
                .map(MetricSampleValue::TerrainClearance),
            Self::SensorHealthSummary(subscription) => subscription
                .recv()
                .await
                .map(MetricSampleValue::SensorHealthSummary),
        }
    }
}

#[pyclass(name = "MetricHandle", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyMetricHandle {
    inner: MetricHandleKind,
}

impl PyMetricHandle {
    fn bool(inner: mavkit::MetricHandle<bool>) -> Self {
        Self {
            inner: MetricHandleKind::Bool(inner),
        }
    }

    fn f64(inner: mavkit::MetricHandle<f64>) -> Self {
        Self {
            inner: MetricHandleKind::F64(inner),
        }
    }

    fn u8(inner: mavkit::MetricHandle<u8>) -> Self {
        Self {
            inner: MetricHandleKind::U8(inner),
        }
    }

    fn i32(inner: mavkit::MetricHandle<i32>) -> Self {
        Self {
            inner: MetricHandleKind::I32(inner),
        }
    }

    fn u16(inner: mavkit::MetricHandle<u16>) -> Self {
        Self {
            inner: MetricHandleKind::U16(inner),
        }
    }

    fn global_position(inner: mavkit::MetricHandle<mavkit::GlobalPosition>) -> Self {
        Self {
            inner: MetricHandleKind::GlobalPosition(inner),
        }
    }

    fn euler_attitude(inner: mavkit::MetricHandle<mavkit::EulerAttitude>) -> Self {
        Self {
            inner: MetricHandleKind::EulerAttitude(inner),
        }
    }

    fn gps_quality(inner: mavkit::MetricHandle<mavkit::GpsQuality>) -> Self {
        Self {
            inner: MetricHandleKind::GpsQuality(inner),
        }
    }

    pub(crate) fn geo_point_3d_msl(inner: mavkit::MetricHandle<mavkit::GeoPoint3dMsl>) -> Self {
        Self {
            inner: MetricHandleKind::GeoPoint3dMsl(inner),
        }
    }

    fn cell_voltages(inner: mavkit::MetricHandle<mavkit::CellVoltages>) -> Self {
        Self {
            inner: MetricHandleKind::CellVoltages(inner),
        }
    }

    fn waypoint_progress(inner: mavkit::MetricHandle<mavkit::WaypointProgress>) -> Self {
        Self {
            inner: MetricHandleKind::WaypointProgress(inner),
        }
    }

    fn guidance_state(inner: mavkit::MetricHandle<mavkit::GuidanceState>) -> Self {
        Self {
            inner: MetricHandleKind::GuidanceState(inner),
        }
    }

    fn terrain_clearance(inner: mavkit::MetricHandle<mavkit::TerrainClearance>) -> Self {
        Self {
            inner: MetricHandleKind::TerrainClearance(inner),
        }
    }

    fn sensor_health_summary(inner: mavkit::MetricHandle<mavkit::SensorHealthSummary>) -> Self {
        Self {
            inner: MetricHandleKind::SensorHealthSummary(inner),
        }
    }
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

enum MessageSampleValue {
    VfrHud(mavkit::MessageSample<dialect::VFR_HUD_DATA>),
    GlobalPositionInt(mavkit::MessageSample<dialect::GLOBAL_POSITION_INT_DATA>),
    LocalPositionNed(mavkit::MessageSample<dialect::LOCAL_POSITION_NED_DATA>),
    GpsRawInt(mavkit::MessageSample<dialect::GPS_RAW_INT_DATA>),
    Attitude(mavkit::MessageSample<dialect::ATTITUDE_DATA>),
    SysStatus(mavkit::MessageSample<dialect::SYS_STATUS_DATA>),
    BatteryStatus(mavkit::MessageSample<dialect::BATTERY_STATUS_DATA>),
    NavControllerOutput(mavkit::MessageSample<dialect::NAV_CONTROLLER_OUTPUT_DATA>),
    TerrainReport(mavkit::MessageSample<dialect::TERRAIN_REPORT_DATA>),
    RcChannels(mavkit::MessageSample<dialect::RC_CHANNELS_DATA>),
    ServoOutputRaw(mavkit::MessageSample<dialect::SERVO_OUTPUT_RAW_DATA>),
    HomePosition(mavkit::MessageSample<dialect::HOME_POSITION_DATA>),
    GpsGlobalOrigin(mavkit::MessageSample<dialect::GPS_GLOBAL_ORIGIN_DATA>),
    StatusText(mavkit::MessageSample<mavkit::StatusTextEvent>),
}

impl MessageSampleValue {
    fn into_py(self, py: Python<'_>) -> PyResult<PyMessageSample> {
        match self {
            Self::VfrHud(sample) => Ok(PyMessageSample::new(
                serialize_to_py!(py, sample.value)?,
                vehicle_timestamp_to_py(py, sample.vehicle_time)?,
                monotonic_seconds(sample.received_at),
            )),
            Self::GlobalPositionInt(sample) => Ok(PyMessageSample::new(
                serialize_to_py!(py, sample.value)?,
                vehicle_timestamp_to_py(py, sample.vehicle_time)?,
                monotonic_seconds(sample.received_at),
            )),
            Self::LocalPositionNed(sample) => Ok(PyMessageSample::new(
                serialize_to_py!(py, sample.value)?,
                vehicle_timestamp_to_py(py, sample.vehicle_time)?,
                monotonic_seconds(sample.received_at),
            )),
            Self::GpsRawInt(sample) => Ok(PyMessageSample::new(
                serialize_to_py!(py, sample.value)?,
                vehicle_timestamp_to_py(py, sample.vehicle_time)?,
                monotonic_seconds(sample.received_at),
            )),
            Self::Attitude(sample) => Ok(PyMessageSample::new(
                serialize_to_py!(py, sample.value)?,
                vehicle_timestamp_to_py(py, sample.vehicle_time)?,
                monotonic_seconds(sample.received_at),
            )),
            Self::SysStatus(sample) => Ok(PyMessageSample::new(
                serialize_to_py!(py, sample.value)?,
                vehicle_timestamp_to_py(py, sample.vehicle_time)?,
                monotonic_seconds(sample.received_at),
            )),
            Self::BatteryStatus(sample) => Ok(PyMessageSample::new(
                serialize_to_py!(py, sample.value)?,
                vehicle_timestamp_to_py(py, sample.vehicle_time)?,
                monotonic_seconds(sample.received_at),
            )),
            Self::NavControllerOutput(sample) => Ok(PyMessageSample::new(
                serialize_to_py!(py, sample.value)?,
                vehicle_timestamp_to_py(py, sample.vehicle_time)?,
                monotonic_seconds(sample.received_at),
            )),
            Self::TerrainReport(sample) => Ok(PyMessageSample::new(
                serialize_to_py!(py, sample.value)?,
                vehicle_timestamp_to_py(py, sample.vehicle_time)?,
                monotonic_seconds(sample.received_at),
            )),
            Self::RcChannels(sample) => Ok(PyMessageSample::new(
                serialize_to_py!(py, sample.value)?,
                vehicle_timestamp_to_py(py, sample.vehicle_time)?,
                monotonic_seconds(sample.received_at),
            )),
            Self::ServoOutputRaw(sample) => Ok(PyMessageSample::new(
                serialize_to_py!(py, sample.value)?,
                vehicle_timestamp_to_py(py, sample.vehicle_time)?,
                monotonic_seconds(sample.received_at),
            )),
            Self::HomePosition(sample) => Ok(PyMessageSample::new(
                serialize_to_py!(py, sample.value)?,
                vehicle_timestamp_to_py(py, sample.vehicle_time)?,
                monotonic_seconds(sample.received_at),
            )),
            Self::GpsGlobalOrigin(sample) => Ok(PyMessageSample::new(
                serialize_to_py!(py, sample.value)?,
                vehicle_timestamp_to_py(py, sample.vehicle_time)?,
                monotonic_seconds(sample.received_at),
            )),
            Self::StatusText(sample) => Ok(PyMessageSample::new(
                Py::new(py, PyStatusTextEvent::from(sample.value))?.into_any(),
                vehicle_timestamp_to_py(py, sample.vehicle_time)?,
                monotonic_seconds(sample.received_at),
            )),
        }
    }
}

#[derive(Clone)]
enum PeriodicMessageHandleKind {
    VfrHud(mavkit::PeriodicMessageHandle<dialect::VFR_HUD_DATA>),
    GlobalPositionInt(mavkit::PeriodicMessageHandle<dialect::GLOBAL_POSITION_INT_DATA>),
    LocalPositionNed(mavkit::PeriodicMessageHandle<dialect::LOCAL_POSITION_NED_DATA>),
    GpsRawInt(mavkit::PeriodicMessageHandle<dialect::GPS_RAW_INT_DATA>),
    Attitude(mavkit::PeriodicMessageHandle<dialect::ATTITUDE_DATA>),
    SysStatus(mavkit::PeriodicMessageHandle<dialect::SYS_STATUS_DATA>),
    BatteryStatus(mavkit::PeriodicMessageHandle<dialect::BATTERY_STATUS_DATA>),
    NavControllerOutput(mavkit::PeriodicMessageHandle<dialect::NAV_CONTROLLER_OUTPUT_DATA>),
    TerrainReport(mavkit::PeriodicMessageHandle<dialect::TERRAIN_REPORT_DATA>),
    RcChannels(mavkit::PeriodicMessageHandle<dialect::RC_CHANNELS_DATA>),
    ServoOutputRaw(mavkit::PeriodicMessageHandle<dialect::SERVO_OUTPUT_RAW_DATA>),
}

impl PeriodicMessageHandleKind {
    fn latest_value(&self) -> Option<MessageSampleValue> {
        match self {
            Self::VfrHud(handle) => handle.latest().map(MessageSampleValue::VfrHud),
            Self::GlobalPositionInt(handle) => {
                handle.latest().map(MessageSampleValue::GlobalPositionInt)
            }
            Self::LocalPositionNed(handle) => {
                handle.latest().map(MessageSampleValue::LocalPositionNed)
            }
            Self::GpsRawInt(handle) => handle.latest().map(MessageSampleValue::GpsRawInt),
            Self::Attitude(handle) => handle.latest().map(MessageSampleValue::Attitude),
            Self::SysStatus(handle) => handle.latest().map(MessageSampleValue::SysStatus),
            Self::BatteryStatus(handle) => handle.latest().map(MessageSampleValue::BatteryStatus),
            Self::NavControllerOutput(handle) => {
                handle.latest().map(MessageSampleValue::NavControllerOutput)
            }
            Self::TerrainReport(handle) => handle.latest().map(MessageSampleValue::TerrainReport),
            Self::RcChannels(handle) => handle.latest().map(MessageSampleValue::RcChannels),
            Self::ServoOutputRaw(handle) => handle.latest().map(MessageSampleValue::ServoOutputRaw),
        }
    }

    async fn wait_value(self) -> Result<MessageSampleValue, mavkit::VehicleError> {
        match self {
            Self::VfrHud(handle) => handle.wait().await.map(MessageSampleValue::VfrHud),
            Self::GlobalPositionInt(handle) => handle
                .wait()
                .await
                .map(MessageSampleValue::GlobalPositionInt),
            Self::LocalPositionNed(handle) => handle
                .wait()
                .await
                .map(MessageSampleValue::LocalPositionNed),
            Self::GpsRawInt(handle) => handle.wait().await.map(MessageSampleValue::GpsRawInt),
            Self::Attitude(handle) => handle.wait().await.map(MessageSampleValue::Attitude),
            Self::SysStatus(handle) => handle.wait().await.map(MessageSampleValue::SysStatus),
            Self::BatteryStatus(handle) => {
                handle.wait().await.map(MessageSampleValue::BatteryStatus)
            }
            Self::NavControllerOutput(handle) => handle
                .wait()
                .await
                .map(MessageSampleValue::NavControllerOutput),
            Self::TerrainReport(handle) => {
                handle.wait().await.map(MessageSampleValue::TerrainReport)
            }
            Self::RcChannels(handle) => handle.wait().await.map(MessageSampleValue::RcChannels),
            Self::ServoOutputRaw(handle) => {
                handle.wait().await.map(MessageSampleValue::ServoOutputRaw)
            }
        }
    }

    async fn wait_timeout_value(
        self,
        timeout: Duration,
    ) -> Result<MessageSampleValue, mavkit::VehicleError> {
        match self {
            Self::VfrHud(handle) => handle
                .wait_timeout(timeout)
                .await
                .map(MessageSampleValue::VfrHud),
            Self::GlobalPositionInt(handle) => handle
                .wait_timeout(timeout)
                .await
                .map(MessageSampleValue::GlobalPositionInt),
            Self::LocalPositionNed(handle) => handle
                .wait_timeout(timeout)
                .await
                .map(MessageSampleValue::LocalPositionNed),
            Self::GpsRawInt(handle) => handle
                .wait_timeout(timeout)
                .await
                .map(MessageSampleValue::GpsRawInt),
            Self::Attitude(handle) => handle
                .wait_timeout(timeout)
                .await
                .map(MessageSampleValue::Attitude),
            Self::SysStatus(handle) => handle
                .wait_timeout(timeout)
                .await
                .map(MessageSampleValue::SysStatus),
            Self::BatteryStatus(handle) => handle
                .wait_timeout(timeout)
                .await
                .map(MessageSampleValue::BatteryStatus),
            Self::NavControllerOutput(handle) => handle
                .wait_timeout(timeout)
                .await
                .map(MessageSampleValue::NavControllerOutput),
            Self::TerrainReport(handle) => handle
                .wait_timeout(timeout)
                .await
                .map(MessageSampleValue::TerrainReport),
            Self::RcChannels(handle) => handle
                .wait_timeout(timeout)
                .await
                .map(MessageSampleValue::RcChannels),
            Self::ServoOutputRaw(handle) => handle
                .wait_timeout(timeout)
                .await
                .map(MessageSampleValue::ServoOutputRaw),
        }
    }

    async fn request_value(
        self,
        timeout: Duration,
    ) -> Result<MessageSampleValue, mavkit::VehicleError> {
        match self {
            Self::VfrHud(handle) => handle
                .request(timeout)
                .await
                .map(MessageSampleValue::VfrHud),
            Self::GlobalPositionInt(handle) => handle
                .request(timeout)
                .await
                .map(MessageSampleValue::GlobalPositionInt),
            Self::LocalPositionNed(handle) => handle
                .request(timeout)
                .await
                .map(MessageSampleValue::LocalPositionNed),
            Self::GpsRawInt(handle) => handle
                .request(timeout)
                .await
                .map(MessageSampleValue::GpsRawInt),
            Self::Attitude(handle) => handle
                .request(timeout)
                .await
                .map(MessageSampleValue::Attitude),
            Self::SysStatus(handle) => handle
                .request(timeout)
                .await
                .map(MessageSampleValue::SysStatus),
            Self::BatteryStatus(handle) => handle
                .request(timeout)
                .await
                .map(MessageSampleValue::BatteryStatus),
            Self::NavControllerOutput(handle) => handle
                .request(timeout)
                .await
                .map(MessageSampleValue::NavControllerOutput),
            Self::TerrainReport(handle) => handle
                .request(timeout)
                .await
                .map(MessageSampleValue::TerrainReport),
            Self::RcChannels(handle) => handle
                .request(timeout)
                .await
                .map(MessageSampleValue::RcChannels),
            Self::ServoOutputRaw(handle) => handle
                .request(timeout)
                .await
                .map(MessageSampleValue::ServoOutputRaw),
        }
    }

    async fn set_rate(self, hz: f32) -> Result<(), mavkit::VehicleError> {
        match self {
            Self::VfrHud(handle) => handle.set_rate(hz).await,
            Self::GlobalPositionInt(handle) => handle.set_rate(hz).await,
            Self::LocalPositionNed(handle) => handle.set_rate(hz).await,
            Self::GpsRawInt(handle) => handle.set_rate(hz).await,
            Self::Attitude(handle) => handle.set_rate(hz).await,
            Self::SysStatus(handle) => handle.set_rate(hz).await,
            Self::BatteryStatus(handle) => handle.set_rate(hz).await,
            Self::NavControllerOutput(handle) => handle.set_rate(hz).await,
            Self::TerrainReport(handle) => handle.set_rate(hz).await,
            Self::RcChannels(handle) => handle.set_rate(hz).await,
            Self::ServoOutputRaw(handle) => handle.set_rate(hz).await,
        }
    }

    fn subscribe_value(&self) -> MessageSubscriptionKind {
        match self {
            Self::VfrHud(handle) => MessageSubscriptionKind::VfrHud(handle.subscribe()),
            Self::GlobalPositionInt(handle) => {
                MessageSubscriptionKind::GlobalPositionInt(handle.subscribe())
            }
            Self::LocalPositionNed(handle) => {
                MessageSubscriptionKind::LocalPositionNed(handle.subscribe())
            }
            Self::GpsRawInt(handle) => MessageSubscriptionKind::GpsRawInt(handle.subscribe()),
            Self::Attitude(handle) => MessageSubscriptionKind::Attitude(handle.subscribe()),
            Self::SysStatus(handle) => MessageSubscriptionKind::SysStatus(handle.subscribe()),
            Self::BatteryStatus(handle) => {
                MessageSubscriptionKind::BatteryStatus(handle.subscribe())
            }
            Self::NavControllerOutput(handle) => {
                MessageSubscriptionKind::NavControllerOutput(handle.subscribe())
            }
            Self::TerrainReport(handle) => {
                MessageSubscriptionKind::TerrainReport(handle.subscribe())
            }
            Self::RcChannels(handle) => MessageSubscriptionKind::RcChannels(handle.subscribe()),
            Self::ServoOutputRaw(handle) => {
                MessageSubscriptionKind::ServoOutputRaw(handle.subscribe())
            }
        }
    }

    fn support_name(&self) -> Option<&'static str> {
        match self {
            Self::VfrHud(handle) => support_state_name(handle.support().latest()),
            Self::GlobalPositionInt(handle) => support_state_name(handle.support().latest()),
            Self::LocalPositionNed(handle) => support_state_name(handle.support().latest()),
            Self::GpsRawInt(handle) => support_state_name(handle.support().latest()),
            Self::Attitude(handle) => support_state_name(handle.support().latest()),
            Self::SysStatus(handle) => support_state_name(handle.support().latest()),
            Self::BatteryStatus(handle) => support_state_name(handle.support().latest()),
            Self::NavControllerOutput(handle) => support_state_name(handle.support().latest()),
            Self::TerrainReport(handle) => support_state_name(handle.support().latest()),
            Self::RcChannels(handle) => support_state_name(handle.support().latest()),
            Self::ServoOutputRaw(handle) => support_state_name(handle.support().latest()),
        }
    }
}

#[derive(Clone)]
enum EventMessageHandleKind {
    HomePosition(mavkit::EventMessageHandle<dialect::HOME_POSITION_DATA>),
    GpsGlobalOrigin(mavkit::EventMessageHandle<dialect::GPS_GLOBAL_ORIGIN_DATA>),
}

impl EventMessageHandleKind {
    fn latest_value(&self) -> Option<MessageSampleValue> {
        match self {
            Self::HomePosition(handle) => handle.latest().map(MessageSampleValue::HomePosition),
            Self::GpsGlobalOrigin(handle) => {
                handle.latest().map(MessageSampleValue::GpsGlobalOrigin)
            }
        }
    }

    async fn wait_value(self) -> Result<MessageSampleValue, mavkit::VehicleError> {
        match self {
            Self::HomePosition(handle) => handle.wait().await.map(MessageSampleValue::HomePosition),
            Self::GpsGlobalOrigin(handle) => {
                handle.wait().await.map(MessageSampleValue::GpsGlobalOrigin)
            }
        }
    }

    async fn wait_timeout_value(
        self,
        timeout: Duration,
    ) -> Result<MessageSampleValue, mavkit::VehicleError> {
        match self {
            Self::HomePosition(handle) => handle
                .wait_timeout(timeout)
                .await
                .map(MessageSampleValue::HomePosition),
            Self::GpsGlobalOrigin(handle) => handle
                .wait_timeout(timeout)
                .await
                .map(MessageSampleValue::GpsGlobalOrigin),
        }
    }

    async fn request_value(
        self,
        timeout: Duration,
    ) -> Result<MessageSampleValue, mavkit::VehicleError> {
        match self {
            Self::HomePosition(handle) => handle
                .request(timeout)
                .await
                .map(MessageSampleValue::HomePosition),
            Self::GpsGlobalOrigin(handle) => handle
                .request(timeout)
                .await
                .map(MessageSampleValue::GpsGlobalOrigin),
        }
    }

    fn subscribe_value(&self) -> MessageSubscriptionKind {
        match self {
            Self::HomePosition(handle) => MessageSubscriptionKind::HomePosition(handle.subscribe()),
            Self::GpsGlobalOrigin(handle) => {
                MessageSubscriptionKind::GpsGlobalOrigin(handle.subscribe())
            }
        }
    }

    fn support_name(&self) -> Option<&'static str> {
        match self {
            Self::HomePosition(handle) => support_state_name(handle.support().latest()),
            Self::GpsGlobalOrigin(handle) => support_state_name(handle.support().latest()),
        }
    }
}

#[derive(Clone)]
enum PushMessageHandleKind {
    StatusText(mavkit::MessageHandle<mavkit::StatusTextEvent>),
}

impl PushMessageHandleKind {
    fn latest_value(&self) -> Option<MessageSampleValue> {
        match self {
            Self::StatusText(handle) => handle.latest().map(MessageSampleValue::StatusText),
        }
    }

    async fn wait_value(self) -> Result<MessageSampleValue, mavkit::VehicleError> {
        match self {
            Self::StatusText(handle) => handle.wait().await.map(MessageSampleValue::StatusText),
        }
    }

    async fn wait_timeout_value(
        self,
        timeout: Duration,
    ) -> Result<MessageSampleValue, mavkit::VehicleError> {
        match self {
            Self::StatusText(handle) => handle
                .wait_timeout(timeout)
                .await
                .map(MessageSampleValue::StatusText),
        }
    }

    fn subscribe_value(&self) -> MessageSubscriptionKind {
        match self {
            Self::StatusText(handle) => MessageSubscriptionKind::StatusText(handle.subscribe()),
        }
    }

    fn support_name(&self) -> Option<&'static str> {
        match self {
            Self::StatusText(handle) => support_state_name(handle.support().latest()),
        }
    }
}

enum MessageSubscriptionKind {
    VfrHud(mavkit::ObservationSubscription<mavkit::MessageSample<dialect::VFR_HUD_DATA>>),
    GlobalPositionInt(
        mavkit::ObservationSubscription<mavkit::MessageSample<dialect::GLOBAL_POSITION_INT_DATA>>,
    ),
    LocalPositionNed(
        mavkit::ObservationSubscription<mavkit::MessageSample<dialect::LOCAL_POSITION_NED_DATA>>,
    ),
    GpsRawInt(mavkit::ObservationSubscription<mavkit::MessageSample<dialect::GPS_RAW_INT_DATA>>),
    Attitude(mavkit::ObservationSubscription<mavkit::MessageSample<dialect::ATTITUDE_DATA>>),
    SysStatus(mavkit::ObservationSubscription<mavkit::MessageSample<dialect::SYS_STATUS_DATA>>),
    BatteryStatus(
        mavkit::ObservationSubscription<mavkit::MessageSample<dialect::BATTERY_STATUS_DATA>>,
    ),
    NavControllerOutput(
        mavkit::ObservationSubscription<mavkit::MessageSample<dialect::NAV_CONTROLLER_OUTPUT_DATA>>,
    ),
    TerrainReport(
        mavkit::ObservationSubscription<mavkit::MessageSample<dialect::TERRAIN_REPORT_DATA>>,
    ),
    RcChannels(mavkit::ObservationSubscription<mavkit::MessageSample<dialect::RC_CHANNELS_DATA>>),
    ServoOutputRaw(
        mavkit::ObservationSubscription<mavkit::MessageSample<dialect::SERVO_OUTPUT_RAW_DATA>>,
    ),
    HomePosition(
        mavkit::ObservationSubscription<mavkit::MessageSample<dialect::HOME_POSITION_DATA>>,
    ),
    GpsGlobalOrigin(
        mavkit::ObservationSubscription<mavkit::MessageSample<dialect::GPS_GLOBAL_ORIGIN_DATA>>,
    ),
    StatusText(mavkit::ObservationSubscription<mavkit::MessageSample<mavkit::StatusTextEvent>>),
}

impl MessageSubscriptionKind {
    async fn recv_value(&mut self) -> Option<MessageSampleValue> {
        match self {
            Self::VfrHud(subscription) => subscription.recv().await.map(MessageSampleValue::VfrHud),
            Self::GlobalPositionInt(subscription) => subscription
                .recv()
                .await
                .map(MessageSampleValue::GlobalPositionInt),
            Self::LocalPositionNed(subscription) => subscription
                .recv()
                .await
                .map(MessageSampleValue::LocalPositionNed),
            Self::GpsRawInt(subscription) => {
                subscription.recv().await.map(MessageSampleValue::GpsRawInt)
            }
            Self::Attitude(subscription) => {
                subscription.recv().await.map(MessageSampleValue::Attitude)
            }
            Self::SysStatus(subscription) => {
                subscription.recv().await.map(MessageSampleValue::SysStatus)
            }
            Self::BatteryStatus(subscription) => subscription
                .recv()
                .await
                .map(MessageSampleValue::BatteryStatus),
            Self::NavControllerOutput(subscription) => subscription
                .recv()
                .await
                .map(MessageSampleValue::NavControllerOutput),
            Self::TerrainReport(subscription) => subscription
                .recv()
                .await
                .map(MessageSampleValue::TerrainReport),
            Self::RcChannels(subscription) => subscription
                .recv()
                .await
                .map(MessageSampleValue::RcChannels),
            Self::ServoOutputRaw(subscription) => subscription
                .recv()
                .await
                .map(MessageSampleValue::ServoOutputRaw),
            Self::HomePosition(subscription) => subscription
                .recv()
                .await
                .map(MessageSampleValue::HomePosition),
            Self::GpsGlobalOrigin(subscription) => subscription
                .recv()
                .await
                .map(MessageSampleValue::GpsGlobalOrigin),
            Self::StatusText(subscription) => subscription
                .recv()
                .await
                .map(MessageSampleValue::StatusText),
        }
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

impl PyTelemetryHandle {
    pub(crate) fn new(inner: mavkit::Vehicle) -> Self {
        Self { inner }
    }
}

#[pymethods]
impl PyTelemetryHandle {
    fn position(&self) -> PyTelemetryPositionNamespace {
        PyTelemetryPositionNamespace::new(self.inner.clone())
    }

    fn attitude(&self) -> PyTelemetryAttitudeNamespace {
        PyTelemetryAttitudeNamespace::new(self.inner.clone())
    }

    fn battery(&self) -> PyTelemetryBatteryNamespace {
        PyTelemetryBatteryNamespace::new(self.inner.clone())
    }

    fn gps(&self) -> PyTelemetryGpsNamespace {
        PyTelemetryGpsNamespace::new(self.inner.clone())
    }

    fn navigation(&self) -> PyTelemetryNavigationNamespace {
        PyTelemetryNavigationNamespace::new(self.inner.clone())
    }

    fn terrain(&self) -> PyTelemetryTerrainNamespace {
        PyTelemetryTerrainNamespace::new(self.inner.clone())
    }

    fn rc(&self) -> PyTelemetryRcNamespace {
        PyTelemetryRcNamespace::new(self.inner.clone())
    }

    fn actuators(&self) -> PyTelemetryActuatorsNamespace {
        PyTelemetryActuatorsNamespace::new(self.inner.clone())
    }

    fn messages(&self) -> PyTelemetryMessagesHandle {
        PyTelemetryMessagesHandle::new(self.inner.clone())
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

#[pyclass(name = "TelemetryPositionNamespace", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyTelemetryPositionNamespace {
    inner: mavkit::Vehicle,
}

impl PyTelemetryPositionNamespace {
    fn new(inner: mavkit::Vehicle) -> Self {
        Self { inner }
    }
}

#[pyclass(name = "TelemetryAttitudeNamespace", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyTelemetryAttitudeNamespace {
    inner: mavkit::Vehicle,
}

impl PyTelemetryAttitudeNamespace {
    fn new(inner: mavkit::Vehicle) -> Self {
        Self { inner }
    }
}

#[pyclass(name = "TelemetryBatteryNamespace", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyTelemetryBatteryNamespace {
    inner: mavkit::Vehicle,
}

impl PyTelemetryBatteryNamespace {
    fn new(inner: mavkit::Vehicle) -> Self {
        Self { inner }
    }
}

#[pyclass(name = "TelemetryGpsNamespace", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyTelemetryGpsNamespace {
    inner: mavkit::Vehicle,
}

impl PyTelemetryGpsNamespace {
    fn new(inner: mavkit::Vehicle) -> Self {
        Self { inner }
    }
}

#[pyclass(name = "TelemetryNavigationNamespace", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyTelemetryNavigationNamespace {
    inner: mavkit::Vehicle,
}

impl PyTelemetryNavigationNamespace {
    fn new(inner: mavkit::Vehicle) -> Self {
        Self { inner }
    }
}

#[pyclass(name = "TelemetryTerrainNamespace", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyTelemetryTerrainNamespace {
    inner: mavkit::Vehicle,
}

impl PyTelemetryTerrainNamespace {
    fn new(inner: mavkit::Vehicle) -> Self {
        Self { inner }
    }
}

#[pyclass(name = "TelemetryRcNamespace", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyTelemetryRcNamespace {
    inner: mavkit::Vehicle,
}

impl PyTelemetryRcNamespace {
    fn new(inner: mavkit::Vehicle) -> Self {
        Self { inner }
    }
}

#[pyclass(name = "TelemetryActuatorsNamespace", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyTelemetryActuatorsNamespace {
    inner: mavkit::Vehicle,
}

impl PyTelemetryActuatorsNamespace {
    fn new(inner: mavkit::Vehicle) -> Self {
        Self { inner }
    }
}

#[pyclass(name = "TelemetryMessagesHandle", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyTelemetryMessagesHandle {
    inner: mavkit::Vehicle,
}

impl PyTelemetryMessagesHandle {
    fn new(inner: mavkit::Vehicle) -> Self {
        Self { inner }
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
        PyPeriodicMessageHandle {
            inner: PeriodicMessageHandleKind::VfrHud(self.inner.telemetry().messages().vfr_hud()),
        }
    }

    fn global_position_int(&self) -> PyPeriodicMessageHandle {
        PyPeriodicMessageHandle {
            inner: PeriodicMessageHandleKind::GlobalPositionInt(
                self.inner.telemetry().messages().global_position_int(),
            ),
        }
    }

    fn local_position_ned(&self) -> PyPeriodicMessageHandle {
        PyPeriodicMessageHandle {
            inner: PeriodicMessageHandleKind::LocalPositionNed(
                self.inner.telemetry().messages().local_position_ned(),
            ),
        }
    }

    fn gps_raw_int(&self) -> PyPeriodicMessageHandle {
        PyPeriodicMessageHandle {
            inner: PeriodicMessageHandleKind::GpsRawInt(
                self.inner.telemetry().messages().gps_raw_int(),
            ),
        }
    }

    fn attitude(&self) -> PyPeriodicMessageHandle {
        PyPeriodicMessageHandle {
            inner: PeriodicMessageHandleKind::Attitude(
                self.inner.telemetry().messages().attitude(),
            ),
        }
    }

    fn sys_status(&self) -> PyPeriodicMessageHandle {
        PyPeriodicMessageHandle {
            inner: PeriodicMessageHandleKind::SysStatus(
                self.inner.telemetry().messages().sys_status(),
            ),
        }
    }

    fn battery_status(&self, instance: u8) -> PyPeriodicMessageHandle {
        PyPeriodicMessageHandle {
            inner: PeriodicMessageHandleKind::BatteryStatus(
                self.inner.telemetry().messages().battery_status(instance),
            ),
        }
    }

    fn nav_controller_output(&self) -> PyPeriodicMessageHandle {
        PyPeriodicMessageHandle {
            inner: PeriodicMessageHandleKind::NavControllerOutput(
                self.inner.telemetry().messages().nav_controller_output(),
            ),
        }
    }

    fn terrain_report(&self) -> PyPeriodicMessageHandle {
        PyPeriodicMessageHandle {
            inner: PeriodicMessageHandleKind::TerrainReport(
                self.inner.telemetry().messages().terrain_report(),
            ),
        }
    }

    fn rc_channels(&self) -> PyPeriodicMessageHandle {
        PyPeriodicMessageHandle {
            inner: PeriodicMessageHandleKind::RcChannels(
                self.inner.telemetry().messages().rc_channels(),
            ),
        }
    }

    fn servo_output_raw(&self, port: u8) -> PyPeriodicMessageHandle {
        PyPeriodicMessageHandle {
            inner: PeriodicMessageHandleKind::ServoOutputRaw(
                self.inner.telemetry().messages().servo_output_raw(port),
            ),
        }
    }

    fn home_position(&self) -> PyEventMessageHandle {
        PyEventMessageHandle {
            inner: EventMessageHandleKind::HomePosition(
                self.inner.telemetry().messages().home_position(),
            ),
        }
    }

    fn gps_global_origin(&self) -> PyEventMessageHandle {
        PyEventMessageHandle {
            inner: EventMessageHandleKind::GpsGlobalOrigin(
                self.inner.telemetry().messages().gps_global_origin(),
            ),
        }
    }

    fn status_text(&self) -> PyMessageHandle {
        PyMessageHandle {
            inner: PushMessageHandleKind::StatusText(
                self.inner.telemetry().messages().status_text(),
            ),
        }
    }
}
