use pyo3::prelude::*;

use crate::enums::*;

// --- VehicleState ---

#[pyclass(name = "VehicleState", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyVehicleState {
    pub(crate) inner: mavkit::VehicleState,
}

#[pymethods]
impl PyVehicleState {
    #[getter]
    fn armed(&self) -> bool {
        self.inner.armed
    }

    #[getter]
    fn custom_mode(&self) -> u32 {
        self.inner.custom_mode
    }

    #[getter]
    fn mode_name(&self) -> &str {
        &self.inner.mode_name
    }

    #[getter]
    fn system_status(&self) -> PySystemStatus {
        self.inner.system_status.into()
    }

    #[getter]
    fn vehicle_type(&self) -> PyVehicleType {
        self.inner.vehicle_type.into()
    }

    #[getter]
    fn autopilot(&self) -> PyAutopilotType {
        self.inner.autopilot.into()
    }

    #[getter]
    fn system_id(&self) -> u8 {
        self.inner.system_id
    }

    #[getter]
    fn component_id(&self) -> u8 {
        self.inner.component_id
    }

    #[getter]
    fn heartbeat_received(&self) -> bool {
        self.inner.heartbeat_received
    }

    fn __repr__(&self) -> String {
        format!(
            "VehicleState(mode='{}', armed={}, status={:?})",
            self.inner.mode_name, self.inner.armed, self.inner.system_status
        )
    }
}

// --- Telemetry ---

#[pyclass(name = "Telemetry", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyTelemetry {
    pub(crate) inner: mavkit::Telemetry,
}

#[pymethods]
impl PyTelemetry {
    #[getter]
    fn altitude_m(&self) -> Option<f64> {
        self.inner.altitude_m
    }
    #[getter]
    fn speed_mps(&self) -> Option<f64> {
        self.inner.speed_mps
    }
    #[getter]
    fn heading_deg(&self) -> Option<f64> {
        self.inner.heading_deg
    }
    #[getter]
    fn latitude_deg(&self) -> Option<f64> {
        self.inner.latitude_deg
    }
    #[getter]
    fn longitude_deg(&self) -> Option<f64> {
        self.inner.longitude_deg
    }
    #[getter]
    fn battery_pct(&self) -> Option<f64> {
        self.inner.battery_pct
    }
    #[getter]
    fn gps_fix_type(&self) -> Option<PyGpsFixType> {
        self.inner.gps_fix_type.map(Into::into)
    }
    #[getter]
    fn climb_rate_mps(&self) -> Option<f64> {
        self.inner.climb_rate_mps
    }
    #[getter]
    fn throttle_pct(&self) -> Option<f64> {
        self.inner.throttle_pct
    }
    #[getter]
    fn airspeed_mps(&self) -> Option<f64> {
        self.inner.airspeed_mps
    }
    #[getter]
    fn battery_voltage_v(&self) -> Option<f64> {
        self.inner.battery_voltage_v
    }
    #[getter]
    fn battery_current_a(&self) -> Option<f64> {
        self.inner.battery_current_a
    }
    #[getter]
    fn gps_satellites(&self) -> Option<u8> {
        self.inner.gps_satellites
    }
    #[getter]
    fn gps_hdop(&self) -> Option<f64> {
        self.inner.gps_hdop
    }
    #[getter]
    fn roll_deg(&self) -> Option<f64> {
        self.inner.roll_deg
    }
    #[getter]
    fn pitch_deg(&self) -> Option<f64> {
        self.inner.pitch_deg
    }
    #[getter]
    fn yaw_deg(&self) -> Option<f64> {
        self.inner.yaw_deg
    }
    #[getter]
    fn wp_dist_m(&self) -> Option<f64> {
        self.inner.wp_dist_m
    }
    #[getter]
    fn nav_bearing_deg(&self) -> Option<f64> {
        self.inner.nav_bearing_deg
    }
    #[getter]
    fn target_bearing_deg(&self) -> Option<f64> {
        self.inner.target_bearing_deg
    }
    #[getter]
    fn xtrack_error_m(&self) -> Option<f64> {
        self.inner.xtrack_error_m
    }
    #[getter]
    fn terrain_height_m(&self) -> Option<f64> {
        self.inner.terrain_height_m
    }
    #[getter]
    fn height_above_terrain_m(&self) -> Option<f64> {
        self.inner.height_above_terrain_m
    }
    #[getter]
    fn battery_voltage_cells(&self) -> Option<Vec<f64>> {
        self.inner.battery_voltage_cells.clone()
    }
    #[getter]
    fn energy_consumed_wh(&self) -> Option<f64> {
        self.inner.energy_consumed_wh
    }
    #[getter]
    fn battery_time_remaining_s(&self) -> Option<i32> {
        self.inner.battery_time_remaining_s
    }
    #[getter]
    fn rc_channels(&self) -> Option<Vec<u16>> {
        self.inner.rc_channels.clone()
    }
    #[getter]
    fn rc_rssi(&self) -> Option<u8> {
        self.inner.rc_rssi
    }
    #[getter]
    fn servo_outputs(&self) -> Option<Vec<u16>> {
        self.inner.servo_outputs.clone()
    }

    fn __repr__(&self) -> String {
        let alt = self
            .inner
            .altitude_m
            .map_or("None".into(), |v| format!("{v:.1}"));
        let spd = self
            .inner
            .speed_mps
            .map_or("None".into(), |v| format!("{v:.1}"));
        let bat = self
            .inner
            .battery_pct
            .map_or("None".into(), |v| format!("{v:.0}%"));
        format!("Telemetry(alt={alt}m, speed={spd}m/s, battery={bat})")
    }
}

// --- Position ---

#[pyclass(name = "Position", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyPosition {
    pub(crate) inner: mavkit::Position,
}

#[pymethods]
impl PyPosition {
    #[getter]
    fn latitude_deg(&self) -> Option<f64> {
        self.inner.latitude_deg
    }
    #[getter]
    fn longitude_deg(&self) -> Option<f64> {
        self.inner.longitude_deg
    }
    #[getter]
    fn altitude_m(&self) -> Option<f64> {
        self.inner.altitude_m
    }
    #[getter]
    fn speed_mps(&self) -> Option<f64> {
        self.inner.speed_mps
    }
    #[getter]
    fn airspeed_mps(&self) -> Option<f64> {
        self.inner.airspeed_mps
    }
    #[getter]
    fn climb_rate_mps(&self) -> Option<f64> {
        self.inner.climb_rate_mps
    }
    #[getter]
    fn heading_deg(&self) -> Option<f64> {
        self.inner.heading_deg
    }
    #[getter]
    fn throttle_pct(&self) -> Option<f64> {
        self.inner.throttle_pct
    }
    fn __repr__(&self) -> String {
        format!(
            "Position(lat={:?}, lon={:?}, alt={:?}m)",
            self.inner.latitude_deg, self.inner.longitude_deg, self.inner.altitude_m
        )
    }
}

// --- Attitude ---

#[pyclass(name = "Attitude", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyAttitude {
    pub(crate) inner: mavkit::Attitude,
}

#[pymethods]
impl PyAttitude {
    #[getter]
    fn roll_deg(&self) -> Option<f64> {
        self.inner.roll_deg
    }
    #[getter]
    fn pitch_deg(&self) -> Option<f64> {
        self.inner.pitch_deg
    }
    #[getter]
    fn yaw_deg(&self) -> Option<f64> {
        self.inner.yaw_deg
    }
    fn __repr__(&self) -> String {
        format!(
            "Attitude(roll={:?}, pitch={:?}, yaw={:?})",
            self.inner.roll_deg, self.inner.pitch_deg, self.inner.yaw_deg
        )
    }
}

// --- Battery ---

#[pyclass(name = "Battery", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyBattery {
    pub(crate) inner: mavkit::Battery,
}

#[pymethods]
impl PyBattery {
    #[getter]
    fn remaining_pct(&self) -> Option<f64> {
        self.inner.remaining_pct
    }
    #[getter]
    fn voltage_v(&self) -> Option<f64> {
        self.inner.voltage_v
    }
    #[getter]
    fn current_a(&self) -> Option<f64> {
        self.inner.current_a
    }
    #[getter]
    fn voltage_cells(&self) -> Option<Vec<f64>> {
        self.inner.voltage_cells.clone()
    }
    #[getter]
    fn energy_consumed_wh(&self) -> Option<f64> {
        self.inner.energy_consumed_wh
    }
    #[getter]
    fn time_remaining_s(&self) -> Option<i32> {
        self.inner.time_remaining_s
    }
    fn __repr__(&self) -> String {
        let pct = self
            .inner
            .remaining_pct
            .map_or("None".into(), |v| format!("{v:.0}%"));
        let volt = self
            .inner
            .voltage_v
            .map_or("None".into(), |v| format!("{v:.2}V"));
        format!("Battery(remaining={pct}, voltage={volt})")
    }
}

// --- Gps ---

#[pyclass(name = "Gps", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyGps {
    pub(crate) inner: mavkit::Gps,
}

#[pymethods]
impl PyGps {
    #[getter]
    fn fix_type(&self) -> Option<PyGpsFixType> {
        self.inner.fix_type.map(Into::into)
    }
    #[getter]
    fn satellites(&self) -> Option<u8> {
        self.inner.satellites
    }
    #[getter]
    fn hdop(&self) -> Option<f64> {
        self.inner.hdop
    }
    fn __repr__(&self) -> String {
        format!(
            "Gps(fix={:?}, sats={:?}, hdop={:?})",
            self.inner.fix_type, self.inner.satellites, self.inner.hdop
        )
    }
}

// --- Navigation ---

#[pyclass(name = "Navigation", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyNavigation {
    pub(crate) inner: mavkit::Navigation,
}

#[pymethods]
impl PyNavigation {
    #[getter]
    fn wp_dist_m(&self) -> Option<f64> {
        self.inner.wp_dist_m
    }
    #[getter]
    fn nav_bearing_deg(&self) -> Option<f64> {
        self.inner.nav_bearing_deg
    }
    #[getter]
    fn target_bearing_deg(&self) -> Option<f64> {
        self.inner.target_bearing_deg
    }
    #[getter]
    fn xtrack_error_m(&self) -> Option<f64> {
        self.inner.xtrack_error_m
    }
    fn __repr__(&self) -> String {
        format!(
            "Navigation(wp_dist={:?}m, bearing={:?})",
            self.inner.wp_dist_m, self.inner.nav_bearing_deg
        )
    }
}

// --- Terrain ---

#[pyclass(name = "Terrain", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyTerrain {
    pub(crate) inner: mavkit::Terrain,
}

#[pymethods]
impl PyTerrain {
    #[getter]
    fn terrain_height_m(&self) -> Option<f64> {
        self.inner.terrain_height_m
    }
    #[getter]
    fn height_above_terrain_m(&self) -> Option<f64> {
        self.inner.height_above_terrain_m
    }
    fn __repr__(&self) -> String {
        format!(
            "Terrain(height={:?}m, clearance={:?}m)",
            self.inner.terrain_height_m, self.inner.height_above_terrain_m
        )
    }
}

// --- RcChannels ---

#[pyclass(name = "RcChannels", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyRcChannels {
    pub(crate) inner: mavkit::RcChannels,
}

#[pymethods]
impl PyRcChannels {
    #[getter]
    fn channels(&self) -> Option<Vec<u16>> {
        self.inner.channels.clone()
    }
    #[getter]
    fn rssi(&self) -> Option<u8> {
        self.inner.rssi
    }
    #[getter]
    fn servo_outputs(&self) -> Option<Vec<u16>> {
        self.inner.servo_outputs.clone()
    }
    fn __repr__(&self) -> String {
        let ch_count = self.inner.channels.as_ref().map_or(0, |c| c.len());
        format!(
            "RcChannels(channels={ch_count}, rssi={:?})",
            self.inner.rssi
        )
    }
}

// --- MissionState ---

#[pyclass(name = "MissionState", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyMissionState {
    pub(crate) inner: mavkit::MissionState,
}

#[pymethods]
impl PyMissionState {
    #[getter]
    fn current_seq(&self) -> u16 {
        self.inner.current_seq
    }

    #[getter]
    fn total_items(&self) -> u16 {
        self.inner.total_items
    }

    fn __repr__(&self) -> String {
        format!(
            "MissionState(seq={}, total={})",
            self.inner.current_seq, self.inner.total_items
        )
    }
}

// --- LinkState (enum with data → struct wrapper) ---

#[pyclass(name = "LinkState", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyLinkState {
    #[pyo3(get)]
    kind: String,
    #[pyo3(get)]
    message: Option<String>,
}

impl From<mavkit::LinkState> for PyLinkState {
    fn from(v: mavkit::LinkState) -> Self {
        match v {
            mavkit::LinkState::Connecting => PyLinkState {
                kind: "connecting".into(),
                message: None,
            },
            mavkit::LinkState::Connected => PyLinkState {
                kind: "connected".into(),
                message: None,
            },
            mavkit::LinkState::Disconnected => PyLinkState {
                kind: "disconnected".into(),
                message: None,
            },
            mavkit::LinkState::Error(msg) => PyLinkState {
                kind: "error".into(),
                message: Some(msg),
            },
        }
    }
}

#[pymethods]
impl PyLinkState {
    fn __repr__(&self) -> String {
        match &self.message {
            Some(msg) => format!("LinkState(kind='{}', message='{}')", self.kind, msg),
            None => format!("LinkState(kind='{}')", self.kind),
        }
    }
}

// --- VehicleIdentity ---

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

// --- FlightMode ---

#[pyclass(name = "FlightMode", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyFlightMode {
    pub(crate) inner: mavkit::FlightMode,
}

#[pymethods]
impl PyFlightMode {
    #[getter]
    fn custom_mode(&self) -> u32 {
        self.inner.custom_mode
    }

    #[getter]
    fn name(&self) -> &str {
        &self.inner.name
    }

    fn __repr__(&self) -> String {
        format!(
            "FlightMode(name='{}', id={})",
            self.inner.name, self.inner.custom_mode
        )
    }
}

// --- StatusMessage ---

#[pyclass(name = "StatusMessage", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyStatusMessage {
    pub(crate) inner: mavkit::StatusMessage,
}

#[pymethods]
impl PyStatusMessage {
    #[getter]
    fn text(&self) -> &str {
        &self.inner.text
    }

    #[getter]
    fn severity(&self) -> crate::enums::PyMavSeverity {
        self.inner.severity.into()
    }

    fn __repr__(&self) -> String {
        format!(
            "StatusMessage(severity={:?}, text='{}')",
            self.inner.severity, self.inner.text
        )
    }
}
