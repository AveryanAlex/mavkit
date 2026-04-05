use pyo3::prelude::*;

use super::messages::{PyEventMessageHandle, PyMessageHandle, PyPeriodicMessageHandle};
use super::metrics::PyMetricHandle;

#[pyclass(name = "TelemetryHandle", frozen, skip_from_py_object)]
#[derive(Clone)]
pub(crate) struct PyTelemetryHandle {
    pub(crate) inner: mavkit::Vehicle,
}

macro_rules! define_vehicle_shell_wrappers {
    ($($namespace:ident => $py_name:literal;)+) => {
        $(
            #[pyclass(name = $py_name, frozen, skip_from_py_object)]
            #[derive(Clone)]
            pub(crate) struct $namespace {
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
