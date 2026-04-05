use pyo3::prelude::*;

#[pyclass(name = "SensorHealthState", eq, frozen, from_py_object)]
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) enum PySensorHealthState {
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

#[pyclass(name = "SensorHealthSummary", frozen, skip_from_py_object)]
#[derive(Clone)]
pub(crate) struct PySensorHealthSummary {
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
