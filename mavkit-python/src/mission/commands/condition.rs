use pyo3::prelude::*;

use crate::macros::py_frozen_wrapper;

use super::common::{yaw_direction_from_name, yaw_direction_name};

py_frozen_wrapper!(PyCondDelay wraps mavkit::mission::commands::CondDelay as "CondDelay" {
    delay_s: f32,
});

py_frozen_wrapper!(PyCondDistance wraps mavkit::mission::commands::CondDistance as "CondDistance" {
    distance_m: f32,
});

#[pyclass(name = "CondYaw", frozen, from_py_object)]
#[derive(Clone)]
pub struct PyCondYaw {
    pub(crate) inner: mavkit::mission::commands::CondYaw,
}

#[pymethods]
impl PyCondYaw {
    #[new]
    #[pyo3(signature = (*, angle_deg, turn_rate_dps=0.0, direction="clockwise", relative=false))]
    fn new(angle_deg: f32, turn_rate_dps: f32, direction: &str, relative: bool) -> PyResult<Self> {
        Ok(Self {
            inner: mavkit::mission::commands::CondYaw {
                angle_deg,
                turn_rate_dps,
                direction: yaw_direction_from_name(direction)?,
                relative,
            },
        })
    }

    #[getter]
    fn angle_deg(&self) -> f32 {
        self.inner.angle_deg
    }

    #[getter]
    fn turn_rate_dps(&self) -> f32 {
        self.inner.turn_rate_dps
    }

    #[getter]
    fn direction(&self) -> &'static str {
        yaw_direction_name(self.inner.direction)
    }

    #[getter]
    fn relative(&self) -> bool {
        self.inner.relative
    }
}
