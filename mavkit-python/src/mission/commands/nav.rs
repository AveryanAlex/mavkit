use pyo3::prelude::*;

use crate::enums::*;
use crate::geo::{PyGeoPoint3d, position_components};
use crate::macros::py_frozen_wrapper;

use super::common::{
    alt_change_action_from_name, alt_change_action_name, loiter_direction_from_name,
    loiter_direction_name, position_from_components, py_position_wrapper,
};

py_position_wrapper!(
    PyNavWaypoint,
    "NavWaypoint",
    mavkit::mission::commands::NavWaypoint,
    new_signature = (
        *,
        latitude_deg,
        longitude_deg,
        altitude_m,
        frame=PyMissionFrame::GlobalRelativeAltInt,
        hold_time_s=0.0,
        acceptance_radius_m=0.0,
        pass_radius_m=0.0,
        yaw_deg=0.0
    ),
    new_attrs = { #[allow(clippy::too_many_arguments)] },
    fields = {
        hold_time_s: f32,
        acceptance_radius_m: f32,
        pass_radius_m: f32,
        yaw_deg: f32,
    },
    from_point_signature = (
        *,
        position,
        hold_time_s=0.0,
        acceptance_radius_m=0.0,
        pass_radius_m=0.0,
        yaw_deg=0.0
    )
);

py_position_wrapper!(
    PyNavTakeoff,
    "NavTakeoff",
    mavkit::mission::commands::NavTakeoff,
    new_signature = (
        *,
        latitude_deg,
        longitude_deg,
        altitude_m,
        frame=PyMissionFrame::GlobalRelativeAltInt,
        pitch_deg=0.0
    ),
    new_attrs = {},
    fields = { pitch_deg: f32 },
    from_point_signature = (*, position, pitch_deg=0.0)
);

py_position_wrapper!(
    PyNavLand,
    "NavLand",
    mavkit::mission::commands::NavLand,
    new_signature = (
        *,
        latitude_deg,
        longitude_deg,
        altitude_m,
        frame=PyMissionFrame::GlobalRelativeAltInt,
        abort_alt_m=0.0
    ),
    new_attrs = {},
    fields = { abort_alt_m: f32 },
    from_point_signature = (*, position, abort_alt_m=0.0)
);

#[pyclass(name = "NavLoiterTime", frozen, from_py_object)]
#[derive(Clone)]
pub struct PyNavLoiterTime {
    pub(crate) inner: mavkit::mission::commands::NavLoiterTime,
}

#[pymethods]
impl PyNavLoiterTime {
    #[new]
    #[pyo3(signature = (*, latitude_deg, longitude_deg, altitude_m, frame=PyMissionFrame::GlobalRelativeAltInt, time_s=0.0, direction="clockwise", exit_xtrack=false))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        latitude_deg: f64,
        longitude_deg: f64,
        altitude_m: f32,
        frame: PyMissionFrame,
        time_s: f32,
        direction: &str,
        exit_xtrack: bool,
    ) -> PyResult<Self> {
        Ok(Self {
            inner: mavkit::mission::commands::NavLoiterTime {
                position: position_from_components(frame, latitude_deg, longitude_deg, altitude_m),
                time_s,
                direction: loiter_direction_from_name(direction)?,
                exit_xtrack,
            },
        })
    }

    #[staticmethod]
    #[pyo3(signature = (*, position, time_s=0.0, direction="clockwise", exit_xtrack=false))]
    fn from_point(
        position: &PyGeoPoint3d,
        time_s: f32,
        direction: &str,
        exit_xtrack: bool,
    ) -> PyResult<Self> {
        Ok(Self {
            inner: mavkit::mission::commands::NavLoiterTime {
                position: position.inner.clone(),
                time_s,
                direction: loiter_direction_from_name(direction)?,
                exit_xtrack,
            },
        })
    }

    #[getter]
    fn frame(&self) -> PyMissionFrame {
        position_components(&self.inner.position).0
    }

    #[getter]
    fn latitude_deg(&self) -> f64 {
        position_components(&self.inner.position).1
    }

    #[getter]
    fn longitude_deg(&self) -> f64 {
        position_components(&self.inner.position).2
    }

    #[getter]
    fn altitude_m(&self) -> f32 {
        position_components(&self.inner.position).3
    }

    #[getter]
    fn time_s(&self) -> f32 {
        self.inner.time_s
    }

    #[getter]
    fn direction(&self) -> &'static str {
        loiter_direction_name(self.inner.direction)
    }

    #[getter]
    fn exit_xtrack(&self) -> bool {
        self.inner.exit_xtrack
    }
}

py_frozen_wrapper!(PyNavGuidedEnable wraps mavkit::mission::commands::NavGuidedEnable as "NavGuidedEnable" {
    enabled: bool,
});

#[pyclass(name = "NavReturnToLaunch", frozen, from_py_object)]
#[derive(Clone, Default)]
pub struct PyNavReturnToLaunch;

#[pymethods]
impl PyNavReturnToLaunch {
    #[new]
    fn new() -> Self {
        Self
    }
}

py_position_wrapper!(
    PyNavSplineWaypoint,
    "NavSplineWaypoint",
    mavkit::mission::commands::NavSplineWaypoint,
    new_signature = (
        *,
        latitude_deg,
        longitude_deg,
        altitude_m,
        frame=PyMissionFrame::GlobalRelativeAltInt,
        hold_time_s=0.0
    ),
    new_attrs = {},
    fields = { hold_time_s: f32 },
    from_point_signature = (*, position, hold_time_s=0.0)
);

#[pyclass(name = "NavArcWaypoint", frozen, from_py_object)]
#[derive(Clone)]
pub struct PyNavArcWaypoint {
    pub(crate) inner: mavkit::mission::commands::NavArcWaypoint,
}

#[pymethods]
impl PyNavArcWaypoint {
    #[new]
    #[pyo3(signature = (*, latitude_deg, longitude_deg, altitude_m, frame=PyMissionFrame::GlobalRelativeAltInt, arc_angle_deg, direction="clockwise"))]
    fn new(
        latitude_deg: f64,
        longitude_deg: f64,
        altitude_m: f32,
        frame: PyMissionFrame,
        arc_angle_deg: f32,
        direction: &str,
    ) -> PyResult<Self> {
        Ok(Self {
            inner: mavkit::mission::commands::NavArcWaypoint {
                position: position_from_components(frame, latitude_deg, longitude_deg, altitude_m),
                arc_angle_deg,
                direction: loiter_direction_from_name(direction)?,
            },
        })
    }

    #[staticmethod]
    #[pyo3(signature = (*, position, arc_angle_deg, direction="clockwise"))]
    fn from_point(position: &PyGeoPoint3d, arc_angle_deg: f32, direction: &str) -> PyResult<Self> {
        Ok(Self {
            inner: mavkit::mission::commands::NavArcWaypoint {
                position: position.inner.clone(),
                arc_angle_deg,
                direction: loiter_direction_from_name(direction)?,
            },
        })
    }

    #[getter]
    fn frame(&self) -> PyMissionFrame {
        position_components(&self.inner.position).0
    }

    #[getter]
    fn latitude_deg(&self) -> f64 {
        position_components(&self.inner.position).1
    }

    #[getter]
    fn longitude_deg(&self) -> f64 {
        position_components(&self.inner.position).2
    }

    #[getter]
    fn altitude_m(&self) -> f32 {
        position_components(&self.inner.position).3
    }

    #[getter]
    fn arc_angle_deg(&self) -> f32 {
        self.inner.arc_angle_deg
    }

    #[getter]
    fn direction(&self) -> &'static str {
        loiter_direction_name(self.inner.direction)
    }
}

#[pyclass(name = "NavLoiterUnlimited", frozen, from_py_object)]
#[derive(Clone)]
pub struct PyNavLoiterUnlimited {
    pub(crate) inner: mavkit::mission::commands::NavLoiterUnlimited,
}

#[pymethods]
impl PyNavLoiterUnlimited {
    #[new]
    #[pyo3(signature = (*, latitude_deg, longitude_deg, altitude_m, frame=PyMissionFrame::GlobalRelativeAltInt, radius_m=0.0, direction="clockwise"))]
    fn new(
        latitude_deg: f64,
        longitude_deg: f64,
        altitude_m: f32,
        frame: PyMissionFrame,
        radius_m: f32,
        direction: &str,
    ) -> PyResult<Self> {
        Ok(Self {
            inner: mavkit::mission::commands::NavLoiterUnlimited {
                position: position_from_components(frame, latitude_deg, longitude_deg, altitude_m),
                radius_m,
                direction: loiter_direction_from_name(direction)?,
            },
        })
    }

    #[staticmethod]
    #[pyo3(signature = (*, position, radius_m=0.0, direction="clockwise"))]
    fn from_point(position: &PyGeoPoint3d, radius_m: f32, direction: &str) -> PyResult<Self> {
        Ok(Self {
            inner: mavkit::mission::commands::NavLoiterUnlimited {
                position: position.inner.clone(),
                radius_m,
                direction: loiter_direction_from_name(direction)?,
            },
        })
    }

    #[getter]
    fn frame(&self) -> PyMissionFrame {
        position_components(&self.inner.position).0
    }

    #[getter]
    fn latitude_deg(&self) -> f64 {
        position_components(&self.inner.position).1
    }

    #[getter]
    fn longitude_deg(&self) -> f64 {
        position_components(&self.inner.position).2
    }

    #[getter]
    fn altitude_m(&self) -> f32 {
        position_components(&self.inner.position).3
    }

    #[getter]
    fn radius_m(&self) -> f32 {
        self.inner.radius_m
    }

    #[getter]
    fn direction(&self) -> &'static str {
        loiter_direction_name(self.inner.direction)
    }
}

#[pyclass(name = "NavLoiterTurns", frozen, from_py_object)]
#[derive(Clone)]
pub struct PyNavLoiterTurns {
    pub(crate) inner: mavkit::mission::commands::NavLoiterTurns,
}

#[pymethods]
impl PyNavLoiterTurns {
    #[new]
    #[pyo3(signature = (*, latitude_deg, longitude_deg, altitude_m, frame=PyMissionFrame::GlobalRelativeAltInt, turns, radius_m=0.0, direction="clockwise", exit_xtrack=false))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        latitude_deg: f64,
        longitude_deg: f64,
        altitude_m: f32,
        frame: PyMissionFrame,
        turns: f32,
        radius_m: f32,
        direction: &str,
        exit_xtrack: bool,
    ) -> PyResult<Self> {
        Ok(Self {
            inner: mavkit::mission::commands::NavLoiterTurns {
                position: position_from_components(frame, latitude_deg, longitude_deg, altitude_m),
                turns,
                radius_m,
                direction: loiter_direction_from_name(direction)?,
                exit_xtrack,
            },
        })
    }

    #[staticmethod]
    #[pyo3(signature = (*, position, turns, radius_m=0.0, direction="clockwise", exit_xtrack=false))]
    fn from_point(
        position: &PyGeoPoint3d,
        turns: f32,
        radius_m: f32,
        direction: &str,
        exit_xtrack: bool,
    ) -> PyResult<Self> {
        Ok(Self {
            inner: mavkit::mission::commands::NavLoiterTurns {
                position: position.inner.clone(),
                turns,
                radius_m,
                direction: loiter_direction_from_name(direction)?,
                exit_xtrack,
            },
        })
    }

    #[getter]
    fn frame(&self) -> PyMissionFrame {
        position_components(&self.inner.position).0
    }

    #[getter]
    fn latitude_deg(&self) -> f64 {
        position_components(&self.inner.position).1
    }

    #[getter]
    fn longitude_deg(&self) -> f64 {
        position_components(&self.inner.position).2
    }

    #[getter]
    fn altitude_m(&self) -> f32 {
        position_components(&self.inner.position).3
    }

    #[getter]
    fn turns(&self) -> f32 {
        self.inner.turns
    }

    #[getter]
    fn radius_m(&self) -> f32 {
        self.inner.radius_m
    }

    #[getter]
    fn direction(&self) -> &'static str {
        loiter_direction_name(self.inner.direction)
    }

    #[getter]
    fn exit_xtrack(&self) -> bool {
        self.inner.exit_xtrack
    }
}

#[pyclass(name = "NavLoiterToAlt", frozen, from_py_object)]
#[derive(Clone)]
pub struct PyNavLoiterToAlt {
    pub(crate) inner: mavkit::mission::commands::NavLoiterToAlt,
}

#[pymethods]
impl PyNavLoiterToAlt {
    #[new]
    #[pyo3(signature = (*, latitude_deg, longitude_deg, altitude_m, frame=PyMissionFrame::GlobalRelativeAltInt, radius_m=0.0, direction="clockwise", exit_xtrack=false))]
    fn new(
        latitude_deg: f64,
        longitude_deg: f64,
        altitude_m: f32,
        frame: PyMissionFrame,
        radius_m: f32,
        direction: &str,
        exit_xtrack: bool,
    ) -> PyResult<Self> {
        Ok(Self {
            inner: mavkit::mission::commands::NavLoiterToAlt {
                position: position_from_components(frame, latitude_deg, longitude_deg, altitude_m),
                radius_m,
                direction: loiter_direction_from_name(direction)?,
                exit_xtrack,
            },
        })
    }

    #[staticmethod]
    #[pyo3(signature = (*, position, radius_m=0.0, direction="clockwise", exit_xtrack=false))]
    fn from_point(
        position: &PyGeoPoint3d,
        radius_m: f32,
        direction: &str,
        exit_xtrack: bool,
    ) -> PyResult<Self> {
        Ok(Self {
            inner: mavkit::mission::commands::NavLoiterToAlt {
                position: position.inner.clone(),
                radius_m,
                direction: loiter_direction_from_name(direction)?,
                exit_xtrack,
            },
        })
    }

    #[getter]
    fn frame(&self) -> PyMissionFrame {
        position_components(&self.inner.position).0
    }

    #[getter]
    fn latitude_deg(&self) -> f64 {
        position_components(&self.inner.position).1
    }

    #[getter]
    fn longitude_deg(&self) -> f64 {
        position_components(&self.inner.position).2
    }

    #[getter]
    fn altitude_m(&self) -> f32 {
        position_components(&self.inner.position).3
    }

    #[getter]
    fn radius_m(&self) -> f32 {
        self.inner.radius_m
    }

    #[getter]
    fn direction(&self) -> &'static str {
        loiter_direction_name(self.inner.direction)
    }

    #[getter]
    fn exit_xtrack(&self) -> bool {
        self.inner.exit_xtrack
    }
}

#[pyclass(name = "NavContinueAndChangeAlt", frozen, from_py_object)]
#[derive(Clone)]
pub struct PyNavContinueAndChangeAlt {
    pub(crate) inner: mavkit::mission::commands::NavContinueAndChangeAlt,
}

#[pymethods]
impl PyNavContinueAndChangeAlt {
    #[new]
    #[pyo3(signature = (*, latitude_deg, longitude_deg, altitude_m, frame=PyMissionFrame::GlobalRelativeAltInt, action="neutral"))]
    fn new(
        latitude_deg: f64,
        longitude_deg: f64,
        altitude_m: f32,
        frame: PyMissionFrame,
        action: &str,
    ) -> PyResult<Self> {
        Ok(Self {
            inner: mavkit::mission::commands::NavContinueAndChangeAlt {
                position: position_from_components(frame, latitude_deg, longitude_deg, altitude_m),
                action: alt_change_action_from_name(action)?,
            },
        })
    }

    #[staticmethod]
    #[pyo3(signature = (*, position, action="neutral"))]
    fn from_point(position: &PyGeoPoint3d, action: &str) -> PyResult<Self> {
        Ok(Self {
            inner: mavkit::mission::commands::NavContinueAndChangeAlt {
                position: position.inner.clone(),
                action: alt_change_action_from_name(action)?,
            },
        })
    }

    #[getter]
    fn frame(&self) -> PyMissionFrame {
        position_components(&self.inner.position).0
    }

    #[getter]
    fn latitude_deg(&self) -> f64 {
        position_components(&self.inner.position).1
    }

    #[getter]
    fn longitude_deg(&self) -> f64 {
        position_components(&self.inner.position).2
    }

    #[getter]
    fn altitude_m(&self) -> f32 {
        position_components(&self.inner.position).3
    }

    #[getter]
    fn action(&self) -> &'static str {
        alt_change_action_name(self.inner.action)
    }
}

py_frozen_wrapper!(PyNavDelay wraps mavkit::mission::commands::NavDelay as "NavDelay" {
    seconds: f32,
    hour_utc: f32,
    min_utc: f32,
    sec_utc: f32,
});

py_frozen_wrapper!(PyNavAltitudeWait wraps mavkit::mission::commands::NavAltitudeWait as "NavAltitudeWait" {
    altitude_m: f32,
    descent_rate_mps: f32,
    wiggle_time_s: f32,
});

py_position_wrapper!(
    PyNavVtolTakeoff,
    "NavVtolTakeoff",
    mavkit::mission::commands::NavVtolTakeoff,
    new_signature = (
        *,
        latitude_deg,
        longitude_deg,
        altitude_m,
        frame=PyMissionFrame::GlobalRelativeAltInt
    ),
    new_attrs = {},
    fields = {},
    from_point_signature = (*, position)
);

py_position_wrapper!(
    PyNavVtolLand,
    "NavVtolLand",
    mavkit::mission::commands::NavVtolLand,
    new_signature = (
        *,
        latitude_deg,
        longitude_deg,
        altitude_m,
        frame=PyMissionFrame::GlobalRelativeAltInt,
        options=0
    ),
    new_attrs = {},
    fields = { options: u8 },
    from_point_signature = (*, position, options=0)
);

py_position_wrapper!(
    PyNavPayloadPlace,
    "NavPayloadPlace",
    mavkit::mission::commands::NavPayloadPlace,
    new_signature = (
        *,
        latitude_deg,
        longitude_deg,
        altitude_m,
        frame=PyMissionFrame::GlobalRelativeAltInt,
        max_descent_m=0.0
    ),
    new_attrs = {},
    fields = { max_descent_m: f32 },
    from_point_signature = (*, position, max_descent_m=0.0)
);

py_frozen_wrapper!(PyNavSetYawSpeed wraps mavkit::mission::commands::NavSetYawSpeed as "NavSetYawSpeed" {
    angle_deg: f32,
    speed_mps: f32,
    relative: bool,
});

py_frozen_wrapper!(PyNavScriptTime wraps mavkit::mission::commands::NavScriptTime as "NavScriptTime" {
    command: u16,
    timeout_s: f32,
    arg1: f32,
    arg2: f32,
    arg3: i16,
    arg4: i16,
});

py_frozen_wrapper!(PyNavAttitudeTime wraps mavkit::mission::commands::NavAttitudeTime as "NavAttitudeTime" {
    time_s: f32,
    roll_deg: f32,
    pitch_deg: f32,
    yaw_deg: f32,
    climb_rate_mps: f32,
});
