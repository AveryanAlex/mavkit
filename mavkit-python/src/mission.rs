use pyo3::prelude::*;
use pyo3::types::PyAny;

use crate::enums::*;
use crate::geo::{PyGeoPoint3d, position_components};
use crate::macros::py_frozen_wrapper;

fn command_frame_from_py(frame: PyMissionFrame) -> mavkit::mission::commands::MissionFrame {
    command_frame_from_py_raw(frame, None)
}

fn command_frame_from_py_raw(
    frame: PyMissionFrame,
    frame_raw: Option<u8>,
) -> mavkit::mission::commands::MissionFrame {
    match frame {
        PyMissionFrame::Mission => mavkit::mission::commands::MissionFrame::Mission,
        PyMissionFrame::GlobalInt => mavkit::mission::commands::MissionFrame::Global,
        PyMissionFrame::GlobalRelativeAltInt => {
            mavkit::mission::commands::MissionFrame::GlobalRelativeAlt
        }
        PyMissionFrame::GlobalTerrainAltInt => {
            mavkit::mission::commands::MissionFrame::GlobalTerrainAlt
        }
        PyMissionFrame::LocalNed => mavkit::mission::commands::MissionFrame::Other(1),
        PyMissionFrame::Other => {
            mavkit::mission::commands::MissionFrame::Other(frame_raw.unwrap_or(0))
        }
    }
}

fn py_frame_from_command(frame: mavkit::mission::commands::MissionFrame) -> PyMissionFrame {
    match frame {
        mavkit::mission::commands::MissionFrame::Mission => PyMissionFrame::Mission,
        mavkit::mission::commands::MissionFrame::Global => PyMissionFrame::GlobalInt,
        mavkit::mission::commands::MissionFrame::GlobalRelativeAlt => {
            PyMissionFrame::GlobalRelativeAltInt
        }
        mavkit::mission::commands::MissionFrame::GlobalTerrainAlt => {
            PyMissionFrame::GlobalTerrainAltInt
        }
        mavkit::mission::commands::MissionFrame::Other(1) => PyMissionFrame::LocalNed,
        mavkit::mission::commands::MissionFrame::Other(_) => PyMissionFrame::Other,
    }
}

/// Extract the raw MAVLink frame ID from a command frame.
fn raw_frame_id(frame: mavkit::mission::commands::MissionFrame) -> u8 {
    match frame {
        mavkit::mission::commands::MissionFrame::Global => 0,
        mavkit::mission::commands::MissionFrame::Mission => 2,
        mavkit::mission::commands::MissionFrame::GlobalRelativeAlt => 3,
        mavkit::mission::commands::MissionFrame::GlobalTerrainAlt => 10,
        mavkit::mission::commands::MissionFrame::Other(v) => v,
    }
}

fn wire_parts(item: &mavkit::MissionItem) -> (u16, PyMissionFrame, [f32; 4], i32, i32, f32) {
    let (command, frame, params, x, y, z) = item.command.clone().into_wire();
    (command, py_frame_from_command(frame), params, x, y, z)
}

fn position_from_components(
    frame: PyMissionFrame,
    latitude_deg: f64,
    longitude_deg: f64,
    altitude_m: f32,
) -> mavkit::GeoPoint3d {
    match frame {
        PyMissionFrame::GlobalInt => mavkit::GeoPoint3d::Msl(mavkit::GeoPoint3dMsl {
            latitude_deg,
            longitude_deg,
            altitude_msl_m: f64::from(altitude_m),
        }),
        PyMissionFrame::GlobalTerrainAltInt => {
            mavkit::GeoPoint3d::Terrain(mavkit::GeoPoint3dTerrain {
                latitude_deg,
                longitude_deg,
                altitude_terrain_m: f64::from(altitude_m),
            })
        }
        PyMissionFrame::GlobalRelativeAltInt
        | PyMissionFrame::Mission
        | PyMissionFrame::LocalNed
        | PyMissionFrame::Other => mavkit::GeoPoint3d::RelHome(mavkit::GeoPoint3dRelHome {
            latitude_deg,
            longitude_deg,
            relative_alt_m: f64::from(altitude_m),
        }),
    }
}

fn speed_type_from_name(speed_type: &str) -> PyResult<mavkit::mission::commands::SpeedType> {
    if speed_type.eq_ignore_ascii_case("airspeed") {
        Ok(mavkit::mission::commands::SpeedType::Airspeed)
    } else if speed_type.eq_ignore_ascii_case("groundspeed") {
        Ok(mavkit::mission::commands::SpeedType::Groundspeed)
    } else {
        Err(pyo3::exceptions::PyValueError::new_err(
            "speed_type must be 'airspeed' or 'groundspeed'",
        ))
    }
}

fn speed_type_name(speed_type: mavkit::mission::commands::SpeedType) -> &'static str {
    match speed_type {
        mavkit::mission::commands::SpeedType::Airspeed => "airspeed",
        mavkit::mission::commands::SpeedType::Groundspeed => "groundspeed",
    }
}

fn yaw_direction_from_name(direction: &str) -> PyResult<mavkit::mission::commands::YawDirection> {
    if direction.eq_ignore_ascii_case("clockwise") {
        Ok(mavkit::mission::commands::YawDirection::Clockwise)
    } else if direction.eq_ignore_ascii_case("counter_clockwise")
        || direction.eq_ignore_ascii_case("counterclockwise")
    {
        Ok(mavkit::mission::commands::YawDirection::CounterClockwise)
    } else {
        Err(pyo3::exceptions::PyValueError::new_err(
            "direction must be 'clockwise' or 'counter_clockwise'",
        ))
    }
}

fn yaw_direction_name(direction: mavkit::mission::commands::YawDirection) -> &'static str {
    match direction {
        mavkit::mission::commands::YawDirection::Clockwise => "clockwise",
        mavkit::mission::commands::YawDirection::CounterClockwise => "counter_clockwise",
    }
}

fn loiter_direction_from_name(
    direction: &str,
) -> PyResult<mavkit::mission::commands::LoiterDirection> {
    if direction.eq_ignore_ascii_case("clockwise") {
        Ok(mavkit::mission::commands::LoiterDirection::Clockwise)
    } else if direction.eq_ignore_ascii_case("counter_clockwise")
        || direction.eq_ignore_ascii_case("counterclockwise")
    {
        Ok(mavkit::mission::commands::LoiterDirection::CounterClockwise)
    } else {
        Err(pyo3::exceptions::PyValueError::new_err(
            "direction must be 'clockwise' or 'counter_clockwise'",
        ))
    }
}

fn loiter_direction_name(direction: mavkit::mission::commands::LoiterDirection) -> &'static str {
    match direction {
        mavkit::mission::commands::LoiterDirection::Clockwise => "clockwise",
        mavkit::mission::commands::LoiterDirection::CounterClockwise => "counter_clockwise",
    }
}

fn alt_change_action_from_name(
    action: &str,
) -> PyResult<mavkit::mission::commands::AltChangeAction> {
    if action.eq_ignore_ascii_case("neutral") {
        Ok(mavkit::mission::commands::AltChangeAction::Neutral)
    } else if action.eq_ignore_ascii_case("climb") {
        Ok(mavkit::mission::commands::AltChangeAction::Climb)
    } else if action.eq_ignore_ascii_case("descend") {
        Ok(mavkit::mission::commands::AltChangeAction::Descend)
    } else {
        Err(pyo3::exceptions::PyValueError::new_err(
            "action must be 'neutral', 'climb', or 'descend'",
        ))
    }
}

fn alt_change_action_name(action: mavkit::mission::commands::AltChangeAction) -> &'static str {
    match action {
        mavkit::mission::commands::AltChangeAction::Neutral => "neutral",
        mavkit::mission::commands::AltChangeAction::Climb => "climb",
        mavkit::mission::commands::AltChangeAction::Descend => "descend",
    }
}

fn fence_action_from_name(action: &str) -> PyResult<mavkit::mission::commands::FenceAction> {
    if action.eq_ignore_ascii_case("disable") {
        Ok(mavkit::mission::commands::FenceAction::Disable)
    } else if action.eq_ignore_ascii_case("enable") {
        Ok(mavkit::mission::commands::FenceAction::Enable)
    } else if action.eq_ignore_ascii_case("disable_floor") {
        Ok(mavkit::mission::commands::FenceAction::DisableFloor)
    } else {
        Err(pyo3::exceptions::PyValueError::new_err(
            "action must be 'disable', 'enable', or 'disable_floor'",
        ))
    }
}

fn fence_action_name(action: mavkit::mission::commands::FenceAction) -> &'static str {
    match action {
        mavkit::mission::commands::FenceAction::Disable => "disable",
        mavkit::mission::commands::FenceAction::Enable => "enable",
        mavkit::mission::commands::FenceAction::DisableFloor => "disable_floor",
    }
}

fn parachute_action_from_name(
    action: &str,
) -> PyResult<mavkit::mission::commands::ParachuteAction> {
    if action.eq_ignore_ascii_case("disable") {
        Ok(mavkit::mission::commands::ParachuteAction::Disable)
    } else if action.eq_ignore_ascii_case("enable") {
        Ok(mavkit::mission::commands::ParachuteAction::Enable)
    } else if action.eq_ignore_ascii_case("release") {
        Ok(mavkit::mission::commands::ParachuteAction::Release)
    } else {
        Err(pyo3::exceptions::PyValueError::new_err(
            "action must be 'disable', 'enable', or 'release'",
        ))
    }
}

fn parachute_action_name(action: mavkit::mission::commands::ParachuteAction) -> &'static str {
    match action {
        mavkit::mission::commands::ParachuteAction::Disable => "disable",
        mavkit::mission::commands::ParachuteAction::Enable => "enable",
        mavkit::mission::commands::ParachuteAction::Release => "release",
    }
}

fn gripper_action_from_name(action: &str) -> PyResult<mavkit::mission::commands::GripperAction> {
    if action.eq_ignore_ascii_case("release") {
        Ok(mavkit::mission::commands::GripperAction::Release)
    } else if action.eq_ignore_ascii_case("grab") {
        Ok(mavkit::mission::commands::GripperAction::Grab)
    } else {
        Err(pyo3::exceptions::PyValueError::new_err(
            "action must be 'release' or 'grab'",
        ))
    }
}

fn gripper_action_name(action: mavkit::mission::commands::GripperAction) -> &'static str {
    match action {
        mavkit::mission::commands::GripperAction::Release => "release",
        mavkit::mission::commands::GripperAction::Grab => "grab",
    }
}

fn winch_action_from_name(action: &str) -> PyResult<mavkit::mission::commands::WinchAction> {
    if action.eq_ignore_ascii_case("relax") {
        Ok(mavkit::mission::commands::WinchAction::Relax)
    } else if action.eq_ignore_ascii_case("length_control") {
        Ok(mavkit::mission::commands::WinchAction::LengthControl)
    } else if action.eq_ignore_ascii_case("rate_control") {
        Ok(mavkit::mission::commands::WinchAction::RateControl)
    } else {
        Err(pyo3::exceptions::PyValueError::new_err(
            "action must be 'relax', 'length_control', or 'rate_control'",
        ))
    }
}

fn winch_action_name(action: mavkit::mission::commands::WinchAction) -> &'static str {
    match action {
        mavkit::mission::commands::WinchAction::Relax => "relax",
        mavkit::mission::commands::WinchAction::LengthControl => "length_control",
        mavkit::mission::commands::WinchAction::RateControl => "rate_control",
    }
}

#[pyclass(name = "RawMissionCommand", frozen, from_py_object)]
#[derive(Clone)]
pub struct PyRawMissionCommand {
    pub(crate) inner: mavkit::RawMissionCommand,
}

#[pymethods]
impl PyRawMissionCommand {
    #[new]
    #[pyo3(signature = (*, command, frame, x=0, y=0, z=0.0, param1=0.0, param2=0.0, param3=0.0, param4=0.0, frame_raw=None))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        command: u16,
        frame: PyMissionFrame,
        x: i32,
        y: i32,
        z: f32,
        param1: f32,
        param2: f32,
        param3: f32,
        param4: f32,
        frame_raw: Option<u8>,
    ) -> Self {
        Self {
            inner: mavkit::RawMissionCommand {
                command,
                frame: command_frame_from_py_raw(frame, frame_raw),
                param1,
                param2,
                param3,
                param4,
                x,
                y,
                z,
            },
        }
    }

    #[getter]
    fn command(&self) -> u16 {
        self.inner.command
    }

    #[getter]
    fn frame(&self) -> PyMissionFrame {
        py_frame_from_command(self.inner.frame)
    }

    /// Raw MAVLink frame ID, preserving the exact numeric value for
    /// non-standard frames that ``MissionFrame.Other`` would collapse.
    #[getter]
    fn frame_raw(&self) -> u8 {
        raw_frame_id(self.inner.frame)
    }

    #[getter]
    fn x(&self) -> i32 {
        self.inner.x
    }

    #[getter]
    fn y(&self) -> i32 {
        self.inner.y
    }

    #[getter]
    fn z(&self) -> f32 {
        self.inner.z
    }

    #[getter]
    fn param1(&self) -> f32 {
        self.inner.param1
    }

    #[getter]
    fn param2(&self) -> f32 {
        self.inner.param2
    }

    #[getter]
    fn param3(&self) -> f32 {
        self.inner.param3
    }

    #[getter]
    fn param4(&self) -> f32 {
        self.inner.param4
    }
}

/// Generates simple infallible position-backed Python mission wrappers.
///
/// Keep this limited to wrappers whose only custom state is a `position` field plus direct
/// scalar passthrough fields. Wrappers with validation, enum/string conversion, or other
/// non-trivial Python behavior should stay hand-written.
macro_rules! py_position_wrapper {
    (
        $py_name:ident,
        $py_class_name:literal,
        $inner_ty:path,
        new_signature = ($($new_sig:tt)*),
        new_attrs = { $(#[$new_attr:meta])* },
        fields = { $($field:ident : $field_ty:ty),* $(,)? },
        from_point_signature = ($($from_point_sig:tt)*)
    ) => {
        #[pyclass(name = $py_class_name, frozen, from_py_object)]
        #[derive(Clone)]
        pub struct $py_name {
            pub(crate) inner: $inner_ty,
        }

        #[pymethods]
        impl $py_name {
            #[new]
            #[pyo3(signature = ($($new_sig)*))]
            $(#[$new_attr])*
            fn new(
                latitude_deg: f64,
                longitude_deg: f64,
                altitude_m: f32,
                frame: PyMissionFrame,
                $($field: $field_ty),*
            ) -> Self {
                type Inner = $inner_ty;
                Self {
                    inner: Inner {
                        position: position_from_components(
                            frame,
                            latitude_deg,
                            longitude_deg,
                            altitude_m,
                        ),
                        $($field),*
                    },
                }
            }

            #[staticmethod]
            #[pyo3(signature = ($($from_point_sig)*))]
            fn from_point(position: &PyGeoPoint3d $(, $field: $field_ty)*) -> Self {
                type Inner = $inner_ty;
                Self {
                    inner: Inner {
                        position: position.inner.clone(),
                        $($field),*
                    },
                }
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

            $(
                #[getter]
                fn $field(&self) -> $field_ty {
                    self.inner.$field
                }
            )*
        }
    };
}

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

#[pyclass(name = "DoChangeSpeed", frozen, from_py_object)]
#[derive(Clone)]
pub struct PyDoChangeSpeed {
    pub(crate) inner: mavkit::mission::commands::DoChangeSpeed,
}

#[pymethods]
impl PyDoChangeSpeed {
    #[new]
    #[pyo3(signature = (*, speed_mps, throttle_pct=0.0, speed_type="groundspeed"))]
    fn new(speed_mps: f32, throttle_pct: f32, speed_type: &str) -> PyResult<Self> {
        Ok(Self {
            inner: mavkit::mission::commands::DoChangeSpeed {
                speed_type: speed_type_from_name(speed_type)?,
                speed_mps,
                throttle_pct,
            },
        })
    }

    #[getter]
    fn speed_type(&self) -> &'static str {
        speed_type_name(self.inner.speed_type)
    }

    #[getter]
    fn speed_mps(&self) -> f32 {
        self.inner.speed_mps
    }

    #[getter]
    fn throttle_pct(&self) -> f32 {
        self.inner.throttle_pct
    }
}

py_position_wrapper!(
    PyDoSetHome,
    "DoSetHome",
    mavkit::mission::commands::DoSetHome,
    new_signature = (
        *,
        latitude_deg,
        longitude_deg,
        altitude_m,
        frame=PyMissionFrame::GlobalRelativeAltInt,
        use_current=false
    ),
    new_attrs = {},
    fields = { use_current: bool },
    from_point_signature = (*, position, use_current=false)
);

py_frozen_wrapper!(PyDoSetRelay wraps mavkit::mission::commands::DoSetRelay as "DoSetRelay" {
    number: u8,
    state: bool,
});

#[pyclass(name = "DoSetRoiNone", frozen, from_py_object)]
#[derive(Clone, Default)]
pub struct PyDoSetRoiNone;

#[pymethods]
impl PyDoSetRoiNone {
    #[new]
    fn new() -> Self {
        Self
    }
}

py_frozen_wrapper!(PyDoJump wraps mavkit::mission::commands::DoJump as "DoJump" {
    target_index: u16,
    repeat_count: u16,
});

py_frozen_wrapper!(PyDoJumpTag wraps mavkit::mission::commands::DoJumpTag as "DoJumpTag" {
    tag: u16,
    repeat_count: u16,
});

py_frozen_wrapper!(PyDoTag wraps mavkit::mission::commands::DoTag as "DoTag" {
    tag: u16,
});

py_frozen_wrapper!(PyDoPauseContinue wraps mavkit::mission::commands::DoPauseContinue as "DoPauseContinue" {
    pause: bool,
});

py_frozen_wrapper!(PyDoSetReverse wraps mavkit::mission::commands::DoSetReverse as "DoSetReverse" {
    reverse: bool,
});

py_position_wrapper!(
    PyDoLandStart,
    "DoLandStart",
    mavkit::mission::commands::DoLandStart,
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
    PyDoReturnPathStart,
    "DoReturnPathStart",
    mavkit::mission::commands::DoReturnPathStart,
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
    PyDoGoAround,
    "DoGoAround",
    mavkit::mission::commands::DoGoAround,
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
    PyDoSetRoiLocation,
    "DoSetRoiLocation",
    mavkit::mission::commands::DoSetRoiLocation,
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
    PyDoSetRoi,
    "DoSetRoi",
    mavkit::mission::commands::DoSetRoi,
    new_signature = (
        *,
        latitude_deg,
        longitude_deg,
        altitude_m,
        frame=PyMissionFrame::GlobalRelativeAltInt,
        mode=0
    ),
    new_attrs = {},
    fields = { mode: u8 },
    from_point_signature = (*, position, mode=0)
);

py_frozen_wrapper!(PyDoMountControl wraps mavkit::mission::commands::DoMountControl as "DoMountControl" {
    pitch_deg: f32,
    roll_deg: f32,
    yaw_deg: f32,
});

py_frozen_wrapper!(PyDoGimbalManagerPitchYaw wraps mavkit::mission::commands::DoGimbalManagerPitchYaw as "DoGimbalManagerPitchYaw" {
    pitch_deg: f32,
    yaw_deg: f32,
    pitch_rate_dps: f32,
    yaw_rate_dps: f32,
    flags: u32,
    gimbal_id: u8,
});

py_frozen_wrapper!(PyDoCamTriggerDistance wraps mavkit::mission::commands::DoCamTriggerDistance as "DoCamTriggerDistance" {
    meters: f32,
    trigger_now: bool,
});

py_frozen_wrapper!(PyDoDigicamConfigure wraps mavkit::mission::commands::DoDigicamConfigure as "DoDigicamConfigure" {
    shooting_mode: u8,
    shutter_speed: u16,
    aperture: f32,
    iso: u16,
    exposure_type: u8,
    cmd_id: u8,
    cutoff_time: f32,
});

py_frozen_wrapper!(PyDoDigicamControl wraps mavkit::mission::commands::DoDigicamControl as "DoDigicamControl" {
    session: u8,
    zoom_pos: u8,
    zoom_step: i8,
    focus_lock: u8,
    shooting_cmd: u8,
    cmd_id: u8,
});

#[pyclass(name = "DoFenceEnable", frozen, from_py_object)]
#[derive(Clone)]
pub struct PyDoFenceEnable {
    pub(crate) inner: mavkit::mission::commands::DoFenceEnable,
}

#[pymethods]
impl PyDoFenceEnable {
    #[new]
    #[pyo3(signature = (*, action))]
    fn new(action: &str) -> PyResult<Self> {
        Ok(Self {
            inner: mavkit::mission::commands::DoFenceEnable {
                action: fence_action_from_name(action)?,
            },
        })
    }

    #[getter]
    fn action(&self) -> &'static str {
        fence_action_name(self.inner.action)
    }
}

#[pyclass(name = "DoParachute", frozen, from_py_object)]
#[derive(Clone)]
pub struct PyDoParachute {
    pub(crate) inner: mavkit::mission::commands::DoParachute,
}

#[pymethods]
impl PyDoParachute {
    #[new]
    #[pyo3(signature = (*, action))]
    fn new(action: &str) -> PyResult<Self> {
        Ok(Self {
            inner: mavkit::mission::commands::DoParachute {
                action: parachute_action_from_name(action)?,
            },
        })
    }

    #[getter]
    fn action(&self) -> &'static str {
        parachute_action_name(self.inner.action)
    }
}

#[pyclass(name = "DoGripper", frozen, from_py_object)]
#[derive(Clone)]
pub struct PyDoGripper {
    pub(crate) inner: mavkit::mission::commands::DoGripper,
}

#[pymethods]
impl PyDoGripper {
    #[new]
    #[pyo3(signature = (*, number, action))]
    fn new(number: u8, action: &str) -> PyResult<Self> {
        Ok(Self {
            inner: mavkit::mission::commands::DoGripper {
                number,
                action: gripper_action_from_name(action)?,
            },
        })
    }

    #[getter]
    fn number(&self) -> u8 {
        self.inner.number
    }

    #[getter]
    fn action(&self) -> &'static str {
        gripper_action_name(self.inner.action)
    }
}

py_frozen_wrapper!(PyDoSprayer wraps mavkit::mission::commands::DoSprayer as "DoSprayer" {
    enabled: bool,
});

#[pyclass(name = "DoWinch", frozen, from_py_object)]
#[derive(Clone)]
pub struct PyDoWinch {
    pub(crate) inner: mavkit::mission::commands::DoWinch,
}

#[pymethods]
impl PyDoWinch {
    #[new]
    #[pyo3(signature = (*, number, action, release_length_m, release_rate_mps))]
    fn new(
        number: u8,
        action: &str,
        release_length_m: f32,
        release_rate_mps: f32,
    ) -> PyResult<Self> {
        Ok(Self {
            inner: mavkit::mission::commands::DoWinch {
                number,
                action: winch_action_from_name(action)?,
                release_length_m,
                release_rate_mps,
            },
        })
    }

    #[getter]
    fn number(&self) -> u8 {
        self.inner.number
    }

    #[getter]
    fn action(&self) -> &'static str {
        winch_action_name(self.inner.action)
    }

    #[getter]
    fn release_length_m(&self) -> f32 {
        self.inner.release_length_m
    }

    #[getter]
    fn release_rate_mps(&self) -> f32 {
        self.inner.release_rate_mps
    }
}

py_frozen_wrapper!(PyDoEngineControl wraps mavkit::mission::commands::DoEngineControl as "DoEngineControl" {
    start: bool,
    cold_start: bool,
    height_delay_m: f32,
    allow_disarmed: bool,
});

py_frozen_wrapper!(PyDoInvertedFlight wraps mavkit::mission::commands::DoInvertedFlight as "DoInvertedFlight" {
    inverted: bool,
});

py_frozen_wrapper!(PyDoAutotuneEnable wraps mavkit::mission::commands::DoAutotuneEnable as "DoAutotuneEnable" {
    enabled: bool,
});

py_frozen_wrapper!(PyDoSetServo wraps mavkit::mission::commands::DoSetServo as "DoSetServo" {
    channel: u16,
    pwm: u16,
});

py_frozen_wrapper!(PyDoRepeatServo wraps mavkit::mission::commands::DoRepeatServo as "DoRepeatServo" {
    channel: u16,
    pwm: u16,
    count: u16,
    cycle_time_s: f32,
});

py_frozen_wrapper!(PyDoRepeatRelay wraps mavkit::mission::commands::DoRepeatRelay as "DoRepeatRelay" {
    number: u8,
    count: u16,
    cycle_time_s: f32,
});

py_frozen_wrapper!(PyDoSetResumeRepeatDist wraps mavkit::mission::commands::DoSetResumeRepeatDist as "DoSetResumeRepeatDist" {
    distance_m: f32,
});

py_frozen_wrapper!(PyDoAuxFunction wraps mavkit::mission::commands::DoAuxFunction as "DoAuxFunction" {
    function: u16,
    switch_pos: u8,
});

py_frozen_wrapper!(PyDoSendScriptMessage wraps mavkit::mission::commands::DoSendScriptMessage as "DoSendScriptMessage" {
    id: u16,
    p1: f32,
    p2: f32,
    p3: f32,
});

py_frozen_wrapper!(PyDoImageStartCapture wraps mavkit::mission::commands::DoImageStartCapture as "DoImageStartCapture" {
    instance: u8,
    interval_s: f32,
    total_images: u32,
    start_number: u32,
});

py_frozen_wrapper!(PyDoImageStopCapture wraps mavkit::mission::commands::DoImageStopCapture as "DoImageStopCapture" {
    instance: u8,
});

py_frozen_wrapper!(PyDoVideoStartCapture wraps mavkit::mission::commands::DoVideoStartCapture as "DoVideoStartCapture" {
    stream_id: u8,
});

py_frozen_wrapper!(PyDoVideoStopCapture wraps mavkit::mission::commands::DoVideoStopCapture as "DoVideoStopCapture" {
    stream_id: u8,
});

py_frozen_wrapper!(PyDoSetCameraZoom wraps mavkit::mission::commands::DoSetCameraZoom as "DoSetCameraZoom" {
    zoom_type: u8,
    zoom_value: f32,
});

py_frozen_wrapper!(PyDoSetCameraFocus wraps mavkit::mission::commands::DoSetCameraFocus as "DoSetCameraFocus" {
    focus_type: u8,
    focus_value: f32,
});

py_frozen_wrapper!(PyDoSetCameraSource wraps mavkit::mission::commands::DoSetCameraSource as "DoSetCameraSource" {
    instance: u8,
    primary: u8,
    secondary: u8,
});

py_frozen_wrapper!(PyDoGuidedLimits wraps mavkit::mission::commands::DoGuidedLimits as "DoGuidedLimits" {
    max_time_s: f32,
    min_alt_m: f32,
    max_alt_m: f32,
    max_horiz_m: f32,
});

py_frozen_wrapper!(PyDoVtolTransition wraps mavkit::mission::commands::DoVtolTransition as "DoVtolTransition" {
    target_state: u8,
});

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

fn typed_command_from_py(command: &Bound<'_, PyAny>) -> PyResult<Option<mavkit::MissionCommand>> {
    if let Ok(nav_waypoint) = command.extract::<PyRef<'_, PyNavWaypoint>>() {
        return Ok(Some(nav_waypoint.inner.clone().into()));
    }

    if let Ok(nav_takeoff) = command.extract::<PyRef<'_, PyNavTakeoff>>() {
        return Ok(Some(nav_takeoff.inner.clone().into()));
    }

    if let Ok(nav_land) = command.extract::<PyRef<'_, PyNavLand>>() {
        return Ok(Some(nav_land.inner.clone().into()));
    }

    if let Ok(nav_loiter_time) = command.extract::<PyRef<'_, PyNavLoiterTime>>() {
        return Ok(Some(nav_loiter_time.inner.clone().into()));
    }

    if let Ok(nav_guided_enable) = command.extract::<PyRef<'_, PyNavGuidedEnable>>() {
        return Ok(Some(nav_guided_enable.inner.clone().into()));
    }

    if let Ok(nav_spline_waypoint) = command.extract::<PyRef<'_, PyNavSplineWaypoint>>() {
        return Ok(Some(nav_spline_waypoint.inner.clone().into()));
    }

    if let Ok(nav_arc_waypoint) = command.extract::<PyRef<'_, PyNavArcWaypoint>>() {
        return Ok(Some(nav_arc_waypoint.inner.clone().into()));
    }

    if let Ok(nav_loiter_unlimited) = command.extract::<PyRef<'_, PyNavLoiterUnlimited>>() {
        return Ok(Some(nav_loiter_unlimited.inner.clone().into()));
    }

    if let Ok(nav_loiter_turns) = command.extract::<PyRef<'_, PyNavLoiterTurns>>() {
        return Ok(Some(nav_loiter_turns.inner.clone().into()));
    }

    if let Ok(nav_loiter_to_alt) = command.extract::<PyRef<'_, PyNavLoiterToAlt>>() {
        return Ok(Some(nav_loiter_to_alt.inner.clone().into()));
    }

    if let Ok(nav_continue_and_change_alt) =
        command.extract::<PyRef<'_, PyNavContinueAndChangeAlt>>()
    {
        return Ok(Some(nav_continue_and_change_alt.inner.clone().into()));
    }

    if let Ok(nav_delay) = command.extract::<PyRef<'_, PyNavDelay>>() {
        return Ok(Some(nav_delay.inner.clone().into()));
    }

    if let Ok(nav_altitude_wait) = command.extract::<PyRef<'_, PyNavAltitudeWait>>() {
        return Ok(Some(nav_altitude_wait.inner.clone().into()));
    }

    if let Ok(nav_vtol_takeoff) = command.extract::<PyRef<'_, PyNavVtolTakeoff>>() {
        return Ok(Some(nav_vtol_takeoff.inner.clone().into()));
    }

    if let Ok(nav_vtol_land) = command.extract::<PyRef<'_, PyNavVtolLand>>() {
        return Ok(Some(nav_vtol_land.inner.clone().into()));
    }

    if let Ok(nav_payload_place) = command.extract::<PyRef<'_, PyNavPayloadPlace>>() {
        return Ok(Some(nav_payload_place.inner.clone().into()));
    }

    if let Ok(nav_set_yaw_speed) = command.extract::<PyRef<'_, PyNavSetYawSpeed>>() {
        return Ok(Some(nav_set_yaw_speed.inner.clone().into()));
    }

    if let Ok(nav_script_time) = command.extract::<PyRef<'_, PyNavScriptTime>>() {
        return Ok(Some(nav_script_time.inner.clone().into()));
    }

    if let Ok(nav_attitude_time) = command.extract::<PyRef<'_, PyNavAttitudeTime>>() {
        return Ok(Some(nav_attitude_time.inner.clone().into()));
    }

    if command.extract::<PyRef<'_, PyNavReturnToLaunch>>().is_ok() {
        return Ok(Some(
            mavkit::mission::commands::NavCommand::ReturnToLaunch.into(),
        ));
    }

    if let Ok(do_change_speed) = command.extract::<PyRef<'_, PyDoChangeSpeed>>() {
        return Ok(Some(do_change_speed.inner.clone().into()));
    }

    if let Ok(do_set_home) = command.extract::<PyRef<'_, PyDoSetHome>>() {
        return Ok(Some(do_set_home.inner.clone().into()));
    }

    if let Ok(do_set_relay) = command.extract::<PyRef<'_, PyDoSetRelay>>() {
        return Ok(Some(do_set_relay.inner.clone().into()));
    }

    if command.extract::<PyRef<'_, PyDoSetRoiNone>>().is_ok() {
        return Ok(Some(
            mavkit::mission::commands::DoCommand::SetRoiNone.into(),
        ));
    }

    if let Ok(do_jump) = command.extract::<PyRef<'_, PyDoJump>>() {
        return Ok(Some(do_jump.inner.clone().into()));
    }

    if let Ok(do_jump_tag) = command.extract::<PyRef<'_, PyDoJumpTag>>() {
        return Ok(Some(do_jump_tag.inner.clone().into()));
    }

    if let Ok(do_tag) = command.extract::<PyRef<'_, PyDoTag>>() {
        return Ok(Some(do_tag.inner.clone().into()));
    }

    if let Ok(do_pause_continue) = command.extract::<PyRef<'_, PyDoPauseContinue>>() {
        return Ok(Some(do_pause_continue.inner.clone().into()));
    }

    if let Ok(do_set_reverse) = command.extract::<PyRef<'_, PyDoSetReverse>>() {
        return Ok(Some(do_set_reverse.inner.clone().into()));
    }

    if let Ok(do_land_start) = command.extract::<PyRef<'_, PyDoLandStart>>() {
        return Ok(Some(do_land_start.inner.clone().into()));
    }

    if let Ok(do_return_path_start) = command.extract::<PyRef<'_, PyDoReturnPathStart>>() {
        return Ok(Some(do_return_path_start.inner.clone().into()));
    }

    if let Ok(do_go_around) = command.extract::<PyRef<'_, PyDoGoAround>>() {
        return Ok(Some(do_go_around.inner.clone().into()));
    }

    if let Ok(do_set_roi_location) = command.extract::<PyRef<'_, PyDoSetRoiLocation>>() {
        return Ok(Some(do_set_roi_location.inner.clone().into()));
    }

    if let Ok(do_set_roi) = command.extract::<PyRef<'_, PyDoSetRoi>>() {
        return Ok(Some(do_set_roi.inner.clone().into()));
    }

    if let Ok(do_mount_control) = command.extract::<PyRef<'_, PyDoMountControl>>() {
        return Ok(Some(do_mount_control.inner.clone().into()));
    }

    if let Ok(do_gimbal_manager_pitch_yaw) =
        command.extract::<PyRef<'_, PyDoGimbalManagerPitchYaw>>()
    {
        return Ok(Some(do_gimbal_manager_pitch_yaw.inner.clone().into()));
    }

    if let Ok(do_cam_trigger_distance) = command.extract::<PyRef<'_, PyDoCamTriggerDistance>>() {
        return Ok(Some(do_cam_trigger_distance.inner.clone().into()));
    }

    if let Ok(do_digicam_configure) = command.extract::<PyRef<'_, PyDoDigicamConfigure>>() {
        return Ok(Some(do_digicam_configure.inner.clone().into()));
    }

    if let Ok(do_digicam_control) = command.extract::<PyRef<'_, PyDoDigicamControl>>() {
        return Ok(Some(do_digicam_control.inner.clone().into()));
    }

    if let Ok(do_fence_enable) = command.extract::<PyRef<'_, PyDoFenceEnable>>() {
        return Ok(Some(do_fence_enable.inner.clone().into()));
    }

    if let Ok(do_parachute) = command.extract::<PyRef<'_, PyDoParachute>>() {
        return Ok(Some(do_parachute.inner.clone().into()));
    }

    if let Ok(do_gripper) = command.extract::<PyRef<'_, PyDoGripper>>() {
        return Ok(Some(do_gripper.inner.clone().into()));
    }

    if let Ok(do_sprayer) = command.extract::<PyRef<'_, PyDoSprayer>>() {
        return Ok(Some(do_sprayer.inner.clone().into()));
    }

    if let Ok(do_winch) = command.extract::<PyRef<'_, PyDoWinch>>() {
        return Ok(Some(do_winch.inner.clone().into()));
    }

    if let Ok(do_engine_control) = command.extract::<PyRef<'_, PyDoEngineControl>>() {
        return Ok(Some(do_engine_control.inner.clone().into()));
    }

    if let Ok(do_inverted_flight) = command.extract::<PyRef<'_, PyDoInvertedFlight>>() {
        return Ok(Some(do_inverted_flight.inner.clone().into()));
    }

    if let Ok(do_autotune_enable) = command.extract::<PyRef<'_, PyDoAutotuneEnable>>() {
        return Ok(Some(do_autotune_enable.inner.clone().into()));
    }

    if let Ok(do_set_servo) = command.extract::<PyRef<'_, PyDoSetServo>>() {
        return Ok(Some(do_set_servo.inner.clone().into()));
    }

    if let Ok(do_repeat_servo) = command.extract::<PyRef<'_, PyDoRepeatServo>>() {
        return Ok(Some(do_repeat_servo.inner.clone().into()));
    }

    if let Ok(do_repeat_relay) = command.extract::<PyRef<'_, PyDoRepeatRelay>>() {
        return Ok(Some(do_repeat_relay.inner.clone().into()));
    }

    if let Ok(do_set_resume_repeat_dist) = command.extract::<PyRef<'_, PyDoSetResumeRepeatDist>>() {
        return Ok(Some(do_set_resume_repeat_dist.inner.clone().into()));
    }

    if let Ok(do_aux_function) = command.extract::<PyRef<'_, PyDoAuxFunction>>() {
        return Ok(Some(do_aux_function.inner.clone().into()));
    }

    if let Ok(do_send_script_message) = command.extract::<PyRef<'_, PyDoSendScriptMessage>>() {
        return Ok(Some(do_send_script_message.inner.clone().into()));
    }

    if let Ok(do_image_start_capture) = command.extract::<PyRef<'_, PyDoImageStartCapture>>() {
        return Ok(Some(do_image_start_capture.inner.clone().into()));
    }

    if let Ok(do_image_stop_capture) = command.extract::<PyRef<'_, PyDoImageStopCapture>>() {
        return Ok(Some(do_image_stop_capture.inner.clone().into()));
    }

    if let Ok(do_video_start_capture) = command.extract::<PyRef<'_, PyDoVideoStartCapture>>() {
        return Ok(Some(do_video_start_capture.inner.clone().into()));
    }

    if let Ok(do_video_stop_capture) = command.extract::<PyRef<'_, PyDoVideoStopCapture>>() {
        return Ok(Some(do_video_stop_capture.inner.clone().into()));
    }

    if let Ok(do_set_camera_zoom) = command.extract::<PyRef<'_, PyDoSetCameraZoom>>() {
        return Ok(Some(do_set_camera_zoom.inner.clone().into()));
    }

    if let Ok(do_set_camera_focus) = command.extract::<PyRef<'_, PyDoSetCameraFocus>>() {
        return Ok(Some(do_set_camera_focus.inner.clone().into()));
    }

    if let Ok(do_set_camera_source) = command.extract::<PyRef<'_, PyDoSetCameraSource>>() {
        return Ok(Some(do_set_camera_source.inner.clone().into()));
    }

    if let Ok(do_guided_limits) = command.extract::<PyRef<'_, PyDoGuidedLimits>>() {
        return Ok(Some(do_guided_limits.inner.clone().into()));
    }

    if let Ok(do_vtol_transition) = command.extract::<PyRef<'_, PyDoVtolTransition>>() {
        return Ok(Some(do_vtol_transition.inner.clone().into()));
    }

    if let Ok(cond_delay) = command.extract::<PyRef<'_, PyCondDelay>>() {
        return Ok(Some(cond_delay.inner.clone().into()));
    }

    if let Ok(cond_distance) = command.extract::<PyRef<'_, PyCondDistance>>() {
        return Ok(Some(cond_distance.inner.clone().into()));
    }

    if let Ok(cond_yaw) = command.extract::<PyRef<'_, PyCondYaw>>() {
        return Ok(Some(cond_yaw.inner.clone().into()));
    }

    if let Ok(raw) = command.extract::<PyRef<'_, PyRawMissionCommand>>() {
        return Ok(Some(raw.inner.clone().into()));
    }

    Ok(None)
}

// --- MissionItem ---

#[pyclass(name = "MissionItem", frozen, from_py_object)]
#[derive(Clone)]
pub struct PyMissionItem {
    pub(crate) inner: mavkit::MissionItem,
}

#[pymethods]
impl PyMissionItem {
    #[new]
    #[pyo3(signature = (*, command, frame=None, x=0, y=0, z=0.0, param1=0.0, param2=0.0, param3=0.0, param4=0.0, autocontinue=true))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        command: &Bound<'_, PyAny>,
        frame: Option<PyMissionFrame>,
        x: i32,
        y: i32,
        z: f32,
        param1: f32,
        param2: f32,
        param3: f32,
        param4: f32,
        autocontinue: bool,
    ) -> PyResult<Self> {
        let uses_legacy_wire_fields = frame.is_some()
            || x != 0
            || y != 0
            || z != 0.0
            || param1 != 0.0
            || param2 != 0.0
            || param3 != 0.0
            || param4 != 0.0;

        let mission_command = if let Some(typed_command) = typed_command_from_py(command)? {
            if uses_legacy_wire_fields {
                return Err(pyo3::exceptions::PyTypeError::new_err(
                    "frame/x/y/z/param1..4 are only valid when command is an int",
                ));
            }
            typed_command
        } else {
            let command_id = command.extract::<u16>().map_err(|_| {
                pyo3::exceptions::PyTypeError::new_err(
                    "command must be a supported typed mission command, RawMissionCommand, or int",
                )
            })?;

            let frame = frame.ok_or_else(|| {
                pyo3::exceptions::PyTypeError::new_err("frame is required when command is an int")
            })?;

            mavkit::MissionCommand::Other(mavkit::RawMissionCommand {
                command: command_id,
                frame: command_frame_from_py(frame),
                param1,
                param2,
                param3,
                param4,
                x,
                y,
                z,
            })
        };

        Ok(Self {
            inner: mavkit::MissionItem {
                command: mission_command,
                autocontinue,
            },
        })
    }

    #[getter]
    fn command(&self) -> u16 {
        wire_parts(&self.inner).0
    }
    #[getter]
    fn frame(&self) -> PyMissionFrame {
        wire_parts(&self.inner).1
    }
    #[getter]
    fn autocontinue(&self) -> bool {
        self.inner.autocontinue
    }
    #[getter]
    fn param1(&self) -> f32 {
        wire_parts(&self.inner).2[0]
    }
    #[getter]
    fn param2(&self) -> f32 {
        wire_parts(&self.inner).2[1]
    }
    #[getter]
    fn param3(&self) -> f32 {
        wire_parts(&self.inner).2[2]
    }
    #[getter]
    fn param4(&self) -> f32 {
        wire_parts(&self.inner).2[3]
    }
    #[getter]
    fn x(&self) -> i32 {
        wire_parts(&self.inner).3
    }
    #[getter]
    fn y(&self) -> i32 {
        wire_parts(&self.inner).4
    }
    #[getter]
    fn z(&self) -> f32 {
        wire_parts(&self.inner).5
    }

    fn __repr__(&self) -> String {
        format!(
            "MissionItem(cmd={}, frame={:?})",
            self.command(),
            self.frame()
        )
    }
}

// --- HomePosition ---

#[pyclass(name = "HomePosition", frozen, from_py_object)]
#[derive(Clone)]
pub struct PyHomePosition {
    pub(crate) inner: mavkit::HomePosition,
}

#[pymethods]
impl PyHomePosition {
    #[new]
    #[pyo3(signature = (*, latitude_deg, longitude_deg, altitude_m=0.0))]
    fn new(latitude_deg: f64, longitude_deg: f64, altitude_m: f64) -> Self {
        Self {
            inner: mavkit::HomePosition {
                latitude_deg,
                longitude_deg,
                altitude_m,
            },
        }
    }

    #[getter]
    fn latitude_deg(&self) -> f64 {
        self.inner.latitude_deg
    }
    #[getter]
    fn longitude_deg(&self) -> f64 {
        self.inner.longitude_deg
    }
    #[getter]
    fn altitude_m(&self) -> f64 {
        self.inner.altitude_m
    }

    fn __repr__(&self) -> String {
        format!(
            "HomePosition(lat={:.6}, lon={:.6}, alt={:.1})",
            self.inner.latitude_deg, self.inner.longitude_deg, self.inner.altitude_m
        )
    }
}

// --- MissionPlan ---

#[pyclass(name = "MissionPlan", frozen, from_py_object)]
#[derive(Clone)]
pub struct PyMissionPlan {
    pub(crate) inner: mavkit::MissionPlan,
}

#[pymethods]
impl PyMissionPlan {
    #[new]
    #[pyo3(signature = (*, items))]
    fn new(items: Vec<PyMissionItem>) -> Self {
        Self {
            inner: mavkit::MissionPlan {
                items: items.into_iter().map(|i| i.inner).collect(),
            },
        }
    }

    #[getter]
    fn items(&self) -> Vec<PyMissionItem> {
        self.inner
            .items
            .iter()
            .map(|i| PyMissionItem { inner: i.clone() })
            .collect()
    }

    fn __repr__(&self) -> String {
        format!("MissionPlan(items={})", self.inner.items.len())
    }

    fn __len__(&self) -> usize {
        self.inner.items.len()
    }
}

// --- MissionIssue ---

#[pyclass(name = "MissionIssue", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyMissionIssue {
    inner: mavkit::MissionIssue,
}

#[pymethods]
impl PyMissionIssue {
    #[getter]
    fn code(&self) -> &str {
        &self.inner.code
    }
    #[getter]
    fn message(&self) -> &str {
        &self.inner.message
    }
    #[getter]
    fn seq(&self) -> Option<u16> {
        self.inner.seq
    }
    #[getter]
    fn severity(&self) -> PyIssueSeverity {
        self.inner.severity.into()
    }

    fn __repr__(&self) -> String {
        format!(
            "MissionIssue({:?}: {} - '{}')",
            self.inner.severity, self.inner.code, self.inner.message
        )
    }
}

// --- TransferProgress ---

#[pyclass(name = "TransferProgress", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyTransferProgress {
    pub(crate) inner: mavkit::TransferProgress,
}

#[pymethods]
impl PyTransferProgress {
    #[getter]
    fn direction(&self) -> PyTransferDirection {
        self.inner.direction.into()
    }
    #[getter]
    fn mission_type(&self) -> PyMissionType {
        self.inner.mission_type.into()
    }
    #[getter]
    fn phase(&self) -> PyTransferPhase {
        self.inner.phase.into()
    }
    #[getter]
    fn completed_items(&self) -> u16 {
        self.inner.completed_items
    }
    #[getter]
    fn total_items(&self) -> u16 {
        self.inner.total_items
    }
    #[getter]
    fn retries_used(&self) -> u8 {
        self.inner.retries_used
    }

    fn __repr__(&self) -> String {
        format!(
            "TransferProgress({:?} {:?}: {}/{})",
            self.inner.direction,
            self.inner.phase,
            self.inner.completed_items,
            self.inner.total_items
        )
    }
}

// --- TransferError ---

#[pyclass(name = "TransferError", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyTransferError {
    inner: mavkit::TransferError,
}

#[pymethods]
impl PyTransferError {
    #[getter]
    fn code(&self) -> &str {
        &self.inner.code
    }
    #[getter]
    fn message(&self) -> &str {
        &self.inner.message
    }

    fn __repr__(&self) -> String {
        format!(
            "TransferError(code='{}', message='{}')",
            self.inner.code, self.inner.message
        )
    }
}

// --- RetryPolicy ---

#[pyclass(name = "RetryPolicy", frozen, from_py_object)]
#[derive(Clone)]
pub struct PyRetryPolicy {
    pub(crate) inner: mavkit::RetryPolicy,
}

#[pymethods]
impl PyRetryPolicy {
    #[new]
    #[pyo3(signature = (*, request_timeout_ms=1500, item_timeout_ms=250, max_retries=5))]
    fn new(request_timeout_ms: u64, item_timeout_ms: u64, max_retries: u8) -> Self {
        Self {
            inner: mavkit::RetryPolicy {
                request_timeout_ms,
                item_timeout_ms,
                max_retries,
            },
        }
    }

    #[getter]
    fn request_timeout_ms(&self) -> u64 {
        self.inner.request_timeout_ms
    }
    #[getter]
    fn item_timeout_ms(&self) -> u64 {
        self.inner.item_timeout_ms
    }
    #[getter]
    fn max_retries(&self) -> u8 {
        self.inner.max_retries
    }

    fn __repr__(&self) -> String {
        format!(
            "RetryPolicy(request_timeout_ms={}, item_timeout_ms={}, max_retries={})",
            self.inner.request_timeout_ms, self.inner.item_timeout_ms, self.inner.max_retries
        )
    }
}

// --- CompareTolerance ---

#[pyclass(name = "CompareTolerance", frozen, from_py_object)]
#[derive(Clone)]
pub struct PyCompareTolerance {
    inner: mavkit::CompareTolerance,
}

#[pymethods]
impl PyCompareTolerance {
    #[new]
    #[pyo3(signature = (*, param_epsilon=0.0001, altitude_epsilon_m=0.01))]
    fn new(param_epsilon: f32, altitude_epsilon_m: f32) -> Self {
        Self {
            inner: mavkit::CompareTolerance {
                param_epsilon,
                altitude_epsilon_m,
            },
        }
    }

    #[getter]
    fn param_epsilon(&self) -> f32 {
        self.inner.param_epsilon
    }
    #[getter]
    fn altitude_epsilon_m(&self) -> f32 {
        self.inner.altitude_epsilon_m
    }

    fn __repr__(&self) -> String {
        format!(
            "CompareTolerance(param_epsilon={}, altitude_epsilon_m={})",
            self.inner.param_epsilon, self.inner.altitude_epsilon_m
        )
    }
}

// --- Free functions ---

#[pyfunction]
pub fn validate_plan(plan: &PyMissionPlan) -> Vec<PyMissionIssue> {
    mavkit::validate_plan(&plan.inner)
        .into_iter()
        .map(|i| PyMissionIssue { inner: i })
        .collect()
}

#[pyfunction]
#[pyo3(signature = (lhs, rhs, tolerance=None))]
pub fn plans_equivalent(
    lhs: &PyMissionPlan,
    rhs: &PyMissionPlan,
    tolerance: Option<&PyCompareTolerance>,
) -> bool {
    let tol = tolerance.map(|t| t.inner).unwrap_or_default();
    mavkit::plans_equivalent(&lhs.inner, &rhs.inner, tol)
}

#[pyfunction]
pub fn normalize_for_compare(plan: &PyMissionPlan) -> PyMissionPlan {
    PyMissionPlan {
        inner: mavkit::normalize_for_compare(&plan.inner),
    }
}

#[pyfunction]
pub fn mission_items_for_upload(plan: &PyMissionPlan) -> Vec<PyMissionItem> {
    mavkit::mission_items_for_upload(&plan.inner)
        .into_iter()
        .map(|i| PyMissionItem { inner: i })
        .collect()
}

#[pyfunction]
pub fn mission_plan_from_download(items: Vec<PyMissionItem>) -> PyMissionPlan {
    let wire_items: Vec<mavkit::MissionItem> = items.into_iter().map(|i| i.inner).collect();
    PyMissionPlan {
        inner: mavkit::mission_plan_from_download(wire_items),
    }
}
