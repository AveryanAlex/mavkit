use pyo3::prelude::*;

use crate::enums::*;

pub(crate) fn command_frame_from_py(
    frame: PyMissionFrame,
) -> mavkit::mission::commands::MissionFrame {
    command_frame_from_py_raw(frame, None)
}

pub(crate) fn command_frame_from_py_raw(
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

pub(crate) fn py_frame_from_command(
    frame: mavkit::mission::commands::MissionFrame,
) -> PyMissionFrame {
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
pub(crate) fn raw_frame_id(frame: mavkit::mission::commands::MissionFrame) -> u8 {
    match frame {
        mavkit::mission::commands::MissionFrame::Global => 0,
        mavkit::mission::commands::MissionFrame::Mission => 2,
        mavkit::mission::commands::MissionFrame::GlobalRelativeAlt => 3,
        mavkit::mission::commands::MissionFrame::GlobalTerrainAlt => 10,
        mavkit::mission::commands::MissionFrame::Other(v) => v,
    }
}

pub(crate) fn position_from_components(
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

macro_rules! string_enum_translators {
    (
        $from_fn:ident,
        $name_fn:ident,
        $enum_ty:path,
        $error_message:literal,
        {
            $(
                $variant:path => [$canonical:literal $(, $alias:literal)* $(,)?]
            ),+ $(,)?
        }
    ) => {
        pub(crate) fn $from_fn(value: &str) -> PyResult<$enum_ty> {
            match () {
                $(
                    _ if value.eq_ignore_ascii_case($canonical)
                        $(|| value.eq_ignore_ascii_case($alias))* => Ok($variant),
                )+
                _ => Err(pyo3::exceptions::PyValueError::new_err($error_message)),
            }
        }

        pub(crate) fn $name_fn(value: $enum_ty) -> &'static str {
            match value {
                $(
                    $variant => $canonical,
                )+
            }
        }
    };
}

string_enum_translators!(
    speed_type_from_name,
    speed_type_name,
    mavkit::mission::commands::SpeedType,
    "speed_type must be 'airspeed' or 'groundspeed'",
    {
        mavkit::mission::commands::SpeedType::Airspeed => ["airspeed"],
        mavkit::mission::commands::SpeedType::Groundspeed => ["groundspeed"],
    }
);

string_enum_translators!(
    yaw_direction_from_name,
    yaw_direction_name,
    mavkit::mission::commands::YawDirection,
    "direction must be 'clockwise' or 'counter_clockwise'",
    {
        mavkit::mission::commands::YawDirection::Clockwise => ["clockwise"],
        mavkit::mission::commands::YawDirection::CounterClockwise => [
            "counter_clockwise",
            "counterclockwise",
        ],
    }
);

string_enum_translators!(
    loiter_direction_from_name,
    loiter_direction_name,
    mavkit::mission::commands::LoiterDirection,
    "direction must be 'clockwise' or 'counter_clockwise'",
    {
        mavkit::mission::commands::LoiterDirection::Clockwise => ["clockwise"],
        mavkit::mission::commands::LoiterDirection::CounterClockwise => [
            "counter_clockwise",
            "counterclockwise",
        ],
    }
);

string_enum_translators!(
    alt_change_action_from_name,
    alt_change_action_name,
    mavkit::mission::commands::AltChangeAction,
    "action must be 'neutral', 'climb', or 'descend'",
    {
        mavkit::mission::commands::AltChangeAction::Neutral => ["neutral"],
        mavkit::mission::commands::AltChangeAction::Climb => ["climb"],
        mavkit::mission::commands::AltChangeAction::Descend => ["descend"],
    }
);

string_enum_translators!(
    fence_action_from_name,
    fence_action_name,
    mavkit::mission::commands::FenceAction,
    "action must be 'disable', 'enable', or 'disable_floor'",
    {
        mavkit::mission::commands::FenceAction::Disable => ["disable"],
        mavkit::mission::commands::FenceAction::Enable => ["enable"],
        mavkit::mission::commands::FenceAction::DisableFloor => ["disable_floor"],
    }
);

string_enum_translators!(
    parachute_action_from_name,
    parachute_action_name,
    mavkit::mission::commands::ParachuteAction,
    "action must be 'disable', 'enable', or 'release'",
    {
        mavkit::mission::commands::ParachuteAction::Disable => ["disable"],
        mavkit::mission::commands::ParachuteAction::Enable => ["enable"],
        mavkit::mission::commands::ParachuteAction::Release => ["release"],
    }
);

string_enum_translators!(
    gripper_action_from_name,
    gripper_action_name,
    mavkit::mission::commands::GripperAction,
    "action must be 'release' or 'grab'",
    {
        mavkit::mission::commands::GripperAction::Release => ["release"],
        mavkit::mission::commands::GripperAction::Grab => ["grab"],
    }
);

string_enum_translators!(
    winch_action_from_name,
    winch_action_name,
    mavkit::mission::commands::WinchAction,
    "action must be 'relax', 'length_control', or 'rate_control'",
    {
        mavkit::mission::commands::WinchAction::Relax => ["relax"],
        mavkit::mission::commands::WinchAction::LengthControl => ["length_control"],
        mavkit::mission::commands::WinchAction::RateControl => ["rate_control"],
    }
);

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
                        position: crate::mission::commands::position_from_components(
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
            fn from_point(position: &crate::geo::PyGeoPoint3d $(, $field: $field_ty)*) -> Self {
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
                crate::geo::position_components(&self.inner.position).0
            }

            #[getter]
            fn latitude_deg(&self) -> f64 {
                crate::geo::position_components(&self.inner.position).1
            }

            #[getter]
            fn longitude_deg(&self) -> f64 {
                crate::geo::position_components(&self.inner.position).2
            }

            #[getter]
            fn altitude_m(&self) -> f32 {
                crate::geo::position_components(&self.inner.position).3
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

pub(crate) use py_position_wrapper;
