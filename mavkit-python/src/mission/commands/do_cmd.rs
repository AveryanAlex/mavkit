use pyo3::prelude::*;

use crate::enums::*;
use crate::macros::py_frozen_wrapper;

use super::common::{
    fence_action_from_name, fence_action_name, gripper_action_from_name, gripper_action_name,
    parachute_action_from_name, parachute_action_name, py_position_wrapper, speed_type_from_name,
    speed_type_name, winch_action_from_name, winch_action_name,
};

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
