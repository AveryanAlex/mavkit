use pyo3::prelude::*;

use crate::enums::*;

use super::common::{command_frame_from_py_raw, py_frame_from_command, raw_frame_id};

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
