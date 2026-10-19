use pyo3::prelude::*;

use crate::enums::PyMavSeverity;

#[pyclass(name = "StatusTextEvent", frozen, skip_from_py_object)]
#[derive(Clone)]
pub(crate) struct PyStatusTextEvent {
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
            severity: value.severity.into(),
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
