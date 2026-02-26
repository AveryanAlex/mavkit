use std::sync::Arc;

use mavlink::common::MavMessage;
use mavlink::{MavHeader, Message};
use pyo3::prelude::*;

use crate::error::MavkitError;

/// A single raw MAVLink message with header metadata.
#[pyclass(name = "RawMessage", frozen)]
pub struct PyRawMessage {
    system_id: u8,
    component_id: u8,
    sequence: u8,
    message_id: u32,
    message_name: &'static str,
    message_json: String,
}

#[pymethods]
impl PyRawMessage {
    #[getter]
    fn system_id(&self) -> u8 {
        self.system_id
    }

    #[getter]
    fn component_id(&self) -> u8 {
        self.component_id
    }

    #[getter]
    fn sequence(&self) -> u8 {
        self.sequence
    }

    #[getter]
    fn message_id(&self) -> u32 {
        self.message_id
    }

    #[getter]
    fn message_name(&self) -> &str {
        self.message_name
    }

    fn message_json(&self) -> &str {
        &self.message_json
    }

    fn __repr__(&self) -> String {
        format!(
            "RawMessage(name={}, sys={}, comp={}, id={})",
            self.message_name, self.system_id, self.component_id, self.message_id,
        )
    }
}

impl PyRawMessage {
    pub fn from_rust(header: MavHeader, msg: MavMessage) -> PyResult<Self> {
        let message_id = msg.message_id();
        let message_name = msg.message_name();
        let message_json = serde_json::to_string(&msg)
            .map_err(|e| MavkitError::new_err(format!("JSON serialization error: {e}")))?;
        Ok(Self {
            system_id: header.system_id,
            component_id: header.component_id,
            sequence: header.sequence,
            message_id,
            message_name,
            message_json,
        })
    }
}

/// Async stream of raw MAVLink messages from a Vehicle connection.
#[pyclass(name = "RawMessageStream", frozen)]
pub struct PyRawMessageStream {
    pub rx: Arc<tokio::sync::Mutex<tokio::sync::broadcast::Receiver<(MavHeader, MavMessage)>>>,
}

#[pymethods]
impl PyRawMessageStream {
    fn recv<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let rx = self.rx.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let mut guard = rx.lock().await;
            loop {
                match guard.recv().await {
                    Ok((header, msg)) => return PyRawMessage::from_rust(header, msg),
                    Err(tokio::sync::broadcast::error::RecvError::Lagged(_)) => continue,
                    Err(tokio::sync::broadcast::error::RecvError::Closed) => {
                        return Err(MavkitError::new_err("raw message channel closed"));
                    }
                }
            }
        })
    }

    fn __repr__(&self) -> &'static str {
        "RawMessageStream()"
    }
}
