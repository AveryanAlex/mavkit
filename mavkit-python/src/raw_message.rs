use std::sync::Arc;
use std::time::Instant;
use std::{pin::Pin, sync::Mutex};

use mavkit::dialect::MavMessage;
use mavlink::{MavHeader, MavlinkVersion, Message};
use pyo3::exceptions::PyStopAsyncIteration;
use pyo3::prelude::*;
use tokio::sync::mpsc;
use tokio::task::JoinHandle;
use tokio_stream::StreamExt;

use crate::error::MavkitError;

type RawMessageSubscription = Pin<Box<dyn tokio_stream::Stream<Item = mavkit::RawMessage> + Send>>;

/// A single raw MAVLink message with header metadata.
#[pyclass(name = "RawMessage", frozen)]
pub struct PyRawMessage {
    system_id: u8,
    component_id: u8,
    sequence: u8,
    message_id: u32,
    message_name: &'static str,
    message_json: String,
    payload: Vec<u8>,
}

#[pymethods]
impl PyRawMessage {
    #[new]
    #[pyo3(signature = (*, message_id, payload, system_id=1, component_id=1, sequence=0))]
    fn new(
        message_id: u32,
        payload: Vec<u8>,
        system_id: u8,
        component_id: u8,
        sequence: u8,
    ) -> PyResult<Self> {
        Self::from_parts(system_id, component_id, sequence, message_id, payload)
    }

    #[getter]
    pub(crate) fn system_id(&self) -> u8 {
        self.system_id
    }

    #[getter]
    pub(crate) fn component_id(&self) -> u8 {
        self.component_id
    }

    #[getter]
    pub(crate) fn sequence(&self) -> u8 {
        self.sequence
    }

    #[getter]
    pub(crate) fn message_id(&self) -> u32 {
        self.message_id
    }

    #[getter]
    pub(crate) fn payload(&self) -> Vec<u8> {
        self.payload.clone()
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
        let mut payload = [0_u8; 255];
        let message_id = msg.message_id();
        let message_name = msg.message_name();
        let payload_len = msg.ser(MavlinkVersion::V2, &mut payload);
        let message_json = serde_json::to_string(&msg)
            .map_err(|e| MavkitError::new_err(format!("JSON serialization error: {e}")))?;
        Ok(Self {
            system_id: header.system_id,
            component_id: header.component_id,
            sequence: header.sequence,
            message_id,
            message_name,
            message_json,
            payload: payload[..payload_len].to_vec(),
        })
    }

    pub fn from_raw_message(message: mavkit::RawMessage) -> PyResult<Self> {
        Self::from_parts(
            message.system_id,
            message.component_id,
            0,
            message.message_id,
            message.payload,
        )
    }

    pub fn to_rust(&self) -> mavkit::RawMessage {
        mavkit::RawMessage {
            message_id: self.message_id,
            system_id: self.system_id,
            component_id: self.component_id,
            payload: self.payload.clone(),
            received_at: Instant::now(),
        }
    }

    fn from_parts(
        system_id: u8,
        component_id: u8,
        sequence: u8,
        message_id: u32,
        payload: Vec<u8>,
    ) -> PyResult<Self> {
        let msg = MavMessage::parse(MavlinkVersion::V2, message_id, &payload)
            .map_err(|err| MavkitError::new_err(format!("failed to parse raw payload: {err}")))?;
        let message_name = msg.message_name();
        let message_json = serde_json::to_string(&msg)
            .map_err(|e| MavkitError::new_err(format!("JSON serialization error: {e}")))?;
        Ok(Self {
            system_id,
            component_id,
            sequence,
            message_id,
            message_name,
            message_json,
            payload,
        })
    }
}

/// Async stream of raw MAVLink messages from a Vehicle connection.
#[pyclass(name = "RawMessageStream", frozen)]
pub struct PyRawMessageStream {
    rx: Arc<tokio::sync::Mutex<mpsc::Receiver<mavkit::RawMessage>>>,
    forward_task: Mutex<Option<JoinHandle<()>>>,
}

impl PyRawMessageStream {
    pub fn from_subscription(mut subscription: RawMessageSubscription) -> Self {
        let (tx, rx) = mpsc::channel(32);
        let forward_task = pyo3_async_runtimes::tokio::get_runtime().spawn(async move {
            while let Some(message) = subscription.next().await {
                if tx.send(message).await.is_err() {
                    break;
                }
            }
        });

        Self {
            rx: Arc::new(tokio::sync::Mutex::new(rx)),
            forward_task: Mutex::new(Some(forward_task)),
        }
    }
}

impl Drop for PyRawMessageStream {
    fn drop(&mut self) {
        if let Some(handle) = self.forward_task.lock().unwrap().take() {
            handle.abort();
        }
    }
}

#[pymethods]
impl PyRawMessageStream {
    fn recv<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let rx = self.rx.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let mut guard = rx.lock().await;
            match guard.recv().await {
                Some(message) => PyRawMessage::from_raw_message(message),
                None => Err(PyStopAsyncIteration::new_err("raw message stream closed")),
            }
        })
    }

    fn __aiter__(slf: PyRef<'_, Self>) -> PyRef<'_, Self> {
        slf
    }

    fn __anext__<'py>(slf: PyRef<'py, Self>, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        slf.recv(py)
    }

    fn __repr__(&self) -> &'static str {
        "RawMessageStream()"
    }
}
