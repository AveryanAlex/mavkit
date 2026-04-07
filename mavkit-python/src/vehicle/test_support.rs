use mavkit::dialect;
use mavlink::{
    AsyncMavConnection, MAVLinkMessageRaw, MAVLinkV2MessageRaw, MavHeader, MavlinkVersion,
};
use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use std::sync::Arc;
use std::sync::Mutex;
use std::time::Duration;
use tokio::sync::mpsc;
use tokio::time::timeout;

use crate::error::to_py_err;
use crate::mission::PyMissionPlan;
use crate::vehicle::api::PyVehicle;

type TestSentMessages = Arc<Mutex<Vec<(MavHeader, dialect::MavMessage)>>>;

struct MockConnection {
    recv_rx: tokio::sync::Mutex<mpsc::Receiver<(MavHeader, dialect::MavMessage)>>,
    sent: TestSentMessages,
}

impl MockConnection {
    fn new(rx: mpsc::Receiver<(MavHeader, dialect::MavMessage)>) -> (Self, TestSentMessages) {
        let sent = Arc::new(Mutex::new(Vec::new()));
        (
            Self {
                recv_rx: tokio::sync::Mutex::new(rx),
                sent: sent.clone(),
            },
            sent,
        )
    }
}

impl AsyncMavConnection<dialect::MavMessage> for MockConnection {
    fn recv<'life0, 'async_trait>(
        &'life0 self,
    ) -> std::pin::Pin<
        Box<
            dyn std::future::Future<
                    Output = Result<
                        (MavHeader, dialect::MavMessage),
                        mavlink::error::MessageReadError,
                    >,
                > + Send
                + 'async_trait,
        >,
    >
    where
        'life0: 'async_trait,
        Self: 'async_trait,
    {
        Box::pin(async move {
            let mut rx = self.recv_rx.lock().await;
            match rx.recv().await {
                Some(message) => Ok(message),
                None => Err(mavlink::error::MessageReadError::Io(std::io::Error::new(
                    std::io::ErrorKind::ConnectionReset,
                    "mock connection closed",
                ))),
            }
        })
    }

    fn recv_raw<'life0, 'async_trait>(
        &'life0 self,
    ) -> std::pin::Pin<
        Box<
            dyn std::future::Future<
                    Output = Result<mavlink::MAVLinkMessageRaw, mavlink::error::MessageReadError>,
                > + Send
                + 'async_trait,
        >,
    >
    where
        'life0: 'async_trait,
        Self: 'async_trait,
    {
        Box::pin(async move {
            let (header, message) = self.recv().await?;
            let mut raw = MAVLinkV2MessageRaw::new();
            raw.serialize_message(header, &message);
            Ok(MAVLinkMessageRaw::V2(raw))
        })
    }

    fn send<'life0, 'life1, 'life2, 'async_trait>(
        &'life0 self,
        header: &'life1 MavHeader,
        data: &'life2 dialect::MavMessage,
    ) -> std::pin::Pin<
        Box<
            dyn std::future::Future<Output = Result<usize, mavlink::error::MessageWriteError>>
                + Send
                + 'async_trait,
        >,
    >
    where
        'life0: 'async_trait,
        'life1: 'async_trait,
        'life2: 'async_trait,
        Self: 'async_trait,
    {
        let header = *header;
        let data = data.clone();
        Box::pin(async move {
            self.sent.lock().unwrap().push((header, data));
            Ok(0)
        })
    }

    fn set_protocol_version(&mut self, _version: MavlinkVersion) {}

    fn protocol_version(&self) -> MavlinkVersion {
        MavlinkVersion::V2
    }

    fn set_allow_recv_any_version(&mut self, _allow: bool) {}

    fn allow_recv_any_version(&self) -> bool {
        true
    }
}

fn default_header() -> MavHeader {
    MavHeader {
        system_id: 1,
        component_id: 1,
        sequence: 0,
    }
}

fn heartbeat_msg() -> dialect::MavMessage {
    dialect::MavMessage::HEARTBEAT(dialect::HEARTBEAT_DATA {
        custom_mode: 7,
        mavtype: dialect::MavType::MAV_TYPE_QUADROTOR,
        autopilot: dialect::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode: dialect::MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        system_status: dialect::MavState::MAV_STATE_STANDBY,
        mavlink_version: 3,
    })
}

fn fast_config() -> mavkit::VehicleConfig {
    mavkit::VehicleConfig {
        connect_timeout: Duration::from_millis(150),
        command_timeout: Duration::from_millis(50),
        command_completion_timeout: Duration::from_millis(150),
        auto_request_home: false,
        ..mavkit::VehicleConfig::default()
    }
}

fn command_ack_accepted_msg(command: u16) -> PyResult<dialect::MavMessage> {
    let command = match command {
        400 => dialect::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
        511 => dialect::MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
        512 => dialect::MavCmd::MAV_CMD_REQUEST_MESSAGE,
        other => {
            return Err(PyValueError::new_err(format!(
                "unsupported test command ack value {other}",
            )));
        }
    };

    Ok(dialect::MavMessage::COMMAND_ACK(
        dialect::COMMAND_ACK_DATA {
            command,
            result: dialect::MavResult::MAV_RESULT_ACCEPTED,
            progress: 0,
            result_param2: 0,
            target_system: 0,
            target_component: 0,
        },
    ))
}

fn mission_type_msg(mission_type: mavkit::MissionType) -> dialect::MavMissionType {
    match mission_type {
        mavkit::MissionType::Mission => dialect::MavMissionType::MAV_MISSION_TYPE_MISSION,
        mavkit::MissionType::Fence => dialect::MavMissionType::MAV_MISSION_TYPE_FENCE,
        mavkit::MissionType::Rally => dialect::MavMissionType::MAV_MISSION_TYPE_RALLY,
    }
}

fn mission_frame_msg(frame: mavkit::mission::commands::MissionFrame) -> dialect::MavFrame {
    match frame {
        mavkit::mission::commands::MissionFrame::Mission => dialect::MavFrame::MAV_FRAME_MISSION,
        mavkit::mission::commands::MissionFrame::Global => dialect::MavFrame::MAV_FRAME_GLOBAL,
        mavkit::mission::commands::MissionFrame::GlobalRelativeAlt => {
            dialect::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT
        }
        mavkit::mission::commands::MissionFrame::GlobalTerrainAlt => {
            dialect::MavFrame::MAV_FRAME_GLOBAL_TERRAIN_ALT
        }
        mavkit::mission::commands::MissionFrame::Other(_) => dialect::MavFrame::MAV_FRAME_MISSION,
    }
}

fn mission_request_int_msg(seq: u16, mission_type: mavkit::MissionType) -> dialect::MavMessage {
    dialect::MavMessage::MISSION_REQUEST_INT(dialect::MISSION_REQUEST_INT_DATA {
        seq,
        target_system: 0,
        target_component: 0,
        mission_type: mission_type_msg(mission_type),
    })
}

fn mission_count_msg(count: u16, mission_type: mavkit::MissionType) -> dialect::MavMessage {
    dialect::MavMessage::MISSION_COUNT(dialect::MISSION_COUNT_DATA {
        target_system: 0,
        target_component: 0,
        count,
        mission_type: mission_type_msg(mission_type),
        opaque_id: 0,
    })
}

fn mission_ack_accepted_msg_for_type(mission_type: mavkit::MissionType) -> dialect::MavMessage {
    dialect::MavMessage::MISSION_ACK(dialect::MISSION_ACK_DATA {
        target_system: 0,
        target_component: 0,
        mavtype: dialect::MavMissionResult::MAV_MISSION_ACCEPTED,
        mission_type: mission_type_msg(mission_type),
        opaque_id: 0,
    })
}

fn mission_item_int_msg(
    item: &mavkit::MissionItem,
    seq: u16,
    mission_type: mavkit::MissionType,
) -> PyResult<dialect::MavMessage> {
    let (command, frame, params, x, y, z) = item.command.clone().into_wire();
    let command = num_traits::FromPrimitive::from_u16(command).ok_or_else(|| {
        PyValueError::new_err(format!(
            "unsupported test mission command value {}",
            command
        ))
    })?;

    Ok(dialect::MavMessage::MISSION_ITEM_INT(
        dialect::MISSION_ITEM_INT_DATA {
            param1: params[0],
            param2: params[1],
            param3: params[2],
            param4: params[3],
            x,
            y,
            z,
            seq,
            command,
            target_system: 0,
            target_component: 0,
            frame: mission_frame_msg(frame),
            current: 0,
            autocontinue: u8::from(item.autocontinue),
            mission_type: mission_type_msg(mission_type),
        },
    ))
}

fn global_position_int_msg() -> dialect::MavMessage {
    dialect::MavMessage::GLOBAL_POSITION_INT(dialect::GLOBAL_POSITION_INT_DATA {
        time_boot_ms: 42,
        lat: 473_977_420,
        lon: 85_455_940,
        alt: 500_000,
        relative_alt: 15_000,
        vx: 0,
        vy: 0,
        vz: 0,
        hdg: 9_000,
    })
}

#[pyclass(name = "_TestVehicleHarness", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyTestVehicleHarness {
    msg_tx: mpsc::Sender<(MavHeader, dialect::MavMessage)>,
    sent: TestSentMessages,
}

#[pymethods]
impl PyTestVehicleHarness {
    fn push_command_ack_accepted<'py>(
        &self,
        py: Python<'py>,
        command: u16,
    ) -> PyResult<Bound<'py, PyAny>> {
        let msg_tx = self.msg_tx.clone();
        let message = command_ack_accepted_msg(command)?;
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            msg_tx
                .send((default_header(), message))
                .await
                .map_err(|_| PyRuntimeError::new_err("mock vehicle channel closed"))?;
            Ok(())
        })
    }

    fn push_global_position_int<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let msg_tx = self.msg_tx.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            msg_tx
                .send((default_header(), global_position_int_msg()))
                .await
                .map_err(|_| PyRuntimeError::new_err("mock vehicle channel closed"))?;
            Ok(())
        })
    }

    fn push_mission_upload_success<'py>(
        &self,
        py: Python<'py>,
        plan: &PyMissionPlan,
    ) -> PyResult<Bound<'py, PyAny>> {
        let msg_tx = self.msg_tx.clone();
        let mission_type = mavkit::MissionType::Mission;
        let wire_items = mavkit::mission_items_for_upload(&plan.inner);
        let mut responses = Vec::with_capacity(wire_items.len() + 1);
        for seq in 0..wire_items.len() {
            responses.push(mission_request_int_msg(seq as u16, mission_type));
        }
        responses.push(mission_ack_accepted_msg_for_type(mission_type));

        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            for message in responses {
                msg_tx
                    .send((default_header(), message))
                    .await
                    .map_err(|_| PyRuntimeError::new_err("mock vehicle channel closed"))?;
            }
            Ok(())
        })
    }

    fn push_mission_download_success<'py>(
        &self,
        py: Python<'py>,
        plan: &PyMissionPlan,
    ) -> PyResult<Bound<'py, PyAny>> {
        let msg_tx = self.msg_tx.clone();
        let mission_type = mavkit::MissionType::Mission;
        let wire_items = mavkit::mission_items_for_upload(&plan.inner);
        let wire_item_count: u16 = wire_items.len().try_into().map_err(|_| {
            PyValueError::new_err("plan has too many items for MAVLink mission count")
        })?;
        let mut responses = Vec::with_capacity(wire_items.len() + 1);
        responses.push(mission_count_msg(wire_item_count, mission_type));
        for (seq, item) in wire_items.iter().enumerate() {
            responses.push(mission_item_int_msg(item, seq as u16, mission_type)?);
        }

        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            for message in responses {
                msg_tx
                    .send((default_header(), message))
                    .await
                    .map_err(|_| PyRuntimeError::new_err("mock vehicle channel closed"))?;
            }
            Ok(())
        })
    }

    fn sent_message_count(&self) -> usize {
        self.sent.lock().unwrap().len()
    }
}

#[pyfunction]
pub fn _connect_test_vehicle<'py>(py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
    let (msg_tx, msg_rx) = mpsc::channel(16);
    let (conn, sent) = MockConnection::new(msg_rx);
    pyo3_async_runtimes::tokio::future_into_py(py, async move {
        let connect_task = tokio::spawn(async move {
            mavkit::Vehicle::from_connection(Box::new(conn), fast_config()).await
        });

        msg_tx
            .send((default_header(), heartbeat_msg()))
            .await
            .map_err(|_| PyRuntimeError::new_err("mock heartbeat should be delivered"))?;

        let vehicle = timeout(Duration::from_millis(250), connect_task)
            .await
            .map_err(|_| PyRuntimeError::new_err("mock vehicle connect timed out"))?
            .map_err(|_| PyRuntimeError::new_err("mock vehicle connect task failed"))?
            .map_err(to_py_err)?;

        Ok((
            PyVehicle::from_inner(vehicle),
            PyTestVehicleHarness { msg_tx, sent },
        ))
    })
}
