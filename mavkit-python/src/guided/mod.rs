mod copter;
mod plane;
mod rover;
mod session;
mod sub;

pub use copter::PyArduCopterGuidedHandle;
pub use plane::{PyArduPlaneGuidedHandle, PyArduPlaneVtolGuidedHandle};
pub use rover::PyArduRoverGuidedHandle;
pub use session::PyArduGuidedSession;
pub use sub::PyArduSubGuidedHandle;

fn geo_point_2d(latitude_deg: f64, longitude_deg: f64) -> mavkit::GeoPoint2d {
    mavkit::GeoPoint2d {
        latitude_deg,
        longitude_deg,
    }
}

fn geo_point_rel_home(
    latitude_deg: f64,
    longitude_deg: f64,
    relative_alt_m: f64,
) -> mavkit::GeoPoint3dRelHome {
    mavkit::GeoPoint3dRelHome {
        latitude_deg,
        longitude_deg,
        relative_alt_m,
    }
}

fn geo_point_msl(
    latitude_deg: f64,
    longitude_deg: f64,
    altitude_msl_m: f64,
) -> mavkit::GeoPoint3dMsl {
    mavkit::GeoPoint3dMsl {
        latitude_deg,
        longitude_deg,
        altitude_msl_m,
    }
}

fn relative_climb_target(relative_climb_m: f32) -> mavkit::RelativeClimbTarget {
    mavkit::RelativeClimbTarget { relative_climb_m }
}

fn sub_goto_depth_target(
    latitude_deg: f64,
    longitude_deg: f64,
    depth_m: f32,
) -> mavkit::SubGotoDepthTarget {
    mavkit::SubGotoDepthTarget {
        point: geo_point_2d(latitude_deg, longitude_deg),
        depth_m,
    }
}

fn session_closed_error() -> mavkit::VehicleError {
    mavkit::VehicleError::OperationConflict {
        conflicting_domain: "ardupilot_guided".to_string(),
        conflicting_op: "session_closed".to_string(),
    }
}

fn guided_family_unavailable_error(label: &str) -> mavkit::VehicleError {
    mavkit::VehicleError::Unsupported(format!(
        "{label} guided controls are unavailable for this session"
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavkit::dialect;
    use mavlink::{AsyncMavConnection, MavHeader, MavlinkVersion};
    use std::sync::{Arc, Mutex};
    use std::time::Duration;
    use tokio::sync::mpsc;
    use tokio::time::timeout;

    type SentMessages = Arc<Mutex<Vec<(MavHeader, dialect::MavMessage)>>>;

    struct MockConnection {
        recv_rx: tokio::sync::Mutex<mpsc::Receiver<(MavHeader, dialect::MavMessage)>>,
        sent: SentMessages,
    }

    impl MockConnection {
        fn new(rx: mpsc::Receiver<(MavHeader, dialect::MavMessage)>) -> (Self, SentMessages) {
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
                        Output = Result<
                            mavlink::MAVLinkMessageRaw,
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
                let (header, message) = self.recv().await?;
                let mut raw = mavlink::MAVLinkV2MessageRaw::new();
                raw.serialize_message(header, &message);
                Ok(mavlink::MAVLinkMessageRaw::V2(raw))
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

    fn heartbeat_msg_with_mode(mavtype: dialect::MavType, custom_mode: u32) -> dialect::MavMessage {
        dialect::MavMessage::HEARTBEAT(dialect::HEARTBEAT_DATA {
            custom_mode,
            mavtype,
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

    async fn connect_mock_vehicle(mavtype: dialect::MavType, custom_mode: u32) -> mavkit::Vehicle {
        let (msg_tx, msg_rx) = mpsc::channel(16);
        let (conn, _sent) = MockConnection::new(msg_rx);
        let connect_task = tokio::spawn(async move {
            mavkit::Vehicle::from_connection(Box::new(conn), fast_config()).await
        });

        msg_tx
            .send((
                default_header(),
                heartbeat_msg_with_mode(mavtype, custom_mode),
            ))
            .await
            .expect("heartbeat should be delivered");

        timeout(Duration::from_millis(250), connect_task)
            .await
            .expect("connect should complete")
            .expect("connect task should join")
            .expect("mock vehicle should connect")
    }

    #[tokio::test(flavor = "current_thread")]
    async fn close_is_idempotent_and_blocks_future_narrowing() {
        let vehicle = connect_mock_vehicle(dialect::MavType::MAV_TYPE_QUADROTOR, 4).await;
        let session = vehicle
            .ardupilot()
            .guided()
            .await
            .expect("guided session should acquire");
        let py_session = PyArduGuidedSession::new(session, "sys=1, comp=1".to_string());

        py_session
            .close_impl()
            .await
            .expect("first close should succeed");
        py_session
            .close_impl()
            .await
            .expect("second close should also succeed");

        assert!(matches!(
            py_session.copter_impl(),
            Err(mavkit::VehicleError::OperationConflict {
                conflicting_domain,
                conflicting_op,
            }) if conflicting_domain == "ardupilot_guided" && conflicting_op == "session_closed"
        ));
    }

    #[tokio::test(flavor = "current_thread")]
    async fn retained_guided_handle_operations_fail_after_python_close() {
        let vehicle = connect_mock_vehicle(dialect::MavType::MAV_TYPE_GROUND_ROVER, 15).await;
        let session = vehicle
            .ardupilot()
            .guided()
            .await
            .expect("guided session should acquire");
        let py_session = PyArduGuidedSession::new(session, "sys=1, comp=1".to_string());
        let rover = py_session
            .rover_impl()
            .expect("rover narrowing should succeed before close")
            .expect("rover session should expose rover handle");

        py_session.close_impl().await.expect("close should succeed");

        assert!(matches!(
            rover.drive_impl(1.0, 0.0).await,
            Err(mavkit::VehicleError::OperationConflict {
                conflicting_domain,
                conflicting_op,
            }) if conflicting_domain == "ardupilot_guided" && conflicting_op == "session_closed"
        ));
    }

    #[tokio::test(flavor = "current_thread")]
    async fn wrong_family_narrowing_returns_none_for_other_families() {
        let vehicle = connect_mock_vehicle(dialect::MavType::MAV_TYPE_QUADROTOR, 4).await;
        let session = vehicle
            .ardupilot()
            .guided()
            .await
            .expect("guided session should acquire");
        let py_session = PyArduGuidedSession::new(session, "sys=1, comp=1".to_string());

        assert!(
            py_session
                .plane_impl()
                .expect("plane narrowing should not error while open")
                .is_none()
        );
        assert!(
            py_session
                .rover_impl()
                .expect("rover narrowing should not error while open")
                .is_none()
        );
        assert!(
            py_session
                .sub_impl()
                .expect("sub narrowing should not error while open")
                .is_none()
        );
    }

    #[tokio::test(flavor = "current_thread")]
    async fn plane_vtol_narrowing_tracks_plane_kind() {
        let fixed_wing_vehicle =
            connect_mock_vehicle(dialect::MavType::MAV_TYPE_FIXED_WING, 15).await;
        let fixed_wing_session = fixed_wing_vehicle
            .ardupilot()
            .guided()
            .await
            .expect("fixed-wing guided session should acquire");
        let fixed_wing_py_session =
            PyArduGuidedSession::new(fixed_wing_session, "sys=1, comp=1".to_string());
        let fixed_wing_plane = fixed_wing_py_session
            .plane_impl()
            .expect("plane narrowing should succeed")
            .expect("fixed-wing session should expose plane handle");
        assert!(
            fixed_wing_plane
                .vtol_impl()
                .expect("fixed-wing VTOL narrowing should not error")
                .is_none()
        );

        let vtol_vehicle =
            connect_mock_vehicle(dialect::MavType::MAV_TYPE_VTOL_FIXEDROTOR, 15).await;
        let vtol_session = vtol_vehicle
            .ardupilot()
            .guided()
            .await
            .expect("vtol plane guided session should acquire");
        let vtol_py_session = PyArduGuidedSession::new(vtol_session, "sys=1, comp=1".to_string());
        let vtol_plane = vtol_py_session
            .plane_impl()
            .expect("plane narrowing should succeed")
            .expect("vtol session should expose plane handle");
        assert!(
            vtol_plane
                .vtol_impl()
                .expect("vtol narrowing should not error")
                .is_some()
        );
    }
}
