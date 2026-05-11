#![cfg(target_arch = "wasm32")]

use mavkit::byte_connection::ByteConnectionConfig;
use mavkit::{AutopilotType, Vehicle, VehicleConfig, VehicleIdentity, VehicleType, dialect};
use mavlink::{MavHeader, MavlinkVersion, write_versioned_msg};
use wasm_bindgen_test::wasm_bindgen_test;

wasm_bindgen_test::wasm_bindgen_test_configure!(run_in_browser);

fn fast_config() -> VehicleConfig {
    VehicleConfig {
        connect_timeout: std::time::Duration::from_millis(150),
        command_timeout: std::time::Duration::from_millis(50),
        command_completion_timeout: std::time::Duration::from_millis(150),
        auto_request_home: false,
        ..VehicleConfig::default()
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
        custom_mode: 5,
        mavtype: dialect::MavType::MAV_TYPE_QUADROTOR,
        autopilot: dialect::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode: dialect::MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            | dialect::MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED,
        system_status: dialect::MavState::MAV_STATE_STANDBY,
        mavlink_version: 3,
    })
}

fn encode_frame(header: MavHeader, message: &dialect::MavMessage) -> Vec<u8> {
    let mut bytes = Vec::new();
    write_versioned_msg(&mut bytes, MavlinkVersion::V2, header, message)
        .expect("frame should encode");
    bytes
}

#[wasm_bindgen_test]
async fn from_byte_connection_connects_after_heartbeat_via_bridge() {
    let (bridge, vehicle_future) =
        Vehicle::from_byte_connection(fast_config(), ByteConnectionConfig::default());

    bridge
        .push_inbound(encode_frame(default_header(), &heartbeat_msg()))
        .await
        .expect("heartbeat should be delivered through the bridge");

    let vehicle = vehicle_future
        .await
        .expect("vehicle should connect after first heartbeat");

    assert_eq!(
        vehicle.identity(),
        VehicleIdentity {
            system_id: 1,
            component_id: 1,
            autopilot: AutopilotType::ArduPilotMega,
            vehicle_type: VehicleType::Quadrotor,
        }
    );
}
