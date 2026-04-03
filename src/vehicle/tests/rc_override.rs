use super::support::*;
use crate::VehicleError;
use crate::vehicle::{RcOverride, RcOverrideChannelValue};
use mavlink::MavHeader;

#[tokio::test]
async fn rc_override_sends_expected_override_message() {
    let target_header = MavHeader {
        system_id: 42,
        component_id: 17,
        sequence: 9,
    };
    let (vehicle, msg_tx, sent) = connect_mock_vehicle_with_header_and_sent(target_header).await;
    let mut overrides = RcOverride::new();
    overrides
        .set_pwm(1, 1001)
        .expect("channel 1 should accept a PWM override")
        .set_pwm(2, 1002)
        .expect("channel 2 should accept a PWM override")
        .set_pwm(3, 1003)
        .expect("channel 3 should accept a PWM override")
        .set_pwm(4, 1004)
        .expect("channel 4 should accept a PWM override")
        .set_pwm(5, 1005)
        .expect("channel 5 should accept a PWM override")
        .set_pwm(6, 1006)
        .expect("channel 6 should accept a PWM override")
        .release(7)
        .expect("channel 7 should encode an explicit release")
        .ignore(8)
        .expect("channel 8 should encode an explicit ignore")
        .set_pwm(9, 1009)
        .expect("channel 9 should accept a PWM override")
        .set_pwm(10, 1010)
        .expect("channel 10 should accept a PWM override")
        .set_pwm(11, 1011)
        .expect("channel 11 should accept a PWM override")
        .set_pwm(12, 1012)
        .expect("channel 12 should accept a PWM override")
        .set_pwm(13, 1013)
        .expect("channel 13 should accept a PWM override")
        .set_pwm(14, 1014)
        .expect("channel 14 should accept a PWM override")
        .set_pwm(15, 1015)
        .expect("channel 15 should accept a PWM override")
        .set_pwm(16, 1016)
        .expect("channel 16 should accept a PWM override")
        .set_pwm(17, 1017)
        .expect("channel 17 should accept a PWM override")
        .set_pwm(18, 1018)
        .expect("channel 18 should accept a PWM override");

    vehicle
        .rc_override(overrides)
        .await
        .expect("rc_override should enqueue one raw send without waiting for an ACK");

    let sent_override = sent
        .lock()
        .expect("sent messages lock should not poison")
        .iter()
        .find_map(|(_, msg)| match msg {
            crate::dialect::MavMessage::RC_CHANNELS_OVERRIDE(data) => Some(data.clone()),
            _ => None,
        })
        .expect("rc override message should be sent");

    assert_eq!(sent_override.target_system, 42);
    assert_eq!(sent_override.target_component, 17);
    assert_eq!(sent_override.chan1_raw, 1001);
    assert_eq!(sent_override.chan2_raw, 1002);
    assert_eq!(sent_override.chan3_raw, 1003);
    assert_eq!(sent_override.chan4_raw, 1004);
    assert_eq!(sent_override.chan5_raw, 1005);
    assert_eq!(sent_override.chan6_raw, 1006);
    assert_eq!(sent_override.chan7_raw, 0);
    assert_eq!(sent_override.chan8_raw, u16::MAX);
    assert_eq!(sent_override.chan9_raw, 1009);
    assert_eq!(sent_override.chan10_raw, 1010);
    assert_eq!(sent_override.chan11_raw, 1011);
    assert_eq!(sent_override.chan12_raw, 1012);
    assert_eq!(sent_override.chan13_raw, 1013);
    assert_eq!(sent_override.chan14_raw, 1014);
    assert_eq!(sent_override.chan15_raw, 1015);
    assert_eq!(sent_override.chan16_raw, 1016);
    assert_eq!(sent_override.chan17_raw, 1017);
    assert_eq!(sent_override.chan18_raw, 1018);

    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
    drop(msg_tx);
}

#[tokio::test]
async fn rc_override_defaults_unspecified_channels_to_ignore() {
    let (vehicle, msg_tx, sent) = connect_mock_vehicle_with_sent().await;
    let mut overrides = RcOverride::new();
    overrides
        .set_pwm(1, 1500)
        .expect("channel 1 should accept a PWM override")
        .release(18)
        .expect("channel 18 should encode an explicit release");

    assert_eq!(
        overrides.channel(1).expect("channel 1 should exist"),
        RcOverrideChannelValue::Pwm(1500)
    );
    assert_eq!(
        overrides.channel(18).expect("channel 18 should exist"),
        RcOverrideChannelValue::Release
    );
    assert_eq!(
        overrides.channel(2).expect("channel 2 should exist"),
        RcOverrideChannelValue::Ignore
    );

    vehicle
        .rc_override(overrides)
        .await
        .expect("rc_override should send the frame");

    let sent_override = sent
        .lock()
        .expect("sent messages lock should not poison")
        .iter()
        .find_map(|(_, msg)| match msg {
            crate::dialect::MavMessage::RC_CHANNELS_OVERRIDE(data) => Some(data.clone()),
            _ => None,
        })
        .expect("rc override message should be sent");

    assert_eq!(sent_override.chan1_raw, 1500);
    assert_eq!(sent_override.chan18_raw, 0);
    assert_eq!(sent_override.chan2_raw, u16::MAX);
    assert_eq!(sent_override.chan3_raw, u16::MAX);
    assert_eq!(sent_override.chan4_raw, u16::MAX);
    assert_eq!(sent_override.chan5_raw, u16::MAX);
    assert_eq!(sent_override.chan6_raw, u16::MAX);
    assert_eq!(sent_override.chan7_raw, u16::MAX);
    assert_eq!(sent_override.chan8_raw, u16::MAX);
    assert_eq!(sent_override.chan9_raw, u16::MAX);
    assert_eq!(sent_override.chan10_raw, u16::MAX);
    assert_eq!(sent_override.chan11_raw, u16::MAX);
    assert_eq!(sent_override.chan12_raw, u16::MAX);
    assert_eq!(sent_override.chan13_raw, u16::MAX);
    assert_eq!(sent_override.chan14_raw, u16::MAX);
    assert_eq!(sent_override.chan15_raw, u16::MAX);
    assert_eq!(sent_override.chan16_raw, u16::MAX);
    assert_eq!(sent_override.chan17_raw, u16::MAX);

    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
    drop(msg_tx);
}

#[test]
fn rc_override_rejects_invalid_channels_and_reserved_pwm_sentinels() {
    let mut overrides = RcOverride::new();

    assert!(matches!(
        overrides.set_pwm(0, 1500),
        Err(VehicleError::InvalidParameter(message))
            if message == "rc override channel must be 1..=18, got 0"
    ));
    assert!(matches!(
        overrides.release(19),
        Err(VehicleError::InvalidParameter(message))
            if message == "rc override channel must be 1..=18, got 19"
    ));
    assert!(matches!(
        RcOverrideChannelValue::pwm(0),
        Err(VehicleError::InvalidParameter(message))
            if message.contains("reserved for release")
    ));
    assert!(matches!(
        RcOverrideChannelValue::pwm(u16::MAX),
        Err(VehicleError::InvalidParameter(message))
            if message.contains("reserved for ignore")
    ));
}

#[tokio::test]
async fn rc_override_surfaces_disconnected_send_failures() {
    let (vehicle, command_rx) = test_vehicle_with_command_rx();
    drop(command_rx);

    let mut overrides = RcOverride::new();
    overrides
        .set_pwm(1, 1500)
        .expect("channel 1 should accept a PWM override");

    assert!(matches!(
        vehicle.rc_override(overrides).await,
        Err(VehicleError::Disconnected)
    ));
}
