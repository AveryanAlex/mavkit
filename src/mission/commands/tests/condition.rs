use super::super::*;
use super::core::assert_roundtrip;

#[test]
fn all_cond_roundtrip() {
    let cases = [
        (
            MissionCommand::from(CondDelay { delay_s: 8.5 }),
            (
                u16::from(MAV_CMD_CONDITION_DELAY),
                MissionFrame::Mission,
                [8.5, 0.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(CondDistance { distance_m: 125.0 }),
            (
                u16::from(MAV_CMD_CONDITION_DISTANCE),
                MissionFrame::Mission,
                [125.0, 0.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(CondYaw {
                angle_deg: 135.0,
                turn_rate_dps: 20.0,
                direction: YawDirection::CounterClockwise,
                relative: true,
            }),
            (
                u16::from(MAV_CMD_CONDITION_YAW),
                MissionFrame::Mission,
                [135.0, 20.0, -1.0, 1.0],
                0,
                0,
                0.0,
            ),
        ),
    ];

    for (original, expected_wire) in cases {
        assert_roundtrip(original, expected_wire);
    }
}
