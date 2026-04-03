use super::super::*;
use super::core::{assert_roundtrip, geo_msl, geo_rel_home, geo_terrain};

#[test]
fn nav_waypoint_roundtrip() {
    let original = MissionCommand::from(NavWaypoint {
        position: geo_rel_home(473_977_420, 85_455_970, 42.0),
        hold_time_s: 5.0,
        acceptance_radius_m: 2.0,
        pass_radius_m: 0.5,
        yaw_deg: 180.0,
    });

    assert_roundtrip(
        original,
        (
            u16::from(MAV_CMD_NAV_WAYPOINT),
            MissionFrame::GlobalRelativeAlt,
            [5.0, 2.0, 0.5, 180.0],
            473_977_420,
            85_455_970,
            42.0,
        ),
    );
}

#[test]
fn all_nav_roundtrip() {
    let cases = [
        (
            MissionCommand::from(NavWaypoint {
                position: geo_rel_home(473_977_420, 85_455_970, 42.0),
                hold_time_s: 5.0,
                acceptance_radius_m: 2.0,
                pass_radius_m: 0.5,
                yaw_deg: 180.0,
            }),
            (
                u16::from(MAV_CMD_NAV_WAYPOINT),
                MissionFrame::GlobalRelativeAlt,
                [5.0, 2.0, 0.5, 180.0],
                473_977_420,
                85_455_970,
                42.0,
            ),
        ),
        (
            MissionCommand::from(NavSplineWaypoint {
                position: geo_msl(473_977_421, 85_455_971, 120.0),
                hold_time_s: 7.0,
            }),
            (
                u16::from(MAV_CMD_NAV_SPLINE_WAYPOINT),
                MissionFrame::Global,
                [7.0, 0.0, 0.0, 0.0],
                473_977_421,
                85_455_971,
                120.0,
            ),
        ),
        (
            MissionCommand::from(NavArcWaypoint {
                position: geo_terrain(473_977_422, 85_455_972, 18.0),
                arc_angle_deg: 90.0,
                direction: LoiterDirection::CounterClockwise,
            }),
            (
                u16::from(MAV_CMD_NAV_ARC_WAYPOINT),
                MissionFrame::GlobalTerrainAlt,
                [-90.0, 0.0, 0.0, 0.0],
                473_977_422,
                85_455_972,
                18.0,
            ),
        ),
        (
            MissionCommand::from(NavTakeoff {
                position: geo_terrain(473_977_423, 85_455_973, 60.0),
                pitch_deg: 12.0,
            }),
            (
                u16::from(MAV_CMD_NAV_TAKEOFF),
                MissionFrame::GlobalTerrainAlt,
                [12.0, 0.0, 0.0, 0.0],
                473_977_423,
                85_455_973,
                60.0,
            ),
        ),
        (
            MissionCommand::from(NavLand {
                position: geo_msl(473_977_424, 85_455_974, 10.0),
                abort_alt_m: 25.0,
            }),
            (
                u16::from(MAV_CMD_NAV_LAND),
                MissionFrame::Global,
                [25.0, 0.0, 0.0, 0.0],
                473_977_424,
                85_455_974,
                10.0,
            ),
        ),
        (
            MissionCommand::from(NavCommand::ReturnToLaunch),
            (
                u16::from(MAV_CMD_NAV_RETURN_TO_LAUNCH),
                MissionFrame::Mission,
                [0.0, 0.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(NavLoiterUnlimited {
                position: geo_rel_home(473_977_425, 85_455_975, 50.0),
                radius_m: 35.0,
                direction: LoiterDirection::CounterClockwise,
            }),
            (
                u16::from(MAV_CMD_NAV_LOITER_UNLIMITED),
                MissionFrame::GlobalRelativeAlt,
                [0.0, 0.0, -35.0, 0.0],
                473_977_425,
                85_455_975,
                50.0,
            ),
        ),
        (
            MissionCommand::from(NavLoiterTurns {
                position: geo_rel_home(473_977_426, 85_455_976, 55.0),
                turns: 2.5,
                radius_m: 20.0,
                direction: LoiterDirection::Clockwise,
                exit_xtrack: true,
            }),
            (
                u16::from(MAV_CMD_NAV_LOITER_TURNS),
                MissionFrame::GlobalRelativeAlt,
                [2.5, 0.0, 20.0, 1.0],
                473_977_426,
                85_455_976,
                55.0,
            ),
        ),
        (
            MissionCommand::from(NavLoiterTime {
                position: geo_msl(473_977_427, 85_455_977, 65.0),
                time_s: 15.0,
                direction: LoiterDirection::Clockwise,
                exit_xtrack: false,
            }),
            (
                u16::from(MAV_CMD_NAV_LOITER_TIME),
                MissionFrame::Global,
                [15.0, 0.0, 0.0, 0.0],
                473_977_427,
                85_455_977,
                65.0,
            ),
        ),
        (
            MissionCommand::from(NavLoiterToAlt {
                position: geo_terrain(473_977_428, 85_455_978, 75.0),
                radius_m: 40.0,
                direction: LoiterDirection::CounterClockwise,
                exit_xtrack: true,
            }),
            (
                u16::from(MAV_CMD_NAV_LOITER_TO_ALT),
                MissionFrame::GlobalTerrainAlt,
                [0.0, -40.0, 0.0, 1.0],
                473_977_428,
                85_455_978,
                75.0,
            ),
        ),
        (
            MissionCommand::from(NavContinueAndChangeAlt {
                position: geo_terrain(473_977_429, 85_455_979, 80.0),
                action: AltChangeAction::Descend,
            }),
            (
                u16::from(MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT),
                MissionFrame::GlobalTerrainAlt,
                [2.0, 0.0, 0.0, 0.0],
                473_977_429,
                85_455_979,
                80.0,
            ),
        ),
        (
            MissionCommand::from(NavDelay {
                seconds: -1.0,
                hour_utc: 6.0,
                min_utc: 7.0,
                sec_utc: 8.0,
            }),
            (
                u16::from(MAV_CMD_NAV_DELAY),
                MissionFrame::Mission,
                [-1.0, 6.0, 7.0, 8.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(NavGuidedEnable { enabled: true }),
            (
                u16::from(MAV_CMD_NAV_GUIDED_ENABLE),
                MissionFrame::Mission,
                [1.0, 0.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(NavAltitudeWait {
                altitude_m: 1500.0,
                descent_rate_mps: -4.0,
                wiggle_time_s: 3.0,
            }),
            (
                u16::from(MAV_CMD_NAV_ALTITUDE_WAIT),
                MissionFrame::Mission,
                [1500.0, -4.0, 3.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(NavVtolTakeoff {
                position: geo_rel_home(473_977_430, 85_455_980, 90.0),
            }),
            (
                u16::from(MAV_CMD_NAV_VTOL_TAKEOFF),
                MissionFrame::GlobalRelativeAlt,
                [0.0, 0.0, 0.0, 0.0],
                473_977_430,
                85_455_980,
                90.0,
            ),
        ),
        (
            MissionCommand::from(NavVtolLand {
                position: geo_msl(473_977_431, 85_455_981, 8.0),
                options: 2,
            }),
            (
                u16::from(MAV_CMD_NAV_VTOL_LAND),
                MissionFrame::Global,
                [2.0, 0.0, 0.0, 0.0],
                473_977_431,
                85_455_981,
                8.0,
            ),
        ),
        (
            MissionCommand::from(NavPayloadPlace {
                position: geo_terrain(473_977_432, 85_455_982, 12.0),
                max_descent_m: 6.0,
            }),
            (
                u16::from(MAV_CMD_NAV_PAYLOAD_PLACE),
                MissionFrame::GlobalTerrainAlt,
                [6.0, 0.0, 0.0, 0.0],
                473_977_432,
                85_455_982,
                12.0,
            ),
        ),
        (
            MissionCommand::from(NavSetYawSpeed {
                angle_deg: 45.0,
                speed_mps: 3.5,
                relative: true,
            }),
            (
                u16::from(MAV_CMD_NAV_SET_YAW_SPEED),
                MissionFrame::Mission,
                [45.0, 3.5, 1.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(NavScriptTime {
                command: 9,
                timeout_s: 11.0,
                arg1: 1.5,
                arg2: -2.5,
                arg3: 17,
                arg4: -29,
            }),
            (
                u16::from(MAV_CMD_NAV_SCRIPT_TIME),
                MissionFrame::Mission,
                [9.0, 11.0, 1.5, -2.5],
                17,
                -29,
                0.0,
            ),
        ),
        (
            MissionCommand::from(NavAttitudeTime {
                time_s: 12.0,
                roll_deg: 15.0,
                pitch_deg: -5.0,
                yaw_deg: 270.0,
                climb_rate_mps: 3.0,
            }),
            (
                u16::from(MAV_CMD_NAV_ATTITUDE_TIME),
                MissionFrame::Mission,
                [12.0, 15.0, -5.0, 270.0],
                3,
                0,
                0.0,
            ),
        ),
    ];

    for (original, expected_wire) in cases {
        assert_roundtrip(original, expected_wire);
    }
}
