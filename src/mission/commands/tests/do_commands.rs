use super::super::*;
use super::core::{assert_roundtrip, geo_msl, geo_rel_home, geo_terrain};

#[test]
fn do_change_speed_roundtrip() {
    let original = MissionCommand::from(DoChangeSpeed {
        speed_type: SpeedType::Groundspeed,
        speed_mps: 5.5,
        throttle_pct: 66.0,
    });

    assert_roundtrip(
        original,
        (
            u16::from(MAV_CMD_DO_CHANGE_SPEED),
            MissionFrame::Mission,
            [1.0, 5.5, 66.0, 0.0],
            0,
            0,
            0.0,
        ),
    );
}

#[test]
fn do_set_roi_none_roundtrip() {
    assert_roundtrip(
        MissionCommand::from(DoCommand::SetRoiNone),
        (
            u16::from(MAV_CMD_DO_SET_ROI_NONE),
            MissionFrame::Mission,
            [0.0, 0.0, 0.0, 0.0],
            0,
            0,
            0.0,
        ),
    );
}

#[test]
fn do_set_home_roundtrip_uses_geopoint_frame() {
    let original = MissionCommand::from(DoSetHome {
        position: geo_terrain(473_977_520, 85_456_070, 21.5),
        use_current: false,
    });

    assert_roundtrip(
        original,
        (
            u16::from(MAV_CMD_DO_SET_HOME),
            MissionFrame::GlobalTerrainAlt,
            [0.0, 0.0, 0.0, 0.0],
            473_977_520,
            85_456_070,
            21.5,
        ),
    );
}

#[test]
fn all_do_roundtrip() {
    let cases = [
        (
            MissionCommand::from(DoJump {
                target_index: 12,
                repeat_count: 3,
            }),
            (
                u16::from(MAV_CMD_DO_JUMP),
                MissionFrame::Mission,
                [12.0, 3.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoJumpTag {
                tag: 44,
                repeat_count: 5,
            }),
            (
                u16::from(MAV_CMD_DO_JUMP_TAG),
                MissionFrame::Mission,
                [44.0, 5.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoTag { tag: 44 }),
            (
                u16::from(MAV_CMD_JUMP_TAG),
                MissionFrame::Mission,
                [44.0, 0.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoPauseContinue { pause: true }),
            (
                u16::from(MAV_CMD_DO_PAUSE_CONTINUE),
                MissionFrame::Mission,
                [0.0, 0.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoChangeSpeed {
                speed_type: SpeedType::Groundspeed,
                speed_mps: 5.5,
                throttle_pct: 66.0,
            }),
            (
                u16::from(MAV_CMD_DO_CHANGE_SPEED),
                MissionFrame::Mission,
                [1.0, 5.5, 66.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoSetReverse { reverse: true }),
            (
                u16::from(MAV_CMD_DO_SET_REVERSE),
                MissionFrame::Mission,
                [1.0, 0.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoSetHome {
                position: geo_msl(473_977_501, 85_456_001, 110.0),
                use_current: false,
            }),
            (
                u16::from(MAV_CMD_DO_SET_HOME),
                MissionFrame::Global,
                [0.0, 0.0, 0.0, 0.0],
                473_977_501,
                85_456_001,
                110.0,
            ),
        ),
        (
            MissionCommand::from(DoLandStart {
                position: geo_rel_home(473_977_502, 85_456_002, 25.0),
            }),
            (
                u16::from(MAV_CMD_DO_LAND_START),
                MissionFrame::GlobalRelativeAlt,
                [0.0, 0.0, 0.0, 0.0],
                473_977_502,
                85_456_002,
                25.0,
            ),
        ),
        (
            MissionCommand::from(DoReturnPathStart {
                position: geo_terrain(473_977_503, 85_456_003, 15.0),
            }),
            (
                u16::from(MAV_CMD_DO_RETURN_PATH_START),
                MissionFrame::GlobalTerrainAlt,
                [0.0, 0.0, 0.0, 0.0],
                473_977_503,
                85_456_003,
                15.0,
            ),
        ),
        (
            MissionCommand::from(DoGoAround {
                position: geo_rel_home(473_977_504, 85_456_004, 35.0),
            }),
            (
                u16::from(MAV_CMD_DO_GO_AROUND),
                MissionFrame::GlobalRelativeAlt,
                [0.0, 0.0, 0.0, 0.0],
                473_977_504,
                85_456_004,
                35.0,
            ),
        ),
        (
            MissionCommand::from(DoSetRoiLocation {
                position: geo_msl(473_977_505, 85_456_005, 45.0),
            }),
            (
                u16::from(MAV_CMD_DO_SET_ROI_LOCATION),
                MissionFrame::Global,
                [0.0, 0.0, 0.0, 0.0],
                473_977_505,
                85_456_005,
                45.0,
            ),
        ),
        (
            MissionCommand::from(DoCommand::SetRoiNone),
            (
                u16::from(MAV_CMD_DO_SET_ROI_NONE),
                MissionFrame::Mission,
                [0.0, 0.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoSetRoi {
                mode: 3,
                position: geo_terrain(473_977_506, 85_456_006, 55.0),
            }),
            (
                u16::from(MAV_CMD_DO_SET_ROI),
                MissionFrame::GlobalTerrainAlt,
                [3.0, 0.0, 0.0, 0.0],
                473_977_506,
                85_456_006,
                55.0,
            ),
        ),
        (
            MissionCommand::from(DoMountControl {
                pitch_deg: -10.0,
                roll_deg: 2.5,
                yaw_deg: 180.0,
            }),
            (
                u16::from(MAV_CMD_DO_MOUNT_CONTROL),
                MissionFrame::Mission,
                [-10.0, 2.5, 180.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoGimbalManagerPitchYaw {
                pitch_deg: -5.0,
                yaw_deg: 90.0,
                pitch_rate_dps: 3.0,
                yaw_rate_dps: 4.0,
                flags: 7,
                gimbal_id: 5,
            }),
            (
                u16::from(MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW),
                MissionFrame::Mission,
                [-5.0, 90.0, 3.0, 4.0],
                7,
                0,
                5.0,
            ),
        ),
        (
            MissionCommand::from(DoCamTriggerDistance {
                meters: 12.5,
                trigger_now: true,
            }),
            (
                u16::from(MAV_CMD_DO_SET_CAM_TRIGG_DIST),
                MissionFrame::Mission,
                [12.5, 0.0, 1.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoImageStartCapture {
                instance: 2,
                interval_s: 4.0,
                total_images: 6,
                start_number: 8,
            }),
            (
                u16::from(MAV_CMD_IMAGE_START_CAPTURE),
                MissionFrame::Mission,
                [2.0, 4.0, 6.0, 8.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoImageStopCapture { instance: 2 }),
            (
                u16::from(MAV_CMD_IMAGE_STOP_CAPTURE),
                MissionFrame::Mission,
                [2.0, 0.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoVideoStartCapture { stream_id: 3 }),
            (
                u16::from(MAV_CMD_VIDEO_START_CAPTURE),
                MissionFrame::Mission,
                [3.0, 0.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoVideoStopCapture { stream_id: 4 }),
            (
                u16::from(MAV_CMD_VIDEO_STOP_CAPTURE),
                MissionFrame::Mission,
                [4.0, 0.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoSetCameraZoom {
                zoom_type: 2,
                zoom_value: 55.0,
            }),
            (
                u16::from(MAV_CMD_SET_CAMERA_ZOOM),
                MissionFrame::Mission,
                [2.0, 55.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoSetCameraFocus {
                focus_type: 3,
                focus_value: 9.0,
            }),
            (
                u16::from(MAV_CMD_SET_CAMERA_FOCUS),
                MissionFrame::Mission,
                [3.0, 9.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoSetCameraSource {
                instance: 1,
                primary: 2,
                secondary: 3,
            }),
            (
                u16::from(MAV_CMD_SET_CAMERA_SOURCE),
                MissionFrame::Mission,
                [1.0, 2.0, 3.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoDigicamConfigure {
                shooting_mode: 1,
                shutter_speed: 125,
                aperture: 2.8,
                iso: 200,
                exposure_type: 4,
                cmd_id: 5,
                cutoff_time: 0.7,
            }),
            (
                u16::from(MAV_CMD_DO_DIGICAM_CONFIGURE),
                MissionFrame::Mission,
                [1.0, 125.0, 2.8, 200.0],
                4,
                5,
                0.7,
            ),
        ),
        (
            MissionCommand::from(DoDigicamControl {
                session: 1,
                zoom_pos: 2,
                zoom_step: -3,
                focus_lock: 4,
                shooting_cmd: 5,
                cmd_id: 6,
            }),
            (
                u16::from(MAV_CMD_DO_DIGICAM_CONTROL),
                MissionFrame::Mission,
                [1.0, 2.0, -3.0, 4.0],
                5,
                6,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoSetServo {
                channel: 9,
                pwm: 1500,
            }),
            (
                u16::from(MAV_CMD_DO_SET_SERVO),
                MissionFrame::Mission,
                [9.0, 1500.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoSetRelay {
                number: 2,
                state: true,
            }),
            (
                u16::from(MAV_CMD_DO_SET_RELAY),
                MissionFrame::Mission,
                [2.0, 1.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoRepeatServo {
                channel: 10,
                pwm: 1200,
                count: 4,
                cycle_time_s: 2.0,
            }),
            (
                u16::from(MAV_CMD_DO_REPEAT_SERVO),
                MissionFrame::Mission,
                [10.0, 1200.0, 4.0, 2.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoRepeatRelay {
                number: 3,
                count: 5,
                cycle_time_s: 1.5,
            }),
            (
                u16::from(MAV_CMD_DO_REPEAT_RELAY),
                MissionFrame::Mission,
                [3.0, 5.0, 1.5, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoFenceEnable {
                action: FenceAction::Enable,
            }),
            (
                u16::from(MAV_CMD_DO_FENCE_ENABLE),
                MissionFrame::Mission,
                [1.0, 0.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoParachute {
                action: ParachuteAction::Release,
            }),
            (
                u16::from(MAV_CMD_DO_PARACHUTE),
                MissionFrame::Mission,
                [2.0, 0.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoGripper {
                number: 1,
                action: GripperAction::Grab,
            }),
            (
                u16::from(MAV_CMD_DO_GRIPPER),
                MissionFrame::Mission,
                [1.0, 1.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoSprayer { enabled: true }),
            (
                u16::from(MAV_CMD_DO_SPRAYER),
                MissionFrame::Mission,
                [1.0, 0.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoWinch {
                number: 1,
                action: WinchAction::RateControl,
                release_length_m: 12.5,
                release_rate_mps: -1.25,
            }),
            (
                u16::from(MAV_CMD_DO_WINCH),
                MissionFrame::Mission,
                [1.0, 2.0, 12.5, -1.25],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoEngineControl {
                start: true,
                cold_start: false,
                height_delay_m: 6.5,
                allow_disarmed: true,
            }),
            (
                u16::from(MAV_CMD_DO_ENGINE_CONTROL),
                MissionFrame::Mission,
                [1.0, 0.0, 6.5, 1.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoInvertedFlight { inverted: true }),
            (
                u16::from(MAV_CMD_DO_INVERTED_FLIGHT),
                MissionFrame::Mission,
                [1.0, 0.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoAutotuneEnable { enabled: true }),
            (
                u16::from(MAV_CMD_DO_AUTOTUNE_ENABLE),
                MissionFrame::Mission,
                [1.0, 0.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoVtolTransition { target_state: 4 }),
            (
                u16::from(MAV_CMD_DO_VTOL_TRANSITION),
                MissionFrame::Mission,
                [4.0, 0.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoGuidedLimits {
                max_time_s: 10.0,
                min_alt_m: 20.0,
                max_alt_m: 30.0,
                max_horiz_m: 40.0,
            }),
            (
                u16::from(MAV_CMD_DO_GUIDED_LIMITS),
                MissionFrame::Mission,
                [10.0, 20.0, 30.0, 40.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoSetResumeRepeatDist { distance_m: 18.0 }),
            (
                u16::from(MAV_CMD_DO_SET_RESUME_REPEAT_DIST),
                MissionFrame::Mission,
                [18.0, 0.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoAuxFunction {
                function: 42,
                switch_pos: 2,
            }),
            (
                u16::from(MAV_CMD_DO_AUX_FUNCTION),
                MissionFrame::Mission,
                [42.0, 2.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoSendScriptMessage {
                id: 7,
                p1: 1.25,
                p2: -2.5,
                p3: 3.75,
            }),
            (
                u16::from(MAV_CMD_DO_SEND_SCRIPT_MESSAGE),
                MissionFrame::Mission,
                [7.0, 1.25, -2.5, 3.75],
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
