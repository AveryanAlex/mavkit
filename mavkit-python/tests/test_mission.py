import pytest

import mavkit


def nav_waypoint_command(
    *,
    latitude_deg: float = 0.0,
    longitude_deg: float = 0.0,
    altitude_m: float = 0.0,
    frame: mavkit.MissionFrame = mavkit.MissionFrame.GlobalInt,
    hold_time_s: float = 0.0,
    acceptance_radius_m: float = 0.0,
    pass_radius_m: float = 0.0,
    yaw_deg: float = 0.0,
) -> mavkit.NavWaypoint:
    return mavkit.NavWaypoint(
        latitude_deg=latitude_deg,
        longitude_deg=longitude_deg,
        altitude_m=altitude_m,
        frame=frame,
        hold_time_s=hold_time_s,
        acceptance_radius_m=acceptance_radius_m,
        pass_radius_m=pass_radius_m,
        yaw_deg=yaw_deg,
    )


def nav_takeoff_command(
    *,
    latitude_deg: float = 0.0,
    longitude_deg: float = 0.0,
    altitude_m: float = 0.0,
    frame: mavkit.MissionFrame = mavkit.MissionFrame.GlobalRelativeAltInt,
    pitch_deg: float = 0.0,
) -> mavkit.NavTakeoff:
    return mavkit.NavTakeoff(
        latitude_deg=latitude_deg,
        longitude_deg=longitude_deg,
        altitude_m=altitude_m,
        frame=frame,
        pitch_deg=pitch_deg,
    )


def geo_point_rel_home(
    *,
    latitude_deg: float = 0.0,
    longitude_deg: float = 0.0,
    relative_alt_m: float = 0.0,
) -> mavkit.GeoPoint3d:
    return mavkit.GeoPoint3d.rel_home(
        latitude_deg=latitude_deg,
        longitude_deg=longitude_deg,
        relative_alt_m=relative_alt_m,
    )


def raw_command(
    *,
    command: int,
    frame: mavkit.MissionFrame,
    x: int = 0,
    y: int = 0,
    z: float = 0.0,
    param1: float = 0.0,
    param2: float = 0.0,
    param3: float = 0.0,
    param4: float = 0.0,
) -> mavkit.RawMissionCommand:
    return mavkit.RawMissionCommand(
        command=command,
        frame=frame,
        x=x,
        y=y,
        z=z,
        param1=param1,
        param2=param2,
        param3=param3,
        param4=param4,
    )


class TestMissionItem:
    def test_construction_with_defaults(self):
        item = mavkit.MissionItem(command=nav_waypoint_command())
        assert item.command == 16
        assert item.frame == mavkit.MissionFrame.GlobalInt
        assert item.x == 0
        assert item.y == 0
        assert item.z == 0.0
        assert item.param1 == 0.0
        assert item.param2 == 0.0
        assert item.param3 == 0.0
        assert item.param4 == 0.0
        assert item.current is False
        assert item.autocontinue is True

    def test_construction_with_all_fields(self):
        item = mavkit.MissionItem(
            command=nav_takeoff_command(
                latitude_deg=47.42,
                longitude_deg=-122.2,
                altitude_m=100.0,
                pitch_deg=15.0,
            ),
            current=True,
            autocontinue=False,
        )
        assert item.command == 22
        assert item.frame == mavkit.MissionFrame.GlobalRelativeAltInt
        assert item.x == 474200000
        assert item.y == -1222000000
        assert item.z == pytest.approx(100.0)
        assert item.param1 == pytest.approx(15.0)
        assert item.current is True
        assert item.autocontinue is False

    def test_repr(self):
        item = mavkit.MissionItem(command=nav_waypoint_command())
        r = repr(item)
        assert "MissionItem" in r

    def test_keyword_only(self):
        with pytest.raises(TypeError):
            _ = mavkit.MissionItem(0, nav_waypoint_command())  # pyright: ignore[reportCallIssue]

    def test_dege7_coordinates(self):
        lat_deg = 47.42
        lon_deg = -122.2
        lat_dege7 = int(lat_deg * 1e7)
        lon_dege7 = int(lon_deg * 1e7)
        item = mavkit.MissionItem(
            command=nav_waypoint_command(latitude_deg=lat_deg, longitude_deg=lon_deg),
        )
        assert item.x == lat_dege7
        assert item.y == lon_dege7
        assert isinstance(item.x, int)
        assert isinstance(item.y, int)

    def test_command_accepts_all_supported_typed_variants(self):
        command_cases = [
            (nav_waypoint_command(), 16),
            (nav_takeoff_command(), 22),
            (
                mavkit.NavLand(
                    latitude_deg=47.5,
                    longitude_deg=-122.3,
                    altitude_m=12.0,
                    abort_alt_m=25.0,
                ),
                21,
            ),
            (
                mavkit.NavLoiterTime(
                    latitude_deg=47.41,
                    longitude_deg=-122.21,
                    altitude_m=60.0,
                    time_s=18.0,
                    direction="counter_clockwise",
                    exit_xtrack=True,
                ),
                19,
            ),
            (mavkit.NavGuidedEnable(enabled=True), 92),
            (mavkit.NavReturnToLaunch(), 20),
            (
                mavkit.NavSplineWaypoint(
                    latitude_deg=47.41,
                    longitude_deg=-122.22,
                    altitude_m=35.0,
                    hold_time_s=3.0,
                ),
                82,
            ),
            (
                mavkit.NavArcWaypoint(
                    latitude_deg=47.42,
                    longitude_deg=-122.23,
                    altitude_m=36.0,
                    arc_angle_deg=45.0,
                    direction="counter_clockwise",
                ),
                36,
            ),
            (
                mavkit.NavLoiterUnlimited(
                    latitude_deg=47.43,
                    longitude_deg=-122.24,
                    altitude_m=40.0,
                    radius_m=30.0,
                ),
                17,
            ),
            (
                mavkit.NavLoiterTurns(
                    latitude_deg=47.44,
                    longitude_deg=-122.25,
                    altitude_m=41.0,
                    turns=2.0,
                    radius_m=25.0,
                    exit_xtrack=True,
                ),
                18,
            ),
            (
                mavkit.NavLoiterToAlt(
                    latitude_deg=47.45,
                    longitude_deg=-122.26,
                    altitude_m=42.0,
                    radius_m=22.0,
                    direction="counter_clockwise",
                ),
                31,
            ),
            (
                mavkit.NavContinueAndChangeAlt(
                    latitude_deg=47.46,
                    longitude_deg=-122.27,
                    altitude_m=43.0,
                    action="descend",
                ),
                30,
            ),
            (mavkit.NavDelay(seconds=1.0, hour_utc=2.0, min_utc=3.0, sec_utc=4.0), 93),
            (
                mavkit.NavAltitudeWait(
                    altitude_m=120.0,
                    descent_rate_mps=-1.0,
                    wiggle_time_s=2.0,
                ),
                83,
            ),
            (
                mavkit.NavVtolTakeoff(
                    latitude_deg=47.47,
                    longitude_deg=-122.28,
                    altitude_m=44.0,
                ),
                84,
            ),
            (
                mavkit.NavVtolLand(
                    latitude_deg=47.48,
                    longitude_deg=-122.29,
                    altitude_m=12.0,
                    options=2,
                ),
                85,
            ),
            (
                mavkit.NavPayloadPlace(
                    latitude_deg=47.49,
                    longitude_deg=-122.3,
                    altitude_m=15.0,
                    max_descent_m=5.0,
                ),
                94,
            ),
            (mavkit.NavSetYawSpeed(angle_deg=90.0, speed_mps=5.0, relative=True), 213),
            (
                mavkit.NavScriptTime(
                    command=9,
                    timeout_s=2.0,
                    arg1=1.0,
                    arg2=2.0,
                    arg3=3,
                    arg4=4,
                ),
                42702,
            ),
            (
                mavkit.NavAttitudeTime(
                    time_s=3.0,
                    roll_deg=1.0,
                    pitch_deg=2.0,
                    yaw_deg=3.0,
                    climb_rate_mps=1.0,
                ),
                42703,
            ),
            (mavkit.DoChangeSpeed(speed_mps=12.0), 178),
            (
                mavkit.DoSetHome(
                    latitude_deg=47.4,
                    longitude_deg=-122.2,
                    altitude_m=50.0,
                    use_current=True,
                ),
                179,
            ),
            (mavkit.DoSetRelay(number=3, state=True), 181),
            (mavkit.DoSetRoiNone(), 197),
            (mavkit.DoJump(target_index=2, repeat_count=3), 177),
            (mavkit.DoJumpTag(tag=44, repeat_count=2), 601),
            (mavkit.DoTag(tag=44), 600),
            (mavkit.DoPauseContinue(pause=True), 193),
            (mavkit.DoSetReverse(reverse=True), 194),
            (mavkit.DoSetServo(channel=8, pwm=1500), 183),
            (mavkit.DoRepeatServo(channel=8, pwm=1600, count=3, cycle_time_s=1.2), 184),
            (mavkit.DoRepeatRelay(number=4, count=2, cycle_time_s=0.5), 182),
            (mavkit.DoSetResumeRepeatDist(distance_m=12.5), 215),
            (mavkit.DoAuxFunction(function=53, switch_pos=1), 218),
            (mavkit.DoSendScriptMessage(id=7, p1=1.0, p2=2.0, p3=3.0), 217),
            (
                mavkit.DoLandStart(
                    latitude_deg=47.5,
                    longitude_deg=-122.31,
                    altitude_m=20.0,
                ),
                189,
            ),
            (
                mavkit.DoReturnPathStart(
                    latitude_deg=47.51,
                    longitude_deg=-122.32,
                    altitude_m=21.0,
                ),
                188,
            ),
            (
                mavkit.DoGoAround(
                    latitude_deg=47.52,
                    longitude_deg=-122.33,
                    altitude_m=22.0,
                ),
                191,
            ),
            (
                mavkit.DoSetRoiLocation(
                    latitude_deg=47.53,
                    longitude_deg=-122.34,
                    altitude_m=23.0,
                ),
                195,
            ),
            (
                mavkit.DoSetRoi(
                    latitude_deg=47.54,
                    longitude_deg=-122.35,
                    altitude_m=24.0,
                    mode=2,
                ),
                201,
            ),
            (
                mavkit.DoImageStartCapture(
                    instance=0,
                    interval_s=1.0,
                    total_images=10,
                    start_number=1,
                ),
                2000,
            ),
            (mavkit.DoImageStopCapture(instance=0), 2001),
            (mavkit.DoVideoStartCapture(stream_id=1), 2500),
            (mavkit.DoVideoStopCapture(stream_id=1), 2501),
            (mavkit.DoSetCameraZoom(zoom_type=1, zoom_value=2.0), 531),
            (mavkit.DoSetCameraFocus(focus_type=1, focus_value=1.5), 532),
            (mavkit.DoSetCameraSource(instance=0, primary=1, secondary=2), 534),
            (
                mavkit.DoMountControl(pitch_deg=1.0, roll_deg=2.0, yaw_deg=3.0),
                205,
            ),
            (
                mavkit.DoGimbalManagerPitchYaw(
                    pitch_deg=4.0,
                    yaw_deg=5.0,
                    pitch_rate_dps=6.0,
                    yaw_rate_dps=7.0,
                    flags=8,
                    gimbal_id=9,
                ),
                1000,
            ),
            (
                mavkit.DoCamTriggerDistance(meters=12.5, trigger_now=True),
                206,
            ),
            (
                mavkit.DoDigicamConfigure(
                    shooting_mode=1,
                    shutter_speed=2,
                    aperture=3.5,
                    iso=400,
                    exposure_type=4,
                    cmd_id=5,
                    cutoff_time=6.0,
                ),
                202,
            ),
            (
                mavkit.DoDigicamControl(
                    session=1,
                    zoom_pos=2,
                    zoom_step=-3,
                    focus_lock=4,
                    shooting_cmd=5,
                    cmd_id=6,
                ),
                203,
            ),
            (mavkit.DoFenceEnable(action="disable_floor"), 207),
            (mavkit.DoParachute(action="release"), 208),
            (mavkit.DoGripper(number=2, action="grab"), 211),
            (mavkit.DoSprayer(enabled=True), 216),
            (
                mavkit.DoWinch(
                    number=1,
                    action="rate_control",
                    release_length_m=20.0,
                    release_rate_mps=3.0,
                ),
                42600,
            ),
            (
                mavkit.DoEngineControl(
                    start=True,
                    cold_start=False,
                    height_delay_m=15.0,
                    allow_disarmed=True,
                ),
                223,
            ),
            (mavkit.DoInvertedFlight(inverted=True), 210),
            (mavkit.DoAutotuneEnable(enabled=True), 212),
            (
                mavkit.DoGuidedLimits(
                    max_time_s=10.0,
                    min_alt_m=5.0,
                    max_alt_m=50.0,
                    max_horiz_m=100.0,
                ),
                222,
            ),
            (mavkit.DoVtolTransition(target_state=3), 3000),
            (mavkit.CondDelay(delay_s=3.0), 112),
            (mavkit.CondDistance(distance_m=22.5), 114),
            (mavkit.CondYaw(angle_deg=45.0), 115),
            (
                raw_command(
                    command=31000,
                    frame=mavkit.MissionFrame.GlobalRelativeAltInt,
                ),
                31000,
            ),
        ]

        for command, expected_command_id in command_cases:
            item = mavkit.MissionItem(command=command)
            assert item.command == expected_command_id

    def test_geo_point3d_helper_drives_typed_nav_construction(self):
        waypoint_item = mavkit.MissionItem(
            command=mavkit.NavWaypoint.from_point(
                position=mavkit.GeoPoint3d.terrain(
                    latitude_deg=47.42,
                    longitude_deg=-122.2,
                    altitude_terrain_m=12.5,
                )
            ),
        )
        assert waypoint_item.frame == mavkit.MissionFrame.GlobalTerrainAltInt
        assert waypoint_item.z == pytest.approx(12.5)

        takeoff_item = mavkit.MissionItem(
            command=mavkit.NavTakeoff.from_point(
                position=mavkit.GeoPoint3d.msl(
                    latitude_deg=47.5,
                    longitude_deg=-122.4,
                    altitude_msl_m=140.0,
                ),
                pitch_deg=10.0,
            ),
        )
        assert takeoff_item.frame == mavkit.MissionFrame.GlobalInt
        assert takeoff_item.z == pytest.approx(140.0)
        assert takeoff_item.param1 == pytest.approx(10.0)

    def test_do_change_speed_and_cond_yaw_wire_fields(self):
        speed_item = mavkit.MissionItem(
            command=mavkit.DoChangeSpeed(
                speed_mps=8.5,
                throttle_pct=35.0,
                speed_type="airspeed",
            ),
        )
        assert speed_item.frame == mavkit.MissionFrame.Mission
        assert speed_item.param1 == pytest.approx(0.0)
        assert speed_item.param2 == pytest.approx(8.5)
        assert speed_item.param3 == pytest.approx(35.0)

        yaw_item = mavkit.MissionItem(
            command=mavkit.CondYaw(
                angle_deg=90.0,
                turn_rate_dps=20.0,
                direction="counter_clockwise",
                relative=True,
            ),
        )
        assert yaw_item.frame == mavkit.MissionFrame.Mission
        assert yaw_item.param1 == pytest.approx(90.0)
        assert yaw_item.param2 == pytest.approx(20.0)
        assert yaw_item.param3 == pytest.approx(-1.0)
        assert yaw_item.param4 == pytest.approx(1.0)

    def test_broader_typed_family_wire_fields(self):
        land_item = mavkit.MissionItem(
            command=mavkit.NavLand.from_point(
                position=mavkit.GeoPoint3d.msl(
                    latitude_deg=47.61,
                    longitude_deg=-122.33,
                    altitude_msl_m=35.0,
                ),
                abort_alt_m=9.0,
            ),
        )
        assert land_item.command == 21
        assert land_item.frame == mavkit.MissionFrame.GlobalInt
        assert land_item.param1 == pytest.approx(9.0)

        loiter_item = mavkit.MissionItem(
            command=mavkit.NavLoiterTime.from_point(
                position=mavkit.GeoPoint3d.rel_home(
                    latitude_deg=47.62,
                    longitude_deg=-122.34,
                    relative_alt_m=40.0,
                ),
                time_s=22.0,
                direction="counter_clockwise",
                exit_xtrack=True,
            ),
        )
        assert loiter_item.command == 19
        assert loiter_item.param1 == pytest.approx(22.0)
        assert loiter_item.param3 == pytest.approx(-0.0)
        assert loiter_item.param4 == pytest.approx(1.0)

        guided_item = mavkit.MissionItem(
            command=mavkit.NavGuidedEnable(enabled=True)
        )
        assert guided_item.command == 92
        assert guided_item.frame == mavkit.MissionFrame.Mission
        assert guided_item.param1 == pytest.approx(1.0)

        set_home_item = mavkit.MissionItem(
            command=mavkit.DoSetHome.from_point(
                position=mavkit.GeoPoint3d.terrain(
                    latitude_deg=47.63,
                    longitude_deg=-122.35,
                    altitude_terrain_m=12.5,
                ),
                use_current=True,
            ),
        )
        assert set_home_item.command == 179
        assert set_home_item.frame == mavkit.MissionFrame.GlobalTerrainAltInt
        assert set_home_item.param1 == pytest.approx(1.0)

        set_relay_item = mavkit.MissionItem(
            command=mavkit.DoSetRelay(number=2, state=True),
        )
        assert set_relay_item.command == 181
        assert set_relay_item.param1 == pytest.approx(2.0)
        assert set_relay_item.param2 == pytest.approx(1.0)

        roi_none_item = mavkit.MissionItem(command=mavkit.DoSetRoiNone())
        assert roi_none_item.command == 197
        assert roi_none_item.frame == mavkit.MissionFrame.Mission

        cond_delay_item = mavkit.MissionItem(
            command=mavkit.CondDelay(delay_s=4.5)
        )
        assert cond_delay_item.command == 112
        assert cond_delay_item.param1 == pytest.approx(4.5)

        cond_distance_item = mavkit.MissionItem(
            command=mavkit.CondDistance(distance_m=75.0),
        )
        assert cond_distance_item.command == 114
        assert cond_distance_item.param1 == pytest.approx(75.0)


class TestHomePosition:
    def test_construction(self):
        home = mavkit.HomePosition(
            latitude_deg=47.42, longitude_deg=-122.2, altitude_m=30.0
        )
        assert home.latitude_deg == pytest.approx(47.42)
        assert home.longitude_deg == pytest.approx(-122.2)
        assert home.altitude_m == pytest.approx(30.0)

    def test_default_altitude(self):
        home = mavkit.HomePosition(latitude_deg=0.0, longitude_deg=0.0)
        assert home.altitude_m == pytest.approx(0.0)

    def test_repr(self):
        home = mavkit.HomePosition(latitude_deg=47.42, longitude_deg=-122.2)
        r = repr(home)
        assert "HomePosition" in r

    def test_frozen(self):
        home = mavkit.HomePosition(latitude_deg=0.0, longitude_deg=0.0)
        with pytest.raises(AttributeError):
            home.latitude_deg = 1.0  # pyright: ignore[reportAttributeAccessIssue]


class TestMissionPlan:
    def test_construction_empty(self):
        plan = mavkit.MissionPlan(mission_type=mavkit.MissionType.Mission, items=[])
        assert plan.mission_type == mavkit.MissionType.Mission
        assert not hasattr(plan, "home")
        assert plan.items == []
        assert len(plan) == 0

    def test_construction_with_items(self):
        items = [
            mavkit.MissionItem(command=nav_waypoint_command()),
            mavkit.MissionItem(command=nav_takeoff_command()),
        ]
        plan = mavkit.MissionPlan(mission_type=mavkit.MissionType.Mission, items=items)
        assert len(plan) == 2

    def test_construction_rejects_home(self):
        home = mavkit.HomePosition(
            latitude_deg=47.42, longitude_deg=-122.2, altitude_m=30.0
        )
        with pytest.raises(TypeError):
            constructor = getattr(mavkit, "MissionPlan")
            _ = constructor(
                mission_type=mavkit.MissionType.Mission,
                items=[],
                home=home,
            )

    def test_repr(self):
        plan = mavkit.MissionPlan(mission_type=mavkit.MissionType.Mission, items=[])
        r = repr(plan)
        assert "MissionPlan" in r

    def test_fence_type(self):
        plan = mavkit.MissionPlan(mission_type=mavkit.MissionType.Fence, items=[])
        assert plan.mission_type == mavkit.MissionType.Fence

    def test_rally_type(self):
        plan = mavkit.MissionPlan(mission_type=mavkit.MissionType.Rally, items=[])
        assert plan.mission_type == mavkit.MissionType.Rally


class TestStoredPlanTypes:
    def test_empty_fence_plan(self):
        plan = mavkit.FencePlan(return_point=None, regions=[])
        assert plan.return_point is None
        assert plan.regions == []

    def test_fence_plan_with_region_helpers(self):
        return_point = mavkit.GeoPoint2d(latitude_deg=47.42, longitude_deg=-122.2)
        inclusion = mavkit.FenceInclusionPolygon(
            vertices=[
                mavkit.GeoPoint2d(latitude_deg=47.42, longitude_deg=-122.2),
                mavkit.GeoPoint2d(latitude_deg=47.43, longitude_deg=-122.2),
                mavkit.GeoPoint2d(latitude_deg=47.43, longitude_deg=-122.1),
            ],
            inclusion_group=2,
        )
        exclusion = mavkit.FenceExclusionCircle(
            center=mavkit.GeoPoint2d(latitude_deg=47.425, longitude_deg=-122.15),
            radius_m=25.0,
        )

        plan = mavkit.FencePlan(
            return_point=return_point,
            regions=[inclusion, exclusion],
        )

        assert plan.return_point is not None
        assert plan.return_point.latitude_deg == pytest.approx(47.42)
        assert len(plan.regions) == 2
        assert isinstance(plan.regions[0], mavkit.FenceInclusionPolygon)
        assert isinstance(plan.regions[1], mavkit.FenceExclusionCircle)
        assert plan.regions[0].inclusion_group == 2
        assert plan.regions[1].radius_m == pytest.approx(25.0)

    def test_empty_rally_plan(self):
        plan = mavkit.RallyPlan(points=[])
        assert plan.points == []

    def test_rally_plan_with_multiple_point_variants(self):
        plan = mavkit.RallyPlan(
            points=[
                mavkit.GeoPoint3dMsl(47.42, -122.2, 30.0),
                mavkit.GeoPoint3dRelHome(
                    latitude_deg=47.43,
                    longitude_deg=-122.1,
                    relative_alt_m=15.0,
                ),
                mavkit.GeoPoint3dTerrain(
                    latitude_deg=47.44,
                    longitude_deg=-122.0,
                    altitude_terrain_m=8.5,
                ),
            ]
        )

        assert len(plan.points) == 3
        assert isinstance(plan.points[0], mavkit.GeoPoint3dMsl)
        assert isinstance(plan.points[1], mavkit.GeoPoint3dRelHome)
        assert isinstance(plan.points[2], mavkit.GeoPoint3dTerrain)
        assert plan.points[0].altitude_msl_m == pytest.approx(30.0)
        assert plan.points[1].relative_alt_m == pytest.approx(15.0)
        assert plan.points[2].altitude_terrain_m == pytest.approx(8.5)


class TestRetryPolicy:
    def test_defaults(self):
        policy = mavkit.RetryPolicy()
        assert policy.request_timeout_ms == 1500
        assert policy.item_timeout_ms == 250
        assert policy.max_retries == 5

    def test_custom_values(self):
        policy = mavkit.RetryPolicy(
            request_timeout_ms=3000, item_timeout_ms=500, max_retries=10
        )
        assert policy.request_timeout_ms == 3000
        assert policy.item_timeout_ms == 500
        assert policy.max_retries == 10

    def test_repr(self):
        policy = mavkit.RetryPolicy()
        r = repr(policy)
        assert "RetryPolicy" in r
        assert "1500" in r


class TestCompareTolerance:
    def test_defaults(self):
        tol = mavkit.CompareTolerance()
        assert tol.param_epsilon == pytest.approx(0.0001)
        assert tol.altitude_epsilon_m == pytest.approx(0.01)

    def test_custom_values(self):
        tol = mavkit.CompareTolerance(param_epsilon=0.01, altitude_epsilon_m=1.0)
        assert tol.param_epsilon == pytest.approx(0.01)
        assert tol.altitude_epsilon_m == pytest.approx(1.0)


class TestValidatePlan:
    def test_valid_plan_no_issues(self):
        plan = mavkit.MissionPlan(mission_type=mavkit.MissionType.Mission, items=[])
        issues = mavkit.validate_plan(plan)
        assert issues == []

    def test_invalid_item_latitude(self):
        item = mavkit.MissionItem(
            command=nav_waypoint_command(latitude_deg=100.0),
        )
        plan = mavkit.MissionPlan(
            mission_type=mavkit.MissionType.Mission,
            items=[item],
        )
        issues = mavkit.validate_plan(plan)
        assert len(issues) >= 1
        codes = [i.code for i in issues]
        assert "item.latitude_out_of_range" in codes
        lat_issue = next(i for i in issues if i.code == "item.latitude_out_of_range")
        assert lat_issue.severity == mavkit.IssueSeverity.Error
        assert lat_issue.seq == 0

    def test_invalid_item_longitude(self):
        item = mavkit.MissionItem(
            command=nav_waypoint_command(longitude_deg=200.0),
        )
        plan = mavkit.MissionPlan(
            mission_type=mavkit.MissionType.Mission,
            items=[item],
        )
        issues = mavkit.validate_plan(plan)
        codes = [i.code for i in issues]
        assert "item.longitude_out_of_range" in codes

    def test_issue_repr(self):
        item = mavkit.MissionItem(
            command=nav_waypoint_command(latitude_deg=100.0),
        )
        plan = mavkit.MissionPlan(
            mission_type=mavkit.MissionType.Mission,
            items=[item],
        )
        issues = mavkit.validate_plan(plan)
        assert len(issues) >= 1
        r = repr(issues[0])
        assert "MissionIssue" in r


class TestPlansEquivalent:
    def test_identical_plans(self):
        items = [mavkit.MissionItem(command=nav_waypoint_command())]
        plan_a = mavkit.MissionPlan(
            mission_type=mavkit.MissionType.Mission, items=items
        )
        plan_b = mavkit.MissionPlan(
            mission_type=mavkit.MissionType.Mission, items=items
        )
        assert mavkit.plans_equivalent(plan_a, plan_b) is True

    def test_different_plans(self):
        plan_a = mavkit.MissionPlan(
            mission_type=mavkit.MissionType.Mission,
            items=[mavkit.MissionItem(command=nav_waypoint_command())],
        )
        plan_b = mavkit.MissionPlan(
            mission_type=mavkit.MissionType.Mission,
            items=[mavkit.MissionItem(command=nav_takeoff_command())],
        )
        assert mavkit.plans_equivalent(plan_a, plan_b) is False

    def test_with_tolerance(self):
        tol = mavkit.CompareTolerance(param_epsilon=0.01, altitude_epsilon_m=1.0)
        plan = mavkit.MissionPlan(
            mission_type=mavkit.MissionType.Mission,
            items=[mavkit.MissionItem(command=nav_waypoint_command())],
        )
        assert mavkit.plans_equivalent(plan, plan, tolerance=tol) is True

    def test_empty_plans(self):
        plan_a = mavkit.MissionPlan(mission_type=mavkit.MissionType.Mission, items=[])
        plan_b = mavkit.MissionPlan(mission_type=mavkit.MissionType.Mission, items=[])
        assert mavkit.plans_equivalent(plan_a, plan_b) is True


class TestNormalizeForCompare:
    def test_returns_plan(self):
        plan = mavkit.MissionPlan(
            mission_type=mavkit.MissionType.Mission,
            items=[mavkit.MissionItem(command=nav_waypoint_command())],
        )
        normalized = mavkit.normalize_for_compare(plan)
        assert isinstance(normalized, mavkit.MissionPlan)
        assert len(normalized) == 1


class TestWireUploadDownload:
    def test_mission_upload_prepends_placeholder_item(self):
        items = [mavkit.MissionItem(command=nav_waypoint_command())]
        plan = mavkit.MissionPlan(
            mission_type=mavkit.MissionType.Mission,
            items=items,
        )
        wire_items = mavkit.items_for_wire_upload(plan)
        assert len(wire_items) == 2
        assert wire_items[0].command == 16
        assert wire_items[0].frame == mavkit.MissionFrame.GlobalInt

    def test_mission_upload_empty_items_still_emit_placeholder(self):
        plan = mavkit.MissionPlan(
            mission_type=mavkit.MissionType.Mission,
            items=[],
        )
        wire_items = mavkit.items_for_wire_upload(plan)
        assert len(wire_items) == 1
        assert wire_items[0].command == 16

    def test_mission_upload_resequences_items_after_placeholder(self):
        items = [
            mavkit.MissionItem(command=nav_waypoint_command()),
            mavkit.MissionItem(command=nav_takeoff_command()),
        ]
        plan = mavkit.MissionPlan(
            mission_type=mavkit.MissionType.Mission,
            items=items,
        )
        wire_items = mavkit.items_for_wire_upload(plan)
        assert len(wire_items) == 3

    def test_fence_upload_passthrough(self):
        items = [
            mavkit.MissionItem(
                command=raw_command(
                    command=5001,
                    frame=mavkit.MissionFrame.GlobalInt,
                ),
            ),
            mavkit.MissionItem(
                command=raw_command(
                    command=5001,
                    frame=mavkit.MissionFrame.GlobalInt,
                ),
            ),
        ]
        plan = mavkit.MissionPlan(mission_type=mavkit.MissionType.Fence, items=items)
        wire_items = mavkit.items_for_wire_upload(plan)
        assert len(wire_items) == 2

    def test_rally_upload_passthrough(self):
        items = [
            mavkit.MissionItem(
                command=raw_command(
                    command=5100,
                    frame=mavkit.MissionFrame.GlobalInt,
                ),
            ),
        ]
        plan = mavkit.MissionPlan(mission_type=mavkit.MissionType.Rally, items=items)
        wire_items = mavkit.items_for_wire_upload(plan)
        assert len(wire_items) == 1

    def test_download_strips_hidden_home_and_resequences(self):
        wire_items = [
            mavkit.MissionItem(
                command=raw_command(
                    command=16,
                    frame=mavkit.MissionFrame.GlobalInt,
                    x=int(47.42 * 1e7),
                    y=int(-122.2 * 1e7),
                    z=30.0,
                ),
            ),
            mavkit.MissionItem(
                command=raw_command(
                    command=22,
                    frame=mavkit.MissionFrame.GlobalRelativeAltInt,
                ),
            ),
        ]
        plan = mavkit.plan_from_wire_download(mavkit.MissionType.Mission, wire_items)
        assert not hasattr(plan, "home")
        assert len(plan.items) == 1
        assert plan.items[0].current is True

    def test_download_fence_passthrough(self):
        wire_items = [
            mavkit.MissionItem(
                command=raw_command(
                    command=5001,
                    frame=mavkit.MissionFrame.GlobalInt,
                ),
            ),
        ]
        plan = mavkit.plan_from_wire_download(mavkit.MissionType.Fence, wire_items)
        assert not hasattr(plan, "home")
        assert len(plan.items) == 1

    def test_download_empty_items(self):
        plan = mavkit.plan_from_wire_download(mavkit.MissionType.Mission, [])
        assert not hasattr(plan, "home")
        assert len(plan.items) == 0
