"""Tests for mission types, construction, and free functions."""

import pytest

import mavkit


class TestMissionItem:
    def test_construction_with_defaults(self):
        item = mavkit.MissionItem(
            seq=0, command=16, frame=mavkit.MissionFrame.GlobalInt
        )
        assert item.seq == 0
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
            seq=1,
            command=22,
            frame=mavkit.MissionFrame.GlobalRelativeAltInt,
            x=474200000,
            y=-1222000000,
            z=100.0,
            param1=15.0,
            param2=0.0,
            param3=0.0,
            param4=0.0,
            current=True,
            autocontinue=False,
        )
        assert item.seq == 1
        assert item.command == 22
        assert item.frame == mavkit.MissionFrame.GlobalRelativeAltInt
        assert item.x == 474200000
        assert item.y == -1222000000
        assert item.z == pytest.approx(100.0)
        assert item.param1 == pytest.approx(15.0)
        assert item.current is True
        assert item.autocontinue is False

    def test_repr(self):
        item = mavkit.MissionItem(
            seq=0, command=16, frame=mavkit.MissionFrame.GlobalInt
        )
        r = repr(item)
        assert "MissionItem" in r
        assert "seq=0" in r

    def test_frozen(self):
        item = mavkit.MissionItem(
            seq=0, command=16, frame=mavkit.MissionFrame.GlobalInt
        )
        with pytest.raises(AttributeError):
            item.seq = 5  # type: ignore[misc]

    def test_keyword_only(self):
        with pytest.raises(TypeError):
            mavkit.MissionItem(0, 16, mavkit.MissionFrame.GlobalInt)  # type: ignore[misc]

    def test_dege7_coordinates(self):
        """Verify x/y are stored as degE7 integers, not floats."""
        lat_dege7 = int(47.42 * 1e7)
        lon_dege7 = int(-122.2 * 1e7)
        item = mavkit.MissionItem(
            seq=0,
            command=16,
            frame=mavkit.MissionFrame.GlobalInt,
            x=lat_dege7,
            y=lon_dege7,
        )
        assert item.x == lat_dege7
        assert item.y == lon_dege7
        assert isinstance(item.x, int)
        assert isinstance(item.y, int)


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
            home.latitude_deg = 1.0  # type: ignore[misc]


class TestMissionPlan:
    def test_construction_empty(self):
        plan = mavkit.MissionPlan(mission_type=mavkit.MissionType.Mission, items=[])
        assert plan.mission_type == mavkit.MissionType.Mission
        assert plan.home is None
        assert plan.items == []
        assert len(plan) == 0

    def test_construction_with_items(self):
        items = [
            mavkit.MissionItem(seq=0, command=16, frame=mavkit.MissionFrame.GlobalInt),
            mavkit.MissionItem(
                seq=1, command=22, frame=mavkit.MissionFrame.GlobalRelativeAltInt
            ),
        ]
        plan = mavkit.MissionPlan(mission_type=mavkit.MissionType.Mission, items=items)
        assert len(plan) == 2
        assert plan.items[0].seq == 0
        assert plan.items[1].seq == 1

    def test_construction_with_home(self):
        home = mavkit.HomePosition(
            latitude_deg=47.42, longitude_deg=-122.2, altitude_m=30.0
        )
        plan = mavkit.MissionPlan(
            mission_type=mavkit.MissionType.Mission,
            items=[],
            home=home,
        )
        assert plan.home is not None
        assert plan.home.latitude_deg == pytest.approx(47.42)

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

    def test_invalid_home_latitude(self):
        home = mavkit.HomePosition(latitude_deg=100.0, longitude_deg=0.0)
        plan = mavkit.MissionPlan(
            mission_type=mavkit.MissionType.Mission,
            items=[],
            home=home,
        )
        issues = mavkit.validate_plan(plan)
        assert len(issues) >= 1
        codes = [i.code for i in issues]
        assert "home.latitude_out_of_range" in codes
        lat_issue = next(i for i in issues if i.code == "home.latitude_out_of_range")
        assert lat_issue.severity == mavkit.IssueSeverity.Error
        assert lat_issue.seq is None

    def test_invalid_home_longitude(self):
        home = mavkit.HomePosition(latitude_deg=0.0, longitude_deg=200.0)
        plan = mavkit.MissionPlan(
            mission_type=mavkit.MissionType.Mission,
            items=[],
            home=home,
        )
        issues = mavkit.validate_plan(plan)
        codes = [i.code for i in issues]
        assert "home.longitude_out_of_range" in codes

    def test_issue_repr(self):
        home = mavkit.HomePosition(latitude_deg=100.0, longitude_deg=0.0)
        plan = mavkit.MissionPlan(
            mission_type=mavkit.MissionType.Mission,
            items=[],
            home=home,
        )
        issues = mavkit.validate_plan(plan)
        assert len(issues) >= 1
        r = repr(issues[0])
        assert "MissionIssue" in r


class TestPlansEquivalent:
    def test_identical_plans(self):
        items = [
            mavkit.MissionItem(seq=0, command=16, frame=mavkit.MissionFrame.GlobalInt)
        ]
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
            items=[
                mavkit.MissionItem(
                    seq=0, command=16, frame=mavkit.MissionFrame.GlobalInt
                )
            ],
        )
        plan_b = mavkit.MissionPlan(
            mission_type=mavkit.MissionType.Mission,
            items=[
                mavkit.MissionItem(
                    seq=0, command=22, frame=mavkit.MissionFrame.GlobalInt
                )
            ],
        )
        assert mavkit.plans_equivalent(plan_a, plan_b) is False

    def test_with_tolerance(self):
        tol = mavkit.CompareTolerance(param_epsilon=0.01, altitude_epsilon_m=1.0)
        plan = mavkit.MissionPlan(
            mission_type=mavkit.MissionType.Mission,
            items=[
                mavkit.MissionItem(
                    seq=0, command=16, frame=mavkit.MissionFrame.GlobalInt
                )
            ],
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
            items=[
                mavkit.MissionItem(
                    seq=5, command=16, frame=mavkit.MissionFrame.GlobalInt
                )
            ],
        )
        normalized = mavkit.normalize_for_compare(plan)
        assert isinstance(normalized, mavkit.MissionPlan)
        assert len(normalized) == 1


class TestWireUploadDownload:
    """Test the mission wire boundary rules from CLAUDE.md."""

    def test_mission_upload_prepends_home(self):
        home = mavkit.HomePosition(
            latitude_deg=47.42, longitude_deg=-122.2, altitude_m=30.0
        )
        items = [
            mavkit.MissionItem(seq=0, command=16, frame=mavkit.MissionFrame.GlobalInt),
        ]
        plan = mavkit.MissionPlan(
            mission_type=mavkit.MissionType.Mission,
            items=items,
            home=home,
        )
        wire_items = mavkit.items_for_wire_upload(plan)
        # Home is prepended as seq=0
        assert len(wire_items) == 2
        assert wire_items[0].seq == 0

    def test_mission_upload_no_home_uses_default(self):
        items = [
            mavkit.MissionItem(seq=0, command=16, frame=mavkit.MissionFrame.GlobalInt),
        ]
        plan = mavkit.MissionPlan(
            mission_type=mavkit.MissionType.Mission,
            items=items,
        )
        wire_items = mavkit.items_for_wire_upload(plan)
        # Default home is still prepended
        assert len(wire_items) == 2

    def test_fence_upload_passthrough(self):
        """Fence items pass through unchanged, no home insertion."""
        items = [
            mavkit.MissionItem(
                seq=0, command=5001, frame=mavkit.MissionFrame.GlobalInt
            ),
            mavkit.MissionItem(
                seq=1, command=5001, frame=mavkit.MissionFrame.GlobalInt
            ),
        ]
        plan = mavkit.MissionPlan(mission_type=mavkit.MissionType.Fence, items=items)
        wire_items = mavkit.items_for_wire_upload(plan)
        assert len(wire_items) == 2

    def test_rally_upload_passthrough(self):
        """Rally items pass through unchanged."""
        items = [
            mavkit.MissionItem(
                seq=0, command=5100, frame=mavkit.MissionFrame.GlobalInt
            ),
        ]
        plan = mavkit.MissionPlan(mission_type=mavkit.MissionType.Rally, items=items)
        wire_items = mavkit.items_for_wire_upload(plan)
        assert len(wire_items) == 1

    def test_download_extracts_home(self):
        """Download extracts seq=0 as home and resequences remaining items from 0."""
        wire_items = [
            mavkit.MissionItem(
                seq=0,
                command=16,
                frame=mavkit.MissionFrame.GlobalInt,
                x=int(47.42 * 1e7),
                y=int(-122.2 * 1e7),
                z=30.0,
            ),
            mavkit.MissionItem(
                seq=1, command=22, frame=mavkit.MissionFrame.GlobalRelativeAltInt
            ),
        ]
        plan = mavkit.plan_from_wire_download(mavkit.MissionType.Mission, wire_items)
        assert plan.home is not None
        assert plan.home.latitude_deg == pytest.approx(47.42, abs=0.001)
        assert plan.home.longitude_deg == pytest.approx(-122.2, abs=0.001)
        assert len(plan.items) == 1
        assert plan.items[0].seq == 0  # resequenced from 0

    def test_download_fence_passthrough(self):
        wire_items = [
            mavkit.MissionItem(
                seq=0, command=5001, frame=mavkit.MissionFrame.GlobalInt
            ),
        ]
        plan = mavkit.plan_from_wire_download(mavkit.MissionType.Fence, wire_items)
        assert plan.home is None
        assert len(plan.items) == 1

    def test_download_empty_items(self):
        plan = mavkit.plan_from_wire_download(mavkit.MissionType.Mission, [])
        assert plan.home is None
        assert len(plan.items) == 0
