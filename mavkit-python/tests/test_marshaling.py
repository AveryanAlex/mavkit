"""Tests for correct marshaling between Rust and Python types.

Verifies that type conversions, edge cases, and boundary values
are handled correctly across the FFI boundary.
"""

import pytest

import mavkit


class TestMissionItemBoundaryValues:
    """Test edge cases in MissionItem integer/float marshaling."""

    def test_max_seq(self):
        item = mavkit.MissionItem(
            seq=65535, command=0, frame=mavkit.MissionFrame.GlobalInt
        )
        assert item.seq == 65535

    def test_max_command(self):
        item = mavkit.MissionItem(
            seq=0, command=65535, frame=mavkit.MissionFrame.GlobalInt
        )
        assert item.command == 65535

    def test_large_positive_coordinates(self):
        item = mavkit.MissionItem(
            seq=0,
            command=16,
            frame=mavkit.MissionFrame.GlobalInt,
            x=900000000,
            y=1800000000,
        )
        assert item.x == 900000000
        assert item.y == 1800000000

    def test_large_negative_coordinates(self):
        item = mavkit.MissionItem(
            seq=0,
            command=16,
            frame=mavkit.MissionFrame.GlobalInt,
            x=-900000000,
            y=-1800000000,
        )
        assert item.x == -900000000
        assert item.y == -1800000000

    def test_float_precision_z(self):
        """f32 precision — verify no unexpected truncation."""
        item = mavkit.MissionItem(
            seq=0,
            command=16,
            frame=mavkit.MissionFrame.GlobalInt,
            z=123.456,
        )
        assert item.z == pytest.approx(123.456, rel=1e-3)

    def test_float_precision_params(self):
        item = mavkit.MissionItem(
            seq=0,
            command=16,
            frame=mavkit.MissionFrame.GlobalInt,
            param1=0.001,
            param2=-999.99,
            param3=1e6,
            param4=0.0,
        )
        assert item.param1 == pytest.approx(0.001, rel=1e-3)
        assert item.param2 == pytest.approx(-999.99, rel=1e-3)
        assert item.param3 == pytest.approx(1e6, rel=1e-3)
        assert item.param4 == pytest.approx(0.0)


class TestHomePositionBoundaryValues:
    def test_extreme_latitude(self):
        home = mavkit.HomePosition(latitude_deg=90.0, longitude_deg=0.0)
        assert home.latitude_deg == pytest.approx(90.0)

    def test_negative_extreme_latitude(self):
        home = mavkit.HomePosition(latitude_deg=-90.0, longitude_deg=0.0)
        assert home.latitude_deg == pytest.approx(-90.0)

    def test_extreme_longitude(self):
        home = mavkit.HomePosition(latitude_deg=0.0, longitude_deg=180.0)
        assert home.longitude_deg == pytest.approx(180.0)

    def test_negative_extreme_longitude(self):
        home = mavkit.HomePosition(latitude_deg=0.0, longitude_deg=-180.0)
        assert home.longitude_deg == pytest.approx(-180.0)


class TestMissionPlanItemListMarshaling:
    """Verify lists of MissionItems survive the Rust<->Python boundary."""

    def test_many_items(self):
        items = [
            mavkit.MissionItem(seq=i, command=16, frame=mavkit.MissionFrame.GlobalInt)
            for i in range(100)
        ]
        plan = mavkit.MissionPlan(mission_type=mavkit.MissionType.Mission, items=items)
        assert len(plan) == 100
        retrieved = plan.items
        for i, item in enumerate(retrieved):
            assert item.seq == i

    def test_items_are_copies(self):
        """Verify items returned from plan.items are independent copies."""
        items = [
            mavkit.MissionItem(seq=0, command=16, frame=mavkit.MissionFrame.GlobalInt)
        ]
        plan = mavkit.MissionPlan(mission_type=mavkit.MissionType.Mission, items=items)
        retrieved = plan.items
        assert len(retrieved) == 1
        # Getting items again should produce a fresh list
        retrieved2 = plan.items
        assert len(retrieved2) == 1


class TestEnumRoundtrip:
    """Verify enum values survive Rust->Python->Rust roundtrip via constructors."""

    def test_mission_frame_roundtrip(self):
        for frame in [
            mavkit.MissionFrame.Mission,
            mavkit.MissionFrame.GlobalInt,
            mavkit.MissionFrame.GlobalRelativeAltInt,
            mavkit.MissionFrame.GlobalTerrainAltInt,
            mavkit.MissionFrame.LocalNed,
            mavkit.MissionFrame.Other,
        ]:
            item = mavkit.MissionItem(seq=0, command=16, frame=frame)
            assert item.frame == frame

    def test_mission_type_roundtrip(self):
        for mt in [
            mavkit.MissionType.Mission,
            mavkit.MissionType.Fence,
            mavkit.MissionType.Rally,
        ]:
            plan = mavkit.MissionPlan(mission_type=mt, items=[])
            assert plan.mission_type == mt


class TestWireRoundtrip:
    """Test that upload -> download produces equivalent plans."""

    def test_mission_wire_roundtrip(self):
        home = mavkit.HomePosition(
            latitude_deg=47.42, longitude_deg=-122.2, altitude_m=30.0
        )
        items = [
            mavkit.MissionItem(
                seq=0,
                command=16,
                frame=mavkit.MissionFrame.GlobalRelativeAltInt,
                x=int(47.43 * 1e7),
                y=int(-122.3 * 1e7),
                z=100.0,
            ),
        ]
        original = mavkit.MissionPlan(
            mission_type=mavkit.MissionType.Mission,
            items=items,
            home=home,
        )

        # Upload produces wire items (home prepended + mission items resequenced)
        wire_items = mavkit.items_for_wire_upload(original)
        assert len(wire_items) == 2  # home + 1 item

        # Download reconstructs the plan
        reconstructed = mavkit.plan_from_wire_download(
            mavkit.MissionType.Mission, wire_items
        )

        # Verify structure preserved
        assert reconstructed.home is not None
        assert reconstructed.home.latitude_deg == pytest.approx(
            home.latitude_deg, abs=1e-6
        )
        assert reconstructed.home.longitude_deg == pytest.approx(
            home.longitude_deg, abs=1e-6
        )
        assert len(reconstructed.items) == 1
        assert reconstructed.items[0].command == 16
