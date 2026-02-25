"""Tests for mavkit enum types — verify all variants exist and equality works."""

import mavkit


class TestSystemStatus:
    def test_all_variants_exist(self):
        variants = [
            mavkit.SystemStatus.Unknown,
            mavkit.SystemStatus.Boot,
            mavkit.SystemStatus.Calibrating,
            mavkit.SystemStatus.Standby,
            mavkit.SystemStatus.Active,
            mavkit.SystemStatus.Critical,
            mavkit.SystemStatus.Emergency,
            mavkit.SystemStatus.Poweroff,
        ]
        assert len(variants) == 8

    def test_equality(self):
        assert mavkit.SystemStatus.Active == mavkit.SystemStatus.Active

    def test_inequality(self):
        assert mavkit.SystemStatus.Active != mavkit.SystemStatus.Standby


class TestVehicleType:
    def test_all_variants_exist(self):
        variants = [
            mavkit.VehicleType.Unknown,
            mavkit.VehicleType.FixedWing,
            mavkit.VehicleType.Quadrotor,
            mavkit.VehicleType.Hexarotor,
            mavkit.VehicleType.Octorotor,
            mavkit.VehicleType.Tricopter,
            mavkit.VehicleType.Helicopter,
            mavkit.VehicleType.Coaxial,
            mavkit.VehicleType.GroundRover,
            mavkit.VehicleType.Generic,
        ]
        assert len(variants) == 10

    def test_equality(self):
        assert mavkit.VehicleType.Quadrotor == mavkit.VehicleType.Quadrotor

    def test_inequality(self):
        assert mavkit.VehicleType.Quadrotor != mavkit.VehicleType.FixedWing


class TestAutopilotType:
    def test_all_variants_exist(self):
        variants = [
            mavkit.AutopilotType.Unknown,
            mavkit.AutopilotType.Generic,
            mavkit.AutopilotType.ArduPilotMega,
            mavkit.AutopilotType.Px4,
        ]
        assert len(variants) == 4


class TestGpsFixType:
    def test_all_variants_exist(self):
        variants = [
            mavkit.GpsFixType.NoFix,
            mavkit.GpsFixType.Fix2d,
            mavkit.GpsFixType.Fix3d,
            mavkit.GpsFixType.Dgps,
            mavkit.GpsFixType.RtkFloat,
            mavkit.GpsFixType.RtkFixed,
        ]
        assert len(variants) == 6


class TestMissionType:
    def test_all_variants_exist(self):
        variants = [
            mavkit.MissionType.Mission,
            mavkit.MissionType.Fence,
            mavkit.MissionType.Rally,
        ]
        assert len(variants) == 3


class TestMissionFrame:
    def test_all_variants_exist(self):
        variants = [
            mavkit.MissionFrame.Mission,
            mavkit.MissionFrame.GlobalInt,
            mavkit.MissionFrame.GlobalRelativeAltInt,
            mavkit.MissionFrame.GlobalTerrainAltInt,
            mavkit.MissionFrame.LocalNed,
            mavkit.MissionFrame.Other,
        ]
        assert len(variants) == 6


class TestIssueSeverity:
    def test_all_variants_exist(self):
        variants = [
            mavkit.IssueSeverity.Error,
            mavkit.IssueSeverity.Warning,
        ]
        assert len(variants) == 2


class TestTransferDirection:
    def test_all_variants_exist(self):
        variants = [
            mavkit.TransferDirection.Upload,
            mavkit.TransferDirection.Download,
        ]
        assert len(variants) == 2


class TestTransferPhase:
    def test_all_variants_exist(self):
        variants = [
            mavkit.TransferPhase.Idle,
            mavkit.TransferPhase.RequestCount,
            mavkit.TransferPhase.TransferItems,
            mavkit.TransferPhase.AwaitAck,
            mavkit.TransferPhase.Completed,
            mavkit.TransferPhase.Failed,
            mavkit.TransferPhase.Cancelled,
        ]
        assert len(variants) == 7


class TestParamTransferPhase:
    def test_all_variants_exist(self):
        variants = [
            mavkit.ParamTransferPhase.Idle,
            mavkit.ParamTransferPhase.Downloading,
            mavkit.ParamTransferPhase.Writing,
            mavkit.ParamTransferPhase.Completed,
            mavkit.ParamTransferPhase.Failed,
        ]
        assert len(variants) == 5


class TestParamType:
    def test_all_variants_exist(self):
        variants = [
            mavkit.ParamType.Uint8,
            mavkit.ParamType.Int8,
            mavkit.ParamType.Uint16,
            mavkit.ParamType.Int16,
            mavkit.ParamType.Uint32,
            mavkit.ParamType.Int32,
            mavkit.ParamType.Real32,
        ]
        assert len(variants) == 7
