"""Tests for parameter types and param file I/O functions."""

import pytest

import mavkit


class TestParseParamFile:
    def test_simple_file(self):
        contents = "BATT_ARM_VOLT,10.5\nBATT_CAPACITY,5000\n"
        result = mavkit.parse_param_file(contents)
        assert isinstance(result, dict)
        assert result["BATT_ARM_VOLT"] == pytest.approx(10.5)
        assert result["BATT_CAPACITY"] == pytest.approx(5000.0)

    def test_comments_and_blanks(self):
        contents = "# comment\n\nBATT_ARM_VOLT,10.5\n# another comment\n"
        result = mavkit.parse_param_file(contents)
        assert len(result) == 1
        assert "BATT_ARM_VOLT" in result

    def test_empty_file(self):
        result = mavkit.parse_param_file("")
        assert result == {}

    def test_invalid_format_raises(self):
        with pytest.raises(ValueError):
            mavkit.parse_param_file("INVALID_LINE_NO_COMMA")

    def test_invalid_value_raises(self):
        with pytest.raises(ValueError):
            mavkit.parse_param_file("PARAM,not_a_number")

    def test_whitespace_handling(self):
        contents = "  BATT_ARM_VOLT , 10.5 \n"
        result = mavkit.parse_param_file(contents)
        assert len(result) == 1

    def test_negative_values(self):
        contents = "TRIM_PITCH,-2.5\n"
        result = mavkit.parse_param_file(contents)
        assert result["TRIM_PITCH"] == pytest.approx(-2.5)

    def test_zero_value(self):
        contents = "PARAM,0\n"
        result = mavkit.parse_param_file(contents)
        assert result["PARAM"] == pytest.approx(0.0)

    def test_scientific_notation(self):
        contents = "PARAM,1.5e3\n"
        result = mavkit.parse_param_file(contents)
        assert result["PARAM"] == pytest.approx(1500.0)


class TestFormatAndParseRoundtrip:
    """Test that format_param_file output can be parsed back by parse_param_file."""

    def test_roundtrip(self):
        # parse_param_file returns dict[str, float], format_param_file takes ParamStore
        # We can't directly construct a ParamStore with params from Python,
        # but we can test parse_param_file with known format output.
        contents = "BATT_ARM_VOLT,10.5\nBATT_CAPACITY,5000\nTRIM_PITCH,-2.5\n"
        parsed = mavkit.parse_param_file(contents)
        assert len(parsed) == 3
        assert parsed["BATT_ARM_VOLT"] == pytest.approx(10.5)
        assert parsed["BATT_CAPACITY"] == pytest.approx(5000.0)
        assert parsed["TRIM_PITCH"] == pytest.approx(-2.5)


class TestVehicleConfig:
    def test_defaults(self):
        config = mavkit.VehicleConfig()
        assert config.gcs_system_id == 255
        assert config.gcs_component_id == 190
        assert config.auto_request_home is True
        assert config.command_buffer_size == 32
        assert config.connect_timeout_secs == pytest.approx(10.0)
        retry = config.retry_policy
        assert isinstance(retry, mavkit.RetryPolicy)

    def test_custom_values(self):
        retry = mavkit.RetryPolicy(
            request_timeout_ms=2000, item_timeout_ms=300, max_retries=3
        )
        config = mavkit.VehicleConfig(
            gcs_system_id=1,
            gcs_component_id=1,
            auto_request_home=False,
            command_buffer_size=64,
            connect_timeout_secs=10.0,
            retry_policy=retry,
        )
        assert config.gcs_system_id == 1
        assert config.gcs_component_id == 1
        assert config.auto_request_home is False
        assert config.command_buffer_size == 64
        assert config.connect_timeout_secs == pytest.approx(10.0)
        assert config.retry_policy.request_timeout_ms == 2000

    def test_repr(self):
        config = mavkit.VehicleConfig()
        r = repr(config)
        assert "VehicleConfig" in r

    def test_frozen(self):
        config = mavkit.VehicleConfig()
        with pytest.raises(AttributeError):
            config.gcs_system_id = 1  # type: ignore[misc]


class TestParamsBindings:
    def test_params_handle_exposes_state_and_operation_methods(self):
        params_handle = getattr(mavkit, "ParamsHandle")
        for name in (
            "latest",
            "wait",
            "wait_timeout",
            "subscribe",
            "download_all",
            "write",
            "write_batch",
        ):
            assert hasattr(params_handle, name)
            assert callable(getattr(params_handle, name))

    def test_param_wrapper_classes_are_exported(self):
        for name in (
            "SyncState",
            "ParamOperationKind",
            "ParamState",
            "ParamStateSubscription",
            "ParamProgressSubscription",
            "ParamDownloadOp",
            "ParamWriteBatchOp",
        ):
            assert hasattr(mavkit, name)

    def test_param_progress_exposes_download_and_write_fields(self):
        for name in (
            "phase",
            "received",
            "expected",
            "expected_count",
            "index",
            "total",
            "name",
        ):
            assert hasattr(mavkit.ParamProgress, name)
