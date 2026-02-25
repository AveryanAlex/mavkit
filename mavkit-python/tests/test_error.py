"""Tests for MavkitError exception type."""

import mavkit


class TestMavkitError:
    def test_is_exception_subclass(self):
        assert issubclass(mavkit.MavkitError, Exception)

    def test_can_be_raised_and_caught(self):
        try:
            raise mavkit.MavkitError("test error message")
        except mavkit.MavkitError as e:
            assert str(e) == "test error message"
        except Exception:
            raise AssertionError("MavkitError should be caught by except MavkitError")

    def test_caught_by_generic_exception(self):
        try:
            raise mavkit.MavkitError("test")
        except Exception as e:
            assert isinstance(e, mavkit.MavkitError)

    def test_error_message_preserved(self):
        msg = "connection timed out after 30s"
        try:
            raise mavkit.MavkitError(msg)
        except mavkit.MavkitError as e:
            assert msg in str(e)
