import inspect

import mavkit


class TestArduPilotBindings:
    def test_ardupilot_handle_exposes_domain_methods(self):
        ardupilot_handle = getattr(mavkit, "ArduPilotHandle")
        for name in (
            "copter",
            "plane",
            "rover",
            "sub",
            "guided",
            "request_prearm_checks",
            "motor_test",
            "preflight_calibration",
            "reboot",
            "reboot_to_bootloader",
            "start_mag_cal",
            "accept_mag_cal",
            "cancel_mag_cal",
        ):
            assert hasattr(ardupilot_handle, name)
            assert callable(getattr(ardupilot_handle, name))

    def test_ardupilot_wrapper_classes_are_exported(self):
        for name in (
            "ArduPilotHandle",
            "ArduCopterHandle",
            "ArduPlaneHandle",
            "ArduPlaneVtolHandle",
            "ArduRoverHandle",
            "ArduSubHandle",
            "ArduGuidedSession",
            "ArduCopterGuidedHandle",
            "ArduPlaneGuidedHandle",
            "ArduPlaneVtolGuidedHandle",
            "ArduRoverGuidedHandle",
            "ArduSubGuidedHandle",
        ):
            assert hasattr(mavkit, name)

    def test_ardupilot_guided_entrypoint_signature_stays_session_based(self):
        guided = getattr(mavkit.ArduPilotHandle, "guided")
        assert str(inspect.signature(guided)) == "(self, /)"

    def test_vehicle_keeps_guided_actions_off_root_api(self):
        assert not hasattr(mavkit.Vehicle, "guided")
        assert not hasattr(mavkit.Vehicle, "motor_test")
        assert not hasattr(mavkit.Vehicle, "goto")


class TestArduGuidedBindings:
    def test_guided_session_exposes_close_and_narrowing(self):
        guided_session = getattr(mavkit, "ArduGuidedSession")
        for name in ("close", "copter", "plane", "rover", "sub"):
            assert hasattr(guided_session, name)
            assert callable(getattr(guided_session, name))

    def test_guided_session_supports_async_context_manager(self):
        guided_session = getattr(mavkit, "ArduGuidedSession")
        assert hasattr(guided_session, "__aenter__")
        assert callable(getattr(guided_session, "__aenter__"))
        assert hasattr(guided_session, "__aexit__")
        assert callable(getattr(guided_session, "__aexit__"))

    def test_guided_session_close_signature_is_stable(self):
        close = getattr(mavkit.ArduGuidedSession, "close")
        assert str(inspect.signature(close)) == "(self, /)"

    def test_plane_handles_expose_vtol_narrowing_only_on_plane(self):
        plane_handle = getattr(mavkit, "ArduPlaneHandle")
        copter_handle = getattr(mavkit, "ArduCopterHandle")
        rover_handle = getattr(mavkit, "ArduRoverHandle")
        sub_handle = getattr(mavkit, "ArduSubHandle")
        plane_guided_handle = getattr(mavkit, "ArduPlaneGuidedHandle")
        copter_guided_handle = getattr(mavkit, "ArduCopterGuidedHandle")
        rover_guided_handle = getattr(mavkit, "ArduRoverGuidedHandle")
        sub_guided_handle = getattr(mavkit, "ArduSubGuidedHandle")

        assert hasattr(plane_handle, "vtol")
        assert callable(getattr(plane_handle, "vtol"))
        assert hasattr(plane_guided_handle, "vtol")
        assert callable(getattr(plane_guided_handle, "vtol"))

        assert not hasattr(copter_handle, "vtol")
        assert not hasattr(rover_handle, "vtol")
        assert not hasattr(sub_handle, "vtol")
        assert not hasattr(copter_guided_handle, "vtol")
        assert not hasattr(rover_guided_handle, "vtol")
        assert not hasattr(sub_guided_handle, "vtol")

    def test_guided_specific_handles_expose_session_only_actions(self):
        copter_guided_handle = getattr(mavkit, "ArduCopterGuidedHandle")
        for name in ("takeoff", "goto", "goto_msl", "set_velocity_ned", "hold"):
            assert hasattr(copter_guided_handle, name)
            assert callable(getattr(copter_guided_handle, name))

        plane_guided_handle = getattr(mavkit, "ArduPlaneGuidedHandle")
        for name in ("reposition", "reposition_rel_home"):
            assert hasattr(plane_guided_handle, name)
            assert callable(getattr(plane_guided_handle, name))

        plane_vtol_guided_handle = getattr(mavkit, "ArduPlaneVtolGuidedHandle")
        for name in ("takeoff", "hold"):
            assert hasattr(plane_vtol_guided_handle, name)
            assert callable(getattr(plane_vtol_guided_handle, name))

        rover_guided_handle = getattr(mavkit, "ArduRoverGuidedHandle")
        for name in ("drive_to", "drive", "hold"):
            assert hasattr(rover_guided_handle, name)
            assert callable(getattr(rover_guided_handle, name))

        sub_guided_handle = getattr(mavkit, "ArduSubGuidedHandle")
        for name in ("goto_depth", "set_velocity_body", "hold"):
            assert hasattr(sub_guided_handle, name)
            assert callable(getattr(sub_guided_handle, name))
