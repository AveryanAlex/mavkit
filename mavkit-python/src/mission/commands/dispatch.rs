use pyo3::prelude::*;
use pyo3::types::PyAny;

use super::condition::{PyCondDelay, PyCondDistance, PyCondYaw};
use super::do_cmd::*;
use super::nav::*;
use super::raw::PyRawMissionCommand;

pub(crate) fn typed_command_from_py(
    command: &Bound<'_, PyAny>,
) -> PyResult<Option<mavkit::MissionCommand>> {
    if let Ok(nav_waypoint) = command.extract::<PyRef<'_, PyNavWaypoint>>() {
        return Ok(Some(nav_waypoint.inner.clone().into()));
    }

    if let Ok(nav_takeoff) = command.extract::<PyRef<'_, PyNavTakeoff>>() {
        return Ok(Some(nav_takeoff.inner.clone().into()));
    }

    if let Ok(nav_land) = command.extract::<PyRef<'_, PyNavLand>>() {
        return Ok(Some(nav_land.inner.clone().into()));
    }

    if let Ok(nav_loiter_time) = command.extract::<PyRef<'_, PyNavLoiterTime>>() {
        return Ok(Some(nav_loiter_time.inner.clone().into()));
    }

    if let Ok(nav_guided_enable) = command.extract::<PyRef<'_, PyNavGuidedEnable>>() {
        return Ok(Some(nav_guided_enable.inner.clone().into()));
    }

    if let Ok(nav_spline_waypoint) = command.extract::<PyRef<'_, PyNavSplineWaypoint>>() {
        return Ok(Some(nav_spline_waypoint.inner.clone().into()));
    }

    if let Ok(nav_arc_waypoint) = command.extract::<PyRef<'_, PyNavArcWaypoint>>() {
        return Ok(Some(nav_arc_waypoint.inner.clone().into()));
    }

    if let Ok(nav_loiter_unlimited) = command.extract::<PyRef<'_, PyNavLoiterUnlimited>>() {
        return Ok(Some(nav_loiter_unlimited.inner.clone().into()));
    }

    if let Ok(nav_loiter_turns) = command.extract::<PyRef<'_, PyNavLoiterTurns>>() {
        return Ok(Some(nav_loiter_turns.inner.clone().into()));
    }

    if let Ok(nav_loiter_to_alt) = command.extract::<PyRef<'_, PyNavLoiterToAlt>>() {
        return Ok(Some(nav_loiter_to_alt.inner.clone().into()));
    }

    if let Ok(nav_continue_and_change_alt) =
        command.extract::<PyRef<'_, PyNavContinueAndChangeAlt>>()
    {
        return Ok(Some(nav_continue_and_change_alt.inner.clone().into()));
    }

    if let Ok(nav_delay) = command.extract::<PyRef<'_, PyNavDelay>>() {
        return Ok(Some(nav_delay.inner.clone().into()));
    }

    if let Ok(nav_altitude_wait) = command.extract::<PyRef<'_, PyNavAltitudeWait>>() {
        return Ok(Some(nav_altitude_wait.inner.clone().into()));
    }

    if let Ok(nav_vtol_takeoff) = command.extract::<PyRef<'_, PyNavVtolTakeoff>>() {
        return Ok(Some(nav_vtol_takeoff.inner.clone().into()));
    }

    if let Ok(nav_vtol_land) = command.extract::<PyRef<'_, PyNavVtolLand>>() {
        return Ok(Some(nav_vtol_land.inner.clone().into()));
    }

    if let Ok(nav_payload_place) = command.extract::<PyRef<'_, PyNavPayloadPlace>>() {
        return Ok(Some(nav_payload_place.inner.clone().into()));
    }

    if let Ok(nav_set_yaw_speed) = command.extract::<PyRef<'_, PyNavSetYawSpeed>>() {
        return Ok(Some(nav_set_yaw_speed.inner.clone().into()));
    }

    if let Ok(nav_script_time) = command.extract::<PyRef<'_, PyNavScriptTime>>() {
        return Ok(Some(nav_script_time.inner.clone().into()));
    }

    if let Ok(nav_attitude_time) = command.extract::<PyRef<'_, PyNavAttitudeTime>>() {
        return Ok(Some(nav_attitude_time.inner.clone().into()));
    }

    if command.extract::<PyRef<'_, PyNavReturnToLaunch>>().is_ok() {
        return Ok(Some(
            mavkit::mission::commands::NavCommand::ReturnToLaunch.into(),
        ));
    }

    if let Ok(do_change_speed) = command.extract::<PyRef<'_, PyDoChangeSpeed>>() {
        return Ok(Some(do_change_speed.inner.clone().into()));
    }

    if let Ok(do_set_home) = command.extract::<PyRef<'_, PyDoSetHome>>() {
        return Ok(Some(do_set_home.inner.clone().into()));
    }

    if let Ok(do_set_relay) = command.extract::<PyRef<'_, PyDoSetRelay>>() {
        return Ok(Some(do_set_relay.inner.clone().into()));
    }

    if command.extract::<PyRef<'_, PyDoSetRoiNone>>().is_ok() {
        return Ok(Some(
            mavkit::mission::commands::DoCommand::SetRoiNone.into(),
        ));
    }

    if let Ok(do_jump) = command.extract::<PyRef<'_, PyDoJump>>() {
        return Ok(Some(do_jump.inner.clone().into()));
    }

    if let Ok(do_jump_tag) = command.extract::<PyRef<'_, PyDoJumpTag>>() {
        return Ok(Some(do_jump_tag.inner.clone().into()));
    }

    if let Ok(do_tag) = command.extract::<PyRef<'_, PyDoTag>>() {
        return Ok(Some(do_tag.inner.clone().into()));
    }

    if let Ok(do_pause_continue) = command.extract::<PyRef<'_, PyDoPauseContinue>>() {
        return Ok(Some(do_pause_continue.inner.clone().into()));
    }

    if let Ok(do_set_reverse) = command.extract::<PyRef<'_, PyDoSetReverse>>() {
        return Ok(Some(do_set_reverse.inner.clone().into()));
    }

    if let Ok(do_land_start) = command.extract::<PyRef<'_, PyDoLandStart>>() {
        return Ok(Some(do_land_start.inner.clone().into()));
    }

    if let Ok(do_return_path_start) = command.extract::<PyRef<'_, PyDoReturnPathStart>>() {
        return Ok(Some(do_return_path_start.inner.clone().into()));
    }

    if let Ok(do_go_around) = command.extract::<PyRef<'_, PyDoGoAround>>() {
        return Ok(Some(do_go_around.inner.clone().into()));
    }

    if let Ok(do_set_roi_location) = command.extract::<PyRef<'_, PyDoSetRoiLocation>>() {
        return Ok(Some(do_set_roi_location.inner.clone().into()));
    }

    if let Ok(do_set_roi) = command.extract::<PyRef<'_, PyDoSetRoi>>() {
        return Ok(Some(do_set_roi.inner.clone().into()));
    }

    if let Ok(do_mount_control) = command.extract::<PyRef<'_, PyDoMountControl>>() {
        return Ok(Some(do_mount_control.inner.clone().into()));
    }

    if let Ok(do_gimbal_manager_pitch_yaw) =
        command.extract::<PyRef<'_, PyDoGimbalManagerPitchYaw>>()
    {
        return Ok(Some(do_gimbal_manager_pitch_yaw.inner.clone().into()));
    }

    if let Ok(do_cam_trigger_distance) = command.extract::<PyRef<'_, PyDoCamTriggerDistance>>() {
        return Ok(Some(do_cam_trigger_distance.inner.clone().into()));
    }

    if let Ok(do_digicam_configure) = command.extract::<PyRef<'_, PyDoDigicamConfigure>>() {
        return Ok(Some(do_digicam_configure.inner.clone().into()));
    }

    if let Ok(do_digicam_control) = command.extract::<PyRef<'_, PyDoDigicamControl>>() {
        return Ok(Some(do_digicam_control.inner.clone().into()));
    }

    if let Ok(do_fence_enable) = command.extract::<PyRef<'_, PyDoFenceEnable>>() {
        return Ok(Some(do_fence_enable.inner.clone().into()));
    }

    if let Ok(do_parachute) = command.extract::<PyRef<'_, PyDoParachute>>() {
        return Ok(Some(do_parachute.inner.clone().into()));
    }

    if let Ok(do_gripper) = command.extract::<PyRef<'_, PyDoGripper>>() {
        return Ok(Some(do_gripper.inner.clone().into()));
    }

    if let Ok(do_sprayer) = command.extract::<PyRef<'_, PyDoSprayer>>() {
        return Ok(Some(do_sprayer.inner.clone().into()));
    }

    if let Ok(do_winch) = command.extract::<PyRef<'_, PyDoWinch>>() {
        return Ok(Some(do_winch.inner.clone().into()));
    }

    if let Ok(do_engine_control) = command.extract::<PyRef<'_, PyDoEngineControl>>() {
        return Ok(Some(do_engine_control.inner.clone().into()));
    }

    if let Ok(do_inverted_flight) = command.extract::<PyRef<'_, PyDoInvertedFlight>>() {
        return Ok(Some(do_inverted_flight.inner.clone().into()));
    }

    if let Ok(do_autotune_enable) = command.extract::<PyRef<'_, PyDoAutotuneEnable>>() {
        return Ok(Some(do_autotune_enable.inner.clone().into()));
    }

    if let Ok(do_set_servo) = command.extract::<PyRef<'_, PyDoSetServo>>() {
        return Ok(Some(do_set_servo.inner.clone().into()));
    }

    if let Ok(do_repeat_servo) = command.extract::<PyRef<'_, PyDoRepeatServo>>() {
        return Ok(Some(do_repeat_servo.inner.clone().into()));
    }

    if let Ok(do_repeat_relay) = command.extract::<PyRef<'_, PyDoRepeatRelay>>() {
        return Ok(Some(do_repeat_relay.inner.clone().into()));
    }

    if let Ok(do_set_resume_repeat_dist) = command.extract::<PyRef<'_, PyDoSetResumeRepeatDist>>() {
        return Ok(Some(do_set_resume_repeat_dist.inner.clone().into()));
    }

    if let Ok(do_aux_function) = command.extract::<PyRef<'_, PyDoAuxFunction>>() {
        return Ok(Some(do_aux_function.inner.clone().into()));
    }

    if let Ok(do_send_script_message) = command.extract::<PyRef<'_, PyDoSendScriptMessage>>() {
        return Ok(Some(do_send_script_message.inner.clone().into()));
    }

    if let Ok(do_image_start_capture) = command.extract::<PyRef<'_, PyDoImageStartCapture>>() {
        return Ok(Some(do_image_start_capture.inner.clone().into()));
    }

    if let Ok(do_image_stop_capture) = command.extract::<PyRef<'_, PyDoImageStopCapture>>() {
        return Ok(Some(do_image_stop_capture.inner.clone().into()));
    }

    if let Ok(do_video_start_capture) = command.extract::<PyRef<'_, PyDoVideoStartCapture>>() {
        return Ok(Some(do_video_start_capture.inner.clone().into()));
    }

    if let Ok(do_video_stop_capture) = command.extract::<PyRef<'_, PyDoVideoStopCapture>>() {
        return Ok(Some(do_video_stop_capture.inner.clone().into()));
    }

    if let Ok(do_set_camera_zoom) = command.extract::<PyRef<'_, PyDoSetCameraZoom>>() {
        return Ok(Some(do_set_camera_zoom.inner.clone().into()));
    }

    if let Ok(do_set_camera_focus) = command.extract::<PyRef<'_, PyDoSetCameraFocus>>() {
        return Ok(Some(do_set_camera_focus.inner.clone().into()));
    }

    if let Ok(do_set_camera_source) = command.extract::<PyRef<'_, PyDoSetCameraSource>>() {
        return Ok(Some(do_set_camera_source.inner.clone().into()));
    }

    if let Ok(do_guided_limits) = command.extract::<PyRef<'_, PyDoGuidedLimits>>() {
        return Ok(Some(do_guided_limits.inner.clone().into()));
    }

    if let Ok(do_vtol_transition) = command.extract::<PyRef<'_, PyDoVtolTransition>>() {
        return Ok(Some(do_vtol_transition.inner.clone().into()));
    }

    if let Ok(cond_delay) = command.extract::<PyRef<'_, PyCondDelay>>() {
        return Ok(Some(cond_delay.inner.clone().into()));
    }

    if let Ok(cond_distance) = command.extract::<PyRef<'_, PyCondDistance>>() {
        return Ok(Some(cond_distance.inner.clone().into()));
    }

    if let Ok(cond_yaw) = command.extract::<PyRef<'_, PyCondYaw>>() {
        return Ok(Some(cond_yaw.inner.clone().into()));
    }

    if let Ok(raw) = command.extract::<PyRef<'_, PyRawMissionCommand>>() {
        return Ok(Some(raw.inner.clone().into()));
    }

    Ok(None)
}
