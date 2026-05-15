use super::namespace::{MSG_ID_GPS_RAW_INT, MSG_ID_HOME_POSITION, MSG_ID_VFR_HUD};
use super::{EventMessageHandle, PeriodicMessageHandle};
use crate::command::Command;
use crate::dialect::{self, MavCmd};
use crate::observation::MessageHandle;
use crate::state::create_channels;
use crate::telemetry::TelemetryHandle;
use crate::telemetry::status_text::StatusTextEvent;
use std::time::Duration;
use tokio::sync::mpsc;

fn telemetry_with_commands() -> (
    crate::state::StateWriters,
    TelemetryHandle<'static>,
    mpsc::Receiver<Command>,
) {
    let (writers, channels) = create_channels();
    let (command_tx, command_rx) = mpsc::channel(8);
    let handles = Box::leak(Box::new(channels.telemetry_handles));
    let telemetry = TelemetryHandle::with_command_tx(handles, &command_tx);
    (writers, telemetry, command_rx)
}

#[test]
fn handle_classification() {
    fn assert_periodic<M: Clone + Send + Sync + 'static>(
        handle: PeriodicMessageHandle<M>,
    ) -> PeriodicMessageHandle<M> {
        handle
    }

    fn assert_event<M: Clone + Send + Sync + 'static>(
        handle: EventMessageHandle<M>,
    ) -> EventMessageHandle<M> {
        handle
    }

    fn assert_push_only<M: Clone + Send + Sync + 'static>(
        handle: MessageHandle<M>,
    ) -> MessageHandle<M> {
        handle
    }

    let (writers, channels) = create_channels();
    let telemetry = TelemetryHandle::new(&channels.telemetry_handles);
    let messages = telemetry.messages();

    let periodic = assert_periodic(messages.vfr_hud());
    let _ = periodic.latest();
    let _ = periodic.support();
    let _ = periodic.subscribe();
    let _request = periodic.request(Duration::from_millis(10));
    let _set_rate = periodic.set_rate(2.0);

    let _ = assert_periodic(messages.global_position_int());
    let _ = assert_periodic(messages.local_position_ned());
    let _ = assert_periodic(messages.gps_raw_int());
    let _ = assert_periodic(messages.attitude());
    let _ = assert_periodic(messages.sys_status());
    let _ = assert_periodic(messages.battery_status(0));
    let _ = assert_periodic(messages.nav_controller_output());
    let _ = assert_periodic(messages.terrain_report());
    let _ = assert_periodic(messages.rc_channels());
    let _ = assert_periodic(messages.servo_output_raw(1));

    let event = assert_event(messages.home_position());
    let _ = event.latest();
    let _ = event.support();
    let _event_request = event.request(Duration::from_millis(10));
    let _ = assert_event(messages.gps_global_origin());

    let push_only = assert_push_only(messages.status_text());
    let _ = push_only.latest();
    let _ = push_only.support();
    let _ = push_only.subscribe();

    drop(writers);
}

#[tokio::test]
async fn periodic_request_waits_for_fresh_sample() {
    let (writers, telemetry, mut command_rx) = telemetry_with_commands();
    let handle = telemetry.messages().vfr_hud();

    writers.telemetry_metrics.message_writers.vfr_hud.publish(
        dialect::VFR_HUD_DATA {
            groundspeed: 3.0,
            ..dialect::VFR_HUD_DATA::default()
        },
        None,
    );
    let stale = handle.latest().expect("stale sample should exist");

    let request_task =
        tokio::spawn(async move { handle.request(Duration::from_millis(250)).await });

    let reply = match command_rx
        .recv()
        .await
        .expect("request command should be sent")
    {
        Command::RawCommandLong {
            command,
            params,
            reply,
            ..
        } => {
            assert_eq!(command, MavCmd::MAV_CMD_REQUEST_MESSAGE);
            assert_eq!(params[0], MSG_ID_VFR_HUD as f32);
            assert_eq!(params[1], 0.0);
            reply
        }
        _ => panic!("expected RawCommandLong request command"),
    };
    let _ = reply.send(Ok(()));

    writers.telemetry_metrics.message_writers.vfr_hud.publish(
        dialect::VFR_HUD_DATA {
            groundspeed: 9.0,
            ..dialect::VFR_HUD_DATA::default()
        },
        None,
    );

    let fresh = request_task
        .await
        .expect("request task should join")
        .expect("request should return a fresh sample");
    assert!(fresh.received_at > stale.received_at);
    assert_eq!(fresh.value.groundspeed, 9.0);
}

#[tokio::test]
async fn periodic_set_rate_sends_set_message_interval() {
    let (_writers, telemetry, mut command_rx) = telemetry_with_commands();
    let handle = telemetry.messages().gps_raw_int();

    let set_rate_task = tokio::spawn(async move { handle.set_rate(5.0).await });

    let reply = match command_rx
        .recv()
        .await
        .expect("set_rate command should be sent")
    {
        Command::RawCommandLong {
            command,
            params,
            reply,
            ..
        } => {
            assert_eq!(command, MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL);
            assert_eq!(params[0], MSG_ID_GPS_RAW_INT as f32);
            assert_eq!(params[1], 200_000.0);
            reply
        }
        _ => panic!("expected RawCommandLong set_rate command"),
    };
    let _ = reply.send(Ok(()));

    set_rate_task
        .await
        .expect("set_rate task should join")
        .expect("set_rate should succeed");
}

#[tokio::test]
async fn event_driven_request_sends_request_message() {
    let (writers, telemetry, mut command_rx) = telemetry_with_commands();
    let handle = telemetry.messages().home_position();

    let request_task =
        tokio::spawn(async move { handle.request(Duration::from_millis(250)).await });

    let reply = match command_rx
        .recv()
        .await
        .expect("request command should be sent")
    {
        Command::RawCommandLong {
            command,
            params,
            reply,
            ..
        } => {
            assert_eq!(command, MavCmd::MAV_CMD_REQUEST_MESSAGE);
            assert_eq!(params[0], MSG_ID_HOME_POSITION as f32);
            reply
        }
        _ => panic!("expected RawCommandLong request command"),
    };
    let _ = reply.send(Ok(()));

    writers
        .telemetry_metrics
        .message_writers
        .home_position
        .publish(
            dialect::HOME_POSITION_DATA {
                latitude: 473_977_420,
                longitude: 85_455_940,
                altitude: 500_000,
                ..dialect::HOME_POSITION_DATA::default()
            },
            None,
        );

    let sample = request_task
        .await
        .expect("request task should join")
        .expect("request should resolve after a fresh home position sample");
    assert_eq!(sample.value.latitude, 473_977_420);
}

#[tokio::test]
async fn status_text_is_push_only_future_stream() {
    let (writers, channels) = create_channels();
    let telemetry = TelemetryHandle::new(&channels.telemetry_handles);
    let handle = telemetry.messages().status_text();

    writers
        .telemetry_metrics
        .message_writers
        .status_text
        .publish(
            StatusTextEvent {
                text: "before".to_string(),
                severity: dialect::MavSeverity::MAV_SEVERITY_INFO,
                id: 0,
                source_system: 1,
                source_component: 1,
            },
            None,
        );

    let mut subscription = handle.subscribe();

    writers
        .telemetry_metrics
        .message_writers
        .status_text
        .publish(
            StatusTextEvent {
                text: "after".to_string(),
                severity: dialect::MavSeverity::MAV_SEVERITY_INFO,
                id: 0,
                source_system: 1,
                source_component: 1,
            },
            None,
        );

    let sample = subscription
        .recv()
        .await
        .expect("broadcast-backed status_text should emit future events");
    assert_eq!(sample.value.text, "after");
}

#[tokio::test]
async fn status_text_subscribe_does_not_coalesce_repeated_events() {
    let (writers, channels) = create_channels();
    let telemetry = TelemetryHandle::new(&channels.telemetry_handles);
    let handle = telemetry.messages().status_text();
    let mut subscription = handle.subscribe();

    writers
        .telemetry_metrics
        .message_writers
        .status_text
        .publish(
            StatusTextEvent {
                text: "repeat".to_string(),
                severity: dialect::MavSeverity::MAV_SEVERITY_INFO,
                id: 0,
                source_system: 1,
                source_component: 1,
            },
            None,
        );
    writers
        .telemetry_metrics
        .message_writers
        .status_text
        .publish(
            StatusTextEvent {
                text: "repeat".to_string(),
                severity: dialect::MavSeverity::MAV_SEVERITY_INFO,
                id: 0,
                source_system: 1,
                source_component: 1,
            },
            None,
        );

    let first = subscription
        .recv()
        .await
        .expect("first event should arrive");
    let second = subscription
        .recv()
        .await
        .expect("second event should arrive");
    assert_eq!(first.value.text, "repeat");
    assert_eq!(second.value.text, "repeat");
}
