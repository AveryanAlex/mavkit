use super::handles::TelemetryMetricHandles;
use super::status_text::{StatusTextEvent, create_status_text_backing_store};
use crate::command::Command;
use crate::dialect::{self, MavCmd};
use crate::observation::{
    MessageHandle, MessageSample, ObservationHandle, ObservationSubscription, ObservationWriter,
    SupportState,
};
use crate::{VehicleError, VehicleTimestamp};
use std::collections::HashMap;
use std::hash::Hash;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use tokio::sync::{mpsc, oneshot};

const MSG_ID_LOCAL_POSITION_NED: u32 = 32;
const MSG_ID_GLOBAL_POSITION_INT: u32 = 33;
const MSG_ID_SERVO_OUTPUT_RAW: u32 = 36;
const MSG_ID_GPS_GLOBAL_ORIGIN: u32 = 49;
const MSG_ID_NAV_CONTROLLER_OUTPUT: u32 = 62;
const MSG_ID_RC_CHANNELS: u32 = 65;
const MSG_ID_VFR_HUD: u32 = 74;
const MSG_ID_TERRAIN_REPORT: u32 = 136;
const MSG_ID_BATTERY_STATUS: u32 = 147;
const MSG_ID_ATTITUDE: u32 = 30;
const MSG_ID_GPS_RAW_INT: u32 = 24;
const MSG_ID_SYS_STATUS: u32 = 1;
const MSG_ID_HOME_POSITION: u32 = 242;

#[derive(Clone)]
pub(crate) struct MessageSlotWriter<M: Clone + Send + Sync + 'static> {
    sample: ObservationWriter<MessageSample<M>>,
    support: ObservationWriter<SupportState>,
}

impl<M: Clone + Send + Sync + 'static> MessageSlotWriter<M> {
    pub(crate) fn publish(&self, value: M, vehicle_time: Option<VehicleTimestamp>) {
        let _ = self.support.publish(SupportState::Supported);
        let _ = self.sample.publish(MessageSample {
            value,
            received_at: Instant::now(),
            vehicle_time,
        });
    }
}

fn message_slot_watch<M: Clone + Send + Sync + 'static>() -> (MessageSlotWriter<M>, MessageHandle<M>)
{
    let (sample_writer, sample_observation) = ObservationHandle::watch();
    let (support_writer, support_observation) = ObservationHandle::watch();
    let _ = support_writer.publish(SupportState::Unknown);

    (
        MessageSlotWriter {
            sample: sample_writer,
            support: support_writer,
        },
        MessageHandle::new(sample_observation, support_observation),
    )
}

#[derive(Clone)]
struct IndexedMessageEntry<M: Clone + Send + Sync + 'static> {
    writer: MessageSlotWriter<M>,
    handle: MessageHandle<M>,
}

#[derive(Clone)]
pub(crate) struct IndexedMessageFamily<K, M>
where
    K: Eq + Hash + Copy + Send + Sync + 'static,
    M: Clone + Send + Sync + 'static,
{
    entries: Arc<Mutex<HashMap<K, IndexedMessageEntry<M>>>>,
    factory: fn() -> (MessageSlotWriter<M>, MessageHandle<M>),
}

impl<K, M> IndexedMessageFamily<K, M>
where
    K: Eq + Hash + Copy + Send + Sync + 'static,
    M: Clone + Send + Sync + 'static,
{
    pub(crate) fn watch() -> Self {
        Self {
            entries: Arc::new(Mutex::new(HashMap::new())),
            factory: message_slot_watch::<M>,
        }
    }

    fn entry(&self, key: K) -> IndexedMessageEntry<M> {
        let mut entries = self
            .entries
            .lock()
            .expect("indexed message family mutex poisoned");
        let factory = self.factory;
        entries
            .entry(key)
            .or_insert_with(|| {
                let (writer, handle) = factory();
                IndexedMessageEntry { writer, handle }
            })
            .clone()
    }

    pub(crate) fn handle(&self, key: K) -> MessageHandle<M> {
        self.entry(key).handle
    }

    pub(crate) fn publish(&self, key: K, value: M, vehicle_time: Option<VehicleTimestamp>) {
        self.entry(key).writer.publish(value, vehicle_time);
    }
}

#[derive(Clone, Default)]
pub(crate) struct MessageCommandHub {
    command_tx: Arc<Mutex<Option<mpsc::Sender<Command>>>>,
}

impl MessageCommandHub {
    pub(crate) fn bind(&self, command_tx: &mpsc::Sender<Command>) {
        *self
            .command_tx
            .lock()
            .expect("message command hub mutex poisoned") = Some(command_tx.clone());
    }

    async fn send_raw_command_long(
        &self,
        command: MavCmd,
        params: [f32; 7],
    ) -> Result<(), VehicleError> {
        let command_tx = self
            .command_tx
            .lock()
            .expect("message command hub mutex poisoned")
            .clone()
            .ok_or_else(|| {
                VehicleError::Unsupported(
                    "telemetry message command bridge is not bound to a vehicle".to_string(),
                )
            })?;

        let (reply_tx, reply_rx) = oneshot::channel();
        command_tx
            .send(Command::RawCommandLong {
                command,
                params,
                reply: reply_tx,
            })
            .await
            .map_err(|_| VehicleError::Disconnected)?;

        reply_rx.await.map_err(|_| VehicleError::Disconnected)?
    }
}

/// Generates the paired `TelemetryMessageHandles` and `TelemetryMessageWriters` structs from a
/// single canonical field list, eliminating the need to keep two parallel struct definitions in
/// sync by hand.
///
/// Syntax:
/// ```text
/// define_messages! {
///     slot    { field_name: Type, ... }
///     indexed { field_name: Key, Type, ... }
/// }
/// ```
///
/// `slot` entries expand to `MessageHandle<Type>` on handles and `MessageSlotWriter<Type>` on
/// writers. `indexed` entries expand to `IndexedMessageFamily<Key, Type>` on **both** structs
/// (shared instance, not split into handle/writer).
///
/// Special fields (`commands`, `status_text`) are hardcoded in the expansion because they have
/// asymmetric types between the two structs.
macro_rules! define_messages {
    (
        slot    { $( $sf:ident : $st:ty ),* $(,)? }
        indexed { $( $if:ident : $ik:ty , $it:ty ),* $(,)? }
    ) => {
        #[derive(Clone)]
        pub(crate) struct TelemetryMessageHandles {
            pub(crate) commands: MessageCommandHub,
            $( pub(crate) $sf: MessageHandle<$st>, )*
            $( pub(crate) $if: IndexedMessageFamily<$ik, $it>, )*
            pub(crate) status_text: MessageHandle<StatusTextEvent>,
        }

        #[derive(Clone)]
        pub(crate) struct TelemetryMessageWriters {
            $( pub(crate) $sf: MessageSlotWriter<$st>, )*
            $( pub(crate) $if: IndexedMessageFamily<$ik, $it>, )*
            pub(crate) status_text: super::status_text::StatusTextWriter,
        }
    };
}

define_messages! {
    slot {
        vfr_hud: dialect::VFR_HUD_DATA,
        global_position_int: dialect::GLOBAL_POSITION_INT_DATA,
        local_position_ned: dialect::LOCAL_POSITION_NED_DATA,
        gps_raw_int: dialect::GPS_RAW_INT_DATA,
        attitude: dialect::ATTITUDE_DATA,
        sys_status: dialect::SYS_STATUS_DATA,
        nav_controller_output: dialect::NAV_CONTROLLER_OUTPUT_DATA,
        terrain_report: dialect::TERRAIN_REPORT_DATA,
        rc_channels: dialect::RC_CHANNELS_DATA,
        home_position: dialect::HOME_POSITION_DATA,
        gps_global_origin: dialect::GPS_GLOBAL_ORIGIN_DATA,
    }
    indexed {
        battery_status: u8, dialect::BATTERY_STATUS_DATA,
        servo_output_raw: u8, dialect::SERVO_OUTPUT_RAW_DATA,
    }
}

pub(crate) fn create_telemetry_message_backing_stores()
-> (TelemetryMessageWriters, TelemetryMessageHandles) {
    let (vfr_hud_w, vfr_hud_h) = message_slot_watch();
    let (global_position_int_w, global_position_int_h) = message_slot_watch();
    let (local_position_ned_w, local_position_ned_h) = message_slot_watch();
    let (gps_raw_int_w, gps_raw_int_h) = message_slot_watch();
    let (attitude_w, attitude_h) = message_slot_watch();
    let (sys_status_w, sys_status_h) = message_slot_watch();
    let battery_status = IndexedMessageFamily::watch();
    let (nav_controller_output_w, nav_controller_output_h) = message_slot_watch();
    let (terrain_report_w, terrain_report_h) = message_slot_watch();
    let (rc_channels_w, rc_channels_h) = message_slot_watch();
    let servo_output_raw = IndexedMessageFamily::watch();
    let (home_position_w, home_position_h) = message_slot_watch();
    let (gps_global_origin_w, gps_global_origin_h) = message_slot_watch();
    let (status_text_w, status_text_h) = create_status_text_backing_store();
    let commands = MessageCommandHub::default();

    (
        TelemetryMessageWriters {
            vfr_hud: vfr_hud_w,
            global_position_int: global_position_int_w,
            local_position_ned: local_position_ned_w,
            gps_raw_int: gps_raw_int_w,
            attitude: attitude_w,
            sys_status: sys_status_w,
            battery_status: battery_status.clone(),
            nav_controller_output: nav_controller_output_w,
            terrain_report: terrain_report_w,
            rc_channels: rc_channels_w,
            servo_output_raw: servo_output_raw.clone(),
            home_position: home_position_w,
            gps_global_origin: gps_global_origin_w,
            status_text: status_text_w,
        },
        TelemetryMessageHandles {
            commands,
            vfr_hud: vfr_hud_h,
            global_position_int: global_position_int_h,
            local_position_ned: local_position_ned_h,
            gps_raw_int: gps_raw_int_h,
            attitude: attitude_h,
            sys_status: sys_status_h,
            battery_status,
            nav_controller_output: nav_controller_output_h,
            terrain_report: terrain_report_h,
            rc_channels: rc_channels_h,
            servo_output_raw,
            home_position: home_position_h,
            gps_global_origin: gps_global_origin_h,
            status_text: status_text_h,
        },
    )
}

async fn wait_for_fresh_sample<M: Clone + Send + Sync + 'static>(
    handle: &MessageHandle<M>,
    timeout: Duration,
    cutoff: Option<Instant>,
) -> Result<MessageSample<M>, VehicleError> {
    let mut subscription = handle.subscribe();
    let wait = async {
        loop {
            match subscription.recv().await {
                Some(sample) if cutoff.is_none_or(|at| sample.received_at > at) => {
                    return Ok(sample);
                }
                Some(_) => continue,
                None => return Err(VehicleError::Disconnected),
            }
        }
    };

    match tokio::time::timeout(timeout, wait).await {
        Ok(result) => result,
        Err(_) => Err(VehicleError::Timeout("message wait".into())),
    }
}

fn interval_us_from_hz(hz: f32) -> Result<f32, VehicleError> {
    if !hz.is_finite() || hz <= 0.0 {
        return Err(VehicleError::InvalidParameter(
            "message rate must be a finite positive hz value".to_string(),
        ));
    }

    Ok(1_000_000.0 / hz)
}

/// Message handle for periodic/requestable MAVLink message families.
#[derive(Clone)]
pub struct PeriodicMessageHandle<M: Clone + Send + Sync + 'static> {
    inner: MessageHandle<M>,
    commands: MessageCommandHub,
    message_id: u32,
    request_param2: f32,
}

impl<M: Clone + Send + Sync + 'static> PeriodicMessageHandle<M> {
    fn new(
        inner: MessageHandle<M>,
        commands: MessageCommandHub,
        message_id: u32,
        request_param2: f32,
    ) -> Self {
        Self {
            inner,
            commands,
            message_id,
            request_param2,
        }
    }

    pub fn latest(&self) -> Option<MessageSample<M>> {
        self.inner.latest()
    }

    pub async fn wait(&self) -> Result<MessageSample<M>, VehicleError> {
        self.inner.wait().await
    }

    pub async fn wait_timeout(&self, timeout: Duration) -> Result<MessageSample<M>, VehicleError> {
        self.inner.wait_timeout(timeout).await
    }

    pub fn subscribe(&self) -> ObservationSubscription<MessageSample<M>> {
        self.inner.subscribe()
    }

    pub fn support(&self) -> ObservationHandle<SupportState> {
        self.inner.support()
    }

    pub async fn request(&self, timeout: Duration) -> Result<MessageSample<M>, VehicleError> {
        let cutoff = self.inner.latest().map(|sample| sample.received_at);
        let wait = wait_for_fresh_sample(&self.inner, timeout, cutoff);

        self.commands
            .send_raw_command_long(
                MavCmd::MAV_CMD_REQUEST_MESSAGE,
                [
                    self.message_id as f32,
                    self.request_param2,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
            )
            .await?;

        wait.await
    }

    pub async fn set_rate(&self, hz: f32) -> Result<(), VehicleError> {
        self.commands
            .send_raw_command_long(
                MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
                [
                    self.message_id as f32,
                    interval_us_from_hz(hz)?,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
            )
            .await
    }
}

/// Message handle for event-style MAVLink message families.
#[derive(Clone)]
pub struct EventMessageHandle<M: Clone + Send + Sync + 'static> {
    inner: MessageHandle<M>,
    commands: MessageCommandHub,
    message_id: u32,
    request_param2: f32,
}

impl<M: Clone + Send + Sync + 'static> EventMessageHandle<M> {
    fn new(
        inner: MessageHandle<M>,
        commands: MessageCommandHub,
        message_id: u32,
        request_param2: f32,
    ) -> Self {
        Self {
            inner,
            commands,
            message_id,
            request_param2,
        }
    }

    pub fn latest(&self) -> Option<MessageSample<M>> {
        self.inner.latest()
    }

    pub async fn wait(&self) -> Result<MessageSample<M>, VehicleError> {
        self.inner.wait().await
    }

    pub async fn wait_timeout(&self, timeout: Duration) -> Result<MessageSample<M>, VehicleError> {
        self.inner.wait_timeout(timeout).await
    }

    pub fn subscribe(&self) -> ObservationSubscription<MessageSample<M>> {
        self.inner.subscribe()
    }

    pub fn support(&self) -> ObservationHandle<SupportState> {
        self.inner.support()
    }

    pub async fn request(&self, timeout: Duration) -> Result<MessageSample<M>, VehicleError> {
        let cutoff = self.inner.latest().map(|sample| sample.received_at);
        let wait = wait_for_fresh_sample(&self.inner, timeout, cutoff);

        self.commands
            .send_raw_command_long(
                MavCmd::MAV_CMD_REQUEST_MESSAGE,
                [
                    self.message_id as f32,
                    self.request_param2,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
            )
            .await?;

        wait.await
    }
}

/// Namespace exposing typed raw MAVLink message streams.
pub struct MessagesHandle<'a> {
    pub(crate) handles: &'a TelemetryMetricHandles,
}

impl MessagesHandle<'_> {
    pub fn vfr_hud(&self) -> PeriodicMessageHandle<dialect::VFR_HUD_DATA> {
        PeriodicMessageHandle::new(
            self.handles.message_handles.vfr_hud.clone(),
            self.handles.message_handles.commands.clone(),
            MSG_ID_VFR_HUD,
            0.0,
        )
    }

    pub fn global_position_int(&self) -> PeriodicMessageHandle<dialect::GLOBAL_POSITION_INT_DATA> {
        PeriodicMessageHandle::new(
            self.handles.message_handles.global_position_int.clone(),
            self.handles.message_handles.commands.clone(),
            MSG_ID_GLOBAL_POSITION_INT,
            0.0,
        )
    }

    pub fn local_position_ned(&self) -> PeriodicMessageHandle<dialect::LOCAL_POSITION_NED_DATA> {
        PeriodicMessageHandle::new(
            self.handles.message_handles.local_position_ned.clone(),
            self.handles.message_handles.commands.clone(),
            MSG_ID_LOCAL_POSITION_NED,
            0.0,
        )
    }

    pub fn gps_raw_int(&self) -> PeriodicMessageHandle<dialect::GPS_RAW_INT_DATA> {
        PeriodicMessageHandle::new(
            self.handles.message_handles.gps_raw_int.clone(),
            self.handles.message_handles.commands.clone(),
            MSG_ID_GPS_RAW_INT,
            0.0,
        )
    }

    pub fn attitude(&self) -> PeriodicMessageHandle<dialect::ATTITUDE_DATA> {
        PeriodicMessageHandle::new(
            self.handles.message_handles.attitude.clone(),
            self.handles.message_handles.commands.clone(),
            MSG_ID_ATTITUDE,
            0.0,
        )
    }

    pub fn sys_status(&self) -> PeriodicMessageHandle<dialect::SYS_STATUS_DATA> {
        PeriodicMessageHandle::new(
            self.handles.message_handles.sys_status.clone(),
            self.handles.message_handles.commands.clone(),
            MSG_ID_SYS_STATUS,
            0.0,
        )
    }

    pub fn battery_status(
        &self,
        instance: u8,
    ) -> PeriodicMessageHandle<dialect::BATTERY_STATUS_DATA> {
        PeriodicMessageHandle::new(
            self.handles.message_handles.battery_status.handle(instance),
            self.handles.message_handles.commands.clone(),
            MSG_ID_BATTERY_STATUS,
            f32::from(instance),
        )
    }

    pub fn nav_controller_output(
        &self,
    ) -> PeriodicMessageHandle<dialect::NAV_CONTROLLER_OUTPUT_DATA> {
        PeriodicMessageHandle::new(
            self.handles.message_handles.nav_controller_output.clone(),
            self.handles.message_handles.commands.clone(),
            MSG_ID_NAV_CONTROLLER_OUTPUT,
            0.0,
        )
    }

    pub fn terrain_report(&self) -> PeriodicMessageHandle<dialect::TERRAIN_REPORT_DATA> {
        PeriodicMessageHandle::new(
            self.handles.message_handles.terrain_report.clone(),
            self.handles.message_handles.commands.clone(),
            MSG_ID_TERRAIN_REPORT,
            0.0,
        )
    }

    pub fn rc_channels(&self) -> PeriodicMessageHandle<dialect::RC_CHANNELS_DATA> {
        PeriodicMessageHandle::new(
            self.handles.message_handles.rc_channels.clone(),
            self.handles.message_handles.commands.clone(),
            MSG_ID_RC_CHANNELS,
            0.0,
        )
    }

    pub fn servo_output_raw(
        &self,
        port: u8,
    ) -> PeriodicMessageHandle<dialect::SERVO_OUTPUT_RAW_DATA> {
        PeriodicMessageHandle::new(
            self.handles.message_handles.servo_output_raw.handle(port),
            self.handles.message_handles.commands.clone(),
            MSG_ID_SERVO_OUTPUT_RAW,
            f32::from(port),
        )
    }

    pub fn home_position(&self) -> EventMessageHandle<dialect::HOME_POSITION_DATA> {
        EventMessageHandle::new(
            self.handles.message_handles.home_position.clone(),
            self.handles.message_handles.commands.clone(),
            MSG_ID_HOME_POSITION,
            0.0,
        )
    }

    pub fn gps_global_origin(&self) -> EventMessageHandle<dialect::GPS_GLOBAL_ORIGIN_DATA> {
        EventMessageHandle::new(
            self.handles.message_handles.gps_global_origin.clone(),
            self.handles.message_handles.commands.clone(),
            MSG_ID_GPS_GLOBAL_ORIGIN,
            0.0,
        )
    }

    pub fn status_text(&self) -> MessageHandle<StatusTextEvent> {
        self.handles.message_handles.status_text.clone()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::create_channels;
    use crate::telemetry::TelemetryHandle;

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
}
