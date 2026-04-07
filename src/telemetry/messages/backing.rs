use crate::command::Command;
use crate::dialect::{self, MavCmd};
use crate::observation::{
    MessageHandle, MessageSample, ObservationHandle, ObservationWriter, SupportState,
};
use crate::shared_state::recover_lock;
use crate::telemetry::status_text::{
    StatusTextEvent, StatusTextWriter, create_status_text_backing_store,
};
use crate::{VehicleError, VehicleTimestamp};
use std::collections::HashMap;
use std::hash::Hash;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Instant;
use tokio::sync::{mpsc, oneshot};

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

    pub(crate) fn close(&self) {
        self.sample.close();
        self.support.close();
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
    closed: Arc<AtomicBool>,
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
            closed: Arc::new(AtomicBool::new(false)),
            factory: message_slot_watch::<M>,
        }
    }

    fn entry(&self, key: K) -> IndexedMessageEntry<M> {
        let mut entries = recover_lock(&self.entries);

        if let Some(existing) = entries.get(&key) {
            return existing.clone();
        }

        let factory = self.factory;
        let (writer, handle) = factory();
        if self.closed.load(Ordering::SeqCst) {
            writer.close();
        }
        let created = IndexedMessageEntry { writer, handle };
        entries.insert(key, created.clone());
        created
    }

    pub(crate) fn handle(&self, key: K) -> MessageHandle<M> {
        self.entry(key).handle
    }

    pub(crate) fn publish(&self, key: K, value: M, vehicle_time: Option<VehicleTimestamp>) {
        self.entry(key).writer.publish(value, vehicle_time);
    }

    pub(crate) fn close(&self) {
        self.closed.store(true, Ordering::SeqCst);
        let entries = recover_lock(&self.entries);
        for entry in entries.values() {
            entry.writer.close();
        }
    }
}

#[derive(Clone, Default)]
pub(crate) struct MessageCommandHub {
    command_tx: Arc<Mutex<Option<mpsc::Sender<Command>>>>,
}

impl MessageCommandHub {
    pub(crate) fn bind(&self, command_tx: &mpsc::Sender<Command>) {
        *recover_lock(&self.command_tx) = Some(command_tx.clone());
    }

    pub(super) async fn send_raw_command_long(
        &self,
        command: MavCmd,
        params: [f32; 7],
    ) -> Result<(), VehicleError> {
        let command_tx = recover_lock(&self.command_tx).clone().ok_or_else(|| {
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
            pub(crate) status_text_writer: StatusTextWriter,
        }

        #[derive(Clone)]
        pub(crate) struct TelemetryMessageWriters {
            $( pub(crate) $sf: MessageSlotWriter<$st>, )*
            $( pub(crate) $if: IndexedMessageFamily<$ik, $it>, )*
            pub(crate) status_text: StatusTextWriter,
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

impl TelemetryMessageHandles {
    pub(crate) fn close_indexed_families(&self) {
        self.battery_status.close();
        self.servo_output_raw.close();
        self.status_text_writer.close();
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
            status_text: status_text_w.clone(),
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
            status_text_writer: status_text_w,
        },
    )
}
