//! Writer-side telemetry metric backing stores and battery precedence wiring.

use std::array;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use crate::geo::GeoPoint3dMsl;
use crate::observation::{
    MetricHandle, MetricSample, ObservationHandle, ObservationWriter, SupportState,
};
use crate::telemetry::messages::{
    TelemetryMessageHandles, TelemetryMessageWriters, create_telemetry_message_backing_stores,
};
use crate::telemetry::values::{
    CellVoltages, EulerAttitude, GlobalPosition, GpsQuality, GuidanceState, TerrainClearance,
    WaypointProgress,
};
use crate::time::Instant;
use crate::types::SensorHealthSummary;
use crate::{TelemetryMessageKind, VehicleTimestamp};

const RC_CHANNEL_COUNT: usize = 18;
const SERVO_OUTPUT_COUNT: usize = 16;

#[derive(Clone)]
pub(crate) struct MetricSlotWriter<T: Clone + Send + Sync + 'static> {
    sample: ObservationWriter<MetricSample<T>>,
    support: ObservationWriter<SupportState>,
}

impl<T: Clone + Send + Sync + 'static> MetricSlotWriter<T> {
    pub(crate) fn publish(
        &self,
        value: T,
        source: TelemetryMessageKind,
        vehicle_time: Option<VehicleTimestamp>,
    ) {
        let _ = self.support.publish(SupportState::Supported);
        let _ = self.sample.publish(MetricSample {
            value,
            source,
            vehicle_time,
            received_at: Instant::now(),
        });
    }

    pub(crate) fn clear(&self) {
        let _ = self.support.publish(SupportState::Unsupported);
        self.sample.clear();
    }
}

/// Generates the paired `TelemetryMetricHandles` and `TelemetryMetricWriters` structs from a
/// single canonical field list, eliminating the need to keep two parallel struct definitions in
/// sync by hand.
///
/// Syntax:
/// ```text
/// define_metrics! {
///     scalar { field_name: Type, ... }
///     array  { field_name: Type; COUNT, ... }
/// }
/// ```
///
/// Scalar entries expand to `MetricHandle<Type>` / `MetricSlotWriter<Type>` fields.
/// Array entries expand to `[MetricHandle<Type>; COUNT]` / `[MetricSlotWriter<Type>; COUNT]`.
///
/// Non-metric fields (`message_handles`, `message_writers`, `battery_primary_seen`) are
/// hardcoded in the expansion because they have different types in each struct and do not follow
/// the uniform `MetricHandle` / `MetricSlotWriter` pattern.
macro_rules! define_metrics {
    (
        scalar { $( $sf:ident : $st:ty ),* $(,)? }
        array  { $( $af:ident : $at:ty ; $ac:expr ),* $(,)? }
    ) => {
        #[derive(Clone)]
        pub(crate) struct TelemetryMetricHandles {
            pub(crate) message_handles: TelemetryMessageHandles,
            $( pub(crate) $sf: MetricHandle<$st>, )*
            $( pub(crate) $af: [MetricHandle<$at>; $ac], )*
        }

        #[derive(Clone)]
        pub(crate) struct TelemetryMetricWriters {
            pub(crate) message_writers: TelemetryMessageWriters,
            $( pub(crate) $sf: MetricSlotWriter<$st>, )*
            $( pub(crate) $af: [MetricSlotWriter<$at>; $ac], )*
            battery_primary_seen: Arc<AtomicBool>,
        }

        pub(crate) fn create_telemetry_backing_stores(
        ) -> (TelemetryMetricWriters, TelemetryMetricHandles) {
            let (message_writers, message_handles) = create_telemetry_message_backing_stores();
            $( let $sf = metric_backing_store::<$st>(); )*
            $( let $af = array::from_fn(|_| metric_backing_store::<$at>()); )*

            let battery_primary_seen = Arc::new(AtomicBool::new(false));

            (
                TelemetryMetricWriters {
                    message_writers,
                    $( $sf: $sf.writer.clone(), )*
                    $( $af: $af.clone().map(|store| store.writer), )*
                    battery_primary_seen,
                },
                TelemetryMetricHandles {
                    message_handles,
                    $( $sf: $sf.handle, )*
                    $( $af: $af.map(|store| store.handle), )*
                },
            )
        }
    };
}

define_metrics! {
    scalar {
        position_global: GlobalPosition,
        position_groundspeed_mps: f64,
        position_airspeed_mps: f64,
        position_climb_rate_mps: f64,
        position_heading_deg: f64,
        position_throttle_pct: f64,
        attitude_euler: EulerAttitude,
        battery_remaining_pct: f64,
        battery_voltage_v: f64,
        battery_current_a: f64,
        battery_energy_consumed_wh: f64,
        battery_time_remaining_s: i32,
        battery_cells: CellVoltages,
        gps_quality: GpsQuality,
        gps_position_msl: GeoPoint3dMsl,
        navigation_waypoint: WaypointProgress,
        navigation_guidance: GuidanceState,
        terrain_clearance: TerrainClearance,
        rc_rssi_pct: u8,
        armed: bool,
        sensor_health: SensorHealthSummary,
        home: GeoPoint3dMsl,
        origin: GeoPoint3dMsl,
    }
    array {
        rc_channels_pwm_us: u16; RC_CHANNEL_COUNT,
        actuator_servo_pwm_us: u16; SERVO_OUTPUT_COUNT,
    }
}

impl TelemetryMetricHandles {
    pub(crate) fn close_indexed_message_families(&self) {
        self.message_handles.close_indexed_families();
    }
}

impl TelemetryMetricWriters {
    pub(crate) fn publish_battery_from_sys_status(
        &self,
        remaining_pct: Option<f64>,
        voltage_v: Option<f64>,
        current_a: Option<f64>,
    ) {
        if self.battery_primary_seen.load(Ordering::SeqCst) {
            return;
        }

        if let Some(value) = remaining_pct {
            self.battery_remaining_pct
                .publish(value, TelemetryMessageKind::SysStatus, None);
        }
        if let Some(value) = voltage_v {
            self.battery_voltage_v
                .publish(value, TelemetryMessageKind::SysStatus, None);
        }
        if let Some(value) = current_a {
            self.battery_current_a
                .publish(value, TelemetryMessageKind::SysStatus, None);
        }
    }

    pub(crate) fn publish_battery_from_primary_status(
        &self,
        remaining_pct: Option<f64>,
        cell_voltages_v: Option<Vec<f64>>,
        current_a: Option<f64>,
        energy_consumed_wh: Option<f64>,
        time_remaining_s: Option<i32>,
    ) {
        self.battery_primary_seen.store(true, Ordering::SeqCst);

        if let Some(value) = remaining_pct {
            self.battery_remaining_pct
                .publish(value, TelemetryMessageKind::BatteryStatus, None);
        }
        if let Some(value) = current_a {
            self.battery_current_a
                .publish(value, TelemetryMessageKind::BatteryStatus, None);
        }
        if let Some(value) = energy_consumed_wh {
            self.battery_energy_consumed_wh.publish(
                value,
                TelemetryMessageKind::BatteryStatus,
                None,
            );
        }
        if let Some(value) = time_remaining_s {
            self.battery_time_remaining_s
                .publish(value, TelemetryMessageKind::BatteryStatus, None);
        }
        if let Some(voltages_v) = cell_voltages_v {
            let summed = voltages_v.iter().sum::<f64>();
            self.battery_voltage_v
                .publish(summed, TelemetryMessageKind::BatteryStatus, None);
            self.battery_cells.publish(
                CellVoltages { voltages_v },
                TelemetryMessageKind::BatteryStatus,
                None,
            );
        }
    }
}

fn metric_slot<T: Clone + Send + Sync + 'static>() -> (MetricSlotWriter<T>, MetricHandle<T>) {
    let (sample_writer, sample_observation) = ObservationHandle::watch();
    let (support_writer, support_observation) = ObservationHandle::watch();
    let _ = support_writer.publish(SupportState::Unknown);

    (
        MetricSlotWriter {
            sample: sample_writer,
            support: support_writer,
        },
        MetricHandle::new(sample_observation, support_observation),
    )
}

#[derive(Clone)]
struct MetricBackingStore<T: Clone + Send + Sync + 'static> {
    writer: MetricSlotWriter<T>,
    handle: MetricHandle<T>,
}

fn metric_backing_store<T: Clone + Send + Sync + 'static>() -> MetricBackingStore<T> {
    let (writer, handle) = metric_slot();
    MetricBackingStore { writer, handle }
}
