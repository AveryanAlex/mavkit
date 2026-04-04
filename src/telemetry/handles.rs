use super::messages::{
    MessagesHandle, TelemetryMessageHandles, TelemetryMessageWriters,
    create_telemetry_message_backing_stores,
};
use super::values::{
    CellVoltages, EulerAttitude, GlobalPosition, GpsQuality, GuidanceState, TerrainClearance,
    WaypointProgress,
};
use crate::command::Command;
use crate::geo::GeoPoint3dMsl;
use crate::observation::{
    MetricHandle, MetricSample, ObservationHandle, ObservationWriter, SupportState,
};
use crate::types::SensorHealthSummary;
use crate::{TelemetryMessageKind, VehicleTimestamp};
use std::array;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Instant;
use tokio::sync::mpsc;

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
/// hardcoded in the expansion because they have different types in each struct and do not
/// follow the uniform `MetricHandle` / `MetricSlotWriter` pattern.
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

/// Root telemetry accessor exposing grouped namespaces and direct metrics.
///
/// Obtained from [`Vehicle::telemetry`](crate::Vehicle::telemetry). Each accessor returns a
/// [`MetricHandle`] that holds an immutable clone of the backing channel — cloning is cheap
/// and the handle stays valid for the lifetime of the vehicle.
///
/// Metrics that have not been received yet return `None` from
/// [`MetricHandle::latest`](crate::observation::MetricHandle::latest).
pub struct TelemetryHandle<'a> {
    handles: &'a TelemetryMetricHandles,
}

impl<'a> TelemetryHandle<'a> {
    pub(crate) fn new(handles: &'a TelemetryMetricHandles) -> Self {
        Self { handles }
    }

    pub(crate) fn with_command_tx(
        handles: &'a TelemetryMetricHandles,
        command_tx: &mpsc::Sender<Command>,
    ) -> Self {
        handles.message_handles.commands.bind(command_tx);
        Self::new(handles)
    }

    /// Position-related metrics: global position, speed, heading, throttle.
    pub fn position(&self) -> PositionNamespace<'a> {
        PositionNamespace {
            handles: self.handles,
        }
    }

    /// Attitude metrics: Euler angles (roll, pitch, yaw).
    pub fn attitude(&self) -> AttitudeNamespace<'a> {
        AttitudeNamespace {
            handles: self.handles,
        }
    }

    /// Battery metrics: voltage, current, remaining charge, cell voltages.
    ///
    /// When both `BATTERY_STATUS` (primary battery) and `SYS_STATUS` messages are present, the
    /// `BATTERY_STATUS`-derived values take precedence for the primary battery.
    pub fn battery(&self) -> BatteryNamespace<'a> {
        BatteryNamespace {
            handles: self.handles,
        }
    }

    /// GPS quality and raw GPS position metrics.
    pub fn gps(&self) -> GpsNamespace<'a> {
        GpsNamespace {
            handles: self.handles,
        }
    }

    /// Navigation-controller metrics: active waypoint progress, guidance state.
    pub fn navigation(&self) -> NavigationNamespace<'a> {
        NavigationNamespace {
            handles: self.handles,
        }
    }

    /// Terrain-avoidance metrics: clearance above terrain.
    pub fn terrain(&self) -> TerrainNamespace<'a> {
        TerrainNamespace {
            handles: self.handles,
        }
    }

    /// RC input metrics: per-channel PWM values and RSSI.
    pub fn rc(&self) -> RcNamespace<'a> {
        RcNamespace {
            handles: self.handles,
        }
    }

    /// Actuator output metrics: servo PWM values.
    pub fn actuators(&self) -> ActuatorsNamespace<'a> {
        ActuatorsNamespace {
            handles: self.handles,
        }
    }

    /// Access to raw MAVLink message streams and on-demand message request APIs.
    pub fn messages(&self) -> MessagesHandle<'a> {
        MessagesHandle {
            handles: self.handles,
        }
    }

    /// Whether the vehicle is currently armed.
    pub fn armed(&self) -> MetricHandle<bool> {
        self.handles.armed.clone()
    }

    /// Aggregated sensor health flags from `SYS_STATUS`.
    pub fn sensor_health(&self) -> MetricHandle<SensorHealthSummary> {
        self.handles.sensor_health.clone()
    }

    /// Home position (MSL), set by the vehicle and updated from `HOME_POSITION` messages.
    pub fn home(&self) -> MetricHandle<GeoPoint3dMsl> {
        self.handles.home.clone()
    }

    /// EKF origin (MSL), updated from `GPS_GLOBAL_ORIGIN` messages.
    pub fn origin(&self) -> MetricHandle<GeoPoint3dMsl> {
        self.handles.origin.clone()
    }
}

/// Position-related telemetry metrics.
pub struct PositionNamespace<'a> {
    handles: &'a TelemetryMetricHandles,
}

impl PositionNamespace<'_> {
    pub fn global(&self) -> MetricHandle<GlobalPosition> {
        self.handles.position_global.clone()
    }

    pub fn groundspeed_mps(&self) -> MetricHandle<f64> {
        self.handles.position_groundspeed_mps.clone()
    }

    pub fn airspeed_mps(&self) -> MetricHandle<f64> {
        self.handles.position_airspeed_mps.clone()
    }

    pub fn climb_rate_mps(&self) -> MetricHandle<f64> {
        self.handles.position_climb_rate_mps.clone()
    }

    pub fn heading_deg(&self) -> MetricHandle<f64> {
        self.handles.position_heading_deg.clone()
    }

    pub fn throttle_pct(&self) -> MetricHandle<f64> {
        self.handles.position_throttle_pct.clone()
    }
}

/// Attitude-related telemetry metrics.
pub struct AttitudeNamespace<'a> {
    handles: &'a TelemetryMetricHandles,
}

impl AttitudeNamespace<'_> {
    pub fn euler(&self) -> MetricHandle<EulerAttitude> {
        self.handles.attitude_euler.clone()
    }
}

/// Battery-related telemetry metrics.
pub struct BatteryNamespace<'a> {
    handles: &'a TelemetryMetricHandles,
}

impl BatteryNamespace<'_> {
    pub fn remaining_pct(&self) -> MetricHandle<f64> {
        self.handles.battery_remaining_pct.clone()
    }

    pub fn voltage_v(&self) -> MetricHandle<f64> {
        self.handles.battery_voltage_v.clone()
    }

    pub fn current_a(&self) -> MetricHandle<f64> {
        self.handles.battery_current_a.clone()
    }

    pub fn energy_consumed_wh(&self) -> MetricHandle<f64> {
        self.handles.battery_energy_consumed_wh.clone()
    }

    pub fn time_remaining_s(&self) -> MetricHandle<i32> {
        self.handles.battery_time_remaining_s.clone()
    }

    pub fn cells(&self) -> MetricHandle<CellVoltages> {
        self.handles.battery_cells.clone()
    }
}

/// GPS-related telemetry metrics.
pub struct GpsNamespace<'a> {
    handles: &'a TelemetryMetricHandles,
}

impl GpsNamespace<'_> {
    pub fn quality(&self) -> MetricHandle<GpsQuality> {
        self.handles.gps_quality.clone()
    }

    pub fn position_msl(&self) -> MetricHandle<GeoPoint3dMsl> {
        self.handles.gps_position_msl.clone()
    }
}

/// Navigation-controller telemetry metrics.
pub struct NavigationNamespace<'a> {
    handles: &'a TelemetryMetricHandles,
}

impl NavigationNamespace<'_> {
    pub fn waypoint(&self) -> MetricHandle<WaypointProgress> {
        self.handles.navigation_waypoint.clone()
    }

    pub fn guidance(&self) -> MetricHandle<GuidanceState> {
        self.handles.navigation_guidance.clone()
    }
}

/// Terrain-related telemetry metrics.
pub struct TerrainNamespace<'a> {
    handles: &'a TelemetryMetricHandles,
}

impl TerrainNamespace<'_> {
    pub fn clearance(&self) -> MetricHandle<TerrainClearance> {
        self.handles.terrain_clearance.clone()
    }
}

/// RC input telemetry metrics.
pub struct RcNamespace<'a> {
    handles: &'a TelemetryMetricHandles,
}

impl RcNamespace<'_> {
    /// Returns the PWM metric for RC channel at `index` (0-based, up to 17).
    ///
    /// Returns `None` if `index` is out of range. The channel may have no value yet even when
    /// in range.
    pub fn channel_pwm_us(&self, index: usize) -> Option<MetricHandle<u16>> {
        self.handles.rc_channels_pwm_us.get(index).cloned()
    }

    /// RC receiver signal strength (0–100 %).
    pub fn rssi_pct(&self) -> MetricHandle<u8> {
        self.handles.rc_rssi_pct.clone()
    }
}

/// Actuator output telemetry metrics.
pub struct ActuatorsNamespace<'a> {
    handles: &'a TelemetryMetricHandles,
}

impl ActuatorsNamespace<'_> {
    /// Returns the PWM metric for servo output at `index` (0-based, up to 15).
    ///
    /// Returns `None` if `index` is out of range. The channel may have no value yet even when
    /// in range.
    pub fn servo_pwm_us(&self, index: usize) -> Option<MetricHandle<u16>> {
        self.handles.actuator_servo_pwm_us.get(index).cloned()
    }
}
