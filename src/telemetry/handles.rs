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
}

impl<T: Clone + Send + Sync + 'static> MetricSlotWriter<T> {
    pub(crate) fn publish(
        &self,
        value: T,
        source: TelemetryMessageKind,
        vehicle_time: Option<VehicleTimestamp>,
    ) {
        let _ = self.sample.publish(MetricSample {
            value,
            source,
            vehicle_time,
            received_at: Instant::now(),
        });
    }
}

#[derive(Clone)]
pub(crate) struct TelemetryMetricHandles {
    pub(crate) message_handles: TelemetryMessageHandles,
    pub(crate) position_global: MetricHandle<GlobalPosition>,
    pub(crate) position_groundspeed_mps: MetricHandle<f64>,
    pub(crate) position_airspeed_mps: MetricHandle<f64>,
    pub(crate) position_climb_rate_mps: MetricHandle<f64>,
    pub(crate) position_heading_deg: MetricHandle<f64>,
    pub(crate) position_throttle_pct: MetricHandle<f64>,
    pub(crate) attitude_euler: MetricHandle<EulerAttitude>,
    pub(crate) battery_remaining_pct: MetricHandle<f64>,
    pub(crate) battery_voltage_v: MetricHandle<f64>,
    pub(crate) battery_current_a: MetricHandle<f64>,
    pub(crate) battery_energy_consumed_wh: MetricHandle<f64>,
    pub(crate) battery_time_remaining_s: MetricHandle<i32>,
    pub(crate) battery_cells: MetricHandle<CellVoltages>,
    pub(crate) gps_quality: MetricHandle<GpsQuality>,
    pub(crate) gps_position_msl: MetricHandle<GeoPoint3dMsl>,
    pub(crate) navigation_waypoint: MetricHandle<WaypointProgress>,
    pub(crate) navigation_guidance: MetricHandle<GuidanceState>,
    pub(crate) terrain_clearance: MetricHandle<TerrainClearance>,
    pub(crate) rc_channels_pwm_us: [MetricHandle<u16>; RC_CHANNEL_COUNT],
    pub(crate) rc_rssi_pct: MetricHandle<u8>,
    pub(crate) actuator_servo_pwm_us: [MetricHandle<u16>; SERVO_OUTPUT_COUNT],
    pub(crate) armed: MetricHandle<bool>,
    pub(crate) sensor_health: MetricHandle<SensorHealthSummary>,
    pub(crate) home: MetricHandle<GeoPoint3dMsl>,
    pub(crate) origin: MetricHandle<GeoPoint3dMsl>,
}

#[derive(Clone)]
pub(crate) struct TelemetryMetricWriters {
    pub(crate) message_writers: TelemetryMessageWriters,
    pub(crate) position_global: MetricSlotWriter<GlobalPosition>,
    pub(crate) position_groundspeed_mps: MetricSlotWriter<f64>,
    pub(crate) position_airspeed_mps: MetricSlotWriter<f64>,
    pub(crate) position_climb_rate_mps: MetricSlotWriter<f64>,
    pub(crate) position_heading_deg: MetricSlotWriter<f64>,
    pub(crate) position_throttle_pct: MetricSlotWriter<f64>,
    pub(crate) attitude_euler: MetricSlotWriter<EulerAttitude>,
    pub(crate) battery_remaining_pct: MetricSlotWriter<f64>,
    pub(crate) battery_voltage_v: MetricSlotWriter<f64>,
    pub(crate) battery_current_a: MetricSlotWriter<f64>,
    pub(crate) battery_energy_consumed_wh: MetricSlotWriter<f64>,
    pub(crate) battery_time_remaining_s: MetricSlotWriter<i32>,
    pub(crate) battery_cells: MetricSlotWriter<CellVoltages>,
    pub(crate) gps_quality: MetricSlotWriter<GpsQuality>,
    pub(crate) gps_position_msl: MetricSlotWriter<GeoPoint3dMsl>,
    pub(crate) navigation_waypoint: MetricSlotWriter<WaypointProgress>,
    pub(crate) navigation_guidance: MetricSlotWriter<GuidanceState>,
    pub(crate) terrain_clearance: MetricSlotWriter<TerrainClearance>,
    pub(crate) rc_channels_pwm_us: [MetricSlotWriter<u16>; RC_CHANNEL_COUNT],
    pub(crate) rc_rssi_pct: MetricSlotWriter<u8>,
    pub(crate) actuator_servo_pwm_us: [MetricSlotWriter<u16>; SERVO_OUTPUT_COUNT],
    pub(crate) armed: MetricSlotWriter<bool>,
    pub(crate) sensor_health: MetricSlotWriter<SensorHealthSummary>,
    pub(crate) home: MetricSlotWriter<GeoPoint3dMsl>,
    pub(crate) origin: MetricSlotWriter<GeoPoint3dMsl>,
    battery_primary_seen: Arc<AtomicBool>,
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
        },
        MetricHandle::new(sample_observation, support_observation),
    )
}

pub(crate) fn create_telemetry_backing_stores() -> (TelemetryMetricWriters, TelemetryMetricHandles)
{
    let (message_writers, message_handles) = create_telemetry_message_backing_stores();
    let (position_global_w, position_global_h) = metric_slot();
    let (position_groundspeed_mps_w, position_groundspeed_mps_h) = metric_slot();
    let (position_airspeed_mps_w, position_airspeed_mps_h) = metric_slot();
    let (position_climb_rate_mps_w, position_climb_rate_mps_h) = metric_slot();
    let (position_heading_deg_w, position_heading_deg_h) = metric_slot();
    let (position_throttle_pct_w, position_throttle_pct_h) = metric_slot();

    let (attitude_euler_w, attitude_euler_h) = metric_slot();

    let (battery_remaining_pct_w, battery_remaining_pct_h) = metric_slot();
    let (battery_voltage_v_w, battery_voltage_v_h) = metric_slot();
    let (battery_current_a_w, battery_current_a_h) = metric_slot();
    let (battery_energy_consumed_wh_w, battery_energy_consumed_wh_h) = metric_slot();
    let (battery_time_remaining_s_w, battery_time_remaining_s_h) = metric_slot();
    let (battery_cells_w, battery_cells_h) = metric_slot();

    let (gps_quality_w, gps_quality_h) = metric_slot();
    let (gps_position_msl_w, gps_position_msl_h) = metric_slot();

    let (navigation_waypoint_w, navigation_waypoint_h) = metric_slot();
    let (navigation_guidance_w, navigation_guidance_h) = metric_slot();

    let (terrain_clearance_w, terrain_clearance_h) = metric_slot();

    let rc_channels = array::from_fn(|_| metric_slot::<u16>());
    let rc_channels_pwm_us_w = rc_channels.clone().map(|(writer, _)| writer);
    let rc_channels_pwm_us_h = rc_channels.map(|(_, handle)| handle);

    let (rc_rssi_pct_w, rc_rssi_pct_h) = metric_slot();

    let servo_outputs = array::from_fn(|_| metric_slot::<u16>());
    let actuator_servo_pwm_us_w = servo_outputs.clone().map(|(writer, _)| writer);
    let actuator_servo_pwm_us_h = servo_outputs.map(|(_, handle)| handle);

    let (armed_w, armed_h) = metric_slot();
    let (sensor_health_w, sensor_health_h) = metric_slot();
    let (home_w, home_h) = metric_slot();
    let (origin_w, origin_h) = metric_slot();

    let battery_primary_seen = Arc::new(AtomicBool::new(false));

    (
        TelemetryMetricWriters {
            message_writers,
            position_global: position_global_w,
            position_groundspeed_mps: position_groundspeed_mps_w,
            position_airspeed_mps: position_airspeed_mps_w,
            position_climb_rate_mps: position_climb_rate_mps_w,
            position_heading_deg: position_heading_deg_w,
            position_throttle_pct: position_throttle_pct_w,
            attitude_euler: attitude_euler_w,
            battery_remaining_pct: battery_remaining_pct_w,
            battery_voltage_v: battery_voltage_v_w,
            battery_current_a: battery_current_a_w,
            battery_energy_consumed_wh: battery_energy_consumed_wh_w,
            battery_time_remaining_s: battery_time_remaining_s_w,
            battery_cells: battery_cells_w,
            gps_quality: gps_quality_w,
            gps_position_msl: gps_position_msl_w,
            navigation_waypoint: navigation_waypoint_w,
            navigation_guidance: navigation_guidance_w,
            terrain_clearance: terrain_clearance_w,
            rc_channels_pwm_us: rc_channels_pwm_us_w,
            rc_rssi_pct: rc_rssi_pct_w,
            actuator_servo_pwm_us: actuator_servo_pwm_us_w,
            armed: armed_w,
            sensor_health: sensor_health_w,
            home: home_w,
            origin: origin_w,
            battery_primary_seen,
        },
        TelemetryMetricHandles {
            message_handles,
            position_global: position_global_h,
            position_groundspeed_mps: position_groundspeed_mps_h,
            position_airspeed_mps: position_airspeed_mps_h,
            position_climb_rate_mps: position_climb_rate_mps_h,
            position_heading_deg: position_heading_deg_h,
            position_throttle_pct: position_throttle_pct_h,
            attitude_euler: attitude_euler_h,
            battery_remaining_pct: battery_remaining_pct_h,
            battery_voltage_v: battery_voltage_v_h,
            battery_current_a: battery_current_a_h,
            battery_energy_consumed_wh: battery_energy_consumed_wh_h,
            battery_time_remaining_s: battery_time_remaining_s_h,
            battery_cells: battery_cells_h,
            gps_quality: gps_quality_h,
            gps_position_msl: gps_position_msl_h,
            navigation_waypoint: navigation_waypoint_h,
            navigation_guidance: navigation_guidance_h,
            terrain_clearance: terrain_clearance_h,
            rc_channels_pwm_us: rc_channels_pwm_us_h,
            rc_rssi_pct: rc_rssi_pct_h,
            actuator_servo_pwm_us: actuator_servo_pwm_us_h,
            armed: armed_h,
            sensor_health: sensor_health_h,
            home: home_h,
            origin: origin_h,
        },
    )
}

/// Root telemetry accessor exposing grouped namespaces and direct metrics.
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

    pub fn position(&self) -> PositionNamespace<'a> {
        PositionNamespace {
            handles: self.handles,
        }
    }

    pub fn attitude(&self) -> AttitudeNamespace<'a> {
        AttitudeNamespace {
            handles: self.handles,
        }
    }

    pub fn battery(&self) -> BatteryNamespace<'a> {
        BatteryNamespace {
            handles: self.handles,
        }
    }

    pub fn gps(&self) -> GpsNamespace<'a> {
        GpsNamespace {
            handles: self.handles,
        }
    }

    pub fn navigation(&self) -> NavigationNamespace<'a> {
        NavigationNamespace {
            handles: self.handles,
        }
    }

    pub fn terrain(&self) -> TerrainNamespace<'a> {
        TerrainNamespace {
            handles: self.handles,
        }
    }

    pub fn rc(&self) -> RcNamespace<'a> {
        RcNamespace {
            handles: self.handles,
        }
    }

    pub fn actuators(&self) -> ActuatorsNamespace<'a> {
        ActuatorsNamespace {
            handles: self.handles,
        }
    }

    pub fn messages(&self) -> MessagesHandle<'a> {
        MessagesHandle {
            handles: self.handles,
        }
    }

    pub fn armed(&self) -> MetricHandle<bool> {
        self.handles.armed.clone()
    }

    pub fn sensor_health(&self) -> MetricHandle<SensorHealthSummary> {
        self.handles.sensor_health.clone()
    }

    pub fn home(&self) -> MetricHandle<GeoPoint3dMsl> {
        self.handles.home.clone()
    }

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
    pub fn channel_pwm_us(&self, index: usize) -> Option<MetricHandle<u16>> {
        self.handles.rc_channels_pwm_us.get(index).cloned()
    }

    pub fn rssi_pct(&self) -> MetricHandle<u8> {
        self.handles.rc_rssi_pct.clone()
    }
}

/// Actuator output telemetry metrics.
pub struct ActuatorsNamespace<'a> {
    handles: &'a TelemetryMetricHandles,
}

impl ActuatorsNamespace<'_> {
    pub fn servo_pwm_us(&self, index: usize) -> Option<MetricHandle<u16>> {
        self.handles.actuator_servo_pwm_us.get(index).cloned()
    }
}
