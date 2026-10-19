pub mod decode;
pub mod handles;
pub mod messages;
pub mod sensor_health;
pub mod status_text;
pub mod types;
pub mod values;

pub(crate) use handles::{
    TelemetryMetricHandles, TelemetryMetricWriters, create_telemetry_backing_stores,
};

pub use handles::{
    ActuatorsNamespace, AttitudeNamespace, BatteryNamespace, GpsNamespace, NavigationNamespace,
    PositionNamespace, RcNamespace, TelemetryHandle, TerrainNamespace,
};

pub use messages::{EventMessageHandle, MessagesHandle, PeriodicMessageHandle};

pub use status_text::StatusTextEvent;

pub use crate::state::GpsFixType;
pub use values::{
    CellVoltages, EulerAttitude, GlobalPosition, GpsQuality, GuidanceState, TerrainClearance,
    WaypointProgress,
};

pub use types::{
    TelemetryMessageKind, VehicleTimestamp, infer_time_boot_ms, infer_time_usec,
    infer_timestamp_from_time_boot_ms, infer_timestamp_from_time_usec,
};

#[cfg(test)]
mod tests {
    use super::TelemetryHandle;
    use crate::geo::{GeoPoint2d, GeoPoint3dMsl, GeoPoint3dRelHome};
    use crate::observation::SupportState;
    use crate::state::create_channels;
    use std::time::Duration;

    #[test]
    fn telemetry_namespace_slots_share_backing_store() {
        let (writers, channels) = create_channels();
        let telemetry = TelemetryHandle::new(&channels.telemetry_handles);

        let groundspeed_a = telemetry.position().groundspeed_mps();
        let groundspeed_b = telemetry.position().groundspeed_mps();

        writers.telemetry_metrics.position_groundspeed_mps.publish(
            10.0,
            crate::telemetry::TelemetryMessageKind::VfrHud,
            None,
        );

        assert_eq!(groundspeed_a.latest().unwrap().value, 10.0);
        assert_eq!(groundspeed_b.latest().unwrap().value, 10.0);
    }

    #[test]
    fn rc_and_actuator_namespaces_return_none_for_out_of_range_indexes() {
        let (_writers, channels) = create_channels();
        let telemetry = TelemetryHandle::new(&channels.telemetry_handles);

        assert!(telemetry.rc().channel_pwm_us(18).is_none());
        assert!(telemetry.actuators().servo_pwm_us(16).is_none());
    }

    #[test]
    fn global_position_converts_to_geo_point_variants() {
        let global = super::GlobalPosition {
            latitude_deg: 47.397742,
            longitude_deg: 8.545594,
            altitude_msl_m: 510.0,
            relative_alt_m: 50.0,
        };

        let point_2d: GeoPoint2d = global.clone().into();
        assert_eq!(point_2d.latitude_deg, 47.397742);
        assert_eq!(point_2d.longitude_deg, 8.545594);

        let point_msl: GeoPoint3dMsl = global.clone().into();
        assert_eq!(point_msl.latitude_deg, 47.397742);
        assert_eq!(point_msl.longitude_deg, 8.545594);
        assert_eq!(point_msl.altitude_msl_m, 510.0);

        let point_rel_home: GeoPoint3dRelHome = global.into();
        assert_eq!(point_rel_home.latitude_deg, 47.397742);
        assert_eq!(point_rel_home.longitude_deg, 8.545594);
        assert_eq!(point_rel_home.relative_alt_m, 50.0);
    }

    #[tokio::test]
    async fn home_metric_handle_exposes_grouped_observation_contract() {
        let (writers, channels) = create_channels();
        let telemetry = TelemetryHandle::new(&channels.telemetry_handles);

        let home = telemetry.home();
        assert_eq!(home.support().latest(), Some(SupportState::Unknown));
        assert!(home.latest().is_none());

        let point = GeoPoint3dMsl {
            latitude_deg: 47.397742,
            longitude_deg: 8.545594,
            altitude_msl_m: 510.0,
        };
        writers.telemetry_metrics.home.publish(
            point.clone(),
            crate::telemetry::TelemetryMessageKind::HomePosition,
            None,
        );

        let latest = home.latest().expect("home sample should be published");
        assert_eq!(latest.value, point);
        assert_eq!(
            latest.source,
            crate::telemetry::TelemetryMessageKind::HomePosition
        );

        let waited = tokio::time::timeout(Duration::from_millis(100), home.wait())
            .await
            .expect("home wait should resolve")
            .expect("home sample should be readable");
        assert_eq!(waited.value, latest.value);
        assert_eq!(waited.source, latest.source);

        let mut subscription = home.subscribe();
        let subscribed = tokio::time::timeout(Duration::from_millis(100), subscription.recv())
            .await
            .expect("home subscription should replay current sample")
            .expect("home subscription sample should exist");
        assert_eq!(subscribed.value, latest.value);
        assert_eq!(subscribed.source, latest.source);
    }

    #[tokio::test]
    async fn origin_metric_handle_exposes_grouped_observation_contract() {
        let (writers, channels) = create_channels();
        let telemetry = TelemetryHandle::new(&channels.telemetry_handles);

        let origin = telemetry.origin();
        assert_eq!(origin.support().latest(), Some(SupportState::Unknown));
        assert!(origin.latest().is_none());

        let point = GeoPoint3dMsl {
            latitude_deg: 47.398123,
            longitude_deg: 8.546321,
            altitude_msl_m: 505.5,
        };
        writers.telemetry_metrics.origin.publish(
            point.clone(),
            crate::telemetry::TelemetryMessageKind::GpsGlobalOrigin,
            None,
        );

        let latest = origin.latest().expect("origin sample should be published");
        assert_eq!(latest.value, point);
        assert_eq!(
            latest.source,
            crate::telemetry::TelemetryMessageKind::GpsGlobalOrigin
        );

        let waited = tokio::time::timeout(Duration::from_millis(100), origin.wait())
            .await
            .expect("origin wait should resolve")
            .expect("origin sample should be readable");
        assert_eq!(waited.value, latest.value);
        assert_eq!(waited.source, latest.source);

        let mut subscription = origin.subscribe();
        let subscribed = tokio::time::timeout(Duration::from_millis(100), subscription.recv())
            .await
            .expect("origin subscription should replay current sample")
            .expect("origin subscription sample should exist");
        assert_eq!(subscribed.value, latest.value);
        assert_eq!(subscribed.source, latest.source);
    }
}
