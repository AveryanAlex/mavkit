use super::UpdateContext;
use crate::dialect;
use crate::geo::GeoPoint3dMsl;
use crate::mission;
use crate::state::GpsFixType;
use crate::telemetry::{GpsQuality, TelemetryMessageKind, infer_timestamp_from_time_usec};

pub(super) fn handle_gps_raw_int(context: &UpdateContext<'_>, data: &dialect::GPS_RAW_INT_DATA) {
    let sats = if data.satellites_visible != u8::MAX {
        Some(data.satellites_visible)
    } else {
        None
    };
    let hdop = if data.eph != u16::MAX {
        Some(f64::from(data.eph) / 100.0)
    } else {
        None
    };

    let vehicle_time =
        infer_timestamp_from_time_usec(TelemetryMessageKind::GpsRawInt, data.time_usec);
    context
        .writers
        .telemetry_metrics
        .message_writers
        .gps_raw_int
        .publish(data.clone(), vehicle_time.clone());
    context.writers.telemetry_metrics.gps_quality.publish(
        GpsQuality {
            fix_type: GpsFixType::from_raw(data.fix_type as u8),
            satellites: sats,
            hdop,
        },
        TelemetryMessageKind::GpsRawInt,
        vehicle_time.clone(),
    );
    context.writers.telemetry_metrics.gps_position_msl.publish(
        GeoPoint3dMsl {
            latitude_deg: f64::from(data.lat) / 1e7,
            longitude_deg: f64::from(data.lon) / 1e7,
            altitude_msl_m: f64::from(data.alt) / 1000.0,
        },
        TelemetryMessageKind::GpsRawInt,
        vehicle_time,
    );
}

pub(super) fn handle_home_position(
    context: &UpdateContext<'_>,
    data: &dialect::HOME_POSITION_DATA,
) {
    let _ = context
        .writers
        .home_position
        .send(Some(mission::HomePosition {
            latitude_deg: f64::from(data.latitude) / 1e7,
            longitude_deg: f64::from(data.longitude) / 1e7,
            altitude_m: f64::from(data.altitude) / 1000.0,
        }));

    let vehicle_time =
        infer_timestamp_from_time_usec(TelemetryMessageKind::HomePosition, data.time_usec);
    context
        .writers
        .telemetry_metrics
        .message_writers
        .home_position
        .publish(data.clone(), vehicle_time.clone());
    context.writers.telemetry_metrics.home.publish(
        GeoPoint3dMsl {
            latitude_deg: f64::from(data.latitude) / 1e7,
            longitude_deg: f64::from(data.longitude) / 1e7,
            altitude_msl_m: f64::from(data.altitude) / 1000.0,
        },
        TelemetryMessageKind::HomePosition,
        vehicle_time,
    );
}

pub(super) fn handle_gps_global_origin(
    context: &UpdateContext<'_>,
    data: &dialect::GPS_GLOBAL_ORIGIN_DATA,
) {
    let vehicle_time =
        infer_timestamp_from_time_usec(TelemetryMessageKind::GpsGlobalOrigin, data.time_usec);
    context
        .writers
        .telemetry_metrics
        .message_writers
        .gps_global_origin
        .publish(data.clone(), vehicle_time.clone());
    context.writers.telemetry_metrics.origin.publish(
        GeoPoint3dMsl {
            latitude_deg: f64::from(data.latitude) / 1e7,
            longitude_deg: f64::from(data.longitude) / 1e7,
            altitude_msl_m: f64::from(data.altitude) / 1000.0,
        },
        TelemetryMessageKind::GpsGlobalOrigin,
        vehicle_time,
    );
}
