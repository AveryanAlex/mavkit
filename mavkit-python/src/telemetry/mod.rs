mod common;
mod messages;
mod metrics;
mod namespaces;
mod sensor_health;
mod status_text;
mod values;

use pyo3::prelude::*;

pub(crate) use namespaces::PyTelemetryHandle;

pub(crate) fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<sensor_health::PySensorHealthState>()?;
    m.add_class::<values::PyGlobalPosition>()?;
    m.add_class::<values::PyEulerAttitude>()?;
    m.add_class::<values::PyGpsQuality>()?;
    m.add_class::<values::PyCellVoltages>()?;
    m.add_class::<values::PyWaypointProgress>()?;
    m.add_class::<values::PyGuidanceState>()?;
    m.add_class::<values::PyTerrainClearance>()?;
    m.add_class::<sensor_health::PySensorHealthSummary>()?;
    m.add_class::<status_text::PyStatusTextEvent>()?;
    m.add_class::<metrics::PyMetricSample>()?;
    m.add_class::<messages::PyMessageSample>()?;
    m.add_class::<metrics::PyMetricSubscription>()?;
    m.add_class::<messages::PyMessageSubscription>()?;
    m.add_class::<metrics::PyMetricHandle>()?;
    m.add_class::<messages::PyPeriodicMessageHandle>()?;
    m.add_class::<messages::PyEventMessageHandle>()?;
    m.add_class::<messages::PyMessageHandle>()?;
    m.add_class::<namespaces::PyTelemetryPositionNamespace>()?;
    m.add_class::<namespaces::PyTelemetryAttitudeNamespace>()?;
    m.add_class::<namespaces::PyTelemetryBatteryNamespace>()?;
    m.add_class::<namespaces::PyTelemetryGpsNamespace>()?;
    m.add_class::<namespaces::PyTelemetryNavigationNamespace>()?;
    m.add_class::<namespaces::PyTelemetryTerrainNamespace>()?;
    m.add_class::<namespaces::PyTelemetryRcNamespace>()?;
    m.add_class::<namespaces::PyTelemetryActuatorsNamespace>()?;
    m.add_class::<namespaces::PyTelemetryMessagesHandle>()?;
    m.add_class::<namespaces::PyTelemetryHandle>()?;
    Ok(())
}
