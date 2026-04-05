mod api;
mod common;
mod plan_handles;
mod progress;
mod raw;
mod states;
mod stored_plans;
#[cfg(feature = "test-support")]
mod test_support;
#[cfg(test)]
mod tests;

use pyo3::prelude::*;

pub(crate) use common::vehicle_label;

pub(crate) fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<states::PyMissionState>()?;
    m.add_class::<api::PyVehicleIdentity>()?;

    m.add_class::<stored_plans::PyFenceInclusionPolygon>()?;
    m.add_class::<stored_plans::PyFenceExclusionPolygon>()?;
    m.add_class::<stored_plans::PyFenceInclusionCircle>()?;
    m.add_class::<stored_plans::PyFenceExclusionCircle>()?;
    m.add_class::<stored_plans::PyFencePlan>()?;
    m.add_class::<stored_plans::PyRallyPlan>()?;

    m.add_class::<raw::PyCommandAck>()?;
    m.add_class::<states::PyFenceState>()?;
    m.add_class::<states::PyRallyState>()?;
    m.add_class::<states::PyMissionStateSubscription>()?;
    m.add_class::<states::PyFenceStateSubscription>()?;
    m.add_class::<states::PyRallyStateSubscription>()?;
    m.add_class::<progress::PyMissionOperationProgress>()?;
    m.add_class::<progress::PyMissionProgressSubscription>()?;
    m.add_class::<progress::PyMissionUploadOp>()?;
    m.add_class::<progress::PyMissionDownloadOp>()?;
    m.add_class::<progress::PyMissionClearOp>()?;
    m.add_class::<progress::PyMissionVerifyOp>()?;
    m.add_class::<progress::PyFenceUploadOp>()?;
    m.add_class::<progress::PyFenceDownloadOp>()?;
    m.add_class::<progress::PyFenceClearOp>()?;
    m.add_class::<progress::PyRallyUploadOp>()?;
    m.add_class::<progress::PyRallyDownloadOp>()?;
    m.add_class::<progress::PyRallyClearOp>()?;
    #[cfg(feature = "test-support")]
    m.add_class::<test_support::PyTestVehicleHarness>()?;

    m.add_class::<plan_handles::PyMissionHandle>()?;
    m.add_class::<plan_handles::PyFenceHandle>()?;
    m.add_class::<plan_handles::PyRallyHandle>()?;
    m.add_class::<raw::PyRawHandle>()?;
    m.add_class::<api::PyVehicle>()?;

    #[cfg(feature = "test-support")]
    m.add_function(wrap_pyfunction!(test_support::_connect_test_vehicle, m)?)?;

    Ok(())
}
