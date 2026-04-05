use pyo3::prelude::*;

use super::model::{PyCompareTolerance, PyMissionIssue, PyMissionItem, PyMissionPlan};

#[pyfunction]
pub fn validate_plan(plan: &PyMissionPlan) -> Vec<PyMissionIssue> {
    mavkit::validate_plan(&plan.inner)
        .into_iter()
        .map(|i| PyMissionIssue { inner: i })
        .collect()
}

#[pyfunction]
#[pyo3(signature = (lhs, rhs, tolerance=None))]
pub fn plans_equivalent(
    lhs: &PyMissionPlan,
    rhs: &PyMissionPlan,
    tolerance: Option<&PyCompareTolerance>,
) -> bool {
    let tol = tolerance.map(|t| t.inner).unwrap_or_default();
    mavkit::plans_equivalent(&lhs.inner, &rhs.inner, tol)
}

#[pyfunction]
pub fn normalize_for_compare(plan: &PyMissionPlan) -> PyMissionPlan {
    PyMissionPlan {
        inner: mavkit::normalize_for_compare(&plan.inner),
    }
}

#[pyfunction]
pub fn mission_items_for_upload(plan: &PyMissionPlan) -> Vec<PyMissionItem> {
    mavkit::mission_items_for_upload(&plan.inner)
        .into_iter()
        .map(|inner| PyMissionItem { inner })
        .collect()
}

#[pyfunction]
pub fn mission_plan_from_download(items: Vec<PyMissionItem>) -> PyMissionPlan {
    let wire_items: Vec<mavkit::MissionItem> = items.into_iter().map(|i| i.inner).collect();
    PyMissionPlan {
        inner: mavkit::mission_plan_from_download(wire_items),
    }
}
