use pyo3::exceptions::PyKeyError;
use pyo3::prelude::*;
use std::collections::HashMap;

use crate::enums::*;

// --- Param ---

#[pyclass(name = "Param", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyParam {
    pub(crate) inner: mavkit::Param,
}

#[pymethods]
impl PyParam {
    #[getter]
    fn name(&self) -> &str {
        &self.inner.name
    }
    #[getter]
    fn value(&self) -> f32 {
        self.inner.value
    }
    #[getter]
    fn param_type(&self) -> PyParamType {
        self.inner.param_type.into()
    }
    #[getter]
    fn index(&self) -> u16 {
        self.inner.index
    }

    fn __repr__(&self) -> String {
        format!(
            "Param(name='{}', value={}, type={:?})",
            self.inner.name, self.inner.value, self.inner.param_type
        )
    }
}

// --- ParamStore ---

#[pyclass(name = "ParamStore", frozen, from_py_object)]
#[derive(Clone)]
pub struct PyParamStore {
    pub(crate) inner: mavkit::ParamStore,
}

#[pymethods]
impl PyParamStore {
    #[getter]
    fn expected_count(&self) -> u16 {
        self.inner.expected_count
    }

    fn get(&self, name: &str) -> Option<PyParam> {
        self.inner
            .params
            .get(name)
            .map(|p| PyParam { inner: p.clone() })
    }

    fn keys(&self) -> Vec<String> {
        let mut keys: Vec<String> = self.inner.params.keys().cloned().collect();
        keys.sort();
        keys
    }

    fn values(&self) -> Vec<PyParam> {
        let mut items: Vec<_> = self.inner.params.values().collect();
        items.sort_by_key(|p| &p.name);
        items
            .into_iter()
            .map(|p| PyParam { inner: p.clone() })
            .collect()
    }

    fn __len__(&self) -> usize {
        self.inner.params.len()
    }

    fn __contains__(&self, name: &str) -> bool {
        self.inner.params.contains_key(name)
    }

    fn __getitem__(&self, name: &str) -> PyResult<PyParam> {
        self.inner
            .params
            .get(name)
            .map(|p| PyParam { inner: p.clone() })
            .ok_or_else(|| PyKeyError::new_err(name.to_string()))
    }

    fn __repr__(&self) -> String {
        format!(
            "ParamStore({}/{} params)",
            self.inner.params.len(),
            self.inner.expected_count
        )
    }
}

// --- ParamProgress ---

#[pyclass(name = "ParamProgress", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyParamProgress {
    pub(crate) inner: mavkit::ParamProgress,
}

#[pymethods]
impl PyParamProgress {
    #[getter]
    fn phase(&self) -> PyParamTransferPhase {
        self.inner.phase.into()
    }
    #[getter]
    fn received(&self) -> u16 {
        self.inner.received
    }
    #[getter]
    fn expected(&self) -> u16 {
        self.inner.expected
    }

    fn __repr__(&self) -> String {
        format!(
            "ParamProgress({:?}: {}/{})",
            self.inner.phase, self.inner.received, self.inner.expected
        )
    }
}

// --- ParamWriteResult ---

#[pyclass(name = "ParamWriteResult", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyParamWriteResult {
    pub(crate) inner: mavkit::ParamWriteResult,
}

#[pymethods]
impl PyParamWriteResult {
    #[getter]
    fn name(&self) -> &str {
        &self.inner.name
    }
    #[getter]
    fn requested_value(&self) -> f32 {
        self.inner.requested_value
    }
    #[getter]
    fn confirmed_value(&self) -> f32 {
        self.inner.confirmed_value
    }
    #[getter]
    fn success(&self) -> bool {
        self.inner.success
    }

    fn __repr__(&self) -> String {
        format!(
            "ParamWriteResult(name='{}', requested={}, confirmed={}, success={})",
            self.inner.name,
            self.inner.requested_value,
            self.inner.confirmed_value,
            self.inner.success
        )
    }
}

// --- Free functions ---

#[pyfunction]
pub fn format_param_file(store: &PyParamStore) -> String {
    mavkit::format_param_file(&store.inner)
}

#[pyfunction]
pub fn parse_param_file(contents: &str) -> PyResult<HashMap<String, f32>> {
    mavkit::parse_param_file(contents).map_err(pyo3::exceptions::PyValueError::new_err)
}
