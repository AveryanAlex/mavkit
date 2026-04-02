use pyo3::prelude::*;
use pyo3::types::PyList;

use crate::macros::define_observation_wrapper;
use crate::vehicle::vehicle_label;

#[pyclass(name = "FirmwareInfo", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyFirmwareInfo {
    version: Option<String>,
    custom_version: Option<Vec<u8>>,
    git_hash: Option<String>,
    os_version: Option<String>,
}

impl From<mavkit::FirmwareInfo> for PyFirmwareInfo {
    fn from(value: mavkit::FirmwareInfo) -> Self {
        Self {
            version: value.version,
            custom_version: value.custom_version.map(|bytes| bytes.to_vec()),
            git_hash: value.git_hash,
            os_version: value.os_version,
        }
    }
}

#[pymethods]
impl PyFirmwareInfo {
    #[new]
    #[pyo3(signature = (*, version=None, custom_version=None, git_hash=None, os_version=None))]
    fn new(
        version: Option<String>,
        custom_version: Option<Vec<u8>>,
        git_hash: Option<String>,
        os_version: Option<String>,
    ) -> Self {
        Self {
            version,
            custom_version,
            git_hash,
            os_version,
        }
    }

    #[getter]
    fn version(&self) -> Option<String> {
        self.version.clone()
    }

    #[getter]
    fn custom_version(&self, py: Python<'_>) -> PyResult<Option<Py<PyAny>>> {
        match &self.custom_version {
            Some(bytes) => {
                let list = PyList::empty(py);
                for byte in bytes {
                    list.append(*byte)?;
                }
                Ok(Some(list.into_any().unbind()))
            }
            None => Ok(None),
        }
    }

    #[getter]
    fn git_hash(&self) -> Option<String> {
        self.git_hash.clone()
    }

    #[getter]
    fn os_version(&self) -> Option<String> {
        self.os_version.clone()
    }

    fn __repr__(&self) -> String {
        format!(
            "FirmwareInfo(version={:?}, git_hash={:?}, os_version={:?})",
            self.version, self.git_hash, self.os_version
        )
    }
}

#[pyclass(name = "HardwareInfo", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyHardwareInfo {
    board_vendor_id: Option<u16>,
    board_product_id: Option<u16>,
    usb_vendor_id: Option<u16>,
    usb_product_id: Option<u16>,
    board_version: Option<u32>,
}

impl From<mavkit::HardwareInfo> for PyHardwareInfo {
    fn from(value: mavkit::HardwareInfo) -> Self {
        Self {
            board_vendor_id: value.board_vendor_id,
            board_product_id: value.board_product_id,
            usb_vendor_id: value.usb_vendor_id,
            usb_product_id: value.usb_product_id,
            board_version: value.board_version,
        }
    }
}

#[pymethods]
impl PyHardwareInfo {
    #[new]
    #[pyo3(signature = (*, board_vendor_id=None, board_product_id=None, usb_vendor_id=None, usb_product_id=None, board_version=None))]
    fn new(
        board_vendor_id: Option<u16>,
        board_product_id: Option<u16>,
        usb_vendor_id: Option<u16>,
        usb_product_id: Option<u16>,
        board_version: Option<u32>,
    ) -> Self {
        Self {
            board_vendor_id,
            board_product_id,
            usb_vendor_id,
            usb_product_id,
            board_version,
        }
    }

    #[getter]
    fn board_vendor_id(&self) -> Option<u16> {
        self.board_vendor_id
    }

    #[getter]
    fn board_product_id(&self) -> Option<u16> {
        self.board_product_id
    }

    #[getter]
    fn usb_vendor_id(&self) -> Option<u16> {
        self.usb_vendor_id
    }

    #[getter]
    fn usb_product_id(&self) -> Option<u16> {
        self.usb_product_id
    }

    #[getter]
    fn board_version(&self) -> Option<u32> {
        self.board_version
    }

    fn __repr__(&self) -> String {
        format!(
            "HardwareInfo(board_vendor_id={:?}, board_product_id={:?}, board_version={:?})",
            self.board_vendor_id, self.board_product_id, self.board_version
        )
    }
}

#[pyclass(name = "UniqueIds", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyUniqueIds {
    hardware_uid: Option<Vec<u8>>,
    uid: Option<u64>,
    remote_id: Option<String>,
    board_id: Option<String>,
}

impl From<mavkit::UniqueIds> for PyUniqueIds {
    fn from(value: mavkit::UniqueIds) -> Self {
        Self {
            hardware_uid: value.hardware_uid,
            uid: value.uid,
            remote_id: value.remote_id,
            board_id: value.board_id,
        }
    }
}

#[pymethods]
impl PyUniqueIds {
    #[new]
    #[pyo3(signature = (*, hardware_uid=None, uid=None, remote_id=None, board_id=None))]
    fn new(
        hardware_uid: Option<Vec<u8>>,
        uid: Option<u64>,
        remote_id: Option<String>,
        board_id: Option<String>,
    ) -> Self {
        Self {
            hardware_uid,
            uid,
            remote_id,
            board_id,
        }
    }

    #[getter]
    fn hardware_uid(&self, py: Python<'_>) -> PyResult<Option<Py<PyAny>>> {
        match &self.hardware_uid {
            Some(bytes) => {
                let list = PyList::empty(py);
                for byte in bytes {
                    list.append(*byte)?;
                }
                Ok(Some(list.into_any().unbind()))
            }
            None => Ok(None),
        }
    }

    #[getter]
    fn uid(&self) -> Option<u64> {
        self.uid
    }

    #[getter]
    fn remote_id(&self) -> Option<String> {
        self.remote_id.clone()
    }

    #[getter]
    fn board_id(&self) -> Option<String> {
        self.board_id.clone()
    }

    fn __repr__(&self) -> String {
        format!(
            "UniqueIds(uid={:?}, remote_id={:?}, board_id={:?})",
            self.uid, self.remote_id, self.board_id
        )
    }
}

#[pyclass(name = "PersistentIdentity", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyPersistentIdentity {
    state: String,
    system_id: Option<u8>,
    component_id: Option<u8>,
    canonical_id: Option<String>,
    aliases: Vec<String>,
}

impl From<mavkit::PersistentIdentity> for PyPersistentIdentity {
    fn from(value: mavkit::PersistentIdentity) -> Self {
        match value {
            mavkit::PersistentIdentity::Pending {
                system_id,
                component_id,
            } => Self {
                state: "pending".to_string(),
                system_id: Some(system_id),
                component_id: Some(component_id),
                canonical_id: None,
                aliases: Vec::new(),
            },
            mavkit::PersistentIdentity::Ready {
                canonical_id,
                aliases,
            } => Self {
                state: "ready".to_string(),
                system_id: None,
                component_id: None,
                canonical_id: Some(canonical_id),
                aliases,
            },
        }
    }
}

#[pymethods]
impl PyPersistentIdentity {
    #[new]
    #[pyo3(signature = (*, state, system_id=None, component_id=None, canonical_id=None, aliases=None))]
    fn new(
        state: String,
        system_id: Option<u8>,
        component_id: Option<u8>,
        canonical_id: Option<String>,
        aliases: Option<Vec<String>>,
    ) -> Self {
        Self {
            state,
            system_id,
            component_id,
            canonical_id,
            aliases: aliases.unwrap_or_default(),
        }
    }

    #[getter]
    fn state(&self) -> &str {
        &self.state
    }

    #[getter]
    fn system_id(&self) -> Option<u8> {
        self.system_id
    }

    #[getter]
    fn component_id(&self) -> Option<u8> {
        self.component_id
    }

    #[getter]
    fn canonical_id(&self) -> Option<String> {
        self.canonical_id.clone()
    }

    #[getter]
    fn aliases(&self) -> Vec<String> {
        self.aliases.clone()
    }

    fn __repr__(&self) -> String {
        format!(
            "PersistentIdentity(state='{}', canonical_id={:?}, aliases={:?})",
            self.state, self.canonical_id, self.aliases
        )
    }
}

define_observation_wrapper!(
    PyFirmwareInfoHandle,
    PyFirmwareInfoSubscription,
    mavkit::FirmwareInfo,
    PyFirmwareInfo,
    "FirmwareInfoHandle",
    "FirmwareInfoSubscription",
    "firmware-info subscription closed"
);
define_observation_wrapper!(
    PyHardwareInfoHandle,
    PyHardwareInfoSubscription,
    mavkit::HardwareInfo,
    PyHardwareInfo,
    "HardwareInfoHandle",
    "HardwareInfoSubscription",
    "hardware-info subscription closed"
);
define_observation_wrapper!(
    PyUniqueIdsHandle,
    PyUniqueIdsSubscription,
    mavkit::UniqueIds,
    PyUniqueIds,
    "UniqueIdsHandle",
    "UniqueIdsSubscription",
    "unique-ids subscription closed"
);
define_observation_wrapper!(
    PyPersistentIdentityHandle,
    PyPersistentIdentitySubscription,
    mavkit::PersistentIdentity,
    PyPersistentIdentity,
    "PersistentIdentityHandle",
    "PersistentIdentitySubscription",
    "persistent-identity subscription closed"
);

#[pyclass(name = "InfoHandle", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyInfoHandle {
    pub(crate) inner: mavkit::Vehicle,
}

impl PyInfoHandle {
    pub(crate) fn new(inner: mavkit::Vehicle) -> Self {
        Self { inner }
    }
}

#[pymethods]
impl PyInfoHandle {
    fn firmware(&self) -> PyFirmwareInfoHandle {
        PyFirmwareInfoHandle::new(self.inner.info().firmware())
    }

    fn hardware(&self) -> PyHardwareInfoHandle {
        PyHardwareInfoHandle::new(self.inner.info().hardware())
    }

    fn unique_ids(&self) -> PyUniqueIdsHandle {
        PyUniqueIdsHandle::new(self.inner.info().unique_ids())
    }

    fn best_unique_id(&self) -> Option<String> {
        self.inner.info().best_unique_id()
    }

    fn best_display_id(&self) -> String {
        self.inner.info().best_display_id()
    }

    fn persistent_identity(&self) -> PyPersistentIdentityHandle {
        PyPersistentIdentityHandle::new(self.inner.info().persistent_identity())
    }

    fn __repr__(&self) -> String {
        format!("InfoHandle({})", vehicle_label(&self.inner))
    }
}
