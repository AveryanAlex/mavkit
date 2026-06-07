use serde::{Deserialize, Serialize};

/// Parsed firmware identity details from `AUTOPILOT_VERSION`.
#[cfg_attr(feature = "typescript", derive(specta::Type))]
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct FirmwareInfo {
    pub version: Option<String>,
    pub custom_version: Option<[u8; 8]>,
    pub git_hash: Option<String>,
    pub os_version: Option<String>,
}

/// Parsed hardware board and USB identity details.
#[cfg_attr(feature = "typescript", derive(specta::Type))]
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct HardwareInfo {
    pub board_vendor_id: Option<u16>,
    pub board_product_id: Option<u16>,
    pub usb_vendor_id: Option<u16>,
    pub usb_product_id: Option<u16>,
    pub board_version: Option<u32>,
}

/// Stable identifiers MAVKit can use for cross-session correlation.
#[cfg_attr(feature = "typescript", derive(specta::Type))]
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct UniqueIds {
    pub hardware_uid: Option<Vec<u8>>,
    #[cfg_attr(feature = "typescript", specta(type = Option<f64>))]
    pub uid: Option<u64>,
    pub remote_id: Option<String>,
    pub board_id: Option<String>,
}

/// Persistent identity readiness for UI labels and cache keys.
#[cfg_attr(feature = "typescript", derive(specta::Type))]
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case", tag = "state")]
pub enum PersistentIdentity {
    Pending {
        system_id: u8,
        component_id: u8,
    },
    Ready {
        canonical_id: String,
        aliases: Vec<String>,
    },
}
