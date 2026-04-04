use serde::{Deserialize, Serialize};

/// Source used to build the current mode catalog.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ModeCatalogSource {
    AvailableModes,
    StaticArduPilotTable,
}

/// One flight mode entry users can inspect or request.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ModeDescriptor {
    pub custom_mode: u32,
    pub name: String,
    pub user_selectable: bool,
    pub source: ModeCatalogSource,
}

/// Source used to determine the currently active mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CurrentModeSource {
    Heartbeat,
    CurrentModeMessage,
}

/// Current mode snapshot, including optional pending target mode.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CurrentMode {
    pub custom_mode: u32,
    pub name: String,
    pub intended_custom_mode: Option<u32>,
    pub source: CurrentModeSource,
}
