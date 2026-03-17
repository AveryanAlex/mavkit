use crate::types::{ParamOperationKind, SyncState};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// MAVLink parameter value type.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ParamType {
    Uint8,
    Int8,
    Uint16,
    Int16,
    Uint32,
    Int32,
    Real32,
}

/// A single vehicle parameter with name, value, type, and index.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Param {
    pub name: String,
    pub value: f32,
    pub param_type: ParamType,
    pub index: u16,
}

/// In-memory store of all downloaded vehicle parameters.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct ParamStore {
    pub params: HashMap<String, Param>,
    pub expected_count: u16,
}

/// Result of a single parameter write, with requested and confirmed values.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ParamWriteResult {
    pub name: String,
    pub requested_value: f32,
    pub confirmed_value: f32,
    pub success: bool,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
/// Cached parameter-domain state plus sync and active-operation markers.
pub struct ParamState {
    pub store: Option<ParamStore>,
    pub sync: SyncState,
    pub active_op: Option<ParamOperationKind>,
}
