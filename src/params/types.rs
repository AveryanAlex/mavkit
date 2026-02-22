use serde::{Deserialize, Serialize};
use std::collections::HashMap;

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

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Param {
    pub name: String,
    pub value: f32,
    pub param_type: ParamType,
    pub index: u16,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct ParamStore {
    pub params: HashMap<String, Param>,
    pub expected_count: u16,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ParamTransferPhase {
    Idle,
    Downloading,
    Writing,
    Completed,
    Failed,
}

impl Default for ParamTransferPhase {
    fn default() -> Self {
        ParamTransferPhase::Idle
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ParamProgress {
    pub phase: ParamTransferPhase,
    pub received: u16,
    pub expected: u16,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ParamWriteResult {
    pub name: String,
    pub requested_value: f32,
    pub confirmed_value: f32,
    pub success: bool,
}

impl Default for ParamProgress {
    fn default() -> Self {
        ParamProgress {
            phase: ParamTransferPhase::Idle,
            received: 0,
            expected: 0,
        }
    }
}
