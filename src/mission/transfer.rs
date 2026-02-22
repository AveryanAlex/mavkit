use super::types::MissionType;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum TransferDirection {
    Upload,
    Download,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum TransferPhase {
    Idle,
    RequestCount,
    TransferItems,
    AwaitAck,
    Completed,
    Failed,
    Cancelled,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub struct RetryPolicy {
    pub request_timeout_ms: u64,
    pub item_timeout_ms: u64,
    pub max_retries: u8,
}

impl Default for RetryPolicy {
    fn default() -> Self {
        Self {
            request_timeout_ms: 1500,
            item_timeout_ms: 250,
            max_retries: 5,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct TransferProgress {
    pub direction: TransferDirection,
    pub mission_type: MissionType,
    pub phase: TransferPhase,
    pub completed_items: u16,
    pub total_items: u16,
    pub retries_used: u8,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct TransferError {
    pub code: String,
    pub message: String,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum TransferEvent {
    Progress { progress: TransferProgress },
    Error { error: TransferError },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MissionTransferMachine {
    direction: TransferDirection,
    mission_type: MissionType,
    phase: TransferPhase,
    total_items: u16,
    completed_items: u16,
    retries_used: u8,
    policy: RetryPolicy,
}

impl MissionTransferMachine {
    pub fn new_upload(mission_type: MissionType, total_items: u16, policy: RetryPolicy) -> Self {
        Self {
            direction: TransferDirection::Upload,
            mission_type,
            phase: TransferPhase::RequestCount,
            total_items,
            completed_items: 0,
            retries_used: 0,
            policy,
        }
    }

    pub fn new_download(mission_type: MissionType, policy: RetryPolicy) -> Self {
        Self {
            direction: TransferDirection::Download,
            mission_type,
            phase: TransferPhase::RequestCount,
            total_items: 0,
            completed_items: 0,
            retries_used: 0,
            policy,
        }
    }

    pub fn set_download_total(&mut self, total_items: u16) {
        self.total_items = total_items;
        self.phase = if total_items == 0 {
            TransferPhase::AwaitAck
        } else {
            TransferPhase::TransferItems
        };
    }

    pub fn on_item_transferred(&mut self) {
        if self.phase == TransferPhase::RequestCount {
            self.phase = TransferPhase::TransferItems;
        }

        if self.phase != TransferPhase::TransferItems {
            return;
        }

        if self.completed_items < self.total_items {
            self.completed_items += 1;
        }

        if self.completed_items >= self.total_items {
            self.phase = TransferPhase::AwaitAck;
        }
    }

    pub fn on_timeout(&mut self) -> Option<TransferError> {
        if self.phase == TransferPhase::Completed
            || self.phase == TransferPhase::Failed
            || self.phase == TransferPhase::Cancelled
        {
            return None;
        }

        self.retries_used = self.retries_used.saturating_add(1);
        if self.retries_used > self.policy.max_retries {
            self.phase = TransferPhase::Failed;
            return Some(TransferError {
                code: "transfer.timeout".to_string(),
                message: "Mission transfer timed out after maximum retries".to_string(),
            });
        }

        None
    }

    pub fn on_ack_success(&mut self) {
        if self.phase == TransferPhase::AwaitAck {
            self.phase = TransferPhase::Completed;
        }
    }

    pub fn on_error(&mut self, code: &str, message: &str) -> TransferError {
        self.phase = TransferPhase::Failed;
        TransferError {
            code: code.to_string(),
            message: message.to_string(),
        }
    }

    pub fn cancel(&mut self) {
        self.phase = TransferPhase::Cancelled;
    }

    pub fn progress(&self) -> TransferProgress {
        TransferProgress {
            direction: self.direction,
            mission_type: self.mission_type,
            phase: self.phase,
            completed_items: self.completed_items,
            total_items: self.total_items,
            retries_used: self.retries_used,
        }
    }

    pub fn is_terminal(&self) -> bool {
        matches!(
            self.phase,
            TransferPhase::Completed | TransferPhase::Failed | TransferPhase::Cancelled
        )
    }

    pub fn timeout_ms(&self) -> u64 {
        if self.phase == TransferPhase::TransferItems {
            self.policy.item_timeout_ms
        } else {
            self.policy.request_timeout_ms
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mission::{MissionFrame, MissionItem, MissionPlan, MissionType};

    fn sample_plan(count: usize) -> MissionPlan {
        let mut items = Vec::with_capacity(count);
        for seq in 0..count {
            items.push(MissionItem {
                seq: seq as u16,
                command: 16,
                frame: MissionFrame::GlobalRelativeAltInt,
                current: seq == 0,
                autocontinue: true,
                param1: 0.0,
                param2: 0.0,
                param3: 0.0,
                param4: 0.0,
                x: 0,
                y: 0,
                z: 10.0,
            });
        }
        MissionPlan {
            mission_type: MissionType::Mission,
            home: None,
            items,
        }
    }

    #[test]
    fn upload_flow_reaches_completed_state() {
        let plan = sample_plan(2);
        let mut machine = MissionTransferMachine::new_upload(
            plan.mission_type,
            plan.items.len() as u16,
            RetryPolicy::default(),
        );

        assert_eq!(machine.progress().phase, TransferPhase::RequestCount);
        machine.on_item_transferred();
        assert_eq!(machine.progress().phase, TransferPhase::TransferItems);
        machine.on_item_transferred();
        assert_eq!(machine.progress().phase, TransferPhase::AwaitAck);
        machine.on_ack_success();
        assert_eq!(machine.progress().phase, TransferPhase::Completed);
    }

    #[test]
    fn timeout_beyond_retry_budget_fails_transfer() {
        let plan = sample_plan(1);
        let mut machine = MissionTransferMachine::new_upload(
            plan.mission_type,
            plan.items.len() as u16,
            RetryPolicy {
                max_retries: 1,
                ..RetryPolicy::default()
            },
        );

        assert!(machine.on_timeout().is_none());
        let err = machine.on_timeout().expect("timeout should fail");
        assert_eq!(err.code, "transfer.timeout");
        assert_eq!(machine.progress().phase, TransferPhase::Failed);
    }

    #[test]
    fn download_flow_uses_item_timeout_after_count() {
        let mut machine =
            MissionTransferMachine::new_download(MissionType::Fence, RetryPolicy::default());
        assert_eq!(machine.timeout_ms(), 1500);
        machine.set_download_total(3);
        assert_eq!(machine.progress().phase, TransferPhase::TransferItems);
        assert_eq!(machine.timeout_ms(), 250);
    }

    #[test]
    fn cancel_sets_cancelled_phase() {
        let mut machine = MissionTransferMachine::new_upload(
            MissionType::Mission,
            3,
            RetryPolicy::default(),
        );
        assert_eq!(machine.progress().phase, TransferPhase::RequestCount);
        machine.cancel();
        assert_eq!(machine.progress().phase, TransferPhase::Cancelled);
    }

    #[test]
    fn timeout_after_cancel_is_noop() {
        let mut machine = MissionTransferMachine::new_upload(
            MissionType::Mission,
            3,
            RetryPolicy::default(),
        );
        machine.cancel();
        assert_eq!(machine.progress().phase, TransferPhase::Cancelled);
        assert!(machine.on_timeout().is_none());
        assert_eq!(machine.progress().phase, TransferPhase::Cancelled);
    }

    #[test]
    fn is_terminal_for_end_states() {
        let mut completed = MissionTransferMachine::new_upload(
            MissionType::Mission,
            2,
            RetryPolicy::default(),
        );
        completed.on_item_transferred();
        completed.on_item_transferred();
        completed.on_ack_success();
        assert!(completed.is_terminal());
        assert_eq!(completed.progress().phase, TransferPhase::Completed);

        let mut failed = MissionTransferMachine::new_upload(
            MissionType::Mission,
            1,
            RetryPolicy {
                max_retries: 0,
                ..RetryPolicy::default()
            },
        );
        let _ = failed.on_timeout();
        assert!(failed.is_terminal());
        assert_eq!(failed.progress().phase, TransferPhase::Failed);

        let mut cancelled = MissionTransferMachine::new_download(
            MissionType::Fence,
            RetryPolicy::default(),
        );
        cancelled.cancel();
        assert!(cancelled.is_terminal());
        assert_eq!(cancelled.progress().phase, TransferPhase::Cancelled);

        let active = MissionTransferMachine::new_upload(
            MissionType::Mission,
            3,
            RetryPolicy::default(),
        );
        assert!(!active.is_terminal());
    }
}
