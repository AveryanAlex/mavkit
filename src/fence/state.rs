use crate::error::VehicleError;
use crate::mission::{MissionType, WireMissionPlan};
use crate::observation::ObservationHandle;
use crate::stored_plan::{StoredPlanAccess, StoredPlanDomain, StoredPlanState};
use crate::types::{StoredPlanOperationKind, SupportState, SyncState};
use crate::vehicle::VehicleInner;

use super::plan::FencePlan;
use super::wire::{fence_plan_from_mission_plan, mission_plan_from_fence_plan};

/// Cached fence-domain state plus sync and active-operation markers.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct FenceState {
    pub plan: Option<FencePlan>,
    pub sync: SyncState,
    pub active_op: Option<StoredPlanOperationKind>,
}

/// Shared fence-domain state stored on `VehicleInner`.
pub(crate) type FenceDomain = StoredPlanDomain<FenceState>;

impl StoredPlanState for FenceState {
    type Plan = FencePlan;
    type OperationKind = StoredPlanOperationKind;

    fn set_active_op(&mut self, kind: Option<Self::OperationKind>) {
        self.active_op = kind;
    }

    fn set_plan(&mut self, plan: Self::Plan) {
        self.plan = Some(plan);
    }

    fn set_sync(&mut self, sync: SyncState) {
        self.sync = sync;
    }

    fn cleared_plan() -> Self::Plan {
        FencePlan {
            return_point: None,
            regions: Vec::new(),
        }
    }
}

impl StoredPlanAccess for FenceState {
    const DOMAIN_NAME: &'static str = "fence";
    const MISSION_TYPE: MissionType = MissionType::Fence;

    fn domain(inner: &VehicleInner) -> &StoredPlanDomain<Self> {
        &inner.fence
    }

    fn support(inner: &VehicleInner) -> ObservationHandle<SupportState> {
        let vehicle_state = inner.stores.vehicle_state.borrow().clone();
        inner.support.seed_from_vehicle_state(&vehicle_state);
        inner.support.mission_fence()
    }

    fn plan_to_wire(plan: &FencePlan) -> Result<WireMissionPlan, VehicleError> {
        mission_plan_from_fence_plan(plan)
    }

    fn plan_from_wire(plan: WireMissionPlan) -> Result<FencePlan, VehicleError> {
        fence_plan_from_mission_plan(plan)
    }
}

#[cfg(test)]
mod tests {
    use crate::error::VehicleError;
    use crate::mission::{MissionDomain, MissionProtocolScope};
    use crate::stored_plan::StoredPlanDomain;
    use crate::types::StoredPlanOperationKind;

    use super::FenceState;

    type FenceDomain = StoredPlanDomain<FenceState>;

    #[test]
    fn mission_upload_conflicts_with_fence_download() {
        let mission_protocol = MissionProtocolScope::new();
        let mission = MissionDomain::new();
        let fence = FenceDomain::new();

        let upload = mission
            .begin_operation(
                &mission_protocol,
                crate::types::MissionOperationKind::Upload,
                "upload",
            )
            .expect("mission upload should start first");

        let conflict = match fence.begin_operation(
            &mission_protocol,
            "fence",
            StoredPlanOperationKind::Download,
            "download",
        ) {
            Ok(_) => panic!("fence download should conflict while mission upload is active"),
            Err(err) => err,
        };

        assert!(matches!(
            conflict,
            VehicleError::OperationConflict {
                conflicting_domain,
                conflicting_op,
            } if conflicting_domain == "mission" && conflicting_op == "upload"
        ));

        mission.finish_operation(&mission_protocol, upload.id);
    }
}
