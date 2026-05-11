use super::{MagCalProgress, MagCalReport, guided, mag_cal};
use crate::error::VehicleError;
use crate::observation::ObservationHandle;
use crate::runtime::TaskHandle;
use crate::state::StateChannels;
use std::sync::Arc;
use tokio_util::sync::CancellationToken;

#[derive(Clone)]
pub(crate) struct ArduPilotDomain {
    inner: Arc<ArduPilotDomainInner>,
}

struct ArduPilotDomainInner {
    mag_cal: mag_cal::MagCalObservations,
    guided_lease: guided::GuidedLeaseScope,
}

impl ArduPilotDomain {
    pub(crate) fn new() -> Self {
        Self {
            inner: Arc::new(ArduPilotDomainInner {
                mag_cal: mag_cal::MagCalObservations::new(),
                guided_lease: guided::GuidedLeaseScope::new(),
            }),
        }
    }

    pub(crate) fn start(
        &self,
        stores: &StateChannels,
        cancel: CancellationToken,
    ) -> Vec<TaskHandle> {
        self.inner.mag_cal.start(stores, cancel)
    }

    pub(crate) fn mag_cal_progress(&self) -> ObservationHandle<Vec<MagCalProgress>> {
        self.inner.mag_cal.progress()
    }

    pub(crate) fn mag_cal_report(&self) -> ObservationHandle<Vec<MagCalReport>> {
        self.inner.mag_cal.report()
    }

    pub(crate) fn begin_guided_lease(&self) -> Result<u64, VehicleError> {
        self.inner.guided_lease.acquire()
    }

    pub(crate) fn release_guided_lease(&self, lease_id: u64) {
        self.inner.guided_lease.release(lease_id);
    }

    pub(crate) fn guided_lease_scope(&self) -> guided::GuidedLeaseScope {
        self.inner.guided_lease.clone()
    }

    pub(crate) fn close(&self) {
        self.inner.mag_cal.close();
    }
}
