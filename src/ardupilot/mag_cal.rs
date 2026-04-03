use crate::observation::{ObservationHandle, ObservationWriter};
use crate::state::{MagCalProgress, MagCalReport, StateChannels};
use std::collections::BTreeMap;
use tokio::sync::watch;
use tokio_util::sync::CancellationToken;

#[derive(Clone)]
pub(super) struct MagCalObservations {
    progress_writer: ObservationWriter<Vec<MagCalProgress>>,
    progress: ObservationHandle<Vec<MagCalProgress>>,
    report_writer: ObservationWriter<Vec<MagCalReport>>,
    report: ObservationHandle<Vec<MagCalReport>>,
}

impl MagCalObservations {
    pub(super) fn new() -> Self {
        let (progress_writer, progress) = ObservationHandle::watch();
        let (report_writer, report) = ObservationHandle::watch();
        let _ = progress_writer.publish(Vec::new());
        let _ = report_writer.publish(Vec::new());

        Self {
            progress_writer,
            progress,
            report_writer,
            report,
        }
    }

    pub(super) fn start(&self, stores: &StateChannels, cancel: CancellationToken) {
        spawn_compass_aggregator(
            stores.mag_cal_progress.clone(),
            self.progress_writer.clone(),
            cancel.clone(),
            |progress| progress.compass_id,
        );
        spawn_compass_aggregator(
            stores.mag_cal_report.clone(),
            self.report_writer.clone(),
            cancel,
            |report| report.compass_id,
        );
    }

    pub(super) fn progress(&self) -> ObservationHandle<Vec<MagCalProgress>> {
        self.progress.clone()
    }

    pub(super) fn report(&self) -> ObservationHandle<Vec<MagCalReport>> {
        self.report.clone()
    }
}

fn spawn_compass_aggregator<T>(
    mut rx: watch::Receiver<Option<T>>,
    writer: ObservationWriter<Vec<T>>,
    cancel: CancellationToken,
    compass_id_of: fn(&T) -> u8,
) where
    T: Clone + Send + Sync + 'static,
{
    tokio::spawn(async move {
        let mut by_compass = BTreeMap::<u8, T>::new();
        if let Some(item) = rx.borrow().clone() {
            by_compass.insert(compass_id_of(&item), item);
            let _ = writer.publish(by_compass.values().cloned().collect());
        }

        loop {
            tokio::select! {
                _ = cancel.cancelled() => break,
                changed = rx.changed() => {
                    if changed.is_err() {
                        break;
                    }

                    if let Some(item) = rx.borrow_and_update().clone() {
                        by_compass.insert(compass_id_of(&item), item);
                        let _ = writer.publish(by_compass.values().cloned().collect());
                    }
                }
            }
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::{self, create_channels};
    use std::time::Duration;
    use tokio::time::timeout;

    #[tokio::test]
    async fn aggregation_keeps_btreemap_order_and_initial_empty_vec() {
        let (writers, channels) = create_channels();
        let cancel = CancellationToken::new();
        let observations = MagCalObservations::new();
        observations.start(&channels, cancel.clone());

        assert_eq!(observations.progress().latest(), Some(Vec::new()));
        assert_eq!(observations.report().latest(), Some(Vec::new()));

        let mut progress_sub = observations.progress().subscribe();

        writers
            .mag_cal_progress
            .send_replace(Some(state::MagCalProgress {
                compass_id: 2,
                completion_pct: 45,
                status: state::MagCalStatus::RunningStepOne,
                attempt: 1,
            }));

        let first_progress = timeout(Duration::from_millis(250), async {
            loop {
                let observed = progress_sub.recv().await.expect("progress update expected");
                if observed.len() == 1 {
                    break observed;
                }
            }
        })
        .await
        .expect("progress aggregation should update");

        assert_eq!(first_progress[0].compass_id, 2);

        writers
            .mag_cal_progress
            .send_replace(Some(state::MagCalProgress {
                compass_id: 1,
                completion_pct: 80,
                status: state::MagCalStatus::RunningStepTwo,
                attempt: 2,
            }));

        let progress = timeout(Duration::from_millis(250), async {
            loop {
                let observed = progress_sub.recv().await.expect("progress update expected");
                if observed.len() == 2 {
                    break observed;
                }
            }
        })
        .await
        .expect("progress aggregation should update");

        assert_eq!(
            progress
                .iter()
                .map(|item| item.compass_id)
                .collect::<Vec<_>>(),
            vec![1, 2]
        );

        cancel.cancel();
    }
}
