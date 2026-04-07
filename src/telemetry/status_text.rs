use crate::VehicleTimestamp;
use crate::dialect;
use crate::observation::{
    MessageHandle, MessageSample, ObservationHandle, ObservationWriter, SupportState,
};
use crate::shared_state::recover_lock;
use mavlink::MavHeader;
use std::collections::{BTreeMap, HashMap};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

pub(crate) const STATUS_TEXT_BROADCAST_CAPACITY: usize = 256;
const STATUS_TEXT_FLUSH_TIMEOUT: Duration = Duration::from_secs(2);

/// Reassembled `STATUSTEXT` event with source identity.
#[derive(Clone, Debug, PartialEq)]
pub struct StatusTextEvent {
    pub text: String,
    pub severity: dialect::MavSeverity,
    pub id: u16,
    pub source_system: u8,
    pub source_component: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct PendingKey {
    source_system: u8,
    source_component: u8,
    id: u16,
}

#[derive(Clone, Debug)]
struct PendingStatusText {
    chunks: BTreeMap<u8, String>,
    severity: dialect::MavSeverity,
    generation: u64,
    vehicle_time: Option<VehicleTimestamp>,
}

#[derive(Clone)]
pub(crate) struct StatusTextWriter {
    sample: ObservationWriter<MessageSample<StatusTextEvent>>,
    support: ObservationWriter<SupportState>,
    pending: Arc<Mutex<HashMap<PendingKey, PendingStatusText>>>,
    closed: Arc<AtomicBool>,
    flush_timeout: Duration,
}

pub(crate) fn create_status_text_backing_store()
-> (StatusTextWriter, MessageHandle<StatusTextEvent>) {
    let (sample_writer, sample_observation) =
        ObservationHandle::broadcast(STATUS_TEXT_BROADCAST_CAPACITY);
    let (support_writer, support_observation) = ObservationHandle::watch();
    let _ = support_writer.publish(SupportState::Unknown);

    (
        StatusTextWriter {
            sample: sample_writer,
            support: support_writer,
            pending: Arc::new(Mutex::new(HashMap::new())),
            closed: Arc::new(AtomicBool::new(false)),
            flush_timeout: STATUS_TEXT_FLUSH_TIMEOUT,
        },
        MessageHandle::new(sample_observation, support_observation),
    )
}

impl StatusTextWriter {
    pub(crate) fn publish(&self, value: StatusTextEvent, vehicle_time: Option<VehicleTimestamp>) {
        if self.closed.load(Ordering::SeqCst) {
            return;
        }
        let _ = self.support.publish(SupportState::Supported);
        let _ = self.sample.publish(MessageSample {
            value,
            received_at: Instant::now(),
            vehicle_time,
        });
    }

    pub(crate) fn close(&self) {
        self.closed.store(true, Ordering::SeqCst);
        recover_lock(&self.pending).clear();
        self.sample.close();
        self.support.close();
    }

    pub(crate) fn ingest(
        &self,
        header: &MavHeader,
        data: &dialect::STATUSTEXT_DATA,
        vehicle_time: Option<VehicleTimestamp>,
    ) -> Option<StatusTextEvent> {
        if self.closed.load(Ordering::SeqCst) {
            return None;
        }

        let chunk_text = data.text.to_str().unwrap_or_default().to_string();

        if data.id == 0 {
            let event = StatusTextEvent {
                text: chunk_text,
                severity: data.severity,
                id: 0,
                source_system: header.system_id,
                source_component: header.component_id,
            };
            if !event.text.is_empty() {
                self.publish(event.clone(), vehicle_time);
                return Some(event);
            }
            return None;
        }

        let key = PendingKey {
            source_system: header.system_id,
            source_component: header.component_id,
            id: data.id,
        };

        let terminal_chunk = data.text.contains(&0);
        let mut flush_now = None;
        let mut schedule_generation = None;

        {
            let mut pending = recover_lock(&self.pending);
            let entry = pending.entry(key).or_insert_with(|| PendingStatusText {
                chunks: BTreeMap::new(),
                severity: data.severity,
                generation: 0,
                vehicle_time: vehicle_time.clone(),
            });
            entry.severity = data.severity;
            entry.vehicle_time = vehicle_time.clone().or(entry.vehicle_time.clone());
            entry.chunks.insert(data.chunk_seq, chunk_text);

            if terminal_chunk {
                let completed = pending
                    .remove(&key)
                    .expect("pending status text must exist for terminal chunk");
                flush_now = Some((
                    StatusTextEvent {
                        text: assemble_text(&completed.chunks),
                        severity: completed.severity,
                        id: key.id,
                        source_system: key.source_system,
                        source_component: key.source_component,
                    },
                    completed.vehicle_time,
                ));
            } else {
                entry.generation += 1;
                schedule_generation = Some(entry.generation);
            }
        }

        if let Some((event, vehicle_time)) = flush_now {
            if event.text.is_empty() {
                return None;
            }
            self.publish(event.clone(), vehicle_time);
            return Some(event);
        }

        if let Some(generation) = schedule_generation {
            self.schedule_flush(key, generation);
        }

        None
    }

    fn schedule_flush(&self, key: PendingKey, generation: u64) {
        let writer = self.clone();
        tokio::spawn(async move {
            tokio::time::sleep(writer.flush_timeout).await;

            if writer.closed.load(Ordering::SeqCst) {
                return;
            }

            let flushed = {
                let mut pending = recover_lock(&writer.pending);
                let should_flush = pending
                    .get(&key)
                    .is_some_and(|entry| entry.generation == generation);

                if !should_flush {
                    None
                } else {
                    pending.remove(&key).map(|entry| {
                        (
                            StatusTextEvent {
                                text: assemble_text(&entry.chunks),
                                severity: entry.severity,
                                id: key.id,
                                source_system: key.source_system,
                                source_component: key.source_component,
                            },
                            entry.vehicle_time,
                        )
                    })
                }
            };

            if let Some((event, vehicle_time)) = flushed
                && !event.text.is_empty()
            {
                writer.publish(event, vehicle_time);
            }
        });
    }
}

fn assemble_text(chunks: &BTreeMap<u8, String>) -> String {
    let mut text = String::new();
    for chunk in chunks.values() {
        text.push_str(chunk);
    }
    text
}
