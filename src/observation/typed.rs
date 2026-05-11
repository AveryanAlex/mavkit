use super::{ObservationHandle, ObservationSubscription, SupportState};
use crate::error::VehicleError;
use crate::telemetry::{TelemetryMessageKind, VehicleTimestamp};
use crate::time::Instant;
use std::time::Duration;

/// A decoded metric value together with the provenance needed to reason about freshness.
///
/// Multiple MAVLink message types can carry the same logical metric (e.g. altitude
/// appears in both `GLOBAL_POSITION_INT` and `VFR_HUD`). `source` records which
/// message produced this sample so callers can filter or prefer a specific source.
///
/// `vehicle_time` is `None` when the autopilot's message did not carry a usable
/// timestamp (e.g. the field was zero or outside the plausible boot-time range).
/// Use `received_at` as a fallback for staleness checks in that case.
#[derive(Clone, Debug, PartialEq)]
pub struct MetricSample<T> {
    pub value: T,
    /// Which MAVLink message type produced this sample.
    pub source: TelemetryMessageKind,
    /// Vehicle boot-relative timestamp extracted from the MAVLink message, if available.
    pub vehicle_time: Option<VehicleTimestamp>,
    /// Wall-clock time at which the event loop received this message.
    pub received_at: Instant,
}

/// A raw MAVLink message value together with receipt timestamps.
///
/// Unlike [`MetricSample`], `MessageSample` does not carry a `source` field because
/// each `MessageHandle` is bound to exactly one MAVLink message type.
///
/// `vehicle_time` is `None` when the message's timestamp field was absent or zero.
#[derive(Clone, Debug, PartialEq)]
pub struct MessageSample<M> {
    pub value: M,
    /// Wall-clock time at which the event loop received this message.
    pub received_at: Instant,
    /// Vehicle boot-relative timestamp extracted from the MAVLink message, if available.
    pub vehicle_time: Option<VehicleTimestamp>,
}

/// A typed [`ObservationHandle`] for a coalesced telemetry metric, paired with a
/// support-state observation.
///
/// Use [`support()`] to check whether the connected vehicle is currently sending this
/// metric at all: a value of [`SupportState::Unsupported`] means the autopilot has not
/// produced the relevant MAVLink messages since connection, so [`wait()`] would block
/// indefinitely.
///
/// [`support()`]: MetricHandle::support
/// [`wait()`]: MetricHandle::wait
#[derive(Clone)]
pub struct MetricHandle<T: Clone + Send + Sync + 'static> {
    observation: ObservationHandle<MetricSample<T>>,
    support: ObservationHandle<SupportState>,
}

impl<T: Clone + Send + Sync + 'static> MetricHandle<T> {
    pub fn new(
        observation: ObservationHandle<MetricSample<T>>,
        support: ObservationHandle<SupportState>,
    ) -> Self {
        Self {
            observation,
            support,
        }
    }

    pub fn latest(&self) -> Option<MetricSample<T>> {
        self.observation.latest()
    }

    pub async fn wait(&self) -> Result<MetricSample<T>, VehicleError> {
        self.observation.wait().await
    }

    pub async fn wait_timeout(&self, timeout: Duration) -> Result<MetricSample<T>, VehicleError> {
        self.observation.wait_timeout(timeout).await
    }

    pub fn subscribe(&self) -> ObservationSubscription<MetricSample<T>> {
        self.observation.subscribe()
    }

    pub fn support(&self) -> ObservationHandle<SupportState> {
        self.support.clone()
    }
}

/// A typed [`ObservationHandle`] for a specific MAVLink message type, paired with a
/// support-state observation.
///
/// Unlike [`MetricHandle`], which coalesces values from multiple message sources,
/// `MessageHandle` is bound to a single MAVLink message type and preserves every
/// received event via broadcast backing. Use [`subscribe()`] when you need an
/// uncoalesced stream of individual messages.
///
/// [`support()`] indicates whether the vehicle has sent this message type since
/// connection. A value of [`SupportState::Unsupported`] means [`wait()`] would block
/// indefinitely.
///
/// [`support()`]: MessageHandle::support
/// [`wait()`]: MessageHandle::wait
/// [`subscribe()`]: MessageHandle::subscribe
#[derive(Clone)]
pub struct MessageHandle<M: Clone + Send + Sync + 'static> {
    observation: ObservationHandle<MessageSample<M>>,
    support: ObservationHandle<SupportState>,
}

impl<M: Clone + Send + Sync + 'static> MessageHandle<M> {
    pub fn new(
        observation: ObservationHandle<MessageSample<M>>,
        support: ObservationHandle<SupportState>,
    ) -> Self {
        Self {
            observation,
            support,
        }
    }

    pub fn latest(&self) -> Option<MessageSample<M>> {
        self.observation.latest()
    }

    pub async fn wait(&self) -> Result<MessageSample<M>, VehicleError> {
        self.observation.wait().await
    }

    pub async fn wait_timeout(&self, timeout: Duration) -> Result<MessageSample<M>, VehicleError> {
        self.observation.wait_timeout(timeout).await
    }

    pub fn subscribe(&self) -> ObservationSubscription<MessageSample<M>> {
        self.observation.subscribe()
    }

    pub fn support(&self) -> ObservationHandle<SupportState> {
        self.support.clone()
    }
}
