use super::backing::MessageCommandHub;
use crate::VehicleError;
use crate::dialect::MavCmd;
use crate::observation::{
    MessageHandle, MessageSample, ObservationHandle, ObservationSubscription, SupportState,
};
use crate::runtime;
use crate::time::Instant;
use std::time::Duration;

async fn wait_for_fresh_sample<M: Clone + Send + Sync + 'static>(
    handle: &MessageHandle<M>,
    timeout: Duration,
    cutoff: Option<Instant>,
) -> Result<MessageSample<M>, VehicleError> {
    let mut subscription = handle.subscribe();
    let wait = async {
        loop {
            match subscription.recv().await {
                Some(sample) if cutoff.is_none_or(|at| sample.received_at > at) => {
                    return Ok(sample);
                }
                Some(_) => continue,
                None => return Err(VehicleError::Disconnected),
            }
        }
    };

    match runtime::timeout(timeout, wait).await {
        Ok(result) => result,
        Err(_) => Err(VehicleError::Timeout("message wait".into())),
    }
}

fn interval_us_from_hz(hz: f32) -> Result<f32, VehicleError> {
    if !hz.is_finite() || hz <= 0.0 {
        return Err(VehicleError::InvalidParameter(
            "message rate must be a finite positive hz value".to_string(),
        ));
    }

    Ok(1_000_000.0 / hz)
}

/// Message handle for periodic/requestable MAVLink message families.
#[derive(Clone)]
pub struct PeriodicMessageHandle<M: Clone + Send + Sync + 'static> {
    inner: MessageHandle<M>,
    commands: MessageCommandHub,
    message_id: u32,
    request_param2: f32,
}

impl<M: Clone + Send + Sync + 'static> PeriodicMessageHandle<M> {
    pub(super) fn new(
        inner: MessageHandle<M>,
        commands: MessageCommandHub,
        message_id: u32,
        request_param2: f32,
    ) -> Self {
        Self {
            inner,
            commands,
            message_id,
            request_param2,
        }
    }

    pub fn latest(&self) -> Option<MessageSample<M>> {
        self.inner.latest()
    }

    pub async fn wait(&self) -> Result<MessageSample<M>, VehicleError> {
        self.inner.wait().await
    }

    pub async fn wait_timeout(&self, timeout: Duration) -> Result<MessageSample<M>, VehicleError> {
        self.inner.wait_timeout(timeout).await
    }

    pub fn subscribe(&self) -> ObservationSubscription<MessageSample<M>> {
        self.inner.subscribe()
    }

    pub fn support(&self) -> ObservationHandle<SupportState> {
        self.inner.support()
    }

    pub async fn request(&self, timeout: Duration) -> Result<MessageSample<M>, VehicleError> {
        let cutoff = self.inner.latest().map(|sample| sample.received_at);
        let wait = wait_for_fresh_sample(&self.inner, timeout, cutoff);

        self.commands
            .send_raw_command_long(
                MavCmd::MAV_CMD_REQUEST_MESSAGE,
                [
                    self.message_id as f32,
                    self.request_param2,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
            )
            .await?;

        wait.await
    }

    pub async fn set_rate(&self, hz: f32) -> Result<(), VehicleError> {
        self.commands
            .send_raw_command_long(
                MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
                [
                    self.message_id as f32,
                    interval_us_from_hz(hz)?,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
            )
            .await
    }
}

/// Message handle for event-style MAVLink message families.
#[derive(Clone)]
pub struct EventMessageHandle<M: Clone + Send + Sync + 'static> {
    inner: MessageHandle<M>,
    commands: MessageCommandHub,
    message_id: u32,
    request_param2: f32,
}

impl<M: Clone + Send + Sync + 'static> EventMessageHandle<M> {
    pub(super) fn new(
        inner: MessageHandle<M>,
        commands: MessageCommandHub,
        message_id: u32,
        request_param2: f32,
    ) -> Self {
        Self {
            inner,
            commands,
            message_id,
            request_param2,
        }
    }

    pub fn latest(&self) -> Option<MessageSample<M>> {
        self.inner.latest()
    }

    pub async fn wait(&self) -> Result<MessageSample<M>, VehicleError> {
        self.inner.wait().await
    }

    pub async fn wait_timeout(&self, timeout: Duration) -> Result<MessageSample<M>, VehicleError> {
        self.inner.wait_timeout(timeout).await
    }

    pub fn subscribe(&self) -> ObservationSubscription<MessageSample<M>> {
        self.inner.subscribe()
    }

    pub fn support(&self) -> ObservationHandle<SupportState> {
        self.inner.support()
    }

    pub async fn request(&self, timeout: Duration) -> Result<MessageSample<M>, VehicleError> {
        let cutoff = self.inner.latest().map(|sample| sample.received_at);
        let wait = wait_for_fresh_sample(&self.inner, timeout, cutoff);

        self.commands
            .send_raw_command_long(
                MavCmd::MAV_CMD_REQUEST_MESSAGE,
                [
                    self.message_id as f32,
                    self.request_param2,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
            )
            .await?;

        wait.await
    }
}
