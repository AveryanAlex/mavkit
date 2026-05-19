use std::time::Duration;

use tokio::sync::{mpsc, watch};

use crate::error::VehicleError;

use super::api::{DemoClock, DemoVehicleSnapshot};
use super::state::{ControlMessage, DemoVehicleConfig, SimulatorCore};
use super::transport::SimulatorEndpoints;

const MICROS_PER_SECOND: u64 = 1_000_000;
const MICROS_PER_MILLISECOND: u64 = 1_000;

pub(crate) async fn run_simulator(
    config: DemoVehicleConfig,
    mut endpoints: SimulatorEndpoints,
    mut control_rx: mpsc::Receiver<ControlMessage>,
    snapshot_tx: watch::Sender<DemoVehicleSnapshot>,
    initial_snapshot: DemoVehicleSnapshot,
) {
    let mut simulator = SimulatorCore::new(
        config.clone(),
        endpoints.to_sdk_tx,
        snapshot_tx,
        initial_snapshot,
    );
    let _ = simulator.emit_bootstrap().await;

    match config.clock {
        DemoClock::RealTime => {
            let tick_period = Duration::from_secs_f64(1.0 / f64::from(config.tick_hz));
            let mut interval = tokio::time::interval(tick_period);
            interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
            interval.tick().await;

            loop {
                tokio::select! {
                    maybe_message = endpoints.from_sdk_rx.recv() => {
                        match maybe_message {
                            Some((header, message)) => {
                                if simulator.handle_inbound_frame(header, message).await.is_err() {
                                    break;
                                }
                            }
                            None => break,
                        }
                    }
                    maybe_control = control_rx.recv() => {
                        match maybe_control {
                            Some(control) => {
                                if handle_control(&mut simulator, control).await {
                                    break;
                                }
                            }
                            None => break,
                        }
                    }
                    _ = interval.tick() => {
                        if simulator.advance_one_tick().await.is_err() {
                            break;
                        }
                    }
                }
            }
        }
        DemoClock::Manual => loop {
            tokio::select! {
                maybe_message = endpoints.from_sdk_rx.recv() => {
                    match maybe_message {
                        Some((header, message)) => {
                            if simulator.handle_inbound_frame(header, message).await.is_err() {
                                break;
                            }
                        }
                        None => break,
                    }
                }
                maybe_control = control_rx.recv() => {
                    match maybe_control {
                        Some(control) => {
                            if handle_control(&mut simulator, control).await {
                                break;
                            }
                        }
                        None => break,
                    }
                }
            }
        },
    }
}

async fn handle_control(simulator: &mut SimulatorCore, control: ControlMessage) -> bool {
    match control {
        ControlMessage::Step { reply } => {
            let result = simulator
                .advance_one_tick()
                .await
                .map(|_| simulator.snapshot.clone());
            let _ = reply.send(result);
            false
        }
        ControlMessage::Shutdown { reply } => {
            let _ = reply.send(Ok(()));
            true
        }
    }
}

impl SimulatorCore {
    pub(crate) async fn emit_bootstrap(&mut self) -> Result<(), VehicleError> {
        self.emit_heartbeat().await?;
        self.emit_current_mode().await?;
        self.emit_home_and_origin().await?;
        self.emit_telemetry_burst().await
    }

    pub(crate) async fn advance_one_tick(&mut self) -> Result<(), VehicleError> {
        let tick_elapsed_us = self.advance_boot_time();
        self.drive_mission_state().await?;
        self.advance_navigation();
        self.advance_power_model();
        self.snapshot.roll_rad = (self.snapshot.time_boot_ms as f32 / 3000.0).sin() * 0.03;
        self.snapshot.pitch_rad = (self.snapshot.time_boot_ms as f32 / 5000.0).cos() * 0.02;
        self.publish_snapshot();

        self.emit_heartbeat().await?;
        self.emit_default_telemetry_burst().await?;
        self.emit_configured_streams(tick_elapsed_us).await
    }

    fn advance_boot_time(&mut self) -> u64 {
        let tick_hz = u64::from(self.config.tick_hz);
        let tick_numerator = MICROS_PER_SECOND + self.tick_time_remainder;
        let tick_elapsed_us = tick_numerator / tick_hz;
        self.tick_time_remainder = tick_numerator % tick_hz;
        self.boot_time_us = self.boot_time_us.saturating_add(tick_elapsed_us);
        let boot_time_ms = (self.boot_time_us / MICROS_PER_MILLISECOND).min(u64::from(u32::MAX));
        self.snapshot.time_boot_ms = boot_time_ms as u32;
        tick_elapsed_us
    }

    async fn emit_configured_streams(&mut self, tick_elapsed_us: u64) -> Result<(), VehicleError> {
        let mut due_message_ids = Vec::new();

        for (message_id, schedule) in &mut self.stream_schedules {
            if !schedule.is_enabled() {
                continue;
            }

            schedule.elapsed_us = schedule.elapsed_us.saturating_add(tick_elapsed_us);
            let interval_us = schedule.interval_us as u64;
            if schedule.elapsed_us >= interval_us {
                schedule.elapsed_us %= interval_us;
                due_message_ids.push(*message_id);
            }
        }

        for message_id in due_message_ids {
            self.emit_message_by_id(message_id).await?;
        }
        Ok(())
    }
}
