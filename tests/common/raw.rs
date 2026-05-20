use crate::common::backend::{disconnect, setup_backend_vehicle};
use crate::common::target::TestTarget;
use std::time::Duration;
use tokio_stream::StreamExt;

pub async fn raw_subscribe_receives_heartbeats_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;
    let result: Result<(), String> = async {
        let stream = vehicle.raw().subscribe_filtered(0);
        tokio::pin!(stream);

        let deadline = tokio::time::sleep(Duration::from_secs(5));
        tokio::pin!(deadline);

        tokio::select! {
            _ = &mut deadline => {
                return Err(String::from("timed out waiting for raw heartbeat"));
            }
            msg = stream.next() => {
                let msg = msg.ok_or_else(|| String::from("raw subscription stream closed"))?;
                if msg.message_id != 0 {
                    return Err(format!("expected message_id 0, got {}", msg.message_id));
                }
            }
        }

        Ok(())
    }
    .await;

    disconnect(backend).await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

pub async fn raw_request_message_autopilot_version_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;
    let result: Result<(), String> = async {
        let msg = vehicle
            .raw()
            .request_message(148, Duration::from_secs(5))
            .await
            .map_err(|e| e.to_string())?;

        if msg.message_id != 148 {
            return Err(format!("expected message_id 148, got {}", msg.message_id));
        }

        Ok(())
    }
    .await;

    disconnect(backend).await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

pub async fn raw_subscribe_unfiltered_receives_multiple_types_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;
    let result: Result<(), String> = async {
        let stream = vehicle.raw().subscribe();
        tokio::pin!(stream);
        let mut seen_ids = std::collections::HashSet::new();
        let deadline = tokio::time::sleep(Duration::from_secs(5));
        tokio::pin!(deadline);

        loop {
            tokio::select! {
                _ = &mut deadline => break,
                msg = stream.next() => {
                    let msg = msg.ok_or_else(|| String::from("raw subscription stream closed"))?;
                    seen_ids.insert(msg.message_id);
                    if seen_ids.len() >= 5 {
                        break;
                    }
                }
            }
        }

        if seen_ids.len() < 3 {
            return Err(format!(
                "expected at least 3 distinct message types, got {}",
                seen_ids.len()
            ));
        }

        Ok(())
    }
    .await;

    disconnect(backend).await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

#[cfg(feature = "sim")]
#[allow(dead_code)]
pub async fn raw_subscribe_receives_power_and_output_messages_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let vehicle = &backend.vehicle;
    let result: Result<(), String> = async {
        let expectations = target
            .raw_power_output_expectations()
            .ok_or_else(|| format!("no raw power/output expectations for {target:?}"))?;
        let stream = vehicle.raw().subscribe();
        tokio::pin!(stream);
        let mut seen_ids = std::collections::HashSet::new();
        let deadline = tokio::time::sleep(Duration::from_secs(5));
        tokio::pin!(deadline);

        loop {
            tokio::select! {
                _ = &mut deadline => break,
                msg = stream.next() => {
                    let msg = msg.ok_or_else(|| String::from("raw subscription stream closed"))?;
                    seen_ids.insert(msg.message_id);
                    if expectations
                        .required_message_ids
                        .iter()
                        .all(|expected_id| seen_ids.contains(expected_id))
                    {
                        break;
                    }
                }
            }
        }

        for expected_id in expectations.required_message_ids {
            if !seen_ids.contains(expected_id) {
                return Err(format!(
                    "expected raw message_id {expected_id}, saw {seen_ids:?}"
                ));
            }
        }

        Ok(())
    }
    .await;

    disconnect(backend).await;
    if let Err(err) = result {
        panic!("{err}");
    }
}
