use crate::mission::RetryPolicy;
use std::time::Duration;

pub struct VehicleConfig {
    pub gcs_system_id: u8,
    pub gcs_component_id: u8,
    pub retry_policy: RetryPolicy,
    pub auto_request_home: bool,
    pub command_buffer_size: usize,
    pub connect_timeout: Duration,
}

impl Default for VehicleConfig {
    fn default() -> Self {
        Self {
            gcs_system_id: 255,
            gcs_component_id: 190,
            retry_policy: RetryPolicy::default(),
            auto_request_home: true,
            command_buffer_size: 32,
            connect_timeout: Duration::from_secs(30),
        }
    }
}
