use crate::mission::RetryPolicy;
use std::time::Duration;

/// Configuration for a [`Vehicle`](crate::Vehicle) connection.
pub struct VehicleConfig {
    /// MAVLink system ID used by this GCS (default 255).
    pub gcs_system_id: u8,
    /// MAVLink component ID used by this GCS (default 190).
    pub gcs_component_id: u8,
    /// Retry policy for mission and parameter transfers.
    pub retry_policy: RetryPolicy,
    /// Automatically request home position after connecting (default `true`).
    pub auto_request_home: bool,
    /// Capacity of the internal command channel (default 32).
    pub command_buffer_size: usize,
    /// Maximum time to wait for the first heartbeat (default 30 s).
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
