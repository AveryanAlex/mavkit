use crate::mission::RetryPolicy;
use std::time::Duration;

/// Startup policy for one initialization domain.
///
/// Controls whether MAVKit probes for a particular piece of optional information at connect time,
/// and how hard it tries. Setting `enabled = false` skips the probe entirely; useful in
/// environments where the vehicle does not support a given capability (e.g. no available-modes
/// support on older firmwares).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct InitDomainPolicy {
    /// When `false` the domain is skipped at connect time. Default: `true`.
    pub enabled: bool,
    /// Maximum number of request attempts before giving up. Must be ≥ 1 when `enabled`.
    pub max_attempts: u8,
    /// Total wall-clock budget for all attempts across this domain. Attempts are distributed
    /// evenly within the budget.
    pub budget: Duration,
}

/// Per-domain initialization policy applied during the connect bootstrap phase.
///
/// Tune individual domains to trade off connect latency against completeness of the initial
/// vehicle snapshot. Disabling a domain means the corresponding state will remain `None` until
/// the vehicle broadcasts it spontaneously.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct InitPolicyConfig {
    /// Policy for fetching `AUTOPILOT_VERSION` (firmware/capabilities). Default: 3 attempts, 5 s.
    pub autopilot_version: InitDomainPolicy,
    /// Policy for fetching `AVAILABLE_MODES` (named flight-mode catalog). Default: 2 attempts, 5 s.
    pub available_modes: InitDomainPolicy,
    /// Policy for fetching the home position. Default: 1 attempt, 5 s.
    pub home: InitDomainPolicy,
    /// Policy for fetching the GPS global origin. Default: 1 attempt, 5 s.
    pub origin: InitDomainPolicy,
}

/// Configuration for a [`Vehicle`](crate::Vehicle) connection.
///
/// Construct via [`VehicleConfig::default()`] and override individual fields. Passed to
/// [`Vehicle::connect_with_config`](crate::Vehicle::connect_with_config) or
/// [`Vehicle::from_connection`](crate::Vehicle::from_connection).
///
/// All timeouts are measured from the moment a request is sent; there is no global session
/// timeout.
#[derive(Clone, Debug)]
pub struct VehicleConfig {
    /// How long to wait for the first heartbeat before failing the connection. Default: 10 s.
    ///
    /// Increase on high-latency links; decrease for fast-fail in automated tests.
    pub connect_timeout: Duration,

    /// How long to wait for a `COMMAND_ACK` after sending a single command. Default: 5 s.
    ///
    /// This covers the round-trip for one MAVLink command-long/int exchange. Does not apply to
    /// multi-step operations such as mode confirmation (see `command_completion_timeout`).
    pub command_timeout: Duration,

    /// How long to wait for the vehicle to report a state change after the command ACK. Default: 10 s.
    ///
    /// Used by operations that send a command and then watch telemetry for confirmation, e.g.
    /// `set_mode` waiting for the mode to appear in heartbeats.
    pub command_completion_timeout: Duration,

    /// How long a multi-message transfer (mission, params, fence, rally) may take in total. Default: 30 s.
    ///
    /// Covers the entire exchange from first item request to final ACK. Increase for large
    /// missions or high packet-loss links.
    pub transfer_timeout: Duration,

    /// Initialization policies applied once after the first heartbeat is received.
    pub init_policy: InitPolicyConfig,

    /// MAVLink system ID used by this GCS in outgoing messages. Default: 255.
    ///
    /// The standard GCS system ID. Change only if multiple GCS instances share a link and need
    /// distinct IDs.
    pub gcs_system_id: u8,

    /// MAVLink component ID used by this GCS in outgoing messages. Default: 190.
    pub gcs_component_id: u8,

    /// Retry policy applied to individual packet exchanges within transfers. Default: see [`RetryPolicy::default`].
    pub retry_policy: RetryPolicy,

    /// When `true`, request the home position immediately after connecting. Default: `true`.
    ///
    /// Set to `false` to reduce connect-time traffic on links with limited bandwidth, or in tests
    /// where the home position is irrelevant.
    pub auto_request_home: bool,

    /// Capacity of the internal async command channel (in commands). Default: 32.
    ///
    /// Must be at least 1. A larger buffer allows more commands to be enqueued before back-pressure
    /// is applied to callers. Increase if many concurrent callers saturate the channel.
    pub command_buffer_size: usize,
}

impl Default for InitPolicyConfig {
    fn default() -> Self {
        Self {
            autopilot_version: InitDomainPolicy {
                enabled: true,
                max_attempts: 3,
                budget: Duration::from_secs(5),
            },
            available_modes: InitDomainPolicy {
                enabled: true,
                max_attempts: 2,
                budget: Duration::from_secs(5),
            },
            home: InitDomainPolicy {
                enabled: true,
                max_attempts: 1,
                budget: Duration::from_secs(5),
            },
            origin: InitDomainPolicy {
                enabled: true,
                max_attempts: 1,
                budget: Duration::from_secs(5),
            },
        }
    }
}

impl Default for VehicleConfig {
    fn default() -> Self {
        Self {
            connect_timeout: Duration::from_secs(10),
            command_timeout: Duration::from_secs(5),
            command_completion_timeout: Duration::from_secs(10),
            transfer_timeout: Duration::from_secs(30),
            init_policy: InitPolicyConfig::default(),
            gcs_system_id: 255,
            gcs_component_id: 190,
            retry_policy: RetryPolicy::default(),
            auto_request_home: true,
            command_buffer_size: 32,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{InitDomainPolicy, InitPolicyConfig, VehicleConfig};
    use std::time::Duration;

    #[test]
    fn defaults_use_connect_time_config_and_init_policy() {
        let cfg = VehicleConfig::default();

        assert_eq!(cfg.connect_timeout, Duration::from_secs(10));
        assert_eq!(cfg.command_timeout, Duration::from_secs(5));
        assert_eq!(cfg.command_completion_timeout, Duration::from_secs(10));
        assert_eq!(cfg.transfer_timeout, Duration::from_secs(30));

        assert_eq!(cfg.init_policy.autopilot_version.max_attempts, 3);
        assert_eq!(cfg.init_policy.available_modes.max_attempts, 2);
        assert!(cfg.init_policy.home.enabled);
        assert!(cfg.init_policy.origin.enabled);
    }

    #[test]
    fn init_policy_can_override_per_domain() {
        let policy = InitPolicyConfig {
            autopilot_version: InitDomainPolicy {
                enabled: false,
                max_attempts: 0,
                budget: Duration::from_secs(1),
            },
            ..InitPolicyConfig::default()
        };

        assert!(!policy.autopilot_version.enabled);
        assert_eq!(policy.autopilot_version.max_attempts, 0);
    }
}
