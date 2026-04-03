use crate::command::{Command, RawCommandIntPayload};
use crate::dialect;
use crate::error::VehicleError;
use crate::geo::{GeoPoint3dMsl, try_latitude_e7, try_longitude_e7};
use crate::modes::mode_number;
use crate::observation::{MetricHandle, MetricSample, ObservationSubscription};
use crate::vehicle::rc_override::RcOverride;
use crate::vehicle::{Vehicle, VehicleInner};
use std::time::Instant;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct WireNormalizedGeo {
    latitude_e7: i32,
    longitude_e7: i32,
    altitude_mm: i32,
}

impl TryFrom<&GeoPoint3dMsl> for WireNormalizedGeo {
    type Error = VehicleError;

    fn try_from(point: &GeoPoint3dMsl) -> Result<Self, Self::Error> {
        Ok(Self {
            latitude_e7: try_latitude_e7(point.latitude_deg)?,
            longitude_e7: try_longitude_e7(point.longitude_deg)?,
            altitude_mm: quantize_meters_mm(point.altitude_msl_m)?,
        })
    }
}

impl WireNormalizedGeo {
    fn matches(self, observed: &GeoPoint3dMsl) -> bool {
        // observed comes from wire-decoded data — valid by construction.
        self.latitude_e7 == crate::geo::quantize_degrees_e7(observed.latitude_deg)
            && self.longitude_e7 == crate::geo::quantize_degrees_e7(observed.longitude_deg)
            && quantize_meters_mm(observed.altitude_msl_m).is_ok_and(|mm| self.altitude_mm == mm)
    }
}

pub(crate) fn quantize_meters_mm(value: f64) -> Result<i32, VehicleError> {
    if !value.is_finite() {
        return Err(VehicleError::InvalidParameter(format!(
            "altitude must be finite, got {value}"
        )));
    }
    let scaled = (value * 1000.0).round();
    if !(i32::MIN as f64..=i32::MAX as f64).contains(&scaled) {
        return Err(VehicleError::InvalidParameter(format!(
            "altitude {value} m overflows i32 millimeter range"
        )));
    }
    Ok(scaled as i32)
}

impl VehicleInner {
    pub(crate) async fn set_mode(
        &self,
        custom_mode: u32,
        wait_for_observation: bool,
    ) -> Result<(), VehicleError> {
        crate::mission::send_domain_command(self.command_tx.clone(), |reply| Command::SetMode {
            custom_mode,
            reply,
        })
        .await?;

        if wait_for_observation {
            self.wait_for_mode_observation(custom_mode).await?;
        }

        Ok(())
    }

    pub(crate) async fn set_mode_by_name(
        &self,
        name: &str,
        wait_for_observation: bool,
    ) -> Result<(), VehicleError> {
        let custom_mode = self
            .resolve_mode_by_name(name)
            .ok_or_else(|| VehicleError::ModeNotAvailable(name.to_string()))?;

        self.set_mode(custom_mode, wait_for_observation).await
    }

    pub(crate) fn resolve_mode_by_name(&self, name: &str) -> Option<u32> {
        let requested = name.trim();
        if requested.is_empty() {
            return None;
        }

        let catalog = self.modes.catalog().latest().unwrap_or_default();
        if let Some(custom_mode) = catalog.into_iter().find_map(|mode| {
            mode.name
                .eq_ignore_ascii_case(requested)
                .then_some(mode.custom_mode)
        }) {
            return Some(custom_mode);
        }

        let identity = self.stores.vehicle_state.borrow().clone();
        mode_number(identity.autopilot, identity.vehicle_type, requested)
    }

    pub(crate) async fn wait_for_mode_observation(
        &self,
        custom_mode: u32,
    ) -> Result<(), VehicleError> {
        let current_mode = self.modes.current();
        if current_mode
            .latest()
            .is_some_and(|mode| mode.custom_mode == custom_mode)
        {
            return Ok(());
        }

        let mut subscription = current_mode.subscribe();
        let wait_for_match = async {
            while let Some(observed) = subscription.recv().await {
                if observed.custom_mode == custom_mode {
                    return Ok(());
                }
            }

            Err(VehicleError::Disconnected)
        };

        tokio::time::timeout(self._config.command_completion_timeout, wait_for_match)
            .await
            .map_err(|_| VehicleError::Timeout("waiting for mode change".into()))?
    }
}

impl Vehicle {
    /// Send one `RC_CHANNELS_OVERRIDE` frame to the connected vehicle.
    ///
    /// Callers must keep resending overrides at their required control rate; mavkit does not own
    /// a continuous-send loop and does not wait for any ACK.
    pub async fn rc_override(&self, overrides: RcOverride) -> Result<(), VehicleError> {
        let target = self.identity();
        let [
            chan1_raw,
            chan2_raw,
            chan3_raw,
            chan4_raw,
            chan5_raw,
            chan6_raw,
            chan7_raw,
            chan8_raw,
            chan9_raw,
            chan10_raw,
            chan11_raw,
            chan12_raw,
            chan13_raw,
            chan14_raw,
            chan15_raw,
            chan16_raw,
            chan17_raw,
            chan18_raw,
        ] = overrides.to_wire_channels();

        self.send_command(|reply| Command::RawSend {
            message: Box::new(dialect::MavMessage::RC_CHANNELS_OVERRIDE(
                dialect::RC_CHANNELS_OVERRIDE_DATA {
                    target_system: target.system_id,
                    target_component: target.component_id,
                    chan1_raw,
                    chan2_raw,
                    chan3_raw,
                    chan4_raw,
                    chan5_raw,
                    chan6_raw,
                    chan7_raw,
                    chan8_raw,
                    chan9_raw,
                    chan10_raw,
                    chan11_raw,
                    chan12_raw,
                    chan13_raw,
                    chan14_raw,
                    chan15_raw,
                    chan16_raw,
                    chan17_raw,
                    chan18_raw,
                },
            )),
            reply,
        })
        .await
    }

    /// Send the arm command and wait for acknowledgment.
    ///
    /// In future versions this may also confirm the armed state via telemetry.
    pub async fn arm(&self) -> Result<(), VehicleError> {
        self.send_command(|reply| Command::Arm {
            force: false,
            reply,
        })
        .await
    }

    /// Send the arm command and wait for acknowledgment (identical to [`arm`](Self::arm) today;
    /// exists for API clarity and future no-wait semantics).
    ///
    /// In future versions this may return immediately after the ACK without waiting for telemetry
    /// confirmation.
    pub async fn arm_no_wait(&self) -> Result<(), VehicleError> {
        self.send_command(|reply| Command::Arm {
            force: false,
            reply,
        })
        .await
    }

    /// Send the arm command with `force=true` and wait for acknowledgment.
    ///
    /// Forces arming even when pre-arm checks fail. Use with care.
    pub async fn force_arm(&self) -> Result<(), VehicleError> {
        self.send_command(|reply| Command::Arm { force: true, reply })
            .await
    }

    /// Send the arm command with `force=true` and wait for acknowledgment (identical to
    /// [`force_arm`](Self::force_arm) today; exists for API clarity and future no-wait semantics).
    pub async fn force_arm_no_wait(&self) -> Result<(), VehicleError> {
        self.send_command(|reply| Command::Arm { force: true, reply })
            .await
    }

    /// Send the disarm command and wait for acknowledgment.
    pub async fn disarm(&self) -> Result<(), VehicleError> {
        self.send_command(|reply| Command::Disarm {
            force: false,
            reply,
        })
        .await
    }

    /// Send the disarm command and wait for acknowledgment (identical to
    /// [`disarm`](Self::disarm) today; exists for API clarity and future no-wait semantics).
    pub async fn disarm_no_wait(&self) -> Result<(), VehicleError> {
        self.send_command(|reply| Command::Disarm {
            force: false,
            reply,
        })
        .await
    }

    /// Send the disarm command with `force=true` and wait for acknowledgment.
    ///
    /// Forces disarming even when the vehicle is in-flight. Use with care.
    pub async fn force_disarm(&self) -> Result<(), VehicleError> {
        self.send_command(|reply| Command::Disarm { force: true, reply })
            .await
    }

    /// Send the disarm command with `force=true` and wait for acknowledgment (identical to
    /// [`force_disarm`](Self::force_disarm) today; exists for API clarity and future no-wait
    /// semantics).
    pub async fn force_disarm_no_wait(&self) -> Result<(), VehicleError> {
        self.send_command(|reply| Command::Disarm { force: true, reply })
            .await
    }

    /// Set the flight mode and wait for the mode change to be observed in telemetry.
    pub async fn set_mode(&self, custom_mode: u32) -> Result<(), VehicleError> {
        self.inner.set_mode(custom_mode, true).await
    }

    /// Set the flight mode and return immediately after the command ACK, without waiting for
    /// telemetry confirmation.
    pub async fn set_mode_no_wait(&self, custom_mode: u32) -> Result<(), VehicleError> {
        self.inner.set_mode(custom_mode, false).await
    }

    /// Set the flight mode by name and wait for the mode change to be observed in telemetry.
    pub async fn set_mode_by_name(&self, name: &str) -> Result<(), VehicleError> {
        self.inner.set_mode_by_name(name, true).await
    }

    /// Set the flight mode by name and return immediately after the command ACK, without waiting
    /// for telemetry confirmation.
    pub async fn set_mode_by_name_no_wait(&self, name: &str) -> Result<(), VehicleError> {
        self.inner.set_mode_by_name(name, false).await
    }

    /// Set the vehicle's home position to a specific global location.
    ///
    /// The position is validated and quantized to the MAVLink wire format (degE7 / mm) before
    /// sending. Returns [`VehicleError::InvalidParameter`] if the coordinates are out of range or
    /// non-finite.
    pub async fn set_home(&self, position: GeoPoint3dMsl) -> Result<(), VehicleError> {
        let lat_e7 = try_latitude_e7(position.latitude_deg)?;
        let lon_e7 = try_longitude_e7(position.longitude_deg)?;
        self.send_command(|reply| Command::RawCommandInt {
            payload: RawCommandIntPayload {
                command: dialect::MavCmd::MAV_CMD_DO_SET_HOME,
                frame: dialect::MavFrame::MAV_FRAME_GLOBAL,
                current: 0,
                autocontinue: 0,
                params: [0.0, 0.0, 0.0, 0.0],
                x: lat_e7,
                y: lon_e7,
                // Altitude narrowed to f32 at wire boundary (MAVLink z field).
                z: position.altitude_msl_m as f32,
            },
            reply,
        })
        .await
        .map(|_| ())
    }

    /// Set the vehicle's home position to its current GPS location.
    ///
    /// Equivalent to sending `MAV_CMD_DO_SET_HOME` with `param1=1`. The deprecated suppression is
    /// needed because the MAVLink crate marked this variant but the wire protocol still requires it.
    // MAVLink crate deprecated this type/variant, but the wire protocol still requires it.
    #[allow(deprecated)]
    pub async fn set_home_current(&self) -> Result<(), VehicleError> {
        self.send_command(|reply| Command::Long {
            command: dialect::MavCmd::MAV_CMD_DO_SET_HOME,
            params: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            reply,
        })
        .await
    }

    /// Set the GPS global origin used by the vehicle's local EKF frame.
    ///
    /// Sends `SET_GPS_GLOBAL_ORIGIN` and waits for the vehicle to echo back the matching
    /// `GPS_GLOBAL_ORIGIN` message as confirmation. Returns [`VehicleError::Timeout`] if the echo
    /// does not arrive within `command_completion_timeout`.
    pub async fn set_origin(&self, origin: GeoPoint3dMsl) -> Result<(), VehicleError> {
        let normalized = WireNormalizedGeo::try_from(&origin)?;
        let origin_handle = self.telemetry().origin();
        let mut subscription = origin_handle.subscribe();

        let sent_after = self
            .send_command(|reply| Command::SetOrigin {
                latitude: normalized.latitude_e7,
                longitude: normalized.longitude_e7,
                altitude: normalized.altitude_mm,
                reply,
            })
            .await?;

        self.wait_for_origin_observation(origin_handle, &mut subscription, normalized, sent_after)
            .await
    }

    async fn wait_for_origin_observation(
        &self,
        origin_handle: MetricHandle<GeoPoint3dMsl>,
        subscription: &mut ObservationSubscription<MetricSample<GeoPoint3dMsl>>,
        requested: WireNormalizedGeo,
        sent_after: Instant,
    ) -> Result<(), VehicleError> {
        if origin_handle.latest().is_some_and(|sample| {
            sample.received_at >= sent_after && requested.matches(&sample.value)
        }) {
            return Ok(());
        }

        let wait_for_match = async {
            while let Some(observed) = subscription.recv().await {
                if observed.received_at >= sent_after && requested.matches(&observed.value) {
                    return Ok(());
                }
            }

            Err(VehicleError::Disconnected)
        };

        tokio::time::timeout(
            self.inner._config.command_completion_timeout,
            wait_for_match,
        )
        .await
        .map_err(|_| VehicleError::Timeout("sending command".into()))?
    }
}
