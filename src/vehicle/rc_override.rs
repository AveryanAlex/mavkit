use crate::error::VehicleError;

const RC_OVERRIDE_CHANNEL_COUNT: usize = 18;
const RC_OVERRIDE_IGNORE_RAW: u16 = u16::MAX;
const RC_OVERRIDE_RELEASE_RAW: u16 = 0;

/// A single RC override slot inside [`RcOverride`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RcOverrideChannelValue {
    /// Leave this channel untouched by encoding MAVLink's ignore sentinel (`65535`).
    Ignore,
    /// Hand control back to the vehicle by encoding MAVLink's release sentinel (`0`).
    Release,
    /// Override the channel with an explicit raw PWM/microsecond value.
    Pwm(u16),
}

impl RcOverrideChannelValue {
    /// Construct a PWM override value, rejecting the two sentinel values.
    ///
    /// Returns `Err` for `0` (reserved for [`Release`](Self::Release)) and `65535` (reserved for
    /// [`Ignore`](Self::Ignore)). All other values in the standard RC PWM range (typically
    /// 1000–2000 µs) are accepted.
    pub fn pwm(pwm_us: u16) -> Result<Self, VehicleError> {
        match pwm_us {
            RC_OVERRIDE_RELEASE_RAW => Err(VehicleError::InvalidParameter(
                "rc override pwm 0 is reserved for release; use RcOverrideChannelValue::Release or RcOverride::release()".into(),
            )),
            RC_OVERRIDE_IGNORE_RAW => Err(VehicleError::InvalidParameter(
                "rc override pwm 65535 is reserved for ignore; use RcOverrideChannelValue::Ignore or RcOverride::ignore()".into(),
            )),
            _ => Ok(Self::Pwm(pwm_us)),
        }
    }

    pub(super) fn to_wire(self) -> u16 {
        match self {
            Self::Ignore => RC_OVERRIDE_IGNORE_RAW,
            Self::Release => RC_OVERRIDE_RELEASE_RAW,
            Self::Pwm(pwm_us) => pwm_us,
        }
    }
}

/// Typed builder for one `RC_CHANNELS_OVERRIDE` frame.
///
/// Channels default to MAVLink's ignore sentinel (`65535`). Use [`RcOverride::release`] to
/// encode an explicit release (`0`) for a channel, or [`RcOverride::set_pwm`] for a concrete raw
/// override. MAVLink treats RC overrides as transient inputs, so callers must keep resending the
/// returned frame shape at their required control cadence; mavkit does not own a refresh loop.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RcOverride {
    channels: [RcOverrideChannelValue; RC_OVERRIDE_CHANNEL_COUNT],
}

impl Default for RcOverride {
    fn default() -> Self {
        Self {
            channels: [RcOverrideChannelValue::Ignore; RC_OVERRIDE_CHANNEL_COUNT],
        }
    }
}

impl RcOverride {
    /// Create a new `RcOverride` with all channels set to [`Ignore`](RcOverrideChannelValue::Ignore).
    pub fn new() -> Self {
        Self::default()
    }

    /// Read the current value for a 1-based channel number (1–18).
    ///
    /// Returns `Err(InvalidParameter)` for channel numbers outside 1–18.
    pub fn channel(&self, channel: u8) -> Result<RcOverrideChannelValue, VehicleError> {
        Ok(self.channels[rc_override_channel_index(channel)?])
    }

    /// Set a channel to an arbitrary [`RcOverrideChannelValue`].
    ///
    /// Returns `Err(InvalidParameter)` for channel numbers outside 1–18. Returns `&mut self` for
    /// method chaining.
    pub fn set(
        &mut self,
        channel: u8,
        value: RcOverrideChannelValue,
    ) -> Result<&mut Self, VehicleError> {
        self.channels[rc_override_channel_index(channel)?] = value;
        Ok(self)
    }

    /// Override a channel with an explicit PWM value in microseconds.
    ///
    /// Typical RC PWM values are 1000–2000 µs (centre ≈ 1500 µs). Returns `Err` for channel
    /// numbers outside 1–18 or for the sentinel values 0 and 65535.
    pub fn set_pwm(&mut self, channel: u8, pwm_us: u16) -> Result<&mut Self, VehicleError> {
        self.set(channel, RcOverrideChannelValue::pwm(pwm_us)?)
    }

    /// Release a channel back to the RC receiver by setting the wire value to `0`.
    ///
    /// Returns `Err(InvalidParameter)` for channel numbers outside 1–18.
    pub fn release(&mut self, channel: u8) -> Result<&mut Self, VehicleError> {
        self.set(channel, RcOverrideChannelValue::Release)
    }

    /// Leave a channel untouched by setting the wire value to the ignore sentinel (`65535`).
    ///
    /// Returns `Err(InvalidParameter)` for channel numbers outside 1–18.
    pub fn ignore(&mut self, channel: u8) -> Result<&mut Self, VehicleError> {
        self.set(channel, RcOverrideChannelValue::Ignore)
    }

    pub(super) fn to_wire_channels(self) -> [u16; RC_OVERRIDE_CHANNEL_COUNT] {
        self.channels.map(RcOverrideChannelValue::to_wire)
    }
}

pub(super) fn rc_override_channel_index(channel: u8) -> Result<usize, VehicleError> {
    if (1..=RC_OVERRIDE_CHANNEL_COUNT as u8).contains(&channel) {
        Ok((channel - 1) as usize)
    } else {
        Err(VehicleError::InvalidParameter(format!(
            "rc override channel must be 1..={}, got {channel}",
            RC_OVERRIDE_CHANNEL_COUNT
        )))
    }
}
