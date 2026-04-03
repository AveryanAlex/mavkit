use crate::command::Command;
use crate::config::VehicleConfig;
use crate::dialect;
use crate::error::VehicleError;
use crate::event_loop::{InitManager, run_event_loop_with_init};
use crate::state::{LinkState, create_channels};
#[cfg(feature = "stream")]
use crate::stream::StreamConnection;
use crate::vehicle::{Vehicle, VehicleInner};
use std::sync::Arc;
#[cfg(feature = "stream")]
use tokio::io::{AsyncRead, AsyncWrite};
use tokio::sync::mpsc;
use tokio_util::sync::CancellationToken;

impl Vehicle {
    /// Connect using a MAVLink URL with the default [`VehicleConfig`].
    ///
    /// Supported URL schemes depend on enabled Cargo features:
    /// - `udpin:<bind_addr>` — listen for incoming UDP packets (`udp` feature)
    /// - `tcpin:<addr>` — outbound TCP connection (`tcp` feature)
    /// - `serial:<port>:<baud>` — serial port (`serial` feature)
    ///
    /// Blocks until the first heartbeat is received or `connect_timeout` elapses.
    /// Returns [`VehicleError::Timeout`] on timeout, [`VehicleError::ConnectionFailed`] on
    /// transport errors.
    pub async fn connect(address: &str) -> Result<Self, VehicleError> {
        Self::connect_with_config(address, VehicleConfig::default()).await
    }

    /// Constructs a vehicle from async stream halves for custom byte-stream transports.
    ///
    /// This is a `stream`-feature convenience wrapper around
    /// [`StreamConnection`] and [`Vehicle::from_connection`]. For callback-driven
    /// transports, [`crate::stream::ChannelBridge`] provides channel-backed stream
    /// halves.
    #[cfg(feature = "stream")]
    pub async fn from_stream_parts<R, W>(
        reader: R,
        writer: W,
        config: VehicleConfig,
    ) -> Result<Self, VehicleError>
    where
        R: AsyncRead + Unpin + Send + 'static,
        W: AsyncWrite + Unpin + Send + 'static,
    {
        Self::from_connection(Box::new(StreamConnection::new(reader, writer)), config).await
    }

    /// Connect by listening on a UDP socket for an incoming MAVLink stream (`udp` feature).
    pub async fn connect_udp(bind_addr: &str) -> Result<Self, VehicleError> {
        Self::connect(&format!("udpin:{bind_addr}")).await
    }

    /// Connect via an outbound TCP connection (`tcp` feature).
    pub async fn connect_tcp(addr: &str) -> Result<Self, VehicleError> {
        Self::connect(&format!("tcpin:{addr}")).await
    }

    /// Connect via a serial port (`serial` feature).
    pub async fn connect_serial(port: &str, baud: u32) -> Result<Self, VehicleError> {
        Self::connect(&format!("serial:{port}:{baud}")).await
    }

    /// Connect using a MAVLink URL with a custom [`VehicleConfig`].
    ///
    /// See [`Vehicle::connect`] for supported URL schemes. Use this when you need to adjust
    /// timeouts, GCS IDs, or initialization policies.
    pub async fn connect_with_config(
        address: &str,
        config: VehicleConfig,
    ) -> Result<Self, VehicleError> {
        let connection = mavlink::connect_async::<dialect::MavMessage>(address)
            .await
            .map_err(|err| VehicleError::ConnectionFailed(err.to_string()))?;

        Self::from_connection(connection, config).await
    }

    /// Attach to an already-opened [`mavlink::AsyncMavConnection`] with a custom config.
    ///
    /// Use this to inject custom transports (e.g. in-process mock connections for testing) without
    /// going through the URL-based `connect` path. The connection must not have been used for any
    /// MAVLink traffic before this call — the event loop starts fresh from the first message.
    pub async fn from_connection(
        connection: Box<dyn mavlink::AsyncMavConnection<dialect::MavMessage> + Sync + Send>,
        config: VehicleConfig,
    ) -> Result<Self, VehicleError> {
        if config.command_buffer_size == 0 {
            return Err(VehicleError::InvalidParameter(
                "command_buffer_size must be at least 1".to_string(),
            ));
        }

        let (writers, stores) = create_channels();
        let cancel = CancellationToken::new();
        let (command_tx, command_rx) = mpsc::channel(config.command_buffer_size);
        let connect_timeout = config.connect_timeout;
        let init_manager = InitManager::new(config.clone());

        let inner = VehicleInner::new(command_tx, cancel, stores, config);
        inner.start_background_domains(&init_manager);

        tokio::spawn(run_event_loop_with_init(
            connection,
            command_rx,
            writers,
            inner._config.clone(),
            init_manager,
            inner.cancel.clone(),
        ));

        let vehicle = Self {
            inner: Arc::new(inner),
        };

        wait_for_first_heartbeat(&vehicle, connect_timeout).await?;
        Ok(vehicle)
    }

    /// Gracefully shut down the event loop and wait for the link to reach `Disconnected`.
    ///
    /// Safe to call from any clone. If the vehicle is already disconnected or in an error state
    /// this is a no-op that returns `Ok(())`. After this returns, all other operations on any
    /// remaining clones will return [`VehicleError::Disconnected`].
    pub async fn disconnect(&self) -> Result<(), VehicleError> {
        let mut link_state = self.inner.stores.link_state.clone();

        if matches!(
            link_state.borrow().clone(),
            LinkState::Disconnected | LinkState::Error(_)
        ) {
            return Ok(());
        }

        self.inner
            .command_tx
            .send(Command::Shutdown)
            .await
            .map_err(|_| VehicleError::Disconnected)?;

        loop {
            let current = link_state.borrow().clone();
            match current {
                LinkState::Disconnected | LinkState::Error(_) => return Ok(()),
                LinkState::Connecting | LinkState::Connected => {
                    link_state
                        .changed()
                        .await
                        .map_err(|_| VehicleError::Disconnected)?;
                }
            }
        }
    }
}

async fn wait_for_first_heartbeat(
    vehicle: &Vehicle,
    connect_timeout: std::time::Duration,
) -> Result<(), VehicleError> {
    let mut vehicle_state = vehicle.inner.stores.vehicle_state.clone();
    let mut link_state = vehicle.inner.stores.link_state.clone();
    let ready_wait = async {
        loop {
            match link_state.borrow().clone() {
                LinkState::Connecting | LinkState::Connected => {}
                LinkState::Disconnected => return Err(VehicleError::Disconnected),
                LinkState::Error(message) => {
                    return Err(VehicleError::ConnectionFailed(message));
                }
            }

            if vehicle_state.borrow().heartbeat_received {
                return Ok::<(), VehicleError>(());
            }

            tokio::select! {
                changed = vehicle_state.changed() => {
                    if changed.is_err() {
                        match link_state.borrow().clone() {
                            LinkState::Error(message) => {
                                return Err(VehicleError::ConnectionFailed(message));
                            }
                            _ => return Err(VehicleError::Disconnected),
                        }
                    }
                }
                changed = link_state.changed() => {
                    if changed.is_err() {
                        return Err(VehicleError::Disconnected);
                    }

                    match link_state.borrow().clone() {
                        LinkState::Connecting | LinkState::Connected => {}
                        LinkState::Disconnected => return Err(VehicleError::Disconnected),
                        LinkState::Error(message) => {
                            return Err(VehicleError::ConnectionFailed(message));
                        }
                    }
                }
            }
        }
    };

    tokio::select! {
        result = ready_wait => result,
        _ = tokio::time::sleep(connect_timeout) => Err(VehicleError::Timeout("connecting to vehicle".into())),
    }
}
