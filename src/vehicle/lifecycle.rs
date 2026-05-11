use crate::command::Command;
use crate::config::VehicleConfig;
use crate::dialect;
use crate::error::VehicleError;
use crate::event_loop::{InitManager, run_event_loop_with_init};
use crate::runtime::{self, TaskHandle, TaskJoinError};
use crate::state::{LinkState, create_channels};
#[cfg(feature = "stream")]
use crate::stream::StreamConnection;
use crate::vehicle::{ConnectionScopedObservationClosers, Vehicle, VehicleInner};
use std::sync::Arc;
#[cfg(feature = "stream")]
use tokio::io::{AsyncRead, AsyncWrite};
use tokio::sync::{mpsc, watch};
use tokio_util::sync::CancellationToken;

impl Vehicle {
    /// Connect using a MAVLink URL with the default [`VehicleConfig`].
    ///
    /// Supported URL schemes depend on enabled Cargo features:
    /// - `udpin:<bind_addr>` — listen for incoming UDP packets (`udp` feature)
    /// - `tcpout:<addr>` — outbound TCP connection (`tcp` feature)
    /// - `tcpin:<addr>` — listen for an incoming TCP connection (`tcp` feature)
    /// - `serial:<port>:<baud>` — serial port (`serial` feature)
    ///
    /// Blocks until the first heartbeat is received or `connect_timeout` elapses.
    /// Returns [`VehicleError::Timeout`] on timeout, [`VehicleError::ConnectionFailed`] on
    /// transport errors.
    #[cfg(not(target_arch = "wasm32"))]
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

    /// Construct a vehicle future plus a byte bridge for callback-style transports.
    ///
    /// This returns the [`crate::byte_connection::ByteBridge`] immediately so callers can feed the
    /// first heartbeat before awaiting vehicle readiness. Await the returned future to complete the
    /// normal first-heartbeat connect path.
    #[cfg(feature = "byte-connection")]
    pub fn from_byte_connection(
        config: VehicleConfig,
        byte_config: crate::byte_connection::ByteConnectionConfig,
    ) -> (
        crate::byte_connection::ByteBridge,
        impl std::future::Future<Output = Result<Self, VehicleError>>,
    ) {
        let (connection, bridge) = crate::byte_connection::ByteConnection::new(byte_config);
        let vehicle = async move { Self::from_connection(Box::new(connection), config).await };
        (bridge, vehicle)
    }

    /// Connect by listening on a UDP socket for an incoming MAVLink stream (`udp` feature).
    #[cfg(all(feature = "udp", not(target_arch = "wasm32")))]
    pub async fn connect_udp(bind_addr: &str) -> Result<Self, VehicleError> {
        Self::connect(&format!("udpin:{bind_addr}")).await
    }

    /// Connect via an outbound TCP connection (`tcp` feature).
    #[cfg(all(feature = "tcp", not(target_arch = "wasm32")))]
    pub async fn connect_tcp(addr: &str) -> Result<Self, VehicleError> {
        Self::connect(&format!("tcpout:{addr}")).await
    }

    /// Connect via a serial port (`serial` feature).
    #[cfg(all(feature = "serial", not(target_arch = "wasm32")))]
    pub async fn connect_serial(port: &str, baud: u32) -> Result<Self, VehicleError> {
        Self::connect(&format!("serial:{port}:{baud}")).await
    }

    /// Connect using a MAVLink URL with a custom [`VehicleConfig`].
    ///
    /// See [`Vehicle::connect`] for supported URL schemes. Use this when you need to adjust
    /// timeouts, GCS IDs, or initialization policies.
    #[cfg(not(target_arch = "wasm32"))]
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

        let mut inner = VehicleInner::new(command_tx, cancel, stores, config);
        let background_handles = inner.start_background_domains(&init_manager);
        let close_bundle = inner.connection_scoped_observation_closers();
        let (shutdown_complete_tx, shutdown_complete_rx) = watch::channel(false);
        inner.shutdown_complete = Some(shutdown_complete_rx);
        let vehicle_cancel = inner.cancel.clone();

        let event_loop_handle = runtime::spawn(run_event_loop_with_init(
            connection,
            command_rx,
            writers,
            inner._config.clone(),
            init_manager.clone(),
            inner.cancel.clone(),
        ));

        let vehicle = Self {
            inner: Arc::new(inner),
        };

        runtime::spawn(supervise_connection_shutdown(
            event_loop_handle,
            background_handles,
            vehicle_cancel,
            init_manager,
            close_bundle,
            shutdown_complete_tx,
        ));

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
            if let Some(shutdown_complete) = &self.inner.shutdown_complete {
                await_shutdown_completion(shutdown_complete.clone()).await?;
            }
            return Ok(());
        }

        if self.inner.command_tx.send(Command::Shutdown).await.is_err() {
            if matches!(
                link_state.borrow().clone(),
                LinkState::Disconnected | LinkState::Error(_)
            ) {
                if let Some(shutdown_complete) = &self.inner.shutdown_complete {
                    await_shutdown_completion(shutdown_complete.clone()).await?;
                }
                return Ok(());
            }

            if let Some(shutdown_complete) = &self.inner.shutdown_complete
                && await_shutdown_completion(shutdown_complete.clone())
                    .await
                    .is_ok()
            {
                return Ok(());
            }

            return Err(VehicleError::Disconnected);
        }

        loop {
            let current = link_state.borrow().clone();
            match current {
                LinkState::Disconnected | LinkState::Error(_) => break,
                LinkState::Connecting | LinkState::Connected => {
                    link_state
                        .changed()
                        .await
                        .map_err(|_| VehicleError::Disconnected)?;
                }
            }
        }

        if let Some(shutdown_complete) = &self.inner.shutdown_complete {
            await_shutdown_completion(shutdown_complete.clone()).await?;
        }

        Ok(())
    }
}

async fn await_shutdown_completion(
    mut shutdown_complete: watch::Receiver<bool>,
) -> Result<(), VehicleError> {
    if *shutdown_complete.borrow() {
        return Ok(());
    }

    loop {
        shutdown_complete
            .changed()
            .await
            .map_err(|_| VehicleError::Disconnected)?;
        if *shutdown_complete.borrow_and_update() {
            return Ok(());
        }
    }
}

async fn supervise_connection_shutdown(
    event_loop_handle: TaskHandle,
    watcher_handles: Vec<TaskHandle>,
    cancel: CancellationToken,
    init_manager: InitManager,
    close_bundle: ConnectionScopedObservationClosers,
    shutdown_complete: watch::Sender<bool>,
) {
    if let Err(err) = event_loop_handle.join().await {
        log_task_join_error("vehicle event-loop", err);
    }

    cancel.cancel();

    for (index, watcher_handle) in watcher_handles.into_iter().enumerate() {
        watcher_handle.abort();
        if let Err(err) = watcher_handle.join().await {
            log_task_join_error(&format!("vehicle background watcher #{index}"), err);
        }
    }

    for (index, init_handle) in init_manager.take_tasks().into_iter().enumerate() {
        init_handle.abort();
        if let Err(err) = init_handle.join().await {
            log_task_join_error(&format!("vehicle init task #{index}"), err);
        }
    }

    close_bundle.close();

    let _ = shutdown_complete.send(true);
}

fn log_task_join_error(task_name: &str, err: TaskJoinError) {
    match err {
        TaskJoinError::Panic => tracing::error!("{task_name} task panicked"),
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

    match runtime::timeout(connect_timeout, ready_wait).await {
        Ok(result) => result,
        Err(_) => Err(VehicleError::Timeout("connecting to vehicle".into())),
    }
}
