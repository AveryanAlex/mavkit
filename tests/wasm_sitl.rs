#![cfg(target_arch = "wasm32")]

use futures::future::{Either, select};
use js_sys::{ArrayBuffer, Uint8Array};
use mavkit::byte_connection::{ByteBridge, ByteConnectionConfig};
use mavkit::{AutopilotType, InitPolicyConfig, LinkState, Vehicle, VehicleConfig};
use std::cell::RefCell;
use std::future::Future;
use std::rc::Rc;
use std::time::Duration;
use wasm_bindgen::JsCast;
use wasm_bindgen::closure::Closure;
use wasm_bindgen_test::wasm_bindgen_test;
use web_sys::{BinaryType, CloseEvent, Event, MessageEvent, WebSocket};

wasm_bindgen_test::wasm_bindgen_test_configure!(run_in_browser);

struct BrowserByteTransport {
    ws: WebSocket,
    bridge: ByteBridge,
    _on_open: Closure<dyn FnMut(Event)>,
    _on_message: Closure<dyn FnMut(MessageEvent)>,
    _on_error: Closure<dyn FnMut(Event)>,
    _on_close: Closure<dyn FnMut(CloseEvent)>,
}

impl BrowserByteTransport {
    async fn connect(url: &str, bridge: ByteBridge) -> Result<Self, String> {
        let ws = WebSocket::new(url).map_err(format_js_error)?;
        ws.set_binary_type(BinaryType::Arraybuffer);

        let (open_tx, open_rx) = tokio::sync::oneshot::channel::<Result<(), String>>();
        let open_tx = Rc::new(RefCell::new(Some(open_tx)));

        let open_sender = Rc::clone(&open_tx);
        let on_open = Closure::<dyn FnMut(Event)>::new(move |_event: Event| {
            if let Some(sender) = open_sender.borrow_mut().take() {
                let _ = sender.send(Ok(()));
            }
        });

        let (inbound_tx, mut inbound_rx) = tokio::sync::mpsc::channel::<Vec<u8>>(128);
        let inbound_bridge = bridge.clone();
        wasm_bindgen_futures::spawn_local(async move {
            while let Some(bytes) = inbound_rx.recv().await {
                if inbound_bridge.push_inbound(bytes).await.is_err() {
                    inbound_bridge.close();
                    break;
                }
            }
        });

        let message_bridge = bridge.clone();
        let on_message = Closure::<dyn FnMut(MessageEvent)>::new(move |event: MessageEvent| {
            let Some(bytes) = websocket_message_bytes(event.data()) else {
                return;
            };

            if inbound_tx.try_send(bytes).is_err() {
                message_bridge.close();
            }
        });

        let error_sender = Rc::clone(&open_tx);
        let error_bridge = bridge.clone();
        let on_error = Closure::<dyn FnMut(Event)>::new(move |_event: Event| {
            error_bridge.close();
            if let Some(sender) = error_sender.borrow_mut().take() {
                let _ = sender.send(Err(String::from("websocket error before open")));
            }
        });

        let close_sender = Rc::clone(&open_tx);
        let close_bridge = bridge.clone();
        let on_close = Closure::<dyn FnMut(CloseEvent)>::new(move |event: CloseEvent| {
            close_bridge.close();
            if let Some(sender) = close_sender.borrow_mut().take() {
                let _ = sender.send(Err(format!(
                    "websocket closed before open: code={} reason={}",
                    event.code(),
                    event.reason()
                )));
            }
        });

        ws.set_onopen(Some(on_open.as_ref().unchecked_ref()));
        ws.set_onmessage(Some(on_message.as_ref().unchecked_ref()));
        ws.set_onerror(Some(on_error.as_ref().unchecked_ref()));
        ws.set_onclose(Some(on_close.as_ref().unchecked_ref()));

        let outbound_ws = ws.clone();
        let outbound_bridge = bridge.clone();
        wasm_bindgen_futures::spawn_local(async move {
            while let Some(bytes) = outbound_bridge.next_outbound().await {
                if outbound_ws.ready_state() != WebSocket::OPEN {
                    outbound_bridge.close();
                    break;
                }

                if outbound_ws.send_with_u8_array(&bytes).is_err() {
                    outbound_bridge.close();
                    break;
                }
            }
        });

        match with_timeout("websocket open", Duration::from_secs(10), open_rx).await? {
            Ok(result) => result?,
            Err(_) => return Err(String::from("websocket open callback dropped")),
        }

        Ok(Self {
            ws,
            bridge,
            _on_open: on_open,
            _on_message: on_message,
            _on_error: on_error,
            _on_close: on_close,
        })
    }

    fn close(&self) {
        self.bridge.close();
        let _ = self.ws.close();
    }
}

fn websocket_message_bytes(data: wasm_bindgen::JsValue) -> Option<Vec<u8>> {
    if let Ok(buffer) = data.clone().dyn_into::<ArrayBuffer>() {
        return Some(Uint8Array::new(&buffer).to_vec());
    }

    data.dyn_into::<Uint8Array>()
        .ok()
        .map(|array| array.to_vec())
}

fn wasm_sitl_url() -> &'static str {
    match option_env!("MAVKIT_WASM_SITL_WS_URL") {
        Some(url) => url,
        None => {
            panic!("MAVKIT_WASM_SITL_WS_URL must be set when compiling/running wasm SITL tests")
        }
    }
}

fn wasm_sitl_config() -> VehicleConfig {
    let mut init_policy = InitPolicyConfig::default();
    init_policy.autopilot_version.enabled = false;
    init_policy.available_modes.enabled = false;
    init_policy.home.enabled = false;
    init_policy.origin.enabled = false;

    VehicleConfig {
        connect_timeout: Duration::from_secs(20),
        command_timeout: Duration::from_secs(10),
        command_completion_timeout: Duration::from_secs(20),
        init_policy,
        auto_request_home: false,
        ..VehicleConfig::default()
    }
}

async fn with_timeout<T>(
    label: &str,
    timeout: Duration,
    future: impl Future<Output = T>,
) -> Result<T, String> {
    let timeout =
        gloo_timers::future::TimeoutFuture::new(timeout.as_millis().min(u32::MAX as u128) as u32);
    futures::pin_mut!(future);
    futures::pin_mut!(timeout);

    match select(future, timeout).await {
        Either::Left((value, _)) => Ok(value),
        Either::Right(((), _)) => Err(format!("timed out waiting for {label}")),
    }
}

fn format_js_error(error: wasm_bindgen::JsValue) -> String {
    error
        .as_string()
        .unwrap_or_else(|| format!("JavaScript error: {error:?}"))
}

#[wasm_bindgen_test]
async fn wasm_sitl_connects_and_receives_telemetry() {
    let (bridge, vehicle_future) =
        Vehicle::from_byte_connection(wasm_sitl_config(), ByteConnectionConfig::default());
    let transport = BrowserByteTransport::connect(wasm_sitl_url(), bridge)
        .await
        .expect("websocket transport should connect");

    let vehicle = with_timeout("vehicle heartbeat", Duration::from_secs(25), vehicle_future)
        .await
        .expect("vehicle connect should complete")
        .expect("vehicle should connect through websocket bridge");

    let identity = vehicle.identity();
    assert_eq!(identity.autopilot, AutopilotType::ArduPilotMega);
    assert_ne!(identity.system_id, 0);

    let link_state = vehicle
        .link()
        .state()
        .latest()
        .expect("link state should be populated after connect");
    assert!(matches!(link_state, LinkState::Connected));

    let telemetry = vehicle.telemetry();
    let messages = telemetry.messages();
    messages
        .attitude()
        .set_rate(5.0)
        .await
        .expect("ATTITUDE stream rate should be set");
    messages
        .global_position_int()
        .set_rate(5.0)
        .await
        .expect("GLOBAL_POSITION_INT stream rate should be set");

    let attitude = telemetry
        .attitude()
        .euler()
        .wait_timeout(Duration::from_secs(15))
        .await
        .expect("attitude telemetry should arrive")
        .value;
    assert!(attitude.roll_deg.is_finite());
    assert!(attitude.pitch_deg.is_finite());
    assert!(attitude.yaw_deg.is_finite());

    let position = telemetry
        .position()
        .global()
        .wait_timeout(Duration::from_secs(15))
        .await
        .expect("global position telemetry should arrive")
        .value;
    assert!(position.latitude_deg.is_finite());
    assert!(position.longitude_deg.is_finite());

    vehicle
        .disconnect()
        .await
        .expect("vehicle should disconnect cleanly");
    transport.close();
}
