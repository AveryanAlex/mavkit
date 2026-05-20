use std::time::Duration;

pub use mavkit::time::Instant;

#[cfg(not(target_arch = "wasm32"))]
pub async fn sleep(duration: Duration) {
    tokio::time::sleep(duration).await;
}

#[cfg(target_arch = "wasm32")]
pub async fn sleep(duration: Duration) {
    let millis = duration.as_millis().min(u32::MAX as u128) as u32;
    gloo_timers::future::TimeoutFuture::new(millis).await;
}
