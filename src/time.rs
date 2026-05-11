pub use std::time::Duration;

#[cfg(not(target_arch = "wasm32"))]
pub use tokio::time::Instant;

#[cfg(target_arch = "wasm32")]
pub use web_time::Instant;
