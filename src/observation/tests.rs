use super::*;
use tokio_stream::Stream;

fn watch_fixture() -> (ObservationWriter<u32>, ObservationHandle<u32>) {
    ObservationHandle::<u32>::watch()
}

fn broadcast_fixture(capacity: usize) -> (ObservationWriter<String>, ObservationHandle<String>) {
    ObservationHandle::<String>::broadcast(capacity)
}

#[tokio::test]
async fn watch_lifecycle() {
    let (writer, handle) = watch_fixture();
    assert_eq!(handle.latest(), None);

    writer.publish(42).unwrap();
    assert_eq!(handle.latest(), Some(42));

    writer.publish(99).unwrap();
    assert_eq!(handle.latest(), Some(99));
}

#[tokio::test]
async fn wait_disconnect() {
    let (writer, handle) = watch_fixture();
    let task = tokio::spawn(async move { handle.wait().await });

    tokio::time::sleep(Duration::from_millis(20)).await;
    drop(writer);

    let result = task.await.unwrap();
    assert!(matches!(result, Err(VehicleError::Disconnected)));
}

#[tokio::test]
async fn subscribe_terminates_when_writer_closed() {
    let (writer, handle) = watch_fixture();
    let mut sub = handle.subscribe();

    writer.close();

    assert_eq!(sub.recv().await, None);
}

#[tokio::test]
async fn unpublished_watch_subscription_terminates_when_closed_later() {
    let (writer, handle) = watch_fixture();
    let mut sub = handle.subscribe();

    tokio::spawn(async move {
        tokio::time::sleep(Duration::from_millis(20)).await;
        writer.close();
    });

    assert_eq!(sub.recv().await, None);
}

#[tokio::test]
async fn wait_timeout() {
    let (_writer, handle) = watch_fixture();
    let result = handle.wait_timeout(Duration::from_millis(20)).await;
    assert!(matches!(result, Err(VehicleError::Timeout(_))));
}

#[tokio::test]
async fn wait_timeout_returns_disconnected_over_cached_latest() {
    let (writer, handle) = watch_fixture();
    writer.publish(7).unwrap();
    writer.close();

    let result = handle.wait_timeout(Duration::from_millis(20)).await;
    assert!(matches!(result, Err(VehicleError::Disconnected)));
}

#[tokio::test]
async fn wait_timeout_returns_disconnected_over_cached_latest_after_last_writer_drop() {
    let (writer, handle) = watch_fixture();
    writer.publish(7).unwrap();
    drop(writer);

    let result = handle.wait_timeout(Duration::from_millis(20)).await;
    assert!(matches!(result, Err(VehicleError::Disconnected)));
}

#[tokio::test]
async fn dropping_non_last_writer_clone_does_not_close_observation() {
    let (writer, handle) = watch_fixture();
    let writer_clone = writer.clone();

    drop(writer);
    writer_clone.publish(7).unwrap();

    assert_eq!(handle.wait().await.unwrap(), 7);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn wait_timeout_returns_disconnected_after_concurrent_final_writer_drop() {
    for _ in 0..64 {
        let (writer, handle) = watch_fixture();
        writer.publish(7).unwrap();

        let writer_a = writer.clone();
        let writer_b = writer;
        let barrier = Arc::new(std::sync::Barrier::new(3));

        let barrier_a = barrier.clone();
        let thread_a = std::thread::spawn(move || {
            barrier_a.wait();
            drop(writer_a);
        });

        let barrier_b = barrier.clone();
        let thread_b = std::thread::spawn(move || {
            barrier_b.wait();
            drop(writer_b);
        });

        barrier.wait();
        thread_a.join().unwrap();
        thread_b.join().unwrap();

        let result = handle.wait_timeout(Duration::from_millis(20)).await;
        assert!(matches!(result, Err(VehicleError::Disconnected)));
    }
}

#[tokio::test]
async fn wait_returns_current_value_immediately() {
    let (writer, handle) = watch_fixture();
    writer.publish(7).unwrap();
    assert_eq!(handle.wait().await.unwrap(), 7);
}

#[tokio::test]
async fn subscribe_watch_emits_current_then_updates() {
    let (writer, handle) = watch_fixture();
    writer.publish(10).unwrap();

    let mut sub = handle.subscribe();
    assert_eq!(sub.recv().await.unwrap(), 10);

    writer.publish(11).unwrap();
    assert_eq!(sub.recv().await.unwrap(), 11);
}

#[tokio::test]
async fn watch_subscribe_after_close_terminates_without_replaying_cached_latest() {
    let (writer, handle) = watch_fixture();
    writer.publish(10).unwrap();
    writer.close();

    let mut sub = handle.subscribe();
    assert_eq!(sub.recv().await, None);
}

#[test]
fn subscribe_exposes_stream_item_t() {
    fn assert_stream<S: Stream<Item = u32>>(_stream: S) {}

    let (_writer, handle) = watch_fixture();
    assert_stream(handle.subscribe());
}

#[tokio::test]
async fn shared_store() {
    let (writer, handle_a) = watch_fixture();
    let handle_b = handle_a.clone();

    writer.publish(123).unwrap();

    assert_eq!(handle_a.latest(), Some(123));
    assert_eq!(handle_b.latest(), Some(123));
}

#[tokio::test]
async fn broadcast_no_coalesce() {
    let (writer, handle) = broadcast_fixture(16);
    let mut sub = handle.subscribe();

    writer.publish("msg1".to_string()).unwrap();
    writer.publish("msg2".to_string()).unwrap();
    writer.publish("msg3".to_string()).unwrap();

    assert_eq!(sub.recv().await.unwrap(), "msg1");
    assert_eq!(sub.recv().await.unwrap(), "msg2");
    assert_eq!(sub.recv().await.unwrap(), "msg3");
}

#[tokio::test]
async fn broadcast_subscribe_only_future_events() {
    let (writer, handle) = broadcast_fixture(16);

    writer.publish("before".to_string()).unwrap();
    let mut sub = handle.subscribe();
    writer.publish("after".to_string()).unwrap();

    assert_eq!(sub.recv().await.unwrap(), "after");
}

#[tokio::test]
async fn broadcast_wait_returns_latest_immediately() {
    let (writer, handle) = broadcast_fixture(16);

    writer.publish("latest".to_string()).unwrap();
    assert_eq!(handle.wait().await.unwrap(), "latest");
}

#[tokio::test]
async fn broadcast_wait_timeout_returns_disconnected_over_cached_latest_after_last_writer_drop() {
    let (writer, handle) = broadcast_fixture(16);

    writer.publish("latest".to_string()).unwrap();
    drop(writer);

    let result = handle.wait_timeout(Duration::from_millis(20)).await;
    assert!(matches!(result, Err(VehicleError::Disconnected)));
}

#[tokio::test]
async fn broadcast_subscribe_after_close_terminates_without_replaying_cached_latest() {
    let (writer, handle) = broadcast_fixture(16);
    writer.publish("before".to_string()).unwrap();
    writer.close();

    let mut sub = handle.subscribe();
    assert_eq!(sub.recv().await, None);
}

#[tokio::test]
async fn broadcast_lag_skips_to_retained_values() {
    let (writer, handle) = broadcast_fixture(2);
    let mut sub = handle.subscribe();

    writer.publish("m1".to_string()).unwrap();
    writer.publish("m2".to_string()).unwrap();
    writer.publish("m3".to_string()).unwrap();
    writer.publish("m4".to_string()).unwrap();

    assert_eq!(sub.recv().await.unwrap(), "m3");
    assert_eq!(sub.recv().await.unwrap(), "m4");
}
