use crate::common::backend::{disconnect, setup_backend_vehicle};
use crate::common::target::TestTarget;
use std::time::Duration;

pub async fn roundtrip_case(target: TestTarget) {
    let backend = setup_backend_vehicle(target).await;
    let path = "mavkit-ftp-roundtrip.bin";
    let expected = b"MAVKit MAVFTP SITL roundtrip".to_vec();

    let _ = backend.vehicle.ftp().remove_file(path).await;
    backend
        .vehicle
        .ftp()
        .upload(path, expected.clone())
        .expect("FTP upload should start")
        .wait_timeout(Duration::from_secs(10))
        .await
        .expect("FTP upload should complete");
    let actual = backend
        .vehicle
        .ftp()
        .download(path)
        .expect("FTP download should start")
        .wait_timeout(Duration::from_secs(10))
        .await
        .expect("FTP download should complete");
    assert_eq!(actual, expected);
    backend
        .vehicle
        .ftp()
        .remove_file(path)
        .await
        .expect("FTP roundtrip fixture should be removed");

    disconnect(backend).await;
}
