use std::sync::{Arc, Mutex};
use std::time::Duration;

use async_trait::async_trait;
use futures::FutureExt;
use mavkit::{FtpEntryKind, FtpOperationProgress, FtpTarget, Vehicle, VehicleConfig, dialect};
use mavlink::{MAVLinkMessageRaw, MAVLinkV2MessageRaw, MavHeader, MavlinkVersion};
use tokio::sync::mpsc;
use tokio::time::{sleep, timeout};

type SentMessages = Arc<Mutex<Vec<(MavHeader, dialect::MavMessage)>>>;

struct MockConnection {
    recv_rx: tokio::sync::Mutex<mpsc::Receiver<(MavHeader, dialect::MavMessage)>>,
    sent: SentMessages,
}

impl MockConnection {
    fn new(rx: mpsc::Receiver<(MavHeader, dialect::MavMessage)>) -> (Self, SentMessages) {
        let sent = Arc::new(Mutex::new(Vec::new()));
        (
            Self {
                recv_rx: tokio::sync::Mutex::new(rx),
                sent: sent.clone(),
            },
            sent,
        )
    }
}

#[async_trait]
impl mavlink::AsyncMavConnection<dialect::MavMessage> for MockConnection {
    async fn recv(
        &self,
    ) -> Result<(MavHeader, dialect::MavMessage), mavlink::error::MessageReadError> {
        let mut rx = self.recv_rx.lock().await;
        rx.recv().await.ok_or_else(|| {
            mavlink::error::MessageReadError::Io(std::io::Error::new(
                std::io::ErrorKind::ConnectionReset,
                "mock connection closed",
            ))
        })
    }

    async fn recv_raw(&self) -> Result<MAVLinkMessageRaw, mavlink::error::MessageReadError> {
        let (header, message) = self.recv().await?;
        let mut raw = MAVLinkV2MessageRaw::new();
        raw.serialize_message(header, &message);
        Ok(MAVLinkMessageRaw::V2(raw))
    }

    async fn try_recv(
        &self,
    ) -> Result<(MavHeader, dialect::MavMessage), mavlink::error::MessageReadError> {
        match self.recv().now_or_never() {
            Some(result) => result,
            None => Err(mavlink::error::MessageReadError::Io(
                std::io::ErrorKind::WouldBlock.into(),
            )),
        }
    }

    async fn send(
        &self,
        header: &MavHeader,
        data: &dialect::MavMessage,
    ) -> Result<usize, mavlink::error::MessageWriteError> {
        self.sent.lock().unwrap().push((*header, data.clone()));
        Ok(0)
    }

    fn set_protocol_version(&mut self, _version: MavlinkVersion) {}

    fn protocol_version(&self) -> MavlinkVersion {
        MavlinkVersion::V2
    }

    fn set_allow_recv_any_version(&mut self, _allow: bool) {}

    fn allow_recv_any_version(&self) -> bool {
        true
    }
}

fn header() -> MavHeader {
    MavHeader {
        system_id: 1,
        component_id: 1,
        sequence: 0,
    }
}

fn heartbeat() -> dialect::MavMessage {
    dialect::MavMessage::HEARTBEAT(dialect::HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: dialect::MavType::MAV_TYPE_QUADROTOR,
        autopilot: dialect::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode: dialect::MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        system_status: dialect::MavState::MAV_STATE_STANDBY,
        mavlink_version: 3,
    })
}

async fn connect() -> (
    Vehicle,
    mpsc::Sender<(MavHeader, dialect::MavMessage)>,
    SentMessages,
) {
    let (msg_tx, msg_rx) = mpsc::channel(16);
    let (connection, sent) = MockConnection::new(msg_rx);
    let config = VehicleConfig {
        connect_timeout: Duration::from_millis(250),
        auto_request_home: false,
        ..VehicleConfig::default()
    };
    let connect_task = tokio::spawn(Vehicle::from_connection(Box::new(connection), config));
    msg_tx
        .send((header(), heartbeat()))
        .await
        .expect("heartbeat should reach mock vehicle");
    let vehicle = connect_task
        .await
        .expect("connect task should join")
        .expect("mock vehicle should connect");
    (vehicle, msg_tx, sent)
}

async fn next_ftp_request(
    sent: &SentMessages,
    cursor: &mut usize,
) -> dialect::FILE_TRANSFER_PROTOCOL_DATA {
    timeout(Duration::from_millis(500), async {
        loop {
            {
                let messages = sent.lock().unwrap();
                while *cursor < messages.len() {
                    let index = *cursor;
                    *cursor += 1;
                    if let dialect::MavMessage::FILE_TRANSFER_PROTOCOL(data) = &messages[index].1 {
                        return data.clone();
                    }
                }
            }
            sleep(Duration::from_millis(1)).await;
        }
    })
    .await
    .expect("FTP request should be sent")
}

fn ftp_ack(
    request: &dialect::FILE_TRANSFER_PROTOCOL_DATA,
    session: u8,
    data: &[u8],
) -> dialect::MavMessage {
    let mut payload = [0_u8; 251];
    let sequence = u16::from_le_bytes(request.payload[0..2].try_into().unwrap()).wrapping_add(1);
    payload[0..2].copy_from_slice(&sequence.to_le_bytes());
    payload[2] = session;
    payload[3] = 128;
    payload[4] = data.len() as u8;
    payload[5] = request.payload[3];
    payload[12..12 + data.len()].copy_from_slice(data);
    dialect::MavMessage::FILE_TRANSFER_PROTOCOL(dialect::FILE_TRANSFER_PROTOCOL_DATA {
        target_network: 0,
        target_system: 255,
        target_component: 190,
        payload,
    })
}

fn ftp_nak(request: &dialect::FILE_TRANSFER_PROTOCOL_DATA, code: u8) -> dialect::MavMessage {
    let mut payload = [0_u8; 251];
    let sequence = u16::from_le_bytes(request.payload[0..2].try_into().unwrap()).wrapping_add(1);
    payload[0..2].copy_from_slice(&sequence.to_le_bytes());
    payload[3] = 129;
    payload[4] = 1;
    payload[5] = request.payload[3];
    payload[12] = code;
    dialect::MavMessage::FILE_TRANSFER_PROTOCOL(dialect::FILE_TRANSFER_PROTOCOL_DATA {
        target_network: 0,
        target_system: 255,
        target_component: 190,
        payload,
    })
}

#[tokio::test]
async fn public_download_runs_open_read_and_terminate_session_exchange() {
    let (vehicle, msg_tx, sent) = connect().await;
    let op = vehicle
        .ftp()
        .download("/logs/test.bin")
        .expect("download should start");
    let mut cursor = 0;

    let open = next_ftp_request(&sent, &mut cursor).await;
    assert_eq!(open.payload[3], 4);
    assert_eq!(
        &open.payload[12..12 + usize::from(open.payload[4])],
        b"/logs/test.bin"
    );
    msg_tx
        .send((header(), ftp_ack(&open, 9, &5_u32.to_le_bytes())))
        .await
        .expect("open ack should reach vehicle");

    let read = next_ftp_request(&sent, &mut cursor).await;
    assert_eq!(read.payload[2], 9);
    assert_eq!(read.payload[3], 5);
    assert_eq!(read.payload[4], 5);
    msg_tx
        .send((header(), ftp_ack(&read, 9, b"hello")))
        .await
        .expect("read ack should reach vehicle");

    let eof_read = next_ftp_request(&sent, &mut cursor).await;
    assert_eq!(eof_read.payload[3], 5);
    assert_eq!(
        u32::from_le_bytes(eof_read.payload[8..12].try_into().unwrap()),
        5
    );
    msg_tx
        .send((header(), ftp_nak(&eof_read, 6)))
        .await
        .expect("EOF NAK should reach vehicle");

    let terminate = next_ftp_request(&sent, &mut cursor).await;
    assert_eq!(terminate.payload[2], 9);
    assert_eq!(terminate.payload[3], 1);
    msg_tx
        .send((header(), ftp_ack(&terminate, 9, &[])))
        .await
        .expect("terminate ack should reach vehicle");

    assert_eq!(op.wait().await.expect("download should complete"), b"hello");
    assert_eq!(op.latest(), Some(FtpOperationProgress::Completed));
    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
}

#[tokio::test]
async fn public_list_directory_collects_pages_until_eof() {
    let (vehicle, msg_tx, sent) = connect().await;
    let list_vehicle = vehicle.clone();
    let list_task = tokio::spawn(async move { list_vehicle.ftp().list_directory("/logs").await });
    let mut cursor = 0;

    let first_page = next_ftp_request(&sent, &mut cursor).await;
    assert_eq!(first_page.payload[3], 3);
    msg_tx
        .send((
            header(),
            ftp_ack(&first_page, 0, b"Fone.bin\t4\0Darchive\0"),
        ))
        .await
        .expect("list ack should reach vehicle");

    let eof_request = next_ftp_request(&sent, &mut cursor).await;
    assert_eq!(
        u32::from_le_bytes(eof_request.payload[8..12].try_into().unwrap()),
        2
    );
    msg_tx
        .send((header(), ftp_nak(&eof_request, 6)))
        .await
        .expect("EOF NAK should reach vehicle");

    let entries = list_task
        .await
        .expect("list task should join")
        .expect("directory list should complete");
    assert_eq!(entries.len(), 2);
    assert_eq!(entries[0].name, "one.bin");
    assert_eq!(entries[0].kind, FtpEntryKind::File);
    assert_eq!(entries[0].size, Some(4));
    assert_eq!(entries[1].kind, FtpEntryKind::Directory);
    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
}

#[tokio::test]
async fn public_upload_writes_chunks_and_terminates_session() {
    let (vehicle, msg_tx, sent) = connect().await;
    let data = vec![7_u8; 240];
    let op = vehicle
        .ftp()
        .upload("/logs/out.bin", data.clone())
        .expect("upload should start");
    let mut cursor = 0;

    let create = next_ftp_request(&sent, &mut cursor).await;
    assert_eq!(create.payload[3], 6);
    msg_tx
        .send((header(), ftp_ack(&create, 4, &[])))
        .await
        .expect("create ack should reach vehicle");

    let first_write = next_ftp_request(&sent, &mut cursor).await;
    assert_eq!(first_write.payload[2], 4);
    assert_eq!(first_write.payload[3], 7);
    assert_eq!(first_write.payload[4], 239);
    assert_eq!(&first_write.payload[12..12 + 239], &data[..239]);
    msg_tx
        .send((header(), ftp_ack(&first_write, 4, &239_u32.to_le_bytes())))
        .await
        .expect("first write ack should reach vehicle");

    let second_write = next_ftp_request(&sent, &mut cursor).await;
    assert_eq!(second_write.payload[3], 7);
    assert_eq!(second_write.payload[4], 1);
    assert_eq!(
        u32::from_le_bytes(second_write.payload[8..12].try_into().unwrap()),
        239
    );
    msg_tx
        .send((header(), ftp_ack(&second_write, 4, &1_u32.to_le_bytes())))
        .await
        .expect("second write ack should reach vehicle");

    let terminate = next_ftp_request(&sent, &mut cursor).await;
    assert_eq!(terminate.payload[3], 1);
    msg_tx
        .send((header(), ftp_ack(&terminate, 4, &[])))
        .await
        .expect("terminate ack should reach vehicle");

    op.wait().await.expect("upload should complete");
    assert_eq!(op.latest(), Some(FtpOperationProgress::Completed));
    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
}

#[tokio::test]
async fn custom_target_is_encoded_and_crc32_returns_server_value() {
    let (vehicle, msg_tx, sent) = connect().await;
    let crc_vehicle = vehicle.clone();
    let crc_task = tokio::spawn(async move {
        crc_vehicle
            .ftp()
            .with_target(FtpTarget {
                network_id: 3,
                system_id: 1,
                component_id: 1,
            })
            .crc32("/logs/out.bin")
            .await
    });
    let mut cursor = 0;

    let crc_request = next_ftp_request(&sent, &mut cursor).await;
    assert_eq!(crc_request.target_network, 3);
    assert_eq!(crc_request.target_system, 1);
    assert_eq!(crc_request.target_component, 1);
    assert_eq!(crc_request.payload[3], 14);
    msg_tx
        .send((
            header(),
            ftp_ack(&crc_request, 0, &0x1234_abcd_u32.to_le_bytes()),
        ))
        .await
        .expect("CRC ack should reach vehicle");

    assert_eq!(
        crc_task
            .await
            .expect("CRC task should join")
            .expect("CRC should complete"),
        0x1234_abcd
    );
    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
}

#[tokio::test]
async fn separate_operations_keep_incrementing_request_sequence() {
    let (vehicle, msg_tx, sent) = connect().await;
    let mut cursor = 0;

    let remove_vehicle = vehicle.clone();
    let remove_task =
        tokio::spawn(async move { remove_vehicle.ftp().remove_file("/logs/old.bin").await });
    let remove = next_ftp_request(&sent, &mut cursor).await;
    assert_eq!(
        u16::from_le_bytes(remove.payload[0..2].try_into().unwrap()),
        0
    );
    msg_tx
        .send((header(), ftp_ack(&remove, 0, &[])))
        .await
        .expect("remove ack should reach vehicle");
    remove_task
        .await
        .expect("remove task should join")
        .expect("remove should complete");

    let create_vehicle = vehicle.clone();
    let create_task = tokio::spawn(async move {
        create_vehicle
            .ftp()
            .create_directory("/logs/new-directory")
            .await
    });
    let create = next_ftp_request(&sent, &mut cursor).await;
    assert_eq!(
        u16::from_le_bytes(create.payload[0..2].try_into().unwrap()),
        1
    );
    msg_tx
        .send((header(), ftp_ack(&create, 0, &[])))
        .await
        .expect("create-directory ack should reach vehicle");
    create_task
        .await
        .expect("create task should join")
        .expect("create directory should complete");

    vehicle
        .disconnect()
        .await
        .expect("disconnect should succeed");
}
