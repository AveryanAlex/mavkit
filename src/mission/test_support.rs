use super::commands::{MissionCommand, MissionFrame, RawMissionCommand};
use super::types::MissionItem;

pub fn sample_item(seq: u16) -> MissionItem {
    MissionItem {
        seq,
        command: MissionCommand::Other(RawMissionCommand {
            command: 16,
            frame: MissionFrame::GlobalRelativeAlt,
            param1: 0.0,
            param2: 1.0,
            param3: 0.0,
            param4: 0.0,
            x: 473_977_420,
            y: 85_455_970,
            z: 42.123_456,
        }),
        current: seq == 0,
        autocontinue: true,
    }
}
