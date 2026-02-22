use super::types::{MissionFrame, MissionItem};

pub fn sample_item(seq: u16) -> MissionItem {
    MissionItem {
        seq,
        command: 16,
        frame: MissionFrame::GlobalRelativeAltInt,
        current: seq == 0,
        autocontinue: true,
        param1: 0.0,
        param2: 1.0,
        param3: 0.0,
        param4: 0.0,
        x: 473_977_420,
        y: 85_455_970,
        z: 42.123_456,
    }
}
