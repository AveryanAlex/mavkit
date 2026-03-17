use super::commands::{MissionCommand, MissionFrame, RawMissionCommand};
use super::types::{MissionItem, MissionPlan, MissionType};

fn mission_home_placeholder() -> MissionItem {
    MissionItem {
        seq: 0,
        command: MissionCommand::Other(RawMissionCommand {
            command: 16,
            frame: MissionFrame::Global,
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            x: 0,
            y: 0,
            z: 0.0,
        }),
        current: false,
        autocontinue: true,
    }
}

fn encode_command_for_wire(command: &MissionCommand) -> MissionCommand {
    let (raw_command, frame, params, x, y, z) = command.clone().into_wire();
    MissionCommand::Other(RawMissionCommand {
        command: raw_command,
        frame,
        param1: params[0],
        param2: params[1],
        param3: params[2],
        param4: params[3],
        x,
        y,
        z,
    })
}

fn decode_command_from_wire(command: &MissionCommand) -> MissionCommand {
    let (raw_command, frame, params, x, y, z) = command.clone().into_wire();
    MissionCommand::from_wire(raw_command, frame, params, x, y, z)
}

pub fn items_for_wire_upload(plan: &MissionPlan) -> Vec<MissionItem> {
    if plan.mission_type != MissionType::Mission {
        return plan.items.clone();
    }

    let mut wire = Vec::with_capacity(plan.items.len() + 1);
    wire.push(mission_home_placeholder());
    for (index, item) in plan.items.iter().enumerate() {
        wire.push(MissionItem {
            seq: (index + 1) as u16,
            command: encode_command_for_wire(&item.command),
            current: item.current,
            autocontinue: item.autocontinue,
        });
    }
    wire
}

pub fn plan_from_wire_download(
    mission_type: MissionType,
    wire_items: Vec<MissionItem>,
) -> MissionPlan {
    if mission_type != MissionType::Mission {
        return MissionPlan {
            mission_type,
            items: wire_items,
        };
    }

    let items: Vec<MissionItem> = wire_items
        .into_iter()
        .skip(1)
        .enumerate()
        .map(|(index, item)| MissionItem {
            seq: index as u16,
            command: decode_command_from_wire(&item.command),
            current: index == 0,
            autocontinue: item.autocontinue,
        })
        .collect();

    MissionPlan {
        mission_type,
        items,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geo::{GeoPoint3d, GeoPoint3dRelHome};
    use crate::mission::commands::{DoCommand, DoJump, NavCommand, NavWaypoint};
    use crate::mission::validate_plan;

    fn typed_waypoint_item(seq: u16, latitude_deg: f64) -> MissionItem {
        MissionItem {
            seq,
            command: MissionCommand::from(NavWaypoint {
                position: GeoPoint3d::RelHome(GeoPoint3dRelHome {
                    latitude_deg,
                    longitude_deg: 8.545_594,
                    relative_alt_m: 50.0,
                }),
                hold_time_s: 0.0,
                acceptance_radius_m: 1.0,
                pass_radius_m: 0.0,
                yaw_deg: 0.0,
            }),
            current: seq == 0,
            autocontinue: true,
        }
    }

    fn typed_jump_item(seq: u16, target_index: u16) -> MissionItem {
        MissionItem {
            seq,
            command: MissionCommand::from(DoJump {
                target_index,
                repeat_count: 1,
            }),
            current: false,
            autocontinue: true,
        }
    }

    fn raw_wire_waypoint(seq: u16) -> MissionItem {
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
                z: 42.0,
            }),
            current: false,
            autocontinue: true,
        }
    }

    fn raw_wire_jump(seq: u16, target_index: u16) -> MissionItem {
        MissionItem {
            seq,
            command: MissionCommand::Other(RawMissionCommand {
                command: 177,
                frame: MissionFrame::Mission,
                param1: target_index as f32,
                param2: 2.0,
                param3: 0.0,
                param4: 0.0,
                x: 0,
                y: 0,
                z: 0.0,
            }),
            current: false,
            autocontinue: true,
        }
    }

    #[test]
    fn wire_upload_prepends_placeholder_for_mission_type() {
        let first = typed_waypoint_item(8, 47.397_742);
        let second = typed_waypoint_item(99, 47.397_842);
        let plan = MissionPlan {
            mission_type: MissionType::Mission,
            items: vec![first, second],
        };

        let wire = items_for_wire_upload(&plan);
        assert_eq!(wire.len(), 3);
        assert_eq!(wire[0].seq, 0);

        let (command, frame, _, _, _, _) = wire[0].command.clone().into_wire();
        assert_eq!(command, 16);
        assert_eq!(frame, MissionFrame::Global);
        assert_eq!(wire[1].seq, 1);
        assert_eq!(wire[2].seq, 2);
        assert!(matches!(&wire[1].command, MissionCommand::Other(_)));

        let expected = plan.items[0].command.clone().into_wire();
        assert_eq!(wire[1].command.clone().into_wire(), expected);
    }

    #[test]
    fn wire_upload_passthrough_for_fence() {
        let item = typed_waypoint_item(12, 47.397_742);
        let plan = MissionPlan {
            mission_type: MissionType::Fence,
            items: vec![item],
        };

        let wire = items_for_wire_upload(&plan);
        assert_eq!(wire, plan.items);
    }

    #[test]
    fn wire_download_strips_hidden_home_for_mission_type() {
        let wire_items = vec![
            mission_home_placeholder(),
            raw_wire_waypoint(1),
            raw_wire_jump(2, 1),
        ];
        let plan = plan_from_wire_download(MissionType::Mission, wire_items);

        assert_eq!(plan.items.len(), 2);
        assert_eq!(plan.items[0].seq, 0);
        assert_eq!(plan.items[1].seq, 1);
        assert!(plan.items[0].current);
        assert!(matches!(
            &plan.items[0].command,
            MissionCommand::Nav(NavCommand::Waypoint(_))
        ));
        assert!(matches!(
            &plan.items[1].command,
            MissionCommand::Do(DoCommand::Jump(DoJump {
                target_index: 1,
                repeat_count: 2,
            }))
        ));
    }

    #[test]
    fn wire_download_passthrough_for_fence() {
        let wire_items = vec![raw_wire_waypoint(0), raw_wire_jump(1, 0)];
        let plan = plan_from_wire_download(MissionType::Fence, wire_items.clone());

        assert_eq!(plan.items, wire_items);
    }

    #[test]
    fn typed_roundtrip() {
        let plan = MissionPlan {
            mission_type: MissionType::Mission,
            items: vec![typed_waypoint_item(0, 47.397_742), typed_jump_item(1, 0)],
        };

        let wire = items_for_wire_upload(&plan);
        let roundtrip = plan_from_wire_download(MissionType::Mission, wire);

        assert_eq!(roundtrip, plan);
    }

    #[test]
    fn jump_validation() {
        let invalid = MissionPlan {
            mission_type: MissionType::Mission,
            items: vec![typed_jump_item(0, 1)],
        };

        let issues = validate_plan(&invalid);
        assert!(
            issues
                .iter()
                .any(|issue| issue.code == "item.do_jump_target_out_of_range")
        );
    }

    #[test]
    fn empty_mission_upload_has_placeholder_only() {
        let plan = MissionPlan {
            mission_type: MissionType::Mission,
            items: Vec::new(),
        };

        let wire = items_for_wire_upload(&plan);
        assert_eq!(wire.len(), 1);
        assert_eq!(wire[0].seq, 0);

        let (command, frame, _, _, _, _) = wire[0].command.clone().into_wire();
        assert_eq!(command, 16);
        assert_eq!(frame, MissionFrame::Global);
    }
}
