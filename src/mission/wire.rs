use super::types::{HomePosition, MissionFrame, MissionItem, MissionPlan, MissionType};

/// Convert a semantic `MissionPlan` into wire items for MAVLink upload.
///
/// For Mission type: prepends home (or a zero placeholder) as seq 0 and
/// resequences semantic items starting from seq 1.
/// For Fence/Rally: returns items unchanged.
pub fn items_for_wire_upload(plan: &MissionPlan) -> Vec<MissionItem> {
    if plan.mission_type != MissionType::Mission {
        return plan.items.clone();
    }

    let home_item = match &plan.home {
        Some(home) => home.to_mission_item(0),
        None => MissionItem {
            seq: 0,
            command: 16,
            frame: MissionFrame::GlobalInt,
            current: false,
            autocontinue: true,
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            x: 0,
            y: 0,
            z: 0.0,
        },
    };

    let mut wire = Vec::with_capacity(plan.items.len() + 1);
    wire.push(home_item);
    for (i, item) in plan.items.iter().enumerate() {
        wire.push(MissionItem {
            seq: (i + 1) as u16,
            ..*item
        });
    }
    wire
}

/// Convert wire items from a MAVLink download into a semantic `MissionPlan`.
///
/// For Mission type: extracts the first item as home position and resequences the
/// remaining items from 0.
/// For Fence/Rally: no home extraction; items pass through unchanged.
pub fn plan_from_wire_download(
    mission_type: MissionType,
    wire_items: Vec<MissionItem>,
) -> MissionPlan {
    if mission_type != MissionType::Mission || wire_items.is_empty() {
        return MissionPlan {
            mission_type,
            home: None,
            items: wire_items,
        };
    }

    let first = &wire_items[0];
    let home = Some(HomePosition {
        latitude_deg: first.x as f64 / 1e7,
        longitude_deg: first.y as f64 / 1e7,
        altitude_m: first.z,
    });

    let items: Vec<MissionItem> = wire_items[1..]
        .iter()
        .enumerate()
        .map(|(i, item)| MissionItem {
            seq: i as u16,
            current: i == 0,
            ..*item
        })
        .collect();

    MissionPlan {
        mission_type,
        home,
        items,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mission::test_support::sample_item;

    #[test]
    fn wire_upload_prepends_home_for_mission_type() {
        let plan = MissionPlan {
            mission_type: MissionType::Mission,
            home: Some(HomePosition {
                latitude_deg: 47.397742,
                longitude_deg: 8.545594,
                altitude_m: 100.0,
            }),
            items: vec![
                MissionItem {
                    param4: 0.0,
                    ..sample_item(0)
                },
                MissionItem {
                    param4: 0.0,
                    ..sample_item(1)
                },
            ],
        };

        let wire = items_for_wire_upload(&plan);
        assert_eq!(wire.len(), 3);
        assert_eq!(wire[0].seq, 0);
        assert_eq!(wire[0].frame, MissionFrame::GlobalInt);
        assert_eq!(wire[1].seq, 1);
        assert_eq!(wire[2].seq, 2);
    }

    #[test]
    fn wire_upload_resequences_items_but_preserves_payload() {
        let plan = MissionPlan {
            mission_type: MissionType::Mission,
            home: Some(HomePosition {
                latitude_deg: 47.397742,
                longitude_deg: 8.545594,
                altitude_m: 100.0,
            }),
            items: vec![
                MissionItem {
                    // Make this item distinct
                    command: 17,
                    param1: 123.0,
                    param2: 4.0,
                    param3: 5.0,
                    param4: 6.0,
                    x: 1,
                    y: 2,
                    z: 3.0,
                    current: true,
                    ..sample_item(9)
                },
                MissionItem {
                    command: 18,
                    frame: MissionFrame::LocalNed,
                    current: false,
                    autocontinue: false,
                    param1: 0.5,
                    param2: 0.25,
                    param3: 0.75,
                    param4: 1.25,
                    x: -10,
                    y: 20,
                    z: -30.0,
                    ..sample_item(42)
                },
            ],
        };

        let wire = items_for_wire_upload(&plan);
        assert_eq!(wire.len(), 3);

        assert_eq!(wire[1].seq, 1);
        assert_eq!(wire[1].command, 17);
        assert_eq!(wire[1].frame, MissionFrame::GlobalRelativeAltInt);
        assert!(wire[1].current);
        assert!(wire[1].autocontinue);
        assert_eq!(wire[1].param1, 123.0);
        assert_eq!(wire[1].param2, 4.0);
        assert_eq!(wire[1].param3, 5.0);
        assert_eq!(wire[1].param4, 6.0);
        assert_eq!(wire[1].x, 1);
        assert_eq!(wire[1].y, 2);
        assert_eq!(wire[1].z, 3.0);

        assert_eq!(wire[2].seq, 2);
        assert_eq!(wire[2].command, 18);
        assert_eq!(wire[2].frame, MissionFrame::LocalNed);
        assert!(!wire[2].current);
        assert!(!wire[2].autocontinue);
        assert_eq!(wire[2].param1, 0.5);
        assert_eq!(wire[2].param2, 0.25);
        assert_eq!(wire[2].param3, 0.75);
        assert_eq!(wire[2].param4, 1.25);
        assert_eq!(wire[2].x, -10);
        assert_eq!(wire[2].y, 20);
        assert_eq!(wire[2].z, -30.0);
    }

    #[test]
    fn wire_upload_uses_placeholder_when_no_home() {
        let plan = MissionPlan {
            mission_type: MissionType::Mission,
            home: None,
            items: vec![MissionItem {
                param4: 0.0,
                ..sample_item(0)
            }],
        };

        let wire = items_for_wire_upload(&plan);
        assert_eq!(wire.len(), 2);
        assert_eq!(wire[0].x, 0);
        assert_eq!(wire[0].y, 0);
    }

    #[test]
    fn wire_upload_passthrough_for_fence() {
        let plan = MissionPlan {
            mission_type: MissionType::Fence,
            home: None,
            items: vec![MissionItem {
                param4: 0.0,
                ..sample_item(0)
            }],
        };

        let wire = items_for_wire_upload(&plan);
        assert_eq!(wire.len(), 1);
    }

    #[test]
    fn wire_download_extracts_home_for_mission_type() {
        let wire = vec![
            MissionItem {
                seq: 0,
                command: 16,
                frame: MissionFrame::GlobalInt,
                current: false,
                autocontinue: true,
                param1: 0.0,
                param2: 0.0,
                param3: 0.0,
                param4: 0.0,
                x: 473977420,
                y: 85455970,
                z: 100.0,
            },
            MissionItem {
                seq: 1,
                param4: 0.0,
                ..sample_item(1)
            },
            MissionItem {
                seq: 2,
                param4: 0.0,
                ..sample_item(2)
            },
        ];

        let plan = plan_from_wire_download(MissionType::Mission, wire);
        assert!(plan.home.is_some());
        let home = plan.home.unwrap();
        assert!((home.latitude_deg - 47.397742).abs() < 0.0001);
        assert_eq!(plan.items.len(), 2);
        assert_eq!(plan.items[0].seq, 0);
        assert_eq!(plan.items[1].seq, 1);
    }

    #[test]
    fn wire_download_empty_mission_returns_no_home_and_no_items() {
        let plan = plan_from_wire_download(MissionType::Mission, Vec::new());
        assert!(plan.home.is_none());
        assert!(plan.items.is_empty());
    }

    #[test]
    fn wire_download_resequences_and_sets_current_on_first_item() {
        let wire = vec![
            HomePosition {
                latitude_deg: 47.397742,
                longitude_deg: 8.545594,
                altitude_m: 100.0,
            }
            .to_mission_item(0),
            MissionItem {
                seq: 10,
                current: false,
                param4: 0.0,
                ..sample_item(10)
            },
            MissionItem {
                seq: 11,
                current: true,
                param4: 0.0,
                ..sample_item(11)
            },
        ];

        let plan = plan_from_wire_download(MissionType::Mission, wire);
        assert!(plan.home.is_some());
        assert_eq!(plan.items.len(), 2);
        assert_eq!(plan.items[0].seq, 0);
        assert!(plan.items[0].current);
        assert_eq!(plan.items[1].seq, 1);
        assert!(!plan.items[1].current);
    }

    #[test]
    fn wire_download_never_extracts_home_for_non_mission_types() {
        let wire = vec![
            HomePosition {
                latitude_deg: 47.397742,
                longitude_deg: 8.545594,
                altitude_m: 100.0,
            }
            .to_mission_item(0),
            MissionItem {
                param4: 0.0,
                ..sample_item(1)
            },
        ];

        let plan_fence = plan_from_wire_download(MissionType::Fence, wire.clone());
        assert!(plan_fence.home.is_none());
        assert_eq!(plan_fence.items.len(), 2);

        let plan_rally = plan_from_wire_download(MissionType::Rally, wire);
        assert!(plan_rally.home.is_none());
        assert_eq!(plan_rally.items.len(), 2);
    }

    #[test]
    fn wire_download_passthrough_for_fence() {
        let wire = vec![MissionItem {
            param4: 0.0,
            ..sample_item(0)
        }];
        let plan = plan_from_wire_download(MissionType::Fence, wire);
        assert!(plan.home.is_none());
        assert_eq!(plan.items.len(), 1);
    }
}
