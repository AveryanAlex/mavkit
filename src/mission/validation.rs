use super::commands::{DoCommand, MissionCommand, MissionFrame};
use super::types::{IssueSeverity, MissionIssue, MissionPlan, MissionType};

#[derive(Debug, Clone, Copy)]
/// Numeric tolerances used when treating near-identical mission items as equivalent after wire roundtrips.
pub struct CompareTolerance {
    pub param_epsilon: f32,
    pub altitude_epsilon_m: f32,
}

impl Default for CompareTolerance {
    fn default() -> Self {
        Self {
            param_epsilon: 0.0001,
            altitude_epsilon_m: 0.01,
        }
    }
}

pub fn validate_plan(plan: &MissionPlan) -> Vec<MissionIssue> {
    let mut issues = Vec::new();

    if plan.items.len() > 4096 {
        issues.push(MissionIssue {
            code: "plan.too_many_items".to_string(),
            message: "Mission exceeds maximum supported item count (4096)".to_string(),
            seq: None,
            severity: IssueSeverity::Error,
        });
    }

    for (expected, item) in plan.items.iter().enumerate() {
        let (wire_command, frame, params, x, y, z) = item.command.clone().into_wire();
        for (name, value) in [
            ("param1", params[0]),
            ("param2", params[1]),
            ("param3", params[2]),
            ("param4", params[3]),
            ("z", z),
        ] {
            if !value.is_finite() {
                issues.push(MissionIssue {
                    code: "item.non_finite_value".to_string(),
                    message: format!("{name} must be finite"),
                    seq: Some(expected as u16),
                    severity: IssueSeverity::Error,
                });
            }
        }

        if plan.mission_type == MissionType::Mission
            && let MissionCommand::Do(DoCommand::Jump(jump)) =
                MissionCommand::from_wire(wire_command, frame, params, x, y, z)
            && jump.target_index as usize >= plan.items.len()
        {
            issues.push(MissionIssue {
                code: "item.do_jump_target_out_of_range".to_string(),
                message: format!(
                    "DoJump target index {} is outside [0, {})",
                    jump.target_index,
                    plan.items.len()
                ),
                seq: Some(expected as u16),
                severity: IssueSeverity::Error,
            });
        }

        if matches!(
            frame,
            MissionFrame::Global | MissionFrame::GlobalRelativeAlt | MissionFrame::GlobalTerrainAlt
        ) {
            let latitude = f64::from(x) / 1e7;
            let longitude = f64::from(y) / 1e7;
            if !(-90.0..=90.0).contains(&latitude) {
                issues.push(MissionIssue {
                    code: "item.latitude_out_of_range".to_string(),
                    message: format!("Latitude {latitude} is outside [-90, 90]"),
                    seq: Some(expected as u16),
                    severity: IssueSeverity::Error,
                });
            }

            if !(-180.0..=180.0).contains(&longitude) {
                issues.push(MissionIssue {
                    code: "item.longitude_out_of_range".to_string(),
                    message: format!("Longitude {longitude} is outside [-180, 180]"),
                    seq: Some(expected as u16),
                    severity: IssueSeverity::Error,
                });
            }
        }
    }

    issues
}

pub fn normalize_for_compare(plan: &MissionPlan) -> MissionPlan {
    let mut normalized = plan.clone();
    for item in normalized.items.iter_mut() {
        let (command, frame, mut params, x, y, mut z) = item.command.clone().into_wire();
        params[0] = round_to(params[0], 1e-4);
        params[1] = round_to(params[1], 1e-4);
        params[2] = round_to(params[2], 1e-4);
        params[3] = round_to(params[3], 1e-4);
        z = round_to(z, 1e-3);
        item.command = MissionCommand::from_wire(command, frame, params, x, y, z);
    }
    normalized
}

pub fn plans_equivalent(lhs: &MissionPlan, rhs: &MissionPlan, tolerance: CompareTolerance) -> bool {
    if lhs.mission_type != rhs.mission_type || lhs.items.len() != rhs.items.len() {
        return false;
    }

    lhs.items.iter().zip(&rhs.items).all(|(left, right)| {
        let (lc, lf, lp, lx, ly, lz) = left.command.clone().into_wire();
        let (rc, rf, rp, rx, ry, rz) = right.command.clone().into_wire();

        left.current == right.current
            && left.autocontinue == right.autocontinue
            && lc == rc
            && lf == rf
            && lx == rx
            && ly == ry
            && float_eq(lp[0], rp[0], tolerance.param_epsilon)
            && float_eq(lp[1], rp[1], tolerance.param_epsilon)
            && float_eq(lp[2], rp[2], tolerance.param_epsilon)
            && float_eq(lp[3], rp[3], tolerance.param_epsilon)
            && float_eq(lz, rz, tolerance.altitude_epsilon_m)
    })
}

fn float_eq(a: f32, b: f32, epsilon: f32) -> bool {
    (a - b).abs() <= epsilon
}

fn round_to(value: f32, step: f32) -> f32 {
    (value / step).round() * step
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mission::commands::{DoJump, MissionCommand, MissionFrame, RawMissionCommand};
    use crate::mission::test_support::sample_item;
    use crate::mission::{MissionItem, MissionType};

    #[test]
    fn detects_invalid_global_coordinates_and_nan() {
        let plan = MissionPlan {
            mission_type: MissionType::Mission,
            items: vec![MissionItem {
                command: MissionCommand::Other(RawMissionCommand {
                    command: 16,
                    frame: MissionFrame::Global,
                    param1: f32::NAN,
                    param2: 0.0,
                    param3: 0.0,
                    param4: 0.0,
                    x: 1_200_000_000,
                    y: 2_100_000_000,
                    z: f32::NAN,
                }),
                ..sample_item()
            }],
        };

        let issues = validate_plan(&plan);
        assert!(
            issues
                .iter()
                .any(|issue| issue.code == "item.non_finite_value")
        );
        assert!(
            issues
                .iter()
                .any(|issue| issue.code == "item.latitude_out_of_range")
        );
        assert!(
            issues
                .iter()
                .any(|issue| issue.code == "item.longitude_out_of_range")
        );
    }

    #[test]
    fn normalize_and_equivalent_tolerates_small_float_drift() {
        let base = MissionPlan {
            mission_type: MissionType::Mission,
            items: vec![MissionItem {
                command: MissionCommand::Other(RawMissionCommand {
                    command: 16,
                    frame: MissionFrame::GlobalRelativeAlt,
                    param1: 1.00001,
                    param2: 2.0,
                    param3: 3.0,
                    param4: 4.0,
                    x: 473_977_420,
                    y: 85_455_970,
                    z: 42.0001,
                }),
                ..sample_item()
            }],
        };
        let mut drifted = base.clone();
        drifted.items[0].command = MissionCommand::Other(RawMissionCommand {
            command: 16,
            frame: MissionFrame::GlobalRelativeAlt,
            param1: 1.00002,
            param2: 2.0,
            param3: 3.0,
            param4: 4.0,
            x: 473_977_420,
            y: 85_455_970,
            z: 42.0002,
        });

        let lhs = normalize_for_compare(&base);
        let rhs = normalize_for_compare(&drifted);
        assert!(plans_equivalent(&lhs, &rhs, CompareTolerance::default()));
    }

    #[test]
    fn detects_out_of_range_do_jump_target() {
        let plan = MissionPlan {
            mission_type: MissionType::Mission,
            items: vec![MissionItem {
                command: MissionCommand::from(DoJump {
                    target_index: 1,
                    repeat_count: 1,
                }),
                ..sample_item()
            }],
        };

        let issues = validate_plan(&plan);
        assert!(
            issues
                .iter()
                .any(|issue| issue.code == "item.do_jump_target_out_of_range")
        );
    }
}
