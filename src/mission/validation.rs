use super::types::{IssueSeverity, MissionIssue, MissionPlan};

#[derive(Debug, Clone, Copy)]
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

    if let Some(ref home) = plan.home {
        if !(-90.0..=90.0).contains(&home.latitude_deg) {
            issues.push(MissionIssue {
                code: "home.latitude_out_of_range".to_string(),
                message: format!("Home latitude {} is outside [-90, 90]", home.latitude_deg),
                seq: None,
                severity: IssueSeverity::Error,
            });
        }
        if !(-180.0..=180.0).contains(&home.longitude_deg) {
            issues.push(MissionIssue {
                code: "home.longitude_out_of_range".to_string(),
                message: format!(
                    "Home longitude {} is outside [-180, 180]",
                    home.longitude_deg
                ),
                seq: None,
                severity: IssueSeverity::Error,
            });
        }
    }

    if plan.items.len() > 4096 {
        issues.push(MissionIssue {
            code: "plan.too_many_items".to_string(),
            message: "Mission exceeds maximum supported item count (4096)".to_string(),
            seq: None,
            severity: IssueSeverity::Error,
        });
    }

    for (expected, item) in plan.items.iter().enumerate() {
        let expected_seq = expected as u16;
        if item.seq != expected_seq {
            issues.push(MissionIssue {
                code: "plan.non_contiguous_sequence".to_string(),
                message: format!("Expected sequence {} but found {}", expected_seq, item.seq),
                seq: Some(item.seq),
                severity: IssueSeverity::Error,
            });
        }

        for (name, value) in [
            ("param1", item.param1),
            ("param2", item.param2),
            ("param3", item.param3),
            ("param4", item.param4),
            ("z", item.z),
        ] {
            if !value.is_finite() {
                issues.push(MissionIssue {
                    code: "item.non_finite_value".to_string(),
                    message: format!("{name} must be finite"),
                    seq: Some(item.seq),
                    severity: IssueSeverity::Error,
                });
            }
        }

        if item.frame.is_global_position() {
            let latitude = item.x as f64 / 1e7;
            let longitude = item.y as f64 / 1e7;
            if !(-90.0..=90.0).contains(&latitude) {
                issues.push(MissionIssue {
                    code: "item.latitude_out_of_range".to_string(),
                    message: format!("Latitude {latitude} is outside [-90, 90]"),
                    seq: Some(item.seq),
                    severity: IssueSeverity::Error,
                });
            }

            if !(-180.0..=180.0).contains(&longitude) {
                issues.push(MissionIssue {
                    code: "item.longitude_out_of_range".to_string(),
                    message: format!("Longitude {longitude} is outside [-180, 180]"),
                    seq: Some(item.seq),
                    severity: IssueSeverity::Error,
                });
            }
        }
    }

    issues
}

pub fn normalize_for_compare(plan: &MissionPlan) -> MissionPlan {
    let mut normalized = plan.clone();
    for (index, item) in normalized.items.iter_mut().enumerate() {
        item.seq = index as u16;
        item.param1 = round_to(item.param1, 1e-4);
        item.param2 = round_to(item.param2, 1e-4);
        item.param3 = round_to(item.param3, 1e-4);
        item.param4 = round_to(item.param4, 1e-4);
        item.z = round_to(item.z, 1e-3);
    }
    if let Some(ref mut home) = normalized.home {
        home.altitude_m = round_to(home.altitude_m, 1e-3);
    }
    normalized
}

pub fn plans_equivalent(
    lhs: &MissionPlan,
    rhs: &MissionPlan,
    tolerance: CompareTolerance,
) -> bool {
    if lhs.mission_type != rhs.mission_type {
        return false;
    }

    match (&lhs.home, &rhs.home) {
        (Some(lh), Some(rh)) => {
            if lh.latitude_deg != rh.latitude_deg
                || lh.longitude_deg != rh.longitude_deg
                || !float_eq(lh.altitude_m, rh.altitude_m, tolerance.altitude_epsilon_m)
            {
                return false;
            }
        }
        (None, None) => {}
        _ => return false,
    }

    if lhs.items.len() != rhs.items.len() {
        return false;
    }

    lhs.items.iter().zip(&rhs.items).all(|(left, right)| {
        left.seq == right.seq
            && left.command == right.command
            && left.frame == right.frame
            && left.current == right.current
            && left.autocontinue == right.autocontinue
            && float_eq(left.param1, right.param1, tolerance.param_epsilon)
            && float_eq(left.param2, right.param2, tolerance.param_epsilon)
            && float_eq(left.param3, right.param3, tolerance.param_epsilon)
            && float_eq(left.param4, right.param4, tolerance.param_epsilon)
            && left.x == right.x
            && left.y == right.y
            && float_eq(left.z, right.z, tolerance.altitude_epsilon_m)
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
    use crate::mission::{HomePosition, MissionFrame, MissionItem, MissionType};

    fn sample_item(seq: u16) -> MissionItem {
        MissionItem {
            seq,
            command: 16,
            frame: MissionFrame::GlobalRelativeAltInt,
            current: seq == 0,
            autocontinue: true,
            param1: 0.0,
            param2: 1.0,
            param3: 0.0,
            param4: f32::NAN,
            x: 473977420,
            y: 85455970,
            z: 42.123456,
        }
    }

    #[test]
    fn detects_non_contiguous_sequence() {
        let mut second = sample_item(2);
        second.param4 = 0.0;
        let plan = MissionPlan {
            mission_type: MissionType::Mission,
            home: None,
            items: vec![
                MissionItem {
                    param4: 0.0,
                    ..sample_item(0)
                },
                second,
            ],
        };

        let issues = validate_plan(&plan);
        assert!(issues
            .iter()
            .any(|issue| issue.code == "plan.non_contiguous_sequence"));
    }

    #[test]
    fn detects_invalid_global_coordinates_and_nan() {
        let mut item = sample_item(0);
        item.x = 999_000_000;
        let plan = MissionPlan {
            mission_type: MissionType::Mission,
            home: None,
            items: vec![item],
        };

        let issues = validate_plan(&plan);
        assert!(issues
            .iter()
            .any(|issue| issue.code == "item.latitude_out_of_range"));
        assert!(issues
            .iter()
            .any(|issue| issue.code == "item.non_finite_value"));
    }

    #[test]
    fn validates_home_latitude_range() {
        let plan = MissionPlan {
            mission_type: MissionType::Mission,
            home: Some(HomePosition {
                latitude_deg: 95.0,
                longitude_deg: 8.0,
                altitude_m: 0.0,
            }),
            items: Vec::new(),
        };

        let issues = validate_plan(&plan);
        assert!(issues
            .iter()
            .any(|issue| issue.code == "home.latitude_out_of_range"));
    }

    #[test]
    fn normalize_and_equivalent_tolerates_small_float_drift() {
        let mut base = sample_item(0);
        base.param4 = 0.0;

        let mut changed = base.clone();
        changed.param2 += 0.00005;
        changed.z += 0.005;

        let lhs = MissionPlan {
            mission_type: MissionType::Mission,
            home: None,
            items: vec![base],
        };
        let rhs = MissionPlan {
            mission_type: MissionType::Mission,
            home: None,
            items: vec![changed],
        };

        assert!(plans_equivalent(&lhs, &rhs, CompareTolerance::default()));

        let normalized = normalize_for_compare(&lhs);
        assert_eq!(normalized.items[0].seq, 0);
    }

    #[test]
    fn plans_equivalent_compares_home() {
        let home_a = Some(HomePosition {
            latitude_deg: 47.397742,
            longitude_deg: 8.545594,
            altitude_m: 0.0,
        });
        let home_b = Some(HomePosition {
            latitude_deg: 47.397742,
            longitude_deg: 8.545594,
            altitude_m: 0.005,
        });

        let plan_a = MissionPlan {
            mission_type: MissionType::Mission,
            home: home_a,
            items: Vec::new(),
        };
        let plan_b = MissionPlan {
            mission_type: MissionType::Mission,
            home: home_b,
            items: Vec::new(),
        };

        assert!(plans_equivalent(
            &plan_a,
            &plan_b,
            CompareTolerance::default()
        ));
    }
}
