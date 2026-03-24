import mavkit


vehicle_type: type[mavkit.Vehicle] = mavkit.Vehicle
mission_handle_type: type[mavkit.MissionHandle] = mavkit.MissionHandle
mission_item_type: type[mavkit.MissionItem] = mavkit.MissionItem
mission_issue_type: type[mavkit.MissionIssue] = mavkit.MissionIssue
validation_error_type: type[mavkit.ValidationError] = mavkit.ValidationError
status_text_event_type: type[mavkit.StatusTextEvent] = mavkit.StatusTextEvent

mission_plan = mavkit.MissionPlan(items=[])
mission_issues: list[mavkit.MissionIssue] = mavkit.validate_plan(mission_plan)
normalized_plan: mavkit.MissionPlan = mavkit.normalize_for_compare(mission_plan)
upload_items: list[mavkit.MissionItem] = mavkit.mission_items_for_upload(mission_plan)
plans_match: bool = mavkit.plans_equivalent(normalized_plan, normalized_plan)
