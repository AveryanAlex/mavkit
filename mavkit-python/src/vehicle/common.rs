pub(crate) fn vehicle_label(vehicle: &mavkit::Vehicle) -> String {
    let identity = vehicle.identity();
    format!("sys={}, comp={}", identity.system_id, identity.component_id)
}
