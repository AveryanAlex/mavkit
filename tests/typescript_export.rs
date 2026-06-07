#![cfg(feature = "typescript")]

use mavkit::{
    CurrentMode, FenceState, FirmwareInfo, GlobalPosition, LoiterDirection, MissionCommand,
    MissionPlan, ParamState, RallyState, RetryPolicy, StatusTextEvent, SupportState,
    TelemetryMessageKind, TransferEvent, UniqueIds, VehicleIdentity, VehicleTimestamp,
};
use specta::Types;
use specta_typescript::Typescript;

#[test]
fn exported_typescript_uses_serde_renamed_enum_values() {
    let types = Types::default()
        .register::<LoiterDirection>()
        .register::<MissionPlan>()
        .register::<MissionCommand>()
        .register::<ParamState>()
        .register::<FenceState>()
        .register::<RallyState>()
        .register::<GlobalPosition>()
        .register::<StatusTextEvent>()
        .register::<TelemetryMessageKind>()
        .register::<VehicleTimestamp>()
        .register::<CurrentMode>()
        .register::<FirmwareInfo>()
        .register::<UniqueIds>()
        .register::<RetryPolicy>()
        .register::<TransferEvent>()
        .register::<SupportState>()
        .register::<VehicleIdentity>();
    let ts = Typescript::default()
        .export(&types, specta_serde::Format)
        .expect("registered MAVKit types should export to TypeScript");

    assert!(
        ts.contains("export type LoiterDirection = \"clockwise\" | \"counter_clockwise\";"),
        "exported TypeScript did not use serde snake_case values:\n{ts}"
    );
    assert!(
        !ts.contains("\"CounterClockwise\"") && !ts.contains("\"Clockwise\""),
        "exported TypeScript leaked Rust variant casing:\n{ts}"
    );
}
