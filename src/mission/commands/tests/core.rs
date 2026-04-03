use super::super::*;
use crate::geo::{GeoPoint3d, GeoPoint3dMsl, GeoPoint3dRelHome, GeoPoint3dTerrain};
use crate::mission::types::MissionFrame as MissionItemFrame;
use serde::ser::{Error as _, Impossible, Serializer};
use std::fmt;

#[derive(Debug)]
struct TestSerError(String);

impl serde::ser::Error for TestSerError {
    fn custom<T: fmt::Display>(msg: T) -> Self {
        Self(msg.to_string())
    }
}

impl fmt::Display for TestSerError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for TestSerError {}

struct UnitVariantNameSerializer;

impl Serializer for UnitVariantNameSerializer {
    type Ok = String;
    type Error = TestSerError;
    type SerializeSeq = Impossible<String, TestSerError>;
    type SerializeTuple = Impossible<String, TestSerError>;
    type SerializeTupleStruct = Impossible<String, TestSerError>;
    type SerializeTupleVariant = Impossible<String, TestSerError>;
    type SerializeMap = Impossible<String, TestSerError>;
    type SerializeStruct = Impossible<String, TestSerError>;
    type SerializeStructVariant = Impossible<String, TestSerError>;

    fn serialize_unit_variant(
        self,
        _name: &'static str,
        _variant_index: u32,
        variant: &'static str,
    ) -> Result<Self::Ok, Self::Error> {
        Ok(variant.to_owned())
    }

    fn serialize_bool(self, _v: bool) -> Result<Self::Ok, Self::Error> {
        Err(TestSerError::custom("unsupported"))
    }
    fn serialize_i8(self, _v: i8) -> Result<Self::Ok, Self::Error> {
        Err(TestSerError::custom("unsupported"))
    }
    fn serialize_i16(self, _v: i16) -> Result<Self::Ok, Self::Error> {
        Err(TestSerError::custom("unsupported"))
    }
    fn serialize_i32(self, _v: i32) -> Result<Self::Ok, Self::Error> {
        Err(TestSerError::custom("unsupported"))
    }
    fn serialize_i64(self, _v: i64) -> Result<Self::Ok, Self::Error> {
        Err(TestSerError::custom("unsupported"))
    }
    fn serialize_u8(self, _v: u8) -> Result<Self::Ok, Self::Error> {
        Err(TestSerError::custom("unsupported"))
    }
    fn serialize_u16(self, _v: u16) -> Result<Self::Ok, Self::Error> {
        Err(TestSerError::custom("unsupported"))
    }
    fn serialize_u32(self, _v: u32) -> Result<Self::Ok, Self::Error> {
        Err(TestSerError::custom("unsupported"))
    }
    fn serialize_u64(self, _v: u64) -> Result<Self::Ok, Self::Error> {
        Err(TestSerError::custom("unsupported"))
    }
    fn serialize_f32(self, _v: f32) -> Result<Self::Ok, Self::Error> {
        Err(TestSerError::custom("unsupported"))
    }
    fn serialize_f64(self, _v: f64) -> Result<Self::Ok, Self::Error> {
        Err(TestSerError::custom("unsupported"))
    }
    fn serialize_char(self, _v: char) -> Result<Self::Ok, Self::Error> {
        Err(TestSerError::custom("unsupported"))
    }
    fn serialize_str(self, _v: &str) -> Result<Self::Ok, Self::Error> {
        Err(TestSerError::custom("unsupported"))
    }
    fn serialize_bytes(self, _v: &[u8]) -> Result<Self::Ok, Self::Error> {
        Err(TestSerError::custom("unsupported"))
    }
    fn serialize_none(self) -> Result<Self::Ok, Self::Error> {
        Err(TestSerError::custom("unsupported"))
    }
    fn serialize_some<T: ?Sized + serde::Serialize>(
        self,
        _value: &T,
    ) -> Result<Self::Ok, Self::Error> {
        Err(TestSerError::custom("unsupported"))
    }
    fn serialize_unit(self) -> Result<Self::Ok, Self::Error> {
        Err(TestSerError::custom("unsupported"))
    }
    fn serialize_unit_struct(self, _name: &'static str) -> Result<Self::Ok, Self::Error> {
        Err(TestSerError::custom("unsupported"))
    }
    fn serialize_newtype_struct<T: ?Sized + serde::Serialize>(
        self,
        _name: &'static str,
        _value: &T,
    ) -> Result<Self::Ok, Self::Error> {
        Err(TestSerError::custom("unsupported"))
    }
    fn serialize_newtype_variant<T: ?Sized + serde::Serialize>(
        self,
        _name: &'static str,
        _variant_index: u32,
        _variant: &'static str,
        _value: &T,
    ) -> Result<Self::Ok, Self::Error> {
        Err(TestSerError::custom("unsupported"))
    }
    fn serialize_seq(self, _len: Option<usize>) -> Result<Self::SerializeSeq, Self::Error> {
        Err(TestSerError::custom("unsupported"))
    }
    fn serialize_tuple(self, _len: usize) -> Result<Self::SerializeTuple, Self::Error> {
        Err(TestSerError::custom("unsupported"))
    }
    fn serialize_tuple_struct(
        self,
        _name: &'static str,
        _len: usize,
    ) -> Result<Self::SerializeTupleStruct, Self::Error> {
        Err(TestSerError::custom("unsupported"))
    }
    fn serialize_tuple_variant(
        self,
        _name: &'static str,
        _variant_index: u32,
        _variant: &'static str,
        _len: usize,
    ) -> Result<Self::SerializeTupleVariant, Self::Error> {
        Err(TestSerError::custom("unsupported"))
    }
    fn serialize_map(self, _len: Option<usize>) -> Result<Self::SerializeMap, Self::Error> {
        Err(TestSerError::custom("unsupported"))
    }
    fn serialize_struct(
        self,
        _name: &'static str,
        _len: usize,
    ) -> Result<Self::SerializeStruct, Self::Error> {
        Err(TestSerError::custom("unsupported"))
    }
    fn serialize_struct_variant(
        self,
        _name: &'static str,
        _variant_index: u32,
        _variant: &'static str,
        _len: usize,
    ) -> Result<Self::SerializeStructVariant, Self::Error> {
        Err(TestSerError::custom("unsupported"))
    }
}

pub(super) fn assert_unit_enum_serde<T>(value: T, wire_name: &str)
where
    T: serde::Serialize + for<'de> serde::Deserialize<'de> + PartialEq + fmt::Debug,
{
    let serialized = value
        .serialize(UnitVariantNameSerializer)
        .expect("serialize unit enum");
    assert_eq!(serialized, wire_name);

    let decoded: T = serde::Deserialize::deserialize(serde::de::value::StrDeserializer::<
        serde::de::value::Error,
    >::new(wire_name))
    .expect("deserialize unit enum");
    assert_eq!(decoded, value);
}

pub(super) fn geo_msl(latitude_e7: i32, longitude_e7: i32, altitude_msl_m: f64) -> GeoPoint3d {
    GeoPoint3d::Msl(GeoPoint3dMsl {
        latitude_deg: f64::from(latitude_e7) / 1e7,
        longitude_deg: f64::from(longitude_e7) / 1e7,
        altitude_msl_m,
    })
}

pub(super) fn geo_rel_home(latitude_e7: i32, longitude_e7: i32, relative_alt_m: f64) -> GeoPoint3d {
    GeoPoint3d::RelHome(GeoPoint3dRelHome {
        latitude_deg: f64::from(latitude_e7) / 1e7,
        longitude_deg: f64::from(longitude_e7) / 1e7,
        relative_alt_m,
    })
}

pub(super) fn geo_terrain(
    latitude_e7: i32,
    longitude_e7: i32,
    altitude_terrain_m: f64,
) -> GeoPoint3d {
    GeoPoint3d::Terrain(GeoPoint3dTerrain {
        latitude_deg: f64::from(latitude_e7) / 1e7,
        longitude_deg: f64::from(longitude_e7) / 1e7,
        altitude_terrain_m,
    })
}

pub(super) fn assert_roundtrip(
    original: MissionCommand,
    expected_wire: (u16, MissionFrame, [f32; 4], i32, i32, f32),
) {
    let encoded = original.clone().into_wire();
    assert_eq!(encoded, expected_wire);

    let decoded = MissionCommand::from_wire(
        expected_wire.0,
        expected_wire.1,
        expected_wire.2,
        expected_wire.3,
        expected_wire.4,
        expected_wire.5,
    );
    assert_eq!(decoded, original);
}

#[test]
fn from_chain() {
    let waypoint = NavWaypoint {
        position: geo_rel_home(473_977_420, 85_455_970, 42.0),
        hold_time_s: 5.0,
        acceptance_radius_m: 2.0,
        pass_radius_m: 0.0,
        yaw_deg: 180.0,
    };

    let nav_command: NavCommand = waypoint.clone().into();
    assert_eq!(nav_command, NavCommand::Waypoint(waypoint.clone()));

    let mission_command_from_sub_enum: MissionCommand = nav_command.into();
    assert_eq!(
        mission_command_from_sub_enum,
        MissionCommand::Nav(NavCommand::Waypoint(waypoint.clone()))
    );

    let mission_command_from_struct: MissionCommand = waypoint.clone().into();
    assert_eq!(mission_command_from_struct, mission_command_from_sub_enum);

    let mission_item: MissionItem = waypoint.into();
    let (command, frame, params, x, y, z) = mission_item.command.clone().into_wire();
    assert_eq!(command, u16::from(MAV_CMD_NAV_WAYPOINT));
    assert_eq!(
        MissionItemFrame::from(frame),
        MissionItemFrame::GlobalRelativeAltInt
    );
    assert!(mission_item.autocontinue);
    assert_eq!(params[0], 5.0);
    assert_eq!(params[1], 2.0);
    assert_eq!(params[2], 0.0);
    assert_eq!(params[3], 180.0);
    assert_eq!(x, 473_977_420);
    assert_eq!(y, 85_455_970);
    assert_eq!(z, 42.0);
}

#[test]
fn enum_serde() {
    assert_unit_enum_serde(LoiterDirection::Clockwise, "clockwise");
    assert_unit_enum_serde(LoiterDirection::CounterClockwise, "counter_clockwise");
    assert_unit_enum_serde(YawDirection::Clockwise, "clockwise");
    assert_unit_enum_serde(YawDirection::CounterClockwise, "counter_clockwise");
    assert_unit_enum_serde(SpeedType::Airspeed, "airspeed");
    assert_unit_enum_serde(SpeedType::Groundspeed, "groundspeed");
    assert_unit_enum_serde(AltChangeAction::Neutral, "neutral");
    assert_unit_enum_serde(AltChangeAction::Climb, "climb");
    assert_unit_enum_serde(AltChangeAction::Descend, "descend");
    assert_unit_enum_serde(FenceAction::Disable, "disable");
    assert_unit_enum_serde(FenceAction::Enable, "enable");
    assert_unit_enum_serde(FenceAction::DisableFloor, "disable_floor");
    assert_unit_enum_serde(ParachuteAction::Disable, "disable");
    assert_unit_enum_serde(ParachuteAction::Enable, "enable");
    assert_unit_enum_serde(ParachuteAction::Release, "release");
    assert_unit_enum_serde(GripperAction::Release, "release");
    assert_unit_enum_serde(GripperAction::Grab, "grab");
    assert_unit_enum_serde(WinchAction::Relax, "relax");
    assert_unit_enum_serde(WinchAction::LengthControl, "length_control");
    assert_unit_enum_serde(WinchAction::RateControl, "rate_control");
}

#[test]
fn other_roundtrip_preserves_raw_wire_fields() {
    let raw = RawMissionCommand {
        command: 31_337,
        frame: MissionFrame::Other(77),
        param1: 1.0,
        param2: -2.0,
        param3: 3.5,
        param4: -4.25,
        x: -17,
        y: 28,
        z: 900.5,
    };

    let (command, frame, params, x, y, z) = MissionCommand::Other(raw).into_wire();
    let decoded = MissionCommand::from_wire(command, frame, params, x, y, z);
    assert_eq!(decoded, MissionCommand::Other(raw));
}

#[test]
fn known_command_roundtrip_uses_generated_wire_arms() {
    let original = MissionCommand::from(NavWaypoint {
        position: geo_msl(10, 20, 30.0),
        hold_time_s: 1.0,
        acceptance_radius_m: 2.0,
        pass_radius_m: 3.0,
        yaw_deg: 4.0,
    });

    let (command, frame, params, x, y, z) = original.clone().into_wire();
    let decoded = MissionCommand::from_wire(command, frame, params, x, y, z);
    assert_eq!(decoded, original);
}
