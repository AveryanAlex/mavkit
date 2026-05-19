use std::collections::{HashMap, HashSet};
use std::sync::LazyLock;

use serde::Deserialize;

use crate::dialect;
use crate::error::VehicleError;

use super::api::DemoProfile;
use super::state::{DEFAULT_SYSTEM_ID, SimulatorCore};

const COPTER_PARAM_FIXTURE_JSON: &str = include_str!("fixtures/params/copter-defaults.json");
const PLANE_PARAM_FIXTURE_JSON: &str = include_str!("fixtures/params/plane-defaults.json");
const QUADPLANE_PARAM_FIXTURE_JSON: &str = include_str!("fixtures/params/quadplane-defaults.json");
const FIXTURE_SCHEMA_VERSION: u32 = 1;
const FIXTURE_SOURCE_KIND: &str = "sitl_param_download";
const REQUIRED_COMMON_PARAMS: &[&str] =
    &["SYSID_THISMAV", "SR0_EXTRA1", "SR0_EXTRA2", "SR0_EXTRA3"];
const REQUIRED_COPTER_PARAMS: &[&str] = &[
    "RTL_ALT",
    "RTL_ALT_FINAL",
    "RTL_CLIMB_MIN",
    "RTL_SPEED",
    "RTL_LOIT_TIME",
    "FS_THR_ENABLE",
    "FS_GCS_ENABLE",
    "FS_EKF_ACTION",
    "FS_CRASH_CHECK",
    "BATT_FS_LOW_ACT",
    "BATT_FS_CRT_ACT",
];
const REQUIRED_PLANE_PARAMS: &[&str] = &[
    "AIRSPEED_CRUISE",
    "RTL_ALTITUDE",
    "RTL_AUTOLAND",
    "FS_LONG_ACTN",
    "FS_SHORT_ACTN",
    "BATT_FS_LOW_ACT",
    "BATT_FS_CRT_ACT",
    "LAND_FLARE_SEC",
];
const REQUIRED_QUADPLANE_PARAMS: &[&str] = &[
    "AIRSPEED_CRUISE",
    "RTL_ALTITUDE",
    "RTL_AUTOLAND",
    "Q_ENABLE",
    "Q_FRAME_CLASS",
    "Q_FRAME_TYPE",
    "Q_GUIDED_MODE",
    "Q_OPTIONS",
    "Q_RTL_ALT",
    "Q_WP_SPEED",
];

static PARAM_FIXTURES: LazyLock<FixtureCatalog> =
    LazyLock::new(|| FixtureCatalog::load().expect("embedded sim param fixture must be valid"));

#[derive(Debug, Clone, PartialEq)]
pub(crate) struct SimParam {
    pub(crate) name: String,
    pub(crate) value: f32,
    pub(crate) param_type: dialect::MavParamType,
}

#[derive(Debug)]
struct FixtureCatalog {
    profiles: HashMap<String, Vec<SimParam>>,
}

#[derive(Debug, Deserialize)]
struct FixtureFile {
    schema_version: u32,
    vehicle_family: String,
    vehicle_preset: String,
    source: FixtureSource,
    params: Vec<FixtureParam>,
}

#[derive(Debug, Deserialize)]
struct FixtureSource {
    kind: String,
    autopilot: String,
    sitl_image: String,
    defaults: String,
    generated_at: String,
}

#[derive(Debug, Deserialize)]
struct FixtureParam {
    name: String,
    value: f32,
    param_type: FixtureParamType,
}

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(rename_all = "snake_case")]
enum FixtureParamType {
    Uint8,
    Int8,
    Uint16,
    Int16,
    Uint32,
    Int32,
    Real32,
}

impl FixtureCatalog {
    fn load() -> Result<Self, String> {
        let fixtures = [
            (DemoProfile::ArduCopter, COPTER_PARAM_FIXTURE_JSON),
            (DemoProfile::ArduPlane, PLANE_PARAM_FIXTURE_JSON),
            (DemoProfile::ArduQuadPlane, QUADPLANE_PARAM_FIXTURE_JSON),
        ];

        let mut profiles = HashMap::with_capacity(fixtures.len());
        for (profile, fixture_json) in fixtures {
            let key = profile_key(profile);
            let params = load_profile_fixture(profile, fixture_json)?;

            if profiles.insert(key.to_string(), params).is_some() {
                return Err(format!("duplicate sim param profile fixture {key}"));
            }
        }

        for profile in [
            DemoProfile::ArduCopter,
            DemoProfile::ArduPlane,
            DemoProfile::ArduQuadPlane,
        ] {
            let key = profile_key(profile);
            if !profiles.contains_key(key) {
                return Err(format!("missing sim param fixture for profile {key}"));
            }
        }

        Ok(Self { profiles })
    }

    fn params(&self, profile: DemoProfile) -> &[SimParam] {
        self.profiles
            .get(profile_key(profile))
            .map(Vec::as_slice)
            .expect("validated fixture profile must exist")
    }
}

fn load_profile_fixture(profile: DemoProfile, fixture_json: &str) -> Result<Vec<SimParam>, String> {
    let file: FixtureFile = serde_json::from_str(fixture_json)
        .map_err(|err| format!("failed to parse sim param fixture: {err}"))?;

    validate_fixture_metadata(profile, &file)?;

    let key = profile_key(profile);
    let params: Vec<SimParam> = file
        .params
        .into_iter()
        .map(|param| SimParam {
            name: param.name,
            value: param.value,
            param_type: param.param_type.into_mav_param_type(),
        })
        .collect();
    validate_profile_fixture(key, &params)?;
    validate_profile_aliases(profile, &params)?;

    Ok(params)
}

fn validate_fixture_metadata(profile: DemoProfile, file: &FixtureFile) -> Result<(), String> {
    if file.schema_version != FIXTURE_SCHEMA_VERSION {
        return Err(format!(
            "unexpected sim param fixture schema version {} for {}",
            file.schema_version,
            profile_key(profile)
        ));
    }

    let expected = expected_fixture_metadata(profile);
    validate_non_empty("vehicle_family", &file.vehicle_family)?;
    validate_non_empty("vehicle_preset", &file.vehicle_preset)?;
    validate_non_empty("source.kind", &file.source.kind)?;
    validate_non_empty("source.autopilot", &file.source.autopilot)?;
    validate_non_empty("source.sitl_image", &file.source.sitl_image)?;
    validate_non_empty("source.defaults", &file.source.defaults)?;
    validate_non_empty("source.generated_at", &file.source.generated_at)?;

    if file.vehicle_family != expected.vehicle_family {
        return Err(format!(
            "sim param fixture {} has vehicle_family {}, expected {}",
            profile_key(profile),
            file.vehicle_family,
            expected.vehicle_family
        ));
    }
    if file.vehicle_preset != expected.vehicle_preset {
        return Err(format!(
            "sim param fixture {} has vehicle_preset {}, expected {}",
            profile_key(profile),
            file.vehicle_preset,
            expected.vehicle_preset
        ));
    }
    if file.source.kind != FIXTURE_SOURCE_KIND {
        return Err(format!(
            "sim param fixture {} has source.kind {}, expected {}",
            profile_key(profile),
            file.source.kind,
            FIXTURE_SOURCE_KIND
        ));
    }
    if file.source.autopilot != expected.autopilot {
        return Err(format!(
            "sim param fixture {} has source.autopilot {}, expected {}",
            profile_key(profile),
            file.source.autopilot,
            expected.autopilot
        ));
    }

    Ok(())
}

fn validate_non_empty(field: &str, value: &str) -> Result<(), String> {
    if value.trim().is_empty() {
        return Err(format!("sim param fixture {field} must not be empty"));
    }

    Ok(())
}

#[derive(Debug, Clone, Copy)]
struct ExpectedFixtureMetadata {
    vehicle_family: &'static str,
    vehicle_preset: &'static str,
    autopilot: &'static str,
}

fn expected_fixture_metadata(profile: DemoProfile) -> ExpectedFixtureMetadata {
    match profile {
        DemoProfile::ArduCopter => ExpectedFixtureMetadata {
            vehicle_family: "copter",
            vehicle_preset: "quadcopter",
            autopilot: "ArduCopter",
        },
        DemoProfile::ArduPlane => ExpectedFixtureMetadata {
            vehicle_family: "plane",
            vehicle_preset: "airplane",
            autopilot: "ArduPlane",
        },
        DemoProfile::ArduQuadPlane => ExpectedFixtureMetadata {
            vehicle_family: "plane",
            vehicle_preset: "quadplane",
            autopilot: "ArduPlane",
        },
    }
}

impl FixtureParamType {
    fn into_mav_param_type(self) -> dialect::MavParamType {
        match self {
            Self::Uint8 => dialect::MavParamType::MAV_PARAM_TYPE_UINT8,
            Self::Int8 => dialect::MavParamType::MAV_PARAM_TYPE_INT8,
            Self::Uint16 => dialect::MavParamType::MAV_PARAM_TYPE_UINT16,
            Self::Int16 => dialect::MavParamType::MAV_PARAM_TYPE_INT16,
            Self::Uint32 => dialect::MavParamType::MAV_PARAM_TYPE_UINT32,
            Self::Int32 => dialect::MavParamType::MAV_PARAM_TYPE_INT32,
            Self::Real32 => dialect::MavParamType::MAV_PARAM_TYPE_REAL32,
        }
    }
}

fn profile_key(profile: DemoProfile) -> &'static str {
    match profile {
        DemoProfile::ArduCopter => "ArduCopter",
        DemoProfile::ArduPlane => "ArduPlane",
        DemoProfile::ArduQuadPlane => "ArduQuadPlane",
    }
}

fn validate_profile_fixture(profile: &str, params: &[SimParam]) -> Result<(), String> {
    if params.is_empty() {
        return Err(format!("sim param fixture {profile} must not be empty"));
    }

    let mut seen = HashSet::with_capacity(params.len());
    for param in params {
        if param.name.is_empty() {
            return Err(format!(
                "sim param fixture {profile} contains an empty name"
            ));
        }
        if param.name.len() > 16 {
            return Err(format!(
                "sim param fixture {profile} contains overlong param name {}",
                param.name
            ));
        }
        if !param.value.is_finite() {
            return Err(format!(
                "sim param fixture {profile} contains non-finite value for {}",
                param.name
            ));
        }
        if !seen.insert(param.name.as_str()) {
            return Err(format!(
                "sim param fixture {profile} contains duplicate param {}",
                param.name
            ));
        }
    }

    for required in REQUIRED_COMMON_PARAMS {
        if !seen.contains(required) {
            return Err(format!("sim param fixture {profile} is missing {required}"));
        }
    }

    let profile_specific_required = match profile {
        "ArduCopter" => REQUIRED_COPTER_PARAMS,
        "ArduPlane" => REQUIRED_PLANE_PARAMS,
        "ArduQuadPlane" => REQUIRED_QUADPLANE_PARAMS,
        _ => return Err(format!("unknown sim param fixture profile {profile}")),
    };

    for required in profile_specific_required {
        if !seen.contains(required) {
            return Err(format!("sim param fixture {profile} is missing {required}"));
        }
    }

    Ok(())
}

#[derive(Debug, Clone, Copy)]
struct ParamAlias {
    alias: &'static str,
    canonical: &'static str,
}

fn profile_aliases(profile: DemoProfile) -> &'static [ParamAlias] {
    match profile {
        DemoProfile::ArduCopter => &[],
        DemoProfile::ArduPlane | DemoProfile::ArduQuadPlane => &[ParamAlias {
            alias: "ALT_HOLD_RTL",
            canonical: "RTL_ALTITUDE",
        }],
    }
}

fn validate_profile_aliases(profile: DemoProfile, params: &[SimParam]) -> Result<(), String> {
    for alias in profile_aliases(profile) {
        if param_index_by_exact_name(params, alias.alias).is_some() {
            return Err(format!(
                "sim param fixture {} contains compatibility alias {} as a canonical param",
                profile_key(profile),
                alias.alias
            ));
        }
        if param_index_by_exact_name(params, alias.canonical).is_none() {
            return Err(format!(
                "sim param fixture {} compatibility alias {} is missing canonical {}",
                profile_key(profile),
                alias.alias,
                alias.canonical
            ));
        }
    }

    Ok(())
}

fn alias_canonical_name(name: &str) -> Option<&'static str> {
    match name {
        "ALT_HOLD_RTL" => Some("RTL_ALTITUDE"),
        _ => None,
    }
}

fn param_count(params: &[SimParam]) -> u16 {
    u16::try_from(params.len()).expect("sim param fixture count must fit in MAVLink param_count")
}

fn param_value_message(params: &[SimParam], index: usize) -> dialect::PARAM_VALUE_DATA {
    let param = &params[index];
    dialect::PARAM_VALUE_DATA {
        param_id: param.name.as_str().into(),
        param_value: param.value,
        param_type: param.param_type,
        param_count: param_count(params),
        param_index: index as u16,
    }
}

fn param_index_by_name(params: &[SimParam], name: &str) -> Option<usize> {
    param_index_by_exact_name(params, name).or_else(|| {
        alias_canonical_name(name).and_then(|name| param_index_by_exact_name(params, name))
    })
}

fn param_index_by_exact_name(params: &[SimParam], name: &str) -> Option<usize> {
    params.iter().position(|param| param.name == name)
}

fn set_param_value_by_name(params: &mut [SimParam], name: &str, value: f32) -> Option<usize> {
    let index = param_index_by_name(params, name)?;
    params[index].value = value;
    Some(index)
}

fn param_index_from_read_request(data: &dialect::PARAM_REQUEST_READ_DATA) -> Option<usize> {
    if let Ok(name) = data.param_id.to_str()
        && !name.is_empty()
    {
        return None;
    }

    usize::try_from(data.param_index).ok()
}

pub(crate) fn seeded_params(profile: DemoProfile) -> Vec<SimParam> {
    let mut params = PARAM_FIXTURES.params(profile).to_vec();
    if let Some(index) = param_index_by_name(&params, "SYSID_THISMAV") {
        params[index].value = f32::from(DEFAULT_SYSTEM_ID);
    }
    params
}

impl SimulatorCore {
    pub(crate) async fn handle_param_request_list(
        &mut self,
        data: dialect::PARAM_REQUEST_LIST_DATA,
    ) -> Result<(), VehicleError> {
        if !Self::targets_this_vehicle(data.target_system, data.target_component) {
            return Ok(());
        }

        let param_values: Vec<_> = (0..self.params.len())
            .map(|index| param_value_message(&self.params, index))
            .collect();

        for param_value in param_values {
            self.send_message(dialect::MavMessage::PARAM_VALUE(param_value))
                .await?;
        }
        Ok(())
    }

    pub(crate) async fn handle_param_request_read(
        &mut self,
        data: dialect::PARAM_REQUEST_READ_DATA,
    ) -> Result<(), VehicleError> {
        if !Self::targets_this_vehicle(data.target_system, data.target_component) {
            return Ok(());
        }

        let selected = match data.param_id.to_str() {
            Ok(requested_name) if !requested_name.is_empty() => {
                param_index_by_name(&self.params, requested_name)
            }
            _ => param_index_from_read_request(&data),
        };

        if let Some(index) = selected.filter(|index| *index < self.params.len()) {
            self.send_message(dialect::MavMessage::PARAM_VALUE(param_value_message(
                &self.params,
                index,
            )))
            .await?;
        }

        Ok(())
    }

    pub(crate) async fn handle_param_set(
        &mut self,
        data: dialect::PARAM_SET_DATA,
    ) -> Result<(), VehicleError> {
        if !Self::targets_this_vehicle(data.target_system, data.target_component) {
            return Ok(());
        }

        let requested_name = data.param_id.to_str().unwrap_or("");
        if let Some(index) =
            set_param_value_by_name(&mut self.params, requested_name, data.param_value)
        {
            self.send_message(dialect::MavMessage::PARAM_VALUE(param_value_message(
                &self.params,
                index,
            )))
            .await?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use mavlink::MavHeader;
    use tokio::sync::{mpsc, watch};

    use super::*;
    use crate::mission::HomePosition;
    use crate::sim::state::{DEFAULT_COMPONENT_ID, DemoVehicleConfig, default_mode};
    use crate::sim::{DemoClock, DemoVehicleSnapshot};

    fn sim_core() -> (
        SimulatorCore,
        mpsc::Receiver<(MavHeader, dialect::MavMessage)>,
    ) {
        let home = HomePosition {
            latitude_deg: 42.0,
            longitude_deg: -71.0,
            altitude_m: 100.0,
        };
        let snapshot = DemoVehicleSnapshot {
            time_boot_ms: 0,
            armed: false,
            custom_mode: default_mode(DemoProfile::ArduCopter),
            home: home.clone(),
            latitude_deg: home.latitude_deg,
            longitude_deg: home.longitude_deg,
            altitude_msl_m: home.altitude_m,
            relative_alt_m: 0.0,
            roll_rad: 0.0,
            pitch_rad: 0.0,
            yaw_rad: 0.0,
            mission_current_wire_seq: 0,
            mission_total_wire_items: 0,
        };
        let (outbound_tx, outbound_rx) = mpsc::channel(8);
        let (snapshot_tx, _snapshot_rx) = watch::channel(snapshot.clone());

        (
            SimulatorCore::new(
                DemoVehicleConfig {
                    profile: DemoProfile::ArduCopter,
                    clock: DemoClock::Manual,
                    tick_hz: 1,
                    home,
                },
                outbound_tx,
                snapshot_tx,
                snapshot,
            ),
            outbound_rx,
        )
    }

    fn param_set(target_system: u8, value: f32) -> dialect::PARAM_SET_DATA {
        dialect::PARAM_SET_DATA {
            target_system,
            target_component: DEFAULT_COMPONENT_ID,
            param_id: "SYSID_THISMAV".into(),
            param_value: value,
            param_type: dialect::MavParamType::MAV_PARAM_TYPE_REAL32,
        }
    }

    fn param_names(params: &[SimParam]) -> Vec<&str> {
        params.iter().map(|param| param.name.as_str()).collect()
    }

    fn raw_fixture(profile: DemoProfile) -> &'static str {
        match profile {
            DemoProfile::ArduCopter => COPTER_PARAM_FIXTURE_JSON,
            DemoProfile::ArduPlane => PLANE_PARAM_FIXTURE_JSON,
            DemoProfile::ArduQuadPlane => QUADPLANE_PARAM_FIXTURE_JSON,
        }
    }

    #[test]
    fn fixture_schema_metadata_validation_accepts_embedded_profiles() {
        for profile in [
            DemoProfile::ArduCopter,
            DemoProfile::ArduPlane,
            DemoProfile::ArduQuadPlane,
        ] {
            let fixture_file: FixtureFile =
                serde_json::from_str(raw_fixture(profile)).expect("fixture file should parse");
            let expected = expected_fixture_metadata(profile);

            validate_fixture_metadata(profile, &fixture_file)
                .expect("fixture metadata should validate");
            assert_eq!(fixture_file.schema_version, FIXTURE_SCHEMA_VERSION);
            assert_eq!(fixture_file.vehicle_family, expected.vehicle_family);
            assert_eq!(fixture_file.vehicle_preset, expected.vehicle_preset);
            assert_eq!(fixture_file.source.kind, FIXTURE_SOURCE_KIND);
            assert_eq!(fixture_file.source.autopilot, expected.autopilot);
            assert!(fixture_file.params.len() > 1_000);
            assert!(!PARAM_FIXTURES.params(profile).is_empty());
        }
    }

    #[test]
    fn fixture_schema_metadata_validation_rejects_mismatches() {
        let mut fixture_file: FixtureFile =
            serde_json::from_str(PLANE_PARAM_FIXTURE_JSON).expect("fixture file should parse");

        fixture_file.schema_version = 99;
        assert!(validate_fixture_metadata(DemoProfile::ArduPlane, &fixture_file).is_err());

        fixture_file.schema_version = FIXTURE_SCHEMA_VERSION;
        fixture_file.vehicle_preset = "quadplane".to_string();
        assert!(validate_fixture_metadata(DemoProfile::ArduPlane, &fixture_file).is_err());

        fixture_file.vehicle_preset = "airplane".to_string();
        fixture_file.source.kind = "manual".to_string();
        assert!(validate_fixture_metadata(DemoProfile::ArduPlane, &fixture_file).is_err());
    }

    #[test]
    fn fixture_validation_locks_duplicates_lengths_and_required_params() {
        for profile in [
            DemoProfile::ArduCopter,
            DemoProfile::ArduPlane,
            DemoProfile::ArduQuadPlane,
        ] {
            validate_profile_fixture(profile_key(profile), PARAM_FIXTURES.params(profile))
                .expect("fixture profile should validate");
        }
    }

    #[test]
    fn by_name_and_by_index_access_is_stable() {
        let params = seeded_params(DemoProfile::ArduCopter);
        let sysid_index = param_index_by_name(&params, "SYSID_THISMAV").expect("SYSID_THISMAV");

        assert_eq!(param_names(&params)[0], "ACRO_BAL_PITCH");
        assert_eq!(
            param_value_message(&params, sysid_index).param_index,
            sysid_index as u16
        );
        assert_eq!(
            param_value_message(&params, sysid_index).param_count,
            param_count(&params)
        );
        assert_eq!(
            param_value_message(&params, sysid_index)
                .param_id
                .to_str()
                .expect("valid name"),
            "SYSID_THISMAV"
        );
        assert_eq!(
            param_index_from_read_request(&dialect::PARAM_REQUEST_READ_DATA {
                target_system: 1,
                target_component: 1,
                param_id: "".into(),
                param_index: sysid_index as i16,
            }),
            Some(sysid_index)
        );
    }

    #[test]
    fn update_semantics_preserve_type_index_and_count() {
        let mut params = seeded_params(DemoProfile::ArduPlane);
        let index = param_index_by_name(&params, "SR0_EXTRA1").expect("SR0_EXTRA1");
        let original_type = params[index].param_type;
        let original_count = param_count(&params);

        params[index].value = 9.0;
        let response = param_value_message(&params, index);

        assert_eq!(response.param_type, original_type);
        assert_eq!(response.param_index, index as u16);
        assert_eq!(response.param_count, original_count);
        assert_eq!(response.param_value, 9.0);
    }

    #[test]
    fn compatibility_alias_read_resolves_to_canonical_param_without_changing_count() {
        for profile in [DemoProfile::ArduPlane, DemoProfile::ArduQuadPlane] {
            let params = seeded_params(profile);
            let canonical_index =
                param_index_by_name(&params, "RTL_ALTITUDE").expect("RTL_ALTITUDE");
            let alias_index = param_index_by_name(&params, "ALT_HOLD_RTL").expect("ALT_HOLD_RTL");
            let response = param_value_message(&params, alias_index);

            assert_eq!(alias_index, canonical_index);
            assert_eq!(response.param_count, param_count(&params));
            assert_eq!(response.param_index, canonical_index as u16);
            assert_eq!(
                response.param_id.to_str().expect("valid name"),
                "RTL_ALTITUDE"
            );
            assert!(!param_names(&params).contains(&"ALT_HOLD_RTL"));
        }
    }

    #[test]
    fn compatibility_alias_set_updates_canonical_storage() {
        let mut params = seeded_params(DemoProfile::ArduPlane);
        let canonical_index = param_index_by_name(&params, "RTL_ALTITUDE").expect("RTL_ALTITUDE");
        let updated_index = set_param_value_by_name(&mut params, "ALT_HOLD_RTL", 12_345.0)
            .expect("alias should resolve");

        assert_eq!(updated_index, canonical_index);
        assert_eq!(params[canonical_index].name, "RTL_ALTITUDE");
        assert_eq!(params[canonical_index].value, 12_345.0);
    }

    #[tokio::test]
    async fn param_set_for_wrong_target_is_ignored_without_response_or_state_change() {
        let (mut sim, mut outbound_rx) = sim_core();
        let index = param_index_by_name(&sim.params, "SYSID_THISMAV").expect("SYSID_THISMAV");
        let original_value = sim.params[index].value;

        sim.handle_param_set(param_set(DEFAULT_SYSTEM_ID + 1, 42.0))
            .await
            .unwrap();

        assert_eq!(sim.params[index].value, original_value);
        assert!(outbound_rx.try_recv().is_err());
    }

    #[tokio::test]
    async fn param_set_for_valid_target_updates_and_responds() {
        let (mut sim, mut outbound_rx) = sim_core();

        sim.handle_param_set(param_set(DEFAULT_SYSTEM_ID, 42.0))
            .await
            .unwrap();

        let response = match outbound_rx.try_recv().unwrap().1 {
            dialect::MavMessage::PARAM_VALUE(data) => data,
            message => panic!("expected PARAM_VALUE, got {message:?}"),
        };
        assert_eq!(response.param_id.to_str().unwrap(), "SYSID_THISMAV");
        assert_eq!(response.param_value, 42.0);
    }

    #[test]
    fn full_fixtures_include_real_world_profile_params() {
        let copter = param_names(PARAM_FIXTURES.params(DemoProfile::ArduCopter));
        let plane = param_names(PARAM_FIXTURES.params(DemoProfile::ArduPlane));
        let quadplane = param_names(PARAM_FIXTURES.params(DemoProfile::ArduQuadPlane));

        assert!(copter.contains(&"RTL_ALT"));
        assert!(copter.contains(&"RTL_ALT_FINAL"));
        assert!(copter.contains(&"RTL_CLIMB_MIN"));
        assert!(copter.contains(&"FS_THR_ENABLE"));
        assert!(copter.contains(&"FS_GCS_ENABLE"));
        assert!(copter.contains(&"BATT_FS_LOW_ACT"));
        assert!(!copter.contains(&"AIRSPEED_CRUISE"));
        assert!(!copter.iter().any(|name| name.starts_with("Q_")));

        assert!(plane.contains(&"AIRSPEED_CRUISE"));
        assert!(plane.contains(&"RTL_ALTITUDE"));
        assert!(plane.contains(&"RTL_AUTOLAND"));
        assert!(plane.contains(&"FS_LONG_ACTN"));
        assert!(plane.contains(&"FS_SHORT_ACTN"));
        assert!(plane.contains(&"LAND_FLARE_SEC"));

        assert!(quadplane.contains(&"AIRSPEED_CRUISE"));
        assert!(quadplane.contains(&"RTL_ALTITUDE"));
        assert!(quadplane.contains(&"RTL_AUTOLAND"));
        assert!(quadplane.contains(&"Q_ENABLE"));
        assert!(quadplane.contains(&"Q_RTL_ALT"));
        assert!(quadplane.contains(&"Q_GUIDED_MODE"));
        assert!(quadplane.iter().any(|name| name.starts_with("Q_")));
    }
}
