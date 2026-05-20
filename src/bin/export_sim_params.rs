#![cfg_attr(target_arch = "wasm32", allow(dead_code, unused_imports))]

use std::env;
use std::fs;
use std::path::Path;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use mavkit::{Param, ParamType, Vehicle, VehicleConfig};
use serde::Serialize;

const SCHEMA_VERSION: u32 = 1;

#[cfg(target_arch = "wasm32")]
fn main() {
    panic!("export_sim_params is a native-only helper");
}

#[derive(Debug)]
struct Args {
    connect: String,
    vehicle_family: String,
    vehicle_preset: String,
    autopilot: String,
    sitl_image: String,
    defaults: String,
    generated_at: String,
    output: Option<String>,
    connect_timeout_ms: u64,
    transfer_timeout_ms: u64,
}

#[derive(Debug, Serialize)]
struct FixtureFile {
    schema_version: u32,
    vehicle_family: String,
    vehicle_preset: String,
    source: FixtureSource,
    params: Vec<FixtureParam>,
}

#[derive(Debug, Serialize)]
struct FixtureSource {
    kind: &'static str,
    autopilot: String,
    sitl_image: String,
    defaults: String,
    generated_at: String,
}

#[derive(Debug, Serialize)]
struct FixtureParam {
    name: String,
    value: f32,
    param_type: ParamType,
}

#[cfg(not(target_arch = "wasm32"))]
#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let raw_args: Vec<String> = env::args().skip(1).collect();
    if raw_args.iter().any(|arg| arg == "--help" || arg == "-h") {
        println!("{}", usage());
        return Ok(());
    }
    let args = Args::parse(raw_args.into_iter())
        .map_err(|err| std::io::Error::new(std::io::ErrorKind::InvalidInput, err))?;

    let config = VehicleConfig {
        connect_timeout: Duration::from_millis(args.connect_timeout_ms),
        transfer_timeout: Duration::from_millis(args.transfer_timeout_ms),
        ..VehicleConfig::default()
    };

    let vehicle = Vehicle::connect_with_config(&args.connect, config).await?;
    let store = vehicle.params().download_all()?.wait().await?;
    vehicle.disconnect().await?;

    let mut params: Vec<Param> = store.params.into_values().collect();
    params.sort_by_key(|param| param.index);

    let fixture = FixtureFile {
        schema_version: SCHEMA_VERSION,
        vehicle_family: args.vehicle_family,
        vehicle_preset: args.vehicle_preset,
        source: FixtureSource {
            kind: "sitl_param_download",
            autopilot: args.autopilot,
            sitl_image: args.sitl_image,
            defaults: args.defaults,
            generated_at: args.generated_at,
        },
        params: params
            .into_iter()
            .map(|param| FixtureParam {
                name: param.name,
                value: param.value,
                param_type: param.param_type,
            })
            .collect(),
    };

    let json = format!("{}\n", serde_json::to_string_pretty(&fixture)?);
    match args.output.as_deref() {
        Some("-") | None => print!("{json}"),
        Some(output) => {
            if let Some(parent) = Path::new(output).parent()
                && !parent.as_os_str().is_empty()
            {
                fs::create_dir_all(parent)?;
            }
            fs::write(output, json)?;
        }
    }

    Ok(())
}

impl Args {
    fn parse(mut raw: impl Iterator<Item = String>) -> Result<Self, String> {
        let mut args = Self {
            connect: String::new(),
            vehicle_family: String::new(),
            vehicle_preset: String::new(),
            autopilot: String::new(),
            sitl_image: String::new(),
            defaults: String::new(),
            generated_at: default_generated_at(),
            output: None,
            connect_timeout_ms: 30_000,
            transfer_timeout_ms: 120_000,
        };

        while let Some(flag) = raw.next() {
            match flag.as_str() {
                "--connect" => args.connect = take_value(&mut raw, &flag)?,
                "--vehicle-family" => args.vehicle_family = take_value(&mut raw, &flag)?,
                "--vehicle-preset" => args.vehicle_preset = take_value(&mut raw, &flag)?,
                "--autopilot" => args.autopilot = take_value(&mut raw, &flag)?,
                "--sitl-image" => args.sitl_image = take_value(&mut raw, &flag)?,
                "--defaults" => args.defaults = take_value(&mut raw, &flag)?,
                "--generated-at" => args.generated_at = take_value(&mut raw, &flag)?,
                "--output" => args.output = Some(take_value(&mut raw, &flag)?),
                "--connect-timeout-ms" => {
                    args.connect_timeout_ms = parse_u64(&take_value(&mut raw, &flag)?, &flag)?;
                }
                "--transfer-timeout-ms" => {
                    args.transfer_timeout_ms = parse_u64(&take_value(&mut raw, &flag)?, &flag)?;
                }
                _ => return Err(format!("unknown argument {flag}\n\n{}", usage())),
            }
        }

        for (field, value) in [
            ("--connect", &args.connect),
            ("--vehicle-family", &args.vehicle_family),
            ("--vehicle-preset", &args.vehicle_preset),
            ("--autopilot", &args.autopilot),
            ("--sitl-image", &args.sitl_image),
            ("--defaults", &args.defaults),
        ] {
            if value.trim().is_empty() {
                return Err(format!("missing required {field}\n\n{}", usage()));
            }
        }

        Ok(args)
    }
}

fn take_value(raw: &mut impl Iterator<Item = String>, flag: &str) -> Result<String, String> {
    raw.next().ok_or_else(|| format!("{flag} requires a value"))
}

fn parse_u64(value: &str, flag: &str) -> Result<u64, String> {
    value
        .parse()
        .map_err(|err| format!("invalid {flag} value {value:?}: {err}"))
}

fn default_generated_at() -> String {
    let seconds = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |duration| duration.as_secs());
    format!("unix:{seconds}")
}

fn usage() -> String {
    "Export a MAVKit simulator param fixture from a live MAVLink endpoint.\n\n\
Usage:\n  cargo run --features tcp,sim --bin export_sim_params -- \\\n    --connect tcpout:127.0.0.1:5760 \\\n    --vehicle-family plane \\\n    --vehicle-preset airplane \\\n    --autopilot ArduPlane \\\n    --sitl-image radarku/ardupilot-sitl:<tag> \\\n    --defaults /ardupilot/Tools/autotest/models/plane.parm \\\n    --output src/sim/fixtures/params/plane-defaults.json\n\n\
The exporter connects with MAVKit, runs PARAM_REQUEST_LIST through the public params API,\n\
sorts rows by MAVLink param_index, and writes the schema consumed by src/sim/params.rs."
        .to_string()
}
