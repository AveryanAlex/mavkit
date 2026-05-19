#!/usr/bin/env python3

from __future__ import annotations

import argparse
import datetime as dt
import os
import shlex
import signal
import socket
import subprocess
import sys
import time
import uuid
from dataclasses import dataclass
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
FIXTURE_DIR = REPO_ROOT / "src" / "sim" / "fixtures" / "params"
DEFAULT_SITL_IMAGE = "radarku/ardupilot-sitl:eff32c1f98152ac3d1dc09a1e475733b73ce569f"
SITL_HOME = "42.3898,-71.1476,14.0,270.0"
SITL_CONTAINER_PORT = 5760


@dataclass(frozen=True)
class Variant:
    preset: str
    vehicle_family: str
    autopilot: str
    defaults: str
    output: str
    sitl_command: str


VARIANTS: dict[str, Variant] = {
    "quadcopter": Variant(
        preset="quadcopter",
        vehicle_family="copter",
        autopilot="ArduCopter",
        defaults="/ardupilot/Tools/autotest/default_params/copter.parm",
        output="copter-defaults.json",
        sitl_command="cd /ardupilot/ArduCopter && ../Tools/autotest/sim_vehicle.py -v ArduCopter -f quad --no-rebuild --no-mavproxy --speedup 1 --custom-location {home} -I {instance} -w",
    ),
    "airplane": Variant(
        preset="airplane",
        vehicle_family="plane",
        autopilot="ArduPlane",
        defaults="/ardupilot/Tools/autotest/models/plane.parm",
        output="plane-defaults.json",
        sitl_command="cd /ardupilot/ArduPlane && ../Tools/autotest/sim_vehicle.py -v ArduPlane -f plane --no-rebuild --no-mavproxy --speedup 1 --custom-location {home} -I {instance} -w",
    ),
    "quadplane": Variant(
        preset="quadplane",
        vehicle_family="plane",
        autopilot="ArduPlane",
        defaults="/ardupilot/Tools/autotest/default_params/quadplane.parm",
        output="quadplane-defaults.json",
        sitl_command="cd /ardupilot/ArduPlane && ../Tools/autotest/sim_vehicle.py -v ArduPlane -f quadplane --no-rebuild --no-mavproxy --speedup 1 --custom-location {home} -I {instance} -w",
    ),
}


def run(cmd: list[str], *, env: dict[str, str] | None = None) -> subprocess.CompletedProcess[str]:
    print(f"+ {shlex.join(cmd)}", flush=True)
    return subprocess.run(cmd, cwd=REPO_ROOT, env=env, text=True, check=False)


def run_capture(cmd: list[str]) -> subprocess.CompletedProcess[str]:
    print(f"+ {shlex.join(cmd)}", flush=True)
    return subprocess.run(cmd, cwd=REPO_ROOT, capture_output=True, text=True, check=False)


class DockerSitl:
    def __init__(self, *, image: str, timeout_s: int, no_pull: bool) -> None:
        self.image = image
        self.timeout_s = timeout_s
        self.no_pull = no_pull
        self.container: str | None = None
        self.tcp_addr: str | None = None

    def start(self, variant: Variant) -> str:
        if not self.no_pull:
            result = run(["docker", "pull", self.image])
            if result.returncode != 0:
                raise RuntimeError(f"docker pull failed with exit code {result.returncode}")

        # Each preset runs in a fresh container, so keep ArduPilot on instance 0.
        # Higher instances can shift the SITL TCP port away from the published 5760/tcp.
        instance = 0
        self.container = os.environ.get(
            "MAVKIT_SIM_PARAM_SITL_CONTAINER",
            f"mavkit-param-fixture-{os.getpid()}-{variant.preset}-{uuid.uuid4().hex[:8]}",
        )
        subprocess.run(["docker", "rm", "-f", self.container], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        command = variant.sitl_command.format(home=SITL_HOME, instance=instance)
        result = run_capture(
            [
                "docker",
                "run",
                "-d",
                "--rm",
                "--name",
                self.container,
                "-p",
                f"127.0.0.1::{SITL_CONTAINER_PORT}",
                "--entrypoint",
                "/bin/sh",
                self.image,
                "-lc",
                command,
            ]
        )
        if result.returncode != 0:
            sys.stderr.write(result.stderr)
            raise RuntimeError(f"docker run failed with exit code {result.returncode}")

        self.tcp_addr = self._resolve_tcp_addr()
        return self.tcp_addr

    def stop(self) -> None:
        if self.container is not None:
            subprocess.run(["docker", "rm", "-f", self.container], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.container = None
            self.tcp_addr = None

    def dump_logs(self) -> None:
        if self.container is None:
            return
        print(f"--- docker logs {self.container} ---", file=sys.stderr, flush=True)
        subprocess.run(["docker", "logs", self.container], check=False)
        print(f"--- end docker logs {self.container} ---", file=sys.stderr, flush=True)

    def wait_for_exporter_connection(self, variant: Variant, output: Path, features: str) -> None:
        if self.tcp_addr is None:
            raise RuntimeError("SITL TCP address is not available")

        deadline = time.monotonic() + self.timeout_s
        last_error = 1
        while time.monotonic() < deadline:
            result = export_fixture(
                connect=f"tcpout:{self.tcp_addr}",
                variant=variant,
                output=output,
                image=self.image,
                features=features,
            )
            if result.returncode == 0:
                return
            last_error = result.returncode
            time.sleep(2)

        raise RuntimeError(f"exporter failed before timeout, last exit code {last_error}")

    def _resolve_tcp_addr(self) -> str:
        if self.container is None:
            raise RuntimeError("SITL container is not running")

        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            result = run_capture(["docker", "port", self.container, f"{SITL_CONTAINER_PORT}/tcp"])
            if result.returncode == 0:
                for line in result.stdout.splitlines():
                    host, separator, port = line.rpartition(":")
                    if separator and port.isdecimal():
                        connect_host = host.strip("[]")
                        if connect_host in {"0.0.0.0", "::"}:
                            connect_host = "127.0.0.1"
                        return f"{connect_host}:{port}"
            time.sleep(1)

        raise RuntimeError("timed out resolving SITL TCP port")


def export_fixture(*, connect: str, variant: Variant, output: Path, image: str, features: str) -> subprocess.CompletedProcess[str]:
    # This is the only fixture writer path: the Rust binary connects through MAVKit,
    # downloads PARAM_VALUE rows from the live endpoint, sorts by param_index, and
    # writes the runtime fixture schema.
    cmd = [
        os.environ.get("CARGO", "cargo"),
        "run",
        "--quiet",
        "--features",
        features,
        "--bin",
        "export_sim_params",
        "--",
        "--connect",
        connect,
        "--vehicle-family",
        variant.vehicle_family,
        "--vehicle-preset",
        variant.preset,
        "--autopilot",
        variant.autopilot,
        "--sitl-image",
        image,
        "--defaults",
        variant.defaults,
        "--generated-at",
        dt.datetime.now(dt.timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z"),
        "--output",
        str(output),
        "--connect-timeout-ms",
        "5000",
        "--transfer-timeout-ms",
        "120000",
    ]
    return run(cmd)


def selected_variants(preset: str) -> list[Variant]:
    if preset == "all":
        return list(VARIANTS.values())
    return [VARIANTS[preset]]


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Regenerate MAVKit simulator param fixtures from live SITL/MAVLink parameter downloads."
    )
    parser.add_argument("--preset", choices=["all", *VARIANTS.keys()], default="all")
    parser.add_argument(
        "--connect",
        help="existing MAVLink URL, e.g. tcpout:127.0.0.1:5760. Requires --preset other than all.",
    )
    parser.add_argument("--output-dir", type=Path, default=FIXTURE_DIR)
    parser.add_argument("--sitl-image", default=os.environ.get("MAVKIT_SITL_IMAGE", DEFAULT_SITL_IMAGE))
    parser.add_argument("--features", default=os.environ.get("MAVKIT_SIM_PARAM_EXPORT_FEATURES", "tcp,sim"))
    parser.add_argument("--timeout", type=int, default=int(os.environ.get("MAVKIT_SIM_PARAM_READY_TIMEOUT", "180")))
    parser.add_argument("--no-pull", action="store_true", help="skip docker pull before starting SITL")
    args = parser.parse_args()

    if args.connect and args.preset == "all":
        parser.error("--connect exports one endpoint; pass --preset quadcopter, airplane, or quadplane")

    variants = selected_variants(args.preset)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    if args.connect:
        variant = variants[0]
        result = export_fixture(
            connect=args.connect,
            variant=variant,
            output=args.output_dir / variant.output,
            image=args.sitl_image,
            features=args.features,
        )
        return result.returncode

    runner = DockerSitl(image=args.sitl_image, timeout_s=args.timeout, no_pull=args.no_pull)

    def stop_for_signal(signum: int, _frame: object) -> None:
        print(f"received signal {signum}, cleaning up SITL", file=sys.stderr, flush=True)
        runner.stop()
        raise SystemExit(128 + signum)

    signal.signal(signal.SIGINT, stop_for_signal)
    signal.signal(signal.SIGTERM, stop_for_signal)

    try:
        for variant in variants:
            output = args.output_dir / variant.output
            runner.start(variant)
            try:
                # ArduPlane-family startup can reject early TCP clients; retry the real exporter
                # instead of probing readiness with a separate socket that may consume the first accept.
                runner.wait_for_exporter_connection(variant, output, args.features)
            except Exception:
                runner.dump_logs()
                raise
            finally:
                runner.stop()
    except (OSError, RuntimeError, socket.error) as exc:
        print(f"error: {exc}", file=sys.stderr, flush=True)
        runner.dump_logs()
        runner.stop()
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
