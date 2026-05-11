#!/usr/bin/env python3

from __future__ import annotations

import argparse
import atexit
import os
import shlex
import signal
import socket
import subprocess
import sys
import time
import uuid


DEFAULT_SITL_IMAGE = "radarku/ardupilot-sitl:eff32c1f98152ac3d1dc09a1e475733b73ce569f"
SITL_DEFAULTS = "/ardupilot/Tools/autotest/default_params/copter.parm"
SITL_HOME = "42.3898,-71.1476,14.0,270.0"
SITL_CONTAINER_PORT = 5760

_current_child: subprocess.Popen[bytes] | None = None
_runner: SitlTestRunner | None = None


def env_flag(name: str, default: bool) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() not in {"0", "false", "no", "off"}


def run_interactive(cmd: list[str], *, env: dict[str, str] | None = None) -> int:
    global _current_child

    print(f"+ {shlex.join(cmd)}", flush=True)
    _current_child = subprocess.Popen(cmd, env=env)
    try:
        return _current_child.wait()
    finally:
        _current_child = None


def run_capture(cmd: list[str]) -> subprocess.CompletedProcess[str]:
    print(f"+ {shlex.join(cmd)}", flush=True)
    return subprocess.run(cmd, capture_output=True, text=True, check=False)


class SitlTestRunner:
    def __init__(self, *, pull: bool, timeout_s: int) -> None:
        self.image = os.environ.get("MAVKIT_SITL_IMAGE", DEFAULT_SITL_IMAGE)
        self.container = os.environ.get(
            "MAVKIT_SITL_CONTAINER", f"mavkit-sitl-{os.getpid()}-{uuid.uuid4().hex[:8]}"
        )
        self.host = os.environ.get("MAVKIT_SITL_TCP_HOST", "127.0.0.1")
        self.requested_port = os.environ.get("MAVKIT_SITL_TCP_PORT")
        self.pull = pull
        self.timeout_s = timeout_s
        self.started = False
        self.tcp_addr: str | None = None

    def start(self) -> str:
        self._remove_stale_container()
        if self.pull:
            rc = run_interactive(["docker", "pull", self.image])
            if rc != 0:
                raise RuntimeError(f"docker pull failed with exit code {rc}")

        publish = self._publish_arg()
        cmd = [
            "docker",
            "run",
            "-d",
            "--rm",
            "--name",
            self.container,
            "-p",
            publish,
            "--entrypoint",
            "/ardupilot/build/sitl/bin/arducopter",
            self.image,
            "--model",
            "+",
            "--speedup",
            "1",
            "--defaults",
            SITL_DEFAULTS,
            "--home",
            SITL_HOME,
            "-w",
        ]
        result = run_capture(cmd)
        if result.returncode != 0:
            sys.stderr.write(result.stderr)
            raise RuntimeError(f"docker run failed with exit code {result.returncode}")

        self.started = True
        self.tcp_addr = self._resolve_tcp_addr()
        self._wait_for_tcp()
        print(f"SITL ready at tcp:{self.tcp_addr}", flush=True)
        return self.tcp_addr

    def stop(self) -> None:
        if not self.started:
            return

        subprocess.run(
            ["docker", "rm", "-f", self.container],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        self.started = False

    def dump_logs(self) -> None:
        if not self.started:
            return

        print(f"--- docker logs {self.container} ---", file=sys.stderr, flush=True)
        result = subprocess.run(
            ["docker", "logs", self.container],
            capture_output=True,
            text=True,
            check=False,
        )
        if result.stdout:
            sys.stderr.write(result.stdout)
        if result.stderr:
            sys.stderr.write(result.stderr)
        print(f"--- end docker logs {self.container} ---", file=sys.stderr, flush=True)

    def _publish_arg(self) -> str:
        if self.requested_port:
            return f"{self.host}:{self.requested_port}:{SITL_CONTAINER_PORT}"

        return f"{self.host}::{SITL_CONTAINER_PORT}"

    def _remove_stale_container(self) -> None:
        subprocess.run(
            ["docker", "rm", "-f", self.container],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )

    def _resolve_tcp_addr(self) -> str:
        result = run_capture(["docker", "port", self.container, f"{SITL_CONTAINER_PORT}/tcp"])
        if result.returncode != 0:
            sys.stderr.write(result.stderr)
            raise RuntimeError("failed to inspect SITL TCP port")

        for line in result.stdout.splitlines():
            host, separator, port = line.rpartition(":")
            if separator and port.isdecimal():
                connect_host = host.strip("[]")
                if connect_host in {"0.0.0.0", "::"}:
                    connect_host = "127.0.0.1"
                return f"{connect_host}:{port}"

        raise RuntimeError(f"could not parse docker port output: {result.stdout!r}")

    def _wait_for_tcp(self) -> None:
        if self.tcp_addr is None:
            raise RuntimeError("SITL TCP address not resolved")

        host, port_text = self.tcp_addr.rsplit(":", 1)
        port = int(port_text)
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            if not self._is_running():
                self.dump_logs()
                raise RuntimeError("SITL container exited before TCP became ready")
            try:
                with socket.create_connection((host, port), timeout=1):
                    return
            except OSError:
                time.sleep(1)

        self.dump_logs()
        raise RuntimeError(f"timed out waiting for TCP endpoint {self.tcp_addr}")

    def _is_running(self) -> bool:
        result = subprocess.run(
            ["docker", "inspect", "-f", "{{.State.Running}}", self.container],
            capture_output=True,
            text=True,
            check=False,
        )
        return result.returncode == 0 and result.stdout.strip() == "true"


def cargo_command() -> list[str]:
    cargo = os.environ.get("CARGO", "cargo")
    features = os.environ.get("MAVKIT_SITL_RUST_FEATURES", "tcp")
    cargo_test_args = shlex.split(os.environ.get("MAVKIT_SITL_CARGO_TEST_ARGS", "--tests"))
    harness_args = os.environ.get("MAVKIT_SITL_HARNESS_ARGS")
    if harness_args is None:
        test_threads = os.environ.get("MAVKIT_SITL_TEST_THREADS", "1")
        harness = ["--ignored", "--nocapture", f"--test-threads={test_threads}"]
    else:
        harness = shlex.split(harness_args)

    cmd = [cargo, "test"]
    if features:
        cmd.extend(["--features", features])
    cmd.extend(cargo_test_args)
    cmd.append("--")
    cmd.extend(harness)
    return cmd


def install_signal_handlers() -> None:
    def handle_signal(signum: int, _frame: object) -> None:
        print(f"received signal {signum}, cleaning up SITL", file=sys.stderr, flush=True)
        if _current_child is not None and _current_child.poll() is None:
            _current_child.terminate()
            try:
                _current_child.wait(timeout=10)
            except subprocess.TimeoutExpired:
                _current_child.kill()
        if _runner is not None:
            _runner.stop()
        raise SystemExit(128 + signum)

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)


def main() -> int:
    global _runner

    parser = argparse.ArgumentParser(
        description="Start a fresh ArduPilot SITL container, run MAVKit SITL tests over TCP, then clean up."
    )
    parser.add_argument("--strict", action="store_true", help="set MAVKIT_SITL_STRICT=1 for the test run")
    parser.add_argument("--no-pull", action="store_true", help="skip docker pull before starting SITL")
    parser.add_argument(
        "--timeout",
        type=int,
        default=int(os.environ.get("MAVKIT_SITL_READY_TIMEOUT", "120")),
        help="seconds to wait for the SITL TCP endpoint",
    )
    args = parser.parse_args()

    pull = env_flag("MAVKIT_SITL_PULL", True) and not args.no_pull
    _runner = SitlTestRunner(pull=pull, timeout_s=args.timeout)
    atexit.register(_runner.stop)
    install_signal_handlers()

    try:
        tcp_addr = _runner.start()
        test_env = os.environ.copy()
        test_env["MAVKIT_SITL_TCP_ADDR"] = tcp_addr
        test_env.pop("MAVKIT_SITL_UDP_BIND", None)
        if args.strict:
            test_env["MAVKIT_SITL_STRICT"] = "1"

        rc = run_interactive(cargo_command(), env=test_env)
        if rc != 0:
            _runner.dump_logs()
        return rc
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr, flush=True)
        if _runner is not None:
            _runner.dump_logs()
        return 1
    finally:
        if _runner is not None:
            _runner.stop()


if __name__ == "__main__":
    raise SystemExit(main())
