#!/usr/bin/env python3

from __future__ import annotations

import argparse
import atexit
import os
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.run_sitl_tests import SitlTestRunner, env_flag


_current_child: subprocess.Popen[bytes] | None = None
_proxy: subprocess.Popen[bytes] | None = None
_runner: SitlTestRunner | None = None


def reserve_port(host: str) -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind((host, 0))
        return int(sock.getsockname()[1])


def wait_for_tcp(host: str, port: int, timeout_s: int, process: subprocess.Popen[bytes]) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"websocket proxy exited with code {process.returncode}")
        try:
            with socket.create_connection((host, port), timeout=1):
                return
        except OSError:
            time.sleep(0.25)

    raise RuntimeError(f"timed out waiting for websocket proxy at {host}:{port}")


def split_host_port(addr: str) -> tuple[str, int]:
    host, separator, port = addr.rpartition(":")
    if not separator or not port.isdecimal():
        raise ValueError(f"expected host:port address, got {addr!r}")
    return host.strip("[]"), int(port)


def start_proxy(tcp_addr: str, timeout_s: int) -> tuple[subprocess.Popen[bytes], str]:
    global _proxy

    script_dir = Path(__file__).resolve().parent
    target_host, target_port = split_host_port(tcp_addr)
    listen_host = os.environ.get("MAVKIT_WASM_SITL_WS_HOST", "127.0.0.1")
    listen_port = int(os.environ.get("MAVKIT_WASM_SITL_WS_PORT", "0")) or reserve_port(
        listen_host
    )
    proxy_script = script_dir / "ws_tcp_proxy.py"
    cmd = [
        sys.executable,
        str(proxy_script),
        "--listen-host",
        listen_host,
        "--listen-port",
        str(listen_port),
        "--target-host",
        target_host,
        "--target-port",
        str(target_port),
    ]
    print(f"+ {shlex.join(cmd)}", flush=True)
    _proxy = subprocess.Popen(cmd)
    wait_for_tcp(listen_host, listen_port, timeout_s, _proxy)
    ws_url = f"ws://{listen_host}:{listen_port}/mavlink"
    print(f"WASM SITL websocket ready at {ws_url}", flush=True)
    return _proxy, ws_url


def stop_proxy() -> None:
    global _proxy

    if _proxy is None or _proxy.poll() is not None:
        _proxy = None
        return

    _proxy.terminate()
    try:
        _proxy.wait(timeout=10)
    except subprocess.TimeoutExpired:
        _proxy.kill()
        _proxy.wait()
    _proxy = None


def cargo_command() -> list[str]:
    cargo = os.environ.get("CARGO", "cargo")
    features = os.environ.get("MAVKIT_WASM_SITL_RUST_FEATURES", "wasm")
    cargo_test_args = shlex.split(os.environ.get("MAVKIT_WASM_SITL_CARGO_TEST_ARGS", "--test wasm_sitl"))
    harness_args = os.environ.get("MAVKIT_WASM_SITL_HARNESS_ARGS")

    cmd = [
        cargo,
        "test",
        "-p",
        "mavkit",
        "--target",
        "wasm32-unknown-unknown",
        "--no-default-features",
    ]
    if features:
        cmd.extend(["--features", features])
    cmd.extend(cargo_test_args)
    if harness_args:
        cmd.append("--")
        cmd.extend(shlex.split(harness_args))
    return cmd


def has_webdriver() -> bool:
    remote_env = [
        "CHROMEDRIVER_REMOTE",
        "GECKODRIVER_REMOTE",
        "SAFARIDRIVER_REMOTE",
        "MSEDGEDRIVER_REMOTE",
    ]
    driver_env = ["CHROMEDRIVER", "GECKODRIVER", "SAFARIDRIVER", "MSEDGEDRIVER"]
    driver_bins = ["chromedriver", "geckodriver", "safaridriver", "msedgedriver"]

    return (
        any(os.environ.get(name) for name in remote_env)
        or any(os.environ.get(name) and Path(os.environ[name]).exists() for name in driver_env)
        or any(shutil.which(name) is not None for name in driver_bins)
    )


def test_environment(ws_url: str, *, no_headless: bool, strict: bool) -> dict[str, str]:
    env = os.environ.copy()
    env["MAVKIT_WASM_SITL_WS_URL"] = ws_url
    env["CARGO_TARGET_WASM32_UNKNOWN_UNKNOWN_RUNNER"] = "wasm-bindgen-test-runner"
    env["WASM_BINDGEN_USE_BROWSER"] = "1"
    if strict:
        env["MAVKIT_SITL_STRICT"] = "1"
    if no_headless:
        env["NO_HEADLESS"] = "1"
        env.pop("CI", None)
    else:
        env.setdefault("CI", "1")
    return env


def install_signal_handlers() -> None:
    def handle_signal(signum: int, _frame: object) -> None:
        print(f"received signal {signum}, cleaning up WASM SITL", file=sys.stderr, flush=True)
        if _current_child is not None and _current_child.poll() is None:
            _current_child.terminate()
            try:
                _current_child.wait(timeout=10)
            except subprocess.TimeoutExpired:
                _current_child.kill()
        stop_proxy()
        if _runner is not None:
            _runner.stop()
        raise SystemExit(128 + signum)

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)


def main() -> int:
    global _current_child, _runner

    parser = argparse.ArgumentParser(
        description="Start ArduPilot SITL, bridge it to browser WebSocket, run wasm SITL tests."
    )
    parser.add_argument("--strict", action="store_true", help="set MAVKIT_SITL_STRICT=1")
    parser.add_argument("--no-pull", action="store_true", help="skip docker pull before starting SITL")
    parser.add_argument("--no-headless", action="store_true", help="let wasm-bindgen open an interactive browser")
    parser.add_argument(
        "--timeout",
        type=int,
        default=int(os.environ.get("MAVKIT_SITL_READY_TIMEOUT", "120")),
        help="seconds to wait for SITL and websocket proxy readiness",
    )
    args = parser.parse_args()

    if shutil.which("wasm-bindgen-test-runner") is None:
        print("error: wasm-bindgen-test-runner is not in PATH", file=sys.stderr, flush=True)
        return 1
    if not args.no_headless and not has_webdriver():
        print(
            "error: no WebDriver found; install chromedriver/geckodriver or pass --no-headless",
            file=sys.stderr,
            flush=True,
        )
        return 1

    pull = env_flag("MAVKIT_SITL_PULL", True) and not args.no_pull
    _runner = SitlTestRunner(pull=pull, timeout_s=args.timeout)
    atexit.register(_runner.stop)
    atexit.register(stop_proxy)
    install_signal_handlers()

    try:
        tcp_addr = _runner.start()
        _, ws_url = start_proxy(tcp_addr, args.timeout)
        env = test_environment(ws_url, no_headless=args.no_headless, strict=args.strict)

        cmd = cargo_command()
        print(f"+ {shlex.join(cmd)}", flush=True)
        _current_child = subprocess.Popen(cmd, env=env)
        rc = _current_child.wait()
        _current_child = None
        if rc != 0:
            _runner.dump_logs()
        return rc
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr, flush=True)
        if _runner is not None:
            _runner.dump_logs()
        return 1
    finally:
        stop_proxy()
        if _runner is not None:
            _runner.stop()


if __name__ == "__main__":
    raise SystemExit(main())
