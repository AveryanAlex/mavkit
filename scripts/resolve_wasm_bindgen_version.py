#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Print the wasm-bindgen crate version resolved by Cargo.lock."
    )
    parser.add_argument(
        "--features",
        default=os.environ.get("MAVKIT_WASM_FEATURES", ""),
        help="comma or space separated feature list to pass to cargo metadata",
    )
    parser.add_argument(
        "--target",
        default="wasm32-unknown-unknown",
        help="target triple used to filter Cargo metadata",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    cargo = os.environ.get("CARGO", "cargo")
    cmd = [
        cargo,
        "metadata",
        "--locked",
        "--format-version",
        "1",
        "--filter-platform",
        args.target,
        "--no-default-features",
    ]
    if args.features:
        cmd.extend(["--features", args.features])

    try:
        metadata = json.loads(subprocess.check_output(cmd, text=True))
    except subprocess.CalledProcessError as exc:
        return exc.returncode

    versions = sorted(
        {
            package["version"]
            for package in metadata["packages"]
            if package["name"] == "wasm-bindgen"
        }
    )
    if len(versions) != 1:
        print(
            f"expected exactly one wasm-bindgen version, found: {versions}",
            file=sys.stderr,
        )
        return 1

    print(versions[0])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
