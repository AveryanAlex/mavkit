#!/usr/bin/env python3
"""Verify that a git tag matches the version in Cargo.toml."""

import os
import pathlib
import sys
import tomllib


def main() -> int:
    tag = os.environ.get("GIT_TAG", "")
    if not tag.startswith("v"):
        print(f"Tag must start with 'v', got: {tag}", file=sys.stderr)
        return 1

    cargo_toml = tomllib.loads(pathlib.Path("Cargo.toml").read_text())
    version = cargo_toml["package"]["version"]
    if tag[1:] != version:
        print(
            f"Tag version {tag[1:]} does not match Cargo.toml version {version}",
            file=sys.stderr,
        )
        return 1

    print(f"Tag {tag} matches Cargo.toml version {version}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
