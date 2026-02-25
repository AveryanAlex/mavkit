#!/usr/bin/env python3
"""Verify that a git tag matches the versions in all manifests."""

import os
import pathlib
import sys
import tomllib


def main() -> int:
    tag = os.environ.get("GIT_TAG", "")
    if not tag.startswith("v"):
        print(f"Tag must start with 'v', got: {tag}", file=sys.stderr)
        return 1

    tag_version = tag[1:]
    ok = True

    manifests = {
        "Cargo.toml": ("package", "version"),
        "mavkit-python/Cargo.toml": ("package", "version"),
        "mavkit-python/pyproject.toml": ("project", "version"),
    }

    for path, keys in manifests.items():
        data = tomllib.loads(pathlib.Path(path).read_text())
        version = data[keys[0]][keys[1]]
        if tag_version != version:
            print(
                f"Tag version {tag_version} does not match {path} version {version}",
                file=sys.stderr,
            )
            ok = False
        else:
            print(f"Tag {tag} matches {path} version {version}")

    # Verify README.md shows correct major.minor version
    major_minor = ".".join(tag_version.split(".")[:2])
    readme = pathlib.Path("README.md").read_text()
    expected = f'mavkit = "{major_minor}"'
    if expected not in readme:
        print(
            f"README.md does not contain {expected!r}",
            file=sys.stderr,
        )
        ok = False
    else:
        print(f"README.md contains {expected}")

    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
