"""Async Python SDK for MAVLink vehicle control."""

from importlib.metadata import PackageNotFoundError
from importlib.metadata import version as _dist_version

from .mavkit import *  # noqa: F403
from .mavkit import TlogWriter as TlogWriter  # noqa: F401

try:
    __version__ = _dist_version("mavkit")
except (
    PackageNotFoundError
):  # pragma: no cover - source-tree import without installed metadata
    __version__ = "0.5.0"
