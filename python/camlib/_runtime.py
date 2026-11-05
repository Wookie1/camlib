"""Locate the engine installed with camlib, or the current source build."""

import os
from pathlib import Path


PACKAGE = Path(__file__).resolve().parent


def executable(name, override=None):
    if override is not None:
        return Path(override)
    if name != "cam-sim-stream":
        raise ValueError("unknown native CAM driver")
    filename = name + (".exe" if os.name == "nt" else "")
    bundled = PACKAGE / "_bin" / filename
    if bundled.is_file():
        return bundled
    source_build = PACKAGE.parents[1] / "target" / "release" / filename
    if source_build.is_file():
        return source_build
    raise FileNotFoundError(
        f"camlib cannot find {filename}; reinstall its platform wheel, or build "
        "this source checkout with cargo build --release --workspace --all-features"
    )
