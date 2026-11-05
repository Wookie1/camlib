"""Load installed or workspace-built safe native buffer kernels."""

import importlib
import importlib.machinery
import importlib.util
import os
from pathlib import Path
import sys


def _load():
    name = "camlib._native"
    try:
        return importlib.import_module(name)
    except ModuleNotFoundError as error:
        if error.name != name:
            raise
    filename = (
        "_native.dll"
        if sys.platform == "win32"
        else "lib_native.dylib"
        if sys.platform == "darwin"
        else "lib_native.so"
    )
    path = Path(os.environ.get("CAMLIB_NATIVE", Path(__file__).resolve().parents[2] / "target/release" / filename))
    if not path.is_file():
        raise ImportError("Build Python buffer kernels with cargo build --release --workspace --all-features")
    loader = importlib.machinery.ExtensionFileLoader(name, str(path))
    spec = importlib.util.spec_from_file_location(name, path, loader=loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    sys.modules[name] = module
    return module


native = _load()
