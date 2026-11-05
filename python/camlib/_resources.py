"""Memory budget shared by the Python API; mirrors ``camlib::resources``.

camlib does not cap work with fixed constants. Memory is bounded instead:
large or caller-sized allocations are checked against the memory actually
available, so an oversized request fails with a clear error instead of making
the host swap until it is unresponsive.
"""

import os

_STEP = 65536


def _cgroup_headroom():
    """Smallest (memory.max - memory.current) over this process's cgroup v2
    ancestry, or None when no limit applies or it cannot be read."""
    try:
        with open("/proc/self/cgroup") as f:
            path = next((line.split("::", 1)[1].strip() for line in f if line.startswith("0::")), None)
    except OSError:
        return None
    if path is None:
        return None
    best = None
    parts = [p for p in path.split("/") if p]
    for depth in range(len(parts), -1, -1):
        base = os.path.join("/sys/fs/cgroup", *parts[:depth])
        try:
            with open(os.path.join(base, "memory.max")) as f:
                limit = f.read().strip()
            if limit == "max":
                continue
            with open(os.path.join(base, "memory.current")) as f:
                current = int(f.read())
        except (OSError, ValueError):
            continue
        headroom = max(0, int(limit) - current)
        best = headroom if best is None else min(best, headroom)
    return best


def _os_available():
    try:
        with open("/proc/meminfo") as f:
            for line in f:
                if line.startswith("MemAvailable:"):
                    return int(line.split()[1]) * 1024
    except (OSError, ValueError, IndexError):
        pass
    return None


def memory_budget():
    """Bytes camlib may allocate now: CAMLIB_MEMORY_BUDGET_BYTES when set,
    else the smaller of the OS available memory and the cgroup headroom;
    None when neither can be determined."""
    explicit = os.environ.get("CAMLIB_MEMORY_BUDGET_BYTES")
    if explicit:
        return int(explicit)
    known = [v for v in (_os_available(), _cgroup_headroom()) if v is not None]
    return min(known) if known else None


def check_memory(nbytes, what, error=MemoryError):
    budget = memory_budget()
    if budget is not None and nbytes > budget:
        raise error(
            f"{what} needs about {nbytes / 2**30:.1f} GiB but only {budget / 2**30:.1f} GiB is "
            "available; use a coarser resolution or larger step, split the job, or raise "
            "CAMLIB_MEMORY_BUDGET_BYTES"
        )


def bounded_tuple(values, item_bytes, what, convert=lambda v: v):
    """Materialize an iterable incrementally, refusing it before it outgrows memory."""
    result = []
    for value in values:
        result.append(convert(value))
        if len(result) % _STEP == 0:
            check_memory(len(result) * item_bytes, what, ValueError)
    return tuple(result)
