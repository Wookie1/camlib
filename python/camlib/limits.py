"""Machining limits: where a toolpath may cut.

``Limits(contain=region, avoid=(AvoidArea(region, top_z_mm), ...))``: the
whole cutter stays inside ``contain`` while cutting, and out of each avoid
area (clamps, fixtures, features to leave alone) while its tip is below the
area's top. Give them per operation (``OperationNode(op, limits=...)``) or for
every operation of a setup (``Setup(avoid=..., contain=...)``).
"""

from dataclasses import dataclass
import math


def _region(value):
    return value.to_dict() if hasattr(value, "to_dict") else value


@dataclass(frozen=True)
class AvoidArea:
    """Keep the cutter out of ``region`` while its tip is below ``top_z_mm``."""

    region: object
    top_z_mm: float

    def __post_init__(self):
        if not isinstance(self.top_z_mm, (int, float)) or not math.isfinite(self.top_z_mm):
            raise ValueError("avoid area needs a finite top_z_mm")

    def to_dict(self):
        return {"region": _region(self.region), "top_z_mm": float(self.top_z_mm)}


@dataclass(frozen=True)
class Limits:
    contain: object = None
    avoid: tuple = ()

    def __post_init__(self):
        object.__setattr__(self, "avoid", tuple(self.avoid))
        if any(not isinstance(a, AvoidArea) for a in self.avoid):
            raise TypeError("avoid areas must be camlib.AvoidArea")

    def to_dict(self):
        result = {}
        if self.contain is not None:
            result["contain"] = _region(self.contain)
        if self.avoid:
            result["avoid"] = [a.to_dict() for a in self.avoid]
        return result


def merged(setup_contain, setup_avoid, node_limits):
    """The limits of one operation: its own containment (else the setup's),
    and the setup's avoid areas plus its own. None when there are none."""
    contain = setup_contain
    avoid = tuple(setup_avoid or ())
    if node_limits is not None:
        if node_limits.contain is not None:
            contain = node_limits.contain
        avoid += node_limits.avoid
    limits = Limits(contain, avoid)
    return limits.to_dict() or None
