"""Speeds and feeds from cutting data, using the standard machining formulas.

camlib never chooses cutting data. Surface speed and feed per tooth come from
the tool maker for the tool, the material and the machine. These helpers only
apply the usual formulas (as given in tooling handbooks):

* spindle speed ``n = 1000 * Vc / (pi * D)`` (rpm)
* table feed ``vf = n * z * fz`` (mm/min)
* radial chip thinning, for a radial engagement ``ae < D / 2``: the chip is
  thinner than ``fz``, so ``fz`` is raised by ``D / (2 * sqrt(ae * (D - ae)))``
  to keep the recommended chip thickness
* ball end mill effective diameter at axial depth ``ap < D / 2``:
  ``De = 2 * sqrt(ap * (D - ap))``; speed is computed at ``De``

Results go straight into an operation's ``spindle_rpm`` and ``feed_mm_min``.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from .tools import Cutter, ToolAssembly


@dataclass(frozen=True)
class CuttingData:
    spindle_rpm: float
    feed_mm_min: float
    feed_per_tooth_mm: float
    """Programmed feed per tooth (raised by chip-thinning compensation)."""
    effective_diameter_mm: float
    rpm_limited: bool
    """True when ``max_rpm`` capped the speed; the feed then follows the
    capped speed, so the feed per tooth is kept."""


def _positive(name, value):
    if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be a positive finite number, got {value!r}")
    return float(value)


def cutting_data(
    tool,
    *,
    surface_speed_m_min,
    feed_per_tooth_mm,
    radial_engagement_mm=None,
    axial_depth_mm=None,
    max_rpm=None,
    chip_thinning=True,
):
    """Spindle speed and feed for ``tool`` (a ``Cutter`` or ``ToolAssembly``).

    ``radial_engagement_mm`` (ae) enables chip-thinning compensation when it is
    below half the diameter (set ``chip_thinning=False`` to use ``fz`` as given).
    ``axial_depth_mm`` (ap) gives a ball end mill's effective cutting diameter.
    ``max_rpm`` caps the speed at the machine's limit, reported in the result.
    """
    cutter = tool.cutter if isinstance(tool, ToolAssembly) else tool
    if not isinstance(cutter, Cutter):
        raise TypeError("tool must be a Cutter or ToolAssembly")
    vc = _positive("surface_speed_m_min", surface_speed_m_min)
    fz = _positive("feed_per_tooth_mm", feed_per_tooth_mm)
    d = _positive("cutter diameter", cutter.diameter_mm)
    if type(cutter.flutes) is not int or cutter.flutes < 1:
        raise ValueError(f"cutter flutes must be a positive integer, got {cutter.flutes!r}")
    effective = d
    if axial_depth_mm is not None:
        ap = _positive("axial_depth_mm", axial_depth_mm)
        if dict(cutter.shape).get("kind") == "ball" and ap < d / 2:
            effective = 2 * math.sqrt(ap * (d - ap))
    if radial_engagement_mm is not None:
        ae = _positive("radial_engagement_mm", radial_engagement_mm)
        if ae > d:
            raise ValueError(f"radial_engagement_mm {ae} exceeds the cutter diameter {d}")
        if chip_thinning and ae < effective / 2:
            fz *= effective / (2 * math.sqrt(ae * (effective - ae)))
    rpm = 1000 * vc / (math.pi * effective)
    limited = False
    if max_rpm is not None:
        cap = _positive("max_rpm", max_rpm)
        if rpm > cap:
            rpm, limited = cap, True
    return CuttingData(rpm, rpm * cutter.flutes * fz, fz, effective, limited)
