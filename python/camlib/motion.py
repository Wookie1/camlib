"""Controller-independent program authoring. mm, mm/min, radians, RPM.

Positive circular sweep is right-handed about +plane normal. In XZ the oriented
axes are Z,X about +Y. Geometry and state are checked by Engine.validate_program.
"""

from dataclasses import dataclass, asdict


@dataclass(frozen=True)
class Line:
    start: tuple
    end: tuple

    def to_dict(self):
        return {"kind": "line", "start": tuple(self.start), "end": tuple(self.end)}


@dataclass(frozen=True)
class Arc:
    start: tuple
    end: tuple
    center: tuple
    sweep_rad: float
    plane: str = "xy"

    def to_dict(self):
        return {
            "kind": "arc",
            "start": tuple(self.start),
            "end": tuple(self.end),
            "center": tuple(self.center),
            "sweep_rad": self.sweep_rad,
            "plane": self.plane,
        }


@dataclass(frozen=True)
class Helix(Arc):
    def to_dict(self):
        return {**super().to_dict(), "kind": "helix"}


@dataclass(frozen=True)
class Motion:
    """Machining role and typed motion; all values are mm, mm/min, radians.

    ``kind='rapid'`` always requires whole-assembly stock clearance and removes
    no stock. A positive feed makes it a feed-controlled line traverse; None
    selects unprogrammed rapid speed. ``kind='shift'`` is a rapid line with the
    spindle stopped at its oriented angle (fine boring, after
    :class:`SpindleOrient`). Other roles require a positive feed.
    """

    id: int
    operation: str
    tool: str
    curve: Line | Arc | Helix
    feed_mm_min: float | None = None
    kind: str = "cut"

    def to_dict(self):
        return {
            "kind": "motion",
            "motion": {
                "id": self.id,
                "operation": self.operation,
                "tool": self.tool,
                "curve": self.curve.to_dict(),
                "feed_mm_min": self.feed_mm_min,
                "kind": self.kind,
            },
        }


@dataclass(frozen=True)
class ToolChange:
    tool: str
    number: int
    length_register: int
    position_after_mm: tuple

    def to_dict(self):
        return {"kind": "tool_change", **asdict(self)}


@dataclass(frozen=True)
class Spindle:
    rpm: float
    direction: str = "cw"

    def to_dict(self):
        return {"kind": "spindle", **asdict(self)}


@dataclass(frozen=True)
class Coolant:
    mode: str = "off"

    def to_dict(self):
        return {"kind": "coolant", **asdict(self)}


@dataclass(frozen=True)
class WorkOffset:
    number: int = 1

    def to_dict(self):
        return {"kind": "work_offset", **asdict(self)}


@dataclass(frozen=True)
class Stop:
    optional: bool = False

    def to_dict(self):
        return {"kind": "stop", **asdict(self)}


@dataclass(frozen=True)
class Dwell:
    """Pause in place (G4), e.g. at the full depth of a hole."""

    seconds: float

    def to_dict(self):
        return {"kind": "dwell", **asdict(self)}


@dataclass(frozen=True)
class SpindleOrient:
    """Stop the spindle at its oriented angle (M19), the boring bar's cutting
    edge pointing along ``edge_angle_deg`` (work XY, degrees from +X). Only
    ``kind='shift'`` moves follow until the spindle restarts."""

    edge_angle_deg: float

    def to_dict(self):
        return {"kind": "spindle_orient", "edge_angle_deg": self.edge_angle_deg}


@dataclass(frozen=True)
class Beam:
    """Plasma arc or laser beam on at ``power_percent`` (0: off)."""

    power_percent: float

    def to_dict(self):
        return {"kind": "beam", "power_percent": self.power_percent}


@dataclass(frozen=True)
class HeightControl:
    """Torch height control on or off (while the beam is on)."""

    on: bool

    def to_dict(self):
        return {"kind": "height_control", "on": self.on}


@dataclass(frozen=True)
class SurfaceSpeed:
    """Spindle on at constant surface speed (lathe G96): ``m_min`` metres
    per minute at the tool's radius |x|, never above ``max_rpm`` (G50 or
    G96 D). A ``Spindle`` cancels it (G97)."""

    m_min: float
    max_rpm: float
    direction: str = "cw"

    def to_dict(self):
        return {"kind": "surface_speed", **asdict(self)}


@dataclass(frozen=True)
class FeedPerRev:
    """Feed moves that follow run at ``mm_rev`` per spindle revolution (G95);
    0 returns to feeds per minute (G94). Each such move's ``feed_mm_min`` is
    ``mm_rev`` x the highest spindle speed along it."""

    mm_rev: float

    def to_dict(self):
        return {"kind": "feed_per_rev", "mm_rev": self.mm_rev}


@dataclass(frozen=True)
class Cycle:
    """The next ``covers`` events (motions and a dwell) drill one hole exactly as
    the controller's canned cycle: drill (G81), dwell (G82), peck (G83),
    chip_break (G73), bore (G85), bore_dwell (G89), bore_stop (G86) or
    fine_bore (G76, shift ``shift_mm`` at the bottom), returning to
    ``initial_z_mm`` (G98). A post may write them as one cycle block or as the
    explicit moves."""

    cycle: str
    center_mm: tuple
    z_mm: float
    r_mm: float
    initial_z_mm: float
    feed_mm_min: float
    covers: int
    peck_mm: float | None = None
    dwell_s: float | None = None
    reentry_mm: float | None = None
    lift_mm: float | None = None
    shift_mm: float | None = None

    def to_dict(self):
        data = {"kind": "cycle", **asdict(self), "center_mm": tuple(self.center_mm)}
        for key in ("peck_mm", "dwell_s", "reentry_mm", "lift_mm", "shift_mm"):
            if data[key] is None:
                del data[key]
        return data


@dataclass(frozen=True)
class CutterCompensation:
    """Cutter radius compensation: 'left' (G41), 'right' (G42) of the
    programmed path in the direction of travel, or 'off' (G40). The path is
    the tool center; the controller adds only the D register's wear."""

    side: str
    # The plane it works in: "xy" (mills) or "xz" (lathes, G18).
    plane: str = "xy"

    def to_dict(self):
        data = {"kind": "cutter_compensation", "side": self.side}
        if self.plane != "xy":
            data["plane"] = self.plane
        return data


@dataclass(frozen=True)
class WorkFrame:
    origin_mm: tuple = (0, 0, 0)
    x: tuple = (1, 0, 0)
    y: tuple = (0, 1, 0)
    z: tuple = (0, 0, 1)


@dataclass(frozen=True)
class Program:
    id: str
    revision: str
    tools: tuple
    initial_position_mm: tuple
    events: tuple
    tolerance_mm: float = 1e-6
    frame: WorkFrame = WorkFrame()

    def to_dict(self):
        return {
            "version": 1,
            "id": self.id,
            "revision": self.revision,
            "units": "mm",
            "tolerance_mm": self.tolerance_mm,
            "frame": asdict(self.frame),
            "initial_position_mm": self.initial_position_mm,
            "tools": [tool.to_dict() for tool in self.tools],
            "events": [event.to_dict() for event in self.events],
        }
