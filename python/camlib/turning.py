"""Turning (lathe) jobs: axisymmetric stock, turning tools and operations.

Coordinates are the lathe's: x is the radius from the spindle axis, z runs
along it. The part is given as its half-section ``profile``: points
``(radius, z, bulge)`` from the front face (on the axis) toward the chuck,
the bulge being tan(sweep / 4) of the arc to the next point (positive: G3).

A lathe setup is a ``camlib.jobs.Setup`` with ``TurnStockSpec`` stock and a
``TurningSetup``. Its run plans each operation natively when it is reached
and checks every move: the swept insert against the part (eroded by the
tolerance), the holder against the stock, rapids against the stock, nothing
past the chuck face. The run streams every move with the exact stock change
it makes, like a milling run: playback, posting and export work the same.
"""

from dataclasses import dataclass, field
import json

from ._buffers import native


@dataclass(frozen=True)
class TurnStockSpec:
    z_min_mm: float
    z_max_mm: float
    outer_radius_mm: float
    bore_radius_mm: float = 0
    cell_mm: float = 0.25
    max_cells: int = 1_000_000

    def to_dict(self):
        return dict(
            version=1,
            units="mm",
            z_min_mm=self.z_min_mm,
            z_max_mm=self.z_max_mm,
            outer_radius_mm=self.outer_radius_mm,
            bore_radius_mm=self.bore_radius_mm,
            cell_mm=self.cell_mm,
            max_cells=self.max_cells,
        )


@dataclass(frozen=True)
class TurningInsert:
    """An ISO turning insert: ``included_angle_deg`` (80 C, 55 D, 35 V, 90 S,
    60 T) on a holder with ``approach_angle_deg`` (e.g. 95 or 93)."""

    included_angle_deg: float = 80
    approach_angle_deg: float = 95
    edge_length_mm: float = 12

    def to_dict(self):
        return dict(
            kind="turning",
            included_angle_deg=self.included_angle_deg,
            approach_angle_deg=self.approach_angle_deg,
            edge_length_mm=self.edge_length_mm,
        )


@dataclass(frozen=True)
class GrooveInsert:
    width_mm: float
    depth_mm: float

    def to_dict(self):
        return dict(kind="groove", width_mm=self.width_mm, depth_mm=self.depth_mm)


@dataclass(frozen=True)
class FaceGrooveInsert:
    """A face grooving insert: a blade pointing along -z into the front face,
    ``width_mm`` across in r, reaching ``depth_mm``; the nose radius at both
    corners. Its reference is the inner corner (toward the axis)."""

    width_mm: float = 2
    depth_mm: float = 8

    def to_dict(self):
        return dict(kind="face_groove", width_mm=self.width_mm, depth_mm=self.depth_mm)


@dataclass(frozen=True)
class ThreadInsert:
    profile_angle_deg: float = 60
    depth_mm: float = 3

    def to_dict(self):
        return dict(kind="thread", profile_angle_deg=self.profile_angle_deg, depth_mm=self.depth_mm)


@dataclass(frozen=True)
class DrillInsert:
    """A drill on the spindle axis: diameter, point angle (118, 140; 180
    flat) and flute length. Its reference point is the tip."""

    diameter_mm: float
    point_angle_deg: float = 118
    flute_length_mm: float = 50

    def to_dict(self):
        return dict(
            kind="drill",
            diameter_mm=self.diameter_mm,
            point_angle_deg=self.point_angle_deg,
            flute_length_mm=self.flute_length_mm,
        )


@dataclass(frozen=True)
class TurningTool:
    """A turning tool: insert, nose radius, outside ("outer") or boring
    ("inner"), right hand (cuts toward the chuck) or left, and the holder,
    used for collision checks. Outside: the holder's width along z and
    length away from the cut. Boring ("inner", the insert cutting outward):
    an axial bar ``holder_width_mm`` across (its diameter), its top
    ``holder_offset_mm`` under the nose center, ``holder_length_mm`` out of
    the bore; both sides of the bar are checked inside the bore."""

    id: str
    insert: object = TurningInsert()
    nose_radius_mm: float = 0.8
    side: str = "outer"
    hand: str = "right"
    holder_width_mm: float = 20
    holder_length_mm: float = 100
    holder_offset_mm: float = 0

    def to_dict(self):
        return dict(
            id=self.id,
            insert=self.insert.to_dict(),
            nose_radius_mm=self.nose_radius_mm,
            side=self.side,
            hand=self.hand,
            holder_width_mm=self.holder_width_mm,
            holder_length_mm=self.holder_length_mm,
            holder_offset_mm=self.holder_offset_mm,
        )


@dataclass(frozen=True)
class TurnToolSetting:
    """The tool in the turret: T number and offset (T0101 = 1, 1)."""

    tool: TurningTool
    number: int = 1
    offset: int = 1

    def to_dict(self):
        return dict(tool=self.tool.to_dict(), number=self.number, offset=self.offset)


def _op(kind, data):
    # Unset options (None) are left out: the engine's defaults.
    return dict(kind=kind, **{k: v for k, v in data.items() if v is not None})


@dataclass(frozen=True)
class TurnFace:
    """Face the stock end down to ``z_mm`` in ``stepdown_mm`` passes, across
    the axis."""

    id: str
    tool: str
    z_mm: float
    stepdown_mm: float = 1
    feed_mm_rev: float = field(kw_only=True)
    rpm: float = field(kw_only=True)
    surface_speed_m_min: float | None = None

    def to_dict(self):
        return _op("face", self.__dict__)


@dataclass(frozen=True)
class TurnRough:
    """Outside roughing in axial passes of ``depth_of_cut_mm`` (G71 type I),
    leaving ``allowance_mm``; each pass ends where the part (plus allowance)
    is reached and retracts ``retract_mm`` at 45 degrees."""

    id: str
    tool: str
    depth_of_cut_mm: float = 2
    allowance_mm: float = 0.3
    feed_mm_rev: float = field(kw_only=True)
    rpm: float = field(kw_only=True)
    surface_speed_m_min: float | None = None
    retract_mm: float = 0.5

    def to_dict(self):
        return _op("rough", self.__dict__)


@dataclass(frozen=True)
class TurnGroove:
    """Grooving with a ``GrooveInsert`` tool: plunges across the groove
    between ``z_from_mm`` and ``z_to_mm`` down to ``bottom_radius_mm``,
    overlapping by ``overlap_mm``, in pecks of ``peck_mm`` if set. The groove
    must be in the part profile (the plunges are checked against it). With a
    boring-side tool (``side="inner"``) the groove is internal: it must be in
    the job's ``bore``; the plunges go outward from inside the open hole,
    which the tool enters and leaves along the axis."""

    id: str
    tool: str
    z_from_mm: float
    z_to_mm: float
    bottom_radius_mm: float
    peck_mm: float | None = None
    overlap_mm: float = 0.2
    feed_mm_rev: float = field(kw_only=True)
    rpm: float = field(kw_only=True)
    surface_speed_m_min: float | None = None

    def to_dict(self):
        data = _op("groove", self.__dict__)
        if self.peck_mm is None:
            data.pop("peck_mm", None)
        return data


@dataclass(frozen=True)
class TurnPartOff:
    """Parting off with a ``GrooveInsert`` tool: its +z side at ``z_mm`` (the
    part's back face), plunged to the axis. The blade must be deeper than
    the stock radius there (its holder is checked against the stock)."""

    id: str
    tool: str
    z_mm: float
    peck_mm: float | None = None
    feed_mm_rev: float = field(kw_only=True)
    rpm: float = field(kw_only=True)
    surface_speed_m_min: float | None = None

    def to_dict(self):
        data = _op("part_off", self.__dict__)
        if self.peck_mm is None:
            data.pop("peck_mm", None)
        return data


@dataclass(frozen=True)
class TurnThread:
    """Single-point threading with a ``ThreadInsert`` tool: ``passes``
    spindle-synchronized passes (constant chip area) from ``z_start_mm`` to
    ``z_end_mm``, infeeding along ``infeed_angle_deg`` (0 radial, 29.5
    flank), plus ``spring_passes``. Outside: down to ``major_radius_mm -
    depth_mm``. Inside (a boring-side tool, the job's ``bore`` at the
    minor diameter): out to ``major_radius_mm``, pulling out toward the
    axis. Posted as G33 K<lead> (LinuxCNC) or G32 F<lead> where the
    dialect declares ``thread_synchronized``."""

    id: str
    tool: str
    z_start_mm: float
    z_end_mm: float
    major_radius_mm: float
    pitch_mm: float
    depth_mm: float
    passes: int = 6
    infeed_angle_deg: float = 29.5
    spring_passes: int = 1
    rpm: float = field(kw_only=True)
    surface_speed_m_min: float | None = None

    def to_dict(self):
        return _op("thread", self.__dict__)


@dataclass(frozen=True)
class FaceGroove:
    """An annular groove in the part's front face (declared on the job):
    between ``inner_radius_mm`` and ``outer_radius_mm``, from the face down to
    ``bottom_z_mm``, square corners."""

    inner_radius_mm: float
    outer_radius_mm: float
    bottom_z_mm: float

    def to_dict(self):
        return dict(self.__dict__)


@dataclass(frozen=True)
class TurnFaceGroove:
    """Face grooving with a ``FaceGrooveInsert`` tool: axial plunges across a
    groove declared in ``TurningJob.face_grooves`` (same radii and bottom),
    overlapping by ``overlap_mm``, in pecks of ``peck_mm`` if set. The blade
    is checked to stay inside the declared groove below the face."""

    id: str
    tool: str
    inner_radius_mm: float
    outer_radius_mm: float
    bottom_z_mm: float
    peck_mm: float | None = None
    overlap_mm: float = 0.2
    feed_mm_rev: float = field(kw_only=True)
    rpm: float = field(kw_only=True)
    surface_speed_m_min: float | None = None

    def to_dict(self):
        return _op("face_groove", self.__dict__)


@dataclass(frozen=True)
class TurnDrill:
    """Drilling on the spindle axis with a ``DrillInsert`` tool, the tip to
    ``depth_z_mm``, in pecks of ``peck_mm`` (out of the hole after each) if
    set. The job must declare the part's ``bore``: the drill is checked to
    stay inside it."""

    id: str
    tool: str
    depth_z_mm: float
    peck_mm: float | None = None
    feed_mm_rev: float = field(kw_only=True)
    rpm: float = field(kw_only=True)
    surface_speed_m_min: float | None = None

    def to_dict(self):
        data = _op("drill", self.__dict__)
        if self.peck_mm is None:
            data.pop("peck_mm", None)
        return data


@dataclass(frozen=True)
class TurnBoreRough:
    """Inside roughing with a boring tool (``side="inner"``): axial passes
    opening the existing hole by at most ``depth_of_cut_mm``, with a pass at
    every bore diameter, each from the front face to where the bore (less
    ``allowance_mm``) is reached or ``z_end_mm``, retracting ``retract_mm``
    at 45 degrees toward the axis. Needs the job's ``bore`` and an open hole
    (stock bore or a drilled one)."""

    id: str
    tool: str
    z_end_mm: float
    depth_of_cut_mm: float = 1.5
    allowance_mm: float = 0.2
    feed_mm_rev: float = field(kw_only=True)
    rpm: float = field(kw_only=True)
    surface_speed_m_min: float | None = None
    retract_mm: float = 0.5

    def to_dict(self):
        return _op("bore_rough", self.__dict__)


@dataclass(frozen=True)
class TurnBoreFinish:
    """Inside finishing along the job's ``bore`` from the front face to
    ``z_end_mm`` with a boring tool, the nose center ``nose radius +
    allowance_mm`` inside it; out toward the axis and along it."""

    id: str
    tool: str
    z_end_mm: float
    allowance_mm: float = 0
    feed_mm_rev: float = field(kw_only=True)
    rpm: float = field(kw_only=True)
    # "computer": the nose path; "wear": the same path with G41/G42 on, the
    # control applying only the wear in its offset's nose radius.
    compensation: str = "computer"
    surface_speed_m_min: float | None = None

    def to_dict(self):
        return _op("bore_finish", self.__dict__)


@dataclass(frozen=True)
class TurnFinish:
    """Outside finishing along the profile (front face first), the nose
    center ``nose radius + allowance_mm`` outside it."""

    id: str
    tool: str
    allowance_mm: float = 0
    feed_mm_rev: float = field(kw_only=True)
    rpm: float = field(kw_only=True)
    # "computer": the nose path; "wear": the same path with G41/G42 on, the
    # control applying only the wear in its offset's nose radius.
    compensation: str = "computer"
    surface_speed_m_min: float | None = None

    def to_dict(self):
        return _op("finish", self.__dict__)


@dataclass(frozen=True, kw_only=True)
class TurningSetup:
    """The turning definition of a lathe ``camlib.jobs.Setup`` (whose stock
    is a ``TurnStockSpec``, tools ``TurnToolSetting`` and operations the
    ``Turn*`` operations, in order).

    ``profile``: the part's half-section (radius, z, bulge) from the front
    face on the axis toward the chuck (``camlib.cadquery.turning_section``
    takes it from a solid). ``chuck_z_mm``: the chuck face; no tool part may
    reach below it (plus the clearance). ``clearance_mm``: the approach and
    retract clearance. ``tolerance_mm``: the tolerance of every move check.
    ``bore``: the part's hole (radius, z, bulge) from the front face to its
    deepest point; drilling and boring stay inside it.

    Every operation runs at ``rpm``, or at constant surface speed
    ``surface_speed_m_min`` (G96) with ``rpm`` as its spindle limit (G50);
    constant surface speed needs ``feed_per_rev=True`` (G95: each feed is
    ``feed_mm_rev``). Threading and drilling run at a fixed rpm.
    ``controlled_point``: the point the program moves and the start position
    names, "tip" (each tool's imaginary tip, where offsets touched off on a
    diameter and a face are measured) or "nose_center". ``face_grooves``:
    annular grooves in the front face (``FaceGroove``), cut by
    ``TurnFaceGroove``."""

    profile: tuple
    chuck_z_mm: float
    clearance_mm: float
    tolerance_mm: float
    bore: tuple | None = None
    feed_per_rev: bool = False
    controlled_point: str = "tip"
    face_grooves: tuple = ()

    @classmethod
    def from_section(cls, section, **settings):
        """From a ``camlib.cadquery.turning_section`` result: its profile,
        bore and face grooves; ``settings`` give the rest."""
        return cls(profile=section.profile, bore=section.bore, face_grooves=section.face_grooves, **settings)

    def to_dict(self, tools, operations):
        """The native turning setup with the setup's tools and operations."""
        result = dict(
            part=dict(points=[list(p) for p in self.profile]),
            chuck_z_mm=self.chuck_z_mm,
            tools=[t if isinstance(t, dict) else t.to_dict() for t in tools],
            operations=[o if isinstance(o, dict) else o.to_dict() for o in operations],
            clearance_mm=self.clearance_mm,
            tolerance_mm=self.tolerance_mm,
            feed_per_rev=self.feed_per_rev,
            controlled_point=self.controlled_point,
            face_grooves=[g.to_dict() for g in self.face_grooves],
        )
        if self.bore is not None:
            result["bore"] = dict(points=[list(p) for p in self.bore])
        return result


def proxy_tools(turning):
    """The posting stand-ins of a native turning setup's tools (the lathe
    run's tool assemblies: id and identity for the program header)."""
    return json.loads(native.turn_proxy_tools(json.dumps(turning)))
