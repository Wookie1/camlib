"""Synthetic CAM regression input."""

from camlib.operations import Facing
from camlib.simulation import StockSpec
from camlib.tools import EndMill, HolderSection, ToolAssembly


def facing_job():
    tool = ToolAssembly(
        "face8", EndMill("em8", diameter=8, flute_length=30, overall_length=50), 35, 60, (HolderSection(35, 60, 5, 5),)
    )
    face = Facing(
        "face",
        tool,
        floor_z_mm=8,
        stepdown_mm=1,
        stepover_mm=6,
        clearance_z_mm=12,
        feed_mm_min=600,
        plunge_mm_min=150,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant="flood",
    )
    return dict(
        stock=StockSpec((0, 0, 0), (40, 24, 10), 0.25),
        tools=(tool,),
        operations=(face,),
        revision="facing-1",
        initial_position_mm=(0, 0, 12),
    )


def cadquery_target():
    import cadquery as cq

    return cq.Workplane("XY").box(40, 24, 8, centered=(False, False, False))
