"""Synthetic CAM regression input."""

from camlib.operations import TrochoidalSlot
from camlib.simulation import StockSpec
from camlib.tools import EndMill, HolderSection, ToolAssembly


def slot_job():
    tool = ToolAssembly(
        "t1", EndMill("em4", diameter=4, flute_length=30, overall_length=50), 35, 60, (HolderSection(35, 60, 4, 4),)
    )
    slot = TrochoidalSlot(
        "slot",
        tool,
        (8, 12),
        (28, 12),
        width_mm=6,
        top_z_mm=10,
        floor_z_mm=8,
        stepdown_mm=1,
        clearance_z_mm=12,
        max_engagement_deg=90,
        boundary_tolerance_mm=0.01,
        feed_mm_min=600,
        entry_feed_mm_min=150,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant="flood",
    )
    return dict(
        stock=StockSpec((0, 0, 0), (40, 24, 10), 0.25),
        tools=(tool,),
        operations=(slot,),
        revision="trochoidal-slot-1",
        initial_position_mm=(0, 0, 12),
    )


def cadquery_target():
    """Same closed capsule, cut into the stock; requires the CAD environment."""
    import cadquery as cq

    blank = cq.Workplane("XY").box(40, 24, 10, centered=(False, False, False))
    removal = cq.Workplane("XY").slot2D(26, 6).extrude(2).translate((18, 12, 8))
    return blank.cut(removal)
