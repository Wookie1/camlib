"""Synthetic CAM regression input."""

from camlib.cadquery import export_surface_mesh
from camlib.operations import Waterline
from camlib.simulation import StockSpec
from camlib.tools import BallEndMill, HolderSection, ToolAssembly


def target():
    import cadquery as cq

    base = cq.Workplane("XY").box(24, 16, 2, centered=(False, False, False))
    island = cq.Workplane("XY").workplane(offset=2).center(12, 8).rect(16, 10).extrude(6, taper=20)
    return base.union(island)


def waterline_job():
    surface = export_surface_mesh(target(), source=dict(model="CadQuery tapered island", revision="waterline-1"))
    tool = ToolAssembly(
        "ball4",
        BallEndMill("be4", diameter=4, flute_length=20, overall_length=50),
        30,
        60,
        (HolderSection(30, 60, 4, 4),),
    )
    op = Waterline(
        "finish-levels",
        tool,
        surface,
        0,
        8,
        2.5,
        2,
        12,
        feed_mm_min=600,
        plunge_mm_min=150,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant="flood",
    )
    return dict(
        stock=StockSpec((0, 0, 0), (24, 16, 10), 0.4),
        tools=(tool,),
        operations=(op,),
        revision="waterline-1",
        initial_position_mm=(0, 0, 12),
    )
