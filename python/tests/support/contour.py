"""Synthetic CAM regression input."""

from camlib.cadquery import export_floor_region
from camlib.operations import Contour
from camlib.simulation import StockSpec
from camlib.tools import EndMill, HolderSection, ToolAssembly


def retained_part():
    import cadquery as cq

    return cq.Workplane("XY").circle(8).circle(4).extrude(8).translate((20, 20, 2))


def contour_job():
    part = retained_part()
    region = export_floor_region(
        part.faces(">Z").val(), name="retained-annulus", source=dict(revision="contour-1", model="CadQuery annulus")
    )
    tool = ToolAssembly(
        "t1", EndMill("em4", diameter=4, flute_length=30, overall_length=50), 35, 60, (HolderSection(35, 60, 4, 4),)
    )
    op = Contour(
        "profile",
        tool,
        region,
        floor_z_mm=2,
        stepdown_mm=2,
        clearance_z_mm=12,
        boundaries="all",
        feed_mm_min=600,
        plunge_mm_min=150,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant="flood",
    )
    return dict(
        stock=StockSpec((0, 0, 0), (40, 40, 10), 0.25),
        tools=(tool,),
        operations=(op,),
        revision="contour-1",
        initial_position_mm=(0, 0, 12),
    )


def cadquery_target():
    """Nominal swept-band target; native numeric margin is reported separately."""
    import cadquery as cq

    blank = cq.Workplane("XY").box(40, 40, 10, centered=(False, False, False))
    band = cq.Workplane("XY").circle(12).circle(8).extrude(8).translate((20, 20, 2))
    bore = cq.Workplane("XY").circle(4).extrude(8).translate((20, 20, 2))
    return blank.cut(band).cut(bore)
