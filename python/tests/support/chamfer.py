"""Synthetic CAM regression input."""

from camlib.cadquery import export_floor_region
from camlib.operations import Contour, Chamfer
from camlib.simulation import StockSpec
from camlib.tools import EndMill, ChamferMill, HolderSection, ToolAssembly


def retained_part():
    import cadquery as cq

    return cq.Workplane("XY").circle(8).circle(4).extrude(8).translate((20, 20, 2))


def chamfer_job():
    part = retained_part()
    region = export_floor_region(
        part.faces(">Z").val(), name="annular-top", source=dict(revision="chamfer-1", model="CadQuery annular top")
    )
    rough = ToolAssembly(
        "mill4", EndMill("em4", diameter=4, flute_length=30, overall_length=50), 35, 60, (HolderSection(35, 60, 4, 4),)
    )
    bevel = ToolAssembly(
        "chamfer6",
        ChamferMill(
            "bevel-bit",
            diameter=6,
            flute_length=20,
            overall_length=50,
            angle=90,
            tip_diameter=0.5,
            center_cutting=False,
        ),
        30,
        60,
        (HolderSection(30, 60, 4, 4),),
    )
    contour = Contour(
        "rough-profile",
        rough,
        region,
        2,
        2,
        12,
        boundaries="all",
        feed_mm_min=600,
        plunge_mm_min=150,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant="flood",
    )
    chamfer = Chamfer(
        "top-bevel",
        bevel,
        region,
        10,
        1,
        12,
        boundaries="all",
        tool_number=2,
        length_register=2,
        feed_mm_min=600,
        plunge_mm_min=150,
        spindle_rpm=12000,
        coolant="flood",
    )
    return dict(
        stock=StockSpec((0, 0, 0), (40, 40, 10), 0.2),
        tools=(rough, bevel),
        operations=(contour, chamfer),
        revision="chamfer-1",
        initial_position_mm=(0, 0, 12),
    )


def cadquery_target():
    import cadquery as cq

    part = retained_part()
    beveled = part.edges(">Z").chamfer(1)
    blank = cq.Workplane("XY").box(40, 40, 10, centered=(False, False, False))
    band = cq.Workplane("XY").circle(12).circle(8).extrude(8).translate((20, 20, 2))
    hole = cq.Workplane("XY").circle(4).extrude(8).translate((20, 20, 2))
    return blank.cut(band).cut(hole).cut(part.cut(beveled))
