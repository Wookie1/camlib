"""Two independent setups with real CAD and selective regeneration."""

from dataclasses import replace
import cadquery as cq
from camlib.cadquery import export_floor_region
from camlib.jobs import CadSource, Job, Setup, OperationNode
from camlib.operations import Facing, Pocket, RestPocket
from camlib.simulation import StockSpec
from camlib.tools import EndMill, ToolAssembly, HolderSection


def build_job(width=10, *, document_revision=1, feed=600, diameter=4, rough_floor=4):
    part = (
        cq.Workplane("XY")
        .box(24, 16, 5, centered=False)
        .cut(cq.Workplane("XY").box(width, 8, 2, centered=False).translate((4, 4, 3)))
    )
    floor = next(
        f
        for f in part.val().Faces()
        if f.geomType() == "PLANE" and f.normalAt().z > 0.999 and abs(f.Center().z - 3) < 1e-7
    )
    source = CadSource.from_shape(
        "main-part", document_revision, part, document_id="main-part", reference="pocket.bottom"
    )
    region = export_floor_region(
        floor,
        name="pocket-floor",
        source=dict(document_id="main-part", revision=str(document_revision), reference="pocket.bottom"),
    )
    tool = ToolAssembly(
        "t1",
        EndMill("em4", diameter=diameter, flute_length=20, overall_length=50, flutes=3),
        30,
        60,
        (HolderSection(30, 60, 4, 4),),
    )
    face = Facing(
        "face",
        tool,
        5,
        1,
        2,
        8,
        feed_mm_min=600,
        plunge_mm_min=150,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant="flood",
    )
    rough = Pocket(
        "rough",
        tool,
        region,
        6,
        rough_floor,
        1,
        1.5,
        8,
        feed_mm_min=feed,
        plunge_mm_min=150,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant="flood",
    )
    finish = RestPocket(
        "finish",
        tool,
        region,
        6,
        3,
        1,
        1,
        8,
        feed_mm_min=600,
        plunge_mm_min=150,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant="flood",
    )
    primary = Setup(
        "main",
        StockSpec((0, 0, 0), (24, 16, 6), 0.5),
        (tool,),
        (OperationNode(face, sources=()), rough, finish),
        (0, 0, 8),
        sources=(source,),
    )
    plate = cq.Workplane("XY").box(12, 12, 3, centered=False)
    secondary_source = CadSource.from_shape("secondary", 1, plate, document_id="secondary-part")
    independent_tool = replace(tool, cutter=EndMill("em4", diameter=4, flute_length=20, overall_length=50, flutes=3))
    secondary = Setup(
        "secondary",
        StockSpec((0, 0, 0), (12, 12, 4), 0.5),
        (independent_tool,),
        (
            Facing(
                "face",
                independent_tool,
                3,
                1,
                2,
                8,
                feed_mm_min=600,
                plunge_mm_min=150,
                spindle_rpm=12000,
                tool_number=1,
                length_register=1,
                coolant="flood",
            ),
        ),
        (0, 0, 8),
        sources=(secondary_source,),
    )
    return Job("cadquery-job", str(document_revision), (primary, secondary))
