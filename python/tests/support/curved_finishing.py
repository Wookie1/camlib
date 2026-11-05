"""Synthetic CAM regression input."""

from camlib.cadquery import export_hemisphere_surface
from camlib.operations import Parallel, Waterline
from camlib.simulation import StockSpec
from camlib.tools import BallEndMill, HolderSection, ToolAssembly


def target():
    import cadquery as cq

    hemisphere = cq.Solid.makeSphere(4, cq.Vector(8, 8, 2), angleDegrees1=0, angleDegrees2=90, angleDegrees3=360)
    base = cq.Workplane("XY").box(16, 16, 2, centered=(False, False, False))
    return base.union(cq.Workplane("XY").newObject([hemisphere]))


def curved_job(strategy="parallel"):
    part = target()
    face = next(f for f in part.val().Faces() if f.geomType() == "SPHERE")
    surface = export_hemisphere_surface(face, source=dict(model="CadQuery spherical boss", revision="curved-1"))
    tool = ToolAssembly(
        "ball4",
        BallEndMill("be4", diameter=4, flute_length=20, overall_length=50),
        30,
        60,
        (HolderSection(30, 60, 4, 4),),
    )
    if strategy == "parallel":
        op = Parallel(
            "finish-cap",
            tool,
            surface,
            2,
            0.5,
            10,
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        )
    elif strategy == "waterline":
        op = Waterline(
            "finish-levels",
            tool,
            surface,
            2,
            6,
            2.5,
            2,
            10,
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        )
    else:
        raise ValueError("strategy must be parallel or waterline")
    return dict(
        stock=StockSpec((0, 0, 0), (16, 16, 8), 0.4),
        tools=(tool,),
        operations=(op,),
        revision="curved-1",
        initial_position_mm=(0, 0, 10),
    )
