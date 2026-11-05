"""Synthetic CAM regression input."""

from camlib.cadquery import export_floor_region
from camlib.operations import AdaptivePocket, Pocket
from camlib.simulation import StockSpec
from camlib.tools import EndMill, HolderSection, ToolAssembly


def cavity():
    import cadquery as cq

    return cq.Workplane("XY").box(20, 20, 1, centered=(False, False, False)).translate((3, 3, 9))


def target():
    import cadquery as cq

    return cq.Workplane("XY").box(26, 26, 10, centered=(False, False, False)).cut(cavity())


def rest_job():
    region = export_floor_region(
        cavity().faces("<Z").val(),
        name="rectangular pocket",
        source=dict(model="CadQuery corner rest", revision="rest-1"),
    )
    large = ToolAssembly(
        "rough8", EndMill("em8", diameter=8, flute_length=20, overall_length=50), 30, 60, (HolderSection(30, 60, 4, 4),)
    )
    small = ToolAssembly(
        "rest4", EndMill("em4", diameter=4, flute_length=20, overall_length=50), 30, 60, (HolderSection(30, 60, 4, 4),)
    )
    rough = Pocket(
        "rough",
        large,
        region,
        10,
        9,
        1,
        3,
        12,
        tool_number=2,
        length_register=2,
        feed_mm_min=600,
        plunge_mm_min=150,
        spindle_rpm=12000,
        coolant="flood",
    )
    rest = AdaptivePocket(
        "rest",
        small,
        region,
        10,
        9,
        1,
        1.5,
        12,
        path_strategy="stock_front",
        feed_mm_min=600,
        plunge_mm_min=150,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant="flood",
    )
    return dict(
        stock=StockSpec((0, 0, 0), (26, 26, 10), 0.4),
        tools=(large, small),
        operations=(rough, rest),
        revision="rest-1",
        initial_position_mm=(0, 0, 12),
    )
