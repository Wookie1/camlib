"""Synthetic CAM regression input."""

import json
from pathlib import Path

from camlib.operations import Pocket
from camlib.planar import PlanarRegion
from camlib.simulation import StockSpec
from camlib.tools import EndMill, HolderSection, ToolAssembly


def cadquery_job():
    root = Path(__file__).resolve().parents[3]
    region = PlanarRegion.from_dict(json.loads((root / "fixtures/planar/cadquery-round-island.json").read_text()))
    assembly = ToolAssembly(
        "t1", EndMill("em4", diameter=4, flute_length=30, overall_length=50), 35, 60, (HolderSection(35, 60, 4, 4),)
    )
    operation = Pocket(
        "clear-pocket",
        assembly,
        region,
        top_z_mm=20,
        floor_z_mm=region.source["floor_z_mm"],
        stepdown_mm=2,
        stepover_mm=1.5,
        clearance_z_mm=25,
        feed_mm_min=600,
        plunge_mm_min=150,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant="flood",
    )
    return dict(
        stock=StockSpec((0, 0, 0), (80, 60, 20), 0.2),
        tools=(assembly,),
        operations=(operation,),
        revision=region.source["revision"],
        initial_position_mm=(0, 0, 25),
    )
