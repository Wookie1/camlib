"""Synthetic CAM regression input."""

from camlib.postprocessor import GCodeDialect

import cadquery as cq
from camlib.cadquery import export_floor_region
from camlib.jobs import CadSource, Job, Setup
from camlib.motion import WorkFrame
from camlib.operations import Pocket
from camlib.post import Machine, ToolSetting
from camlib.tools import EndMill, HolderSection, ToolAssembly
from camlib.simulation import StockSpec
from camlib.verification import StockContactOptions, StreamChecks


def geometry():
    edge = cq.Edge.makeBezier(
        [cq.Vector(-4, 0, -0.6), cq.Vector(-2, 6, -0.6), cq.Vector(3, 5, -0.6), cq.Vector(5, 0, -0.6)]
    )
    outer = cq.Wire.assembleEdges([edge, cq.Edge.makeLine((5, 0, -0.6), (-4, 0, -0.6))])
    hole = cq.Workplane('XY').center(0, 1.5).circle(0.5).val().translate((0, 0, -0.6))
    floor = cq.Face.makeFromWires(outer, [hole])
    frame = WorkFrame((0, -12, 6), (1, 0, 0), (0, 0, 1), (0, -1, 0))
    placed = floor.rotate((0, 0, 0), (1, 0, 0), 90).translate(frame.origin_mm)
    return placed, frame


def build_job(revision='1', max_error_mm=0.005):
    floor, frame = geometry()
    region = export_floor_region(
        floor, name='curved-pocket', source={'revision': revision}, work_frame=frame, max_error_mm=max_error_mm
    )
    tool = ToolAssembly(
        'small-mill',
        EndMill('flat', diameter=0.8, flute_length=3, overall_length=30),
        8,
        30,
        (HolderSection(8, 30, 3, 3),),
    )
    operation = Pocket(
        'pocket',
        tool,
        region,
        top_z_mm=0,
        floor_z_mm=-0.6,
        stepdown_mm=0.3,
        stepover_mm=0.3,
        clearance_z_mm=3,
        feed_mm_min=150,
        plunge_mm_min=50,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant="flood",
    )
    setup = Setup(
        'side',
        StockSpec((-5, -1, -1), (6, 5, 0), 0.1),
        (tool,),
        (operation,),
        (0, 0, 3),
        sources=(CadSource.from_shape('sketch', revision, floor),),
        verification=StreamChecks(stock_contact=StockContactOptions()),
    )
    return Job('side-pockets', revision, (setup,))


def machine_for(job):
    setup = job.setups[0]
    return Machine(
        'Example mill',
        tuple(
            ToolSetting.for_tool(t, number=i, length_register=i, position_after_mm=setup.initial_position_mm)
            for i, t in enumerate(setup.tools, 1)
        ),
        tool_change_position_mode='preserve_tip',
        dialect=GCodeDialect.linuxcnc(),
        output_units='mm',
        decimals=3,
        max_feed_mm_min=10000,
        max_spindle_rpm=30000,
        planes=('xy', 'xz', 'yz'),
        arcs=True,
        helices=True,
        max_sweep_rad=6.283185307179586,
        max_tool_number=99999,
        max_length_register=99999,
        tool_changes=True,
        flood=True,
        mist=True,
        rapid_mm_min=6000,
    )
