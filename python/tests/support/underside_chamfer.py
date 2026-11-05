"""Synthetic CAM regression input."""

from camlib.postprocessor import GCodeDialect

import cadquery as cq
from camlib.cadquery import export_floor_region
from camlib.jobs import CadSource, Job, Setup
from camlib.motion import WorkFrame
from camlib.operations import Contour, Chamfer
from camlib.post import Machine, ToolSetting
from camlib.simulation import StockSpec
from camlib.tools import EndMill, ChamferMill, ToolAssembly, HolderSection
from camlib.verification import StreamChecks, StockContactOptions


def geometry():
    blank = cq.Workplane('XY').box(36, 24, 12, centered=(True, True, False)).edges('|Z').fillet(3)
    part = blank.edges('<Z').chamfer(0.8).val()
    # The region is the unbeveled wall at the lower end of the chamfer,
    # not the smaller finished bottom face.
    section = cq.Workplane('XY').newObject([part]).section(0.8).val()
    face = cq.Face.makeFromWires(max(section.Wires(), key=lambda w: w.Length()))
    return part, face, WorkFrame((0, 0, 0), (1, 0, 0), (0, -1, 0), (0, 0, -1))


def build_job():
    part, face, frame = geometry()
    region = export_floor_region(face, name='unbeveled-wall', source={'revision': '1'}, work_frame=frame)

    def assembly(id, cutter):
        return ToolAssembly(id, cutter, 12, 35, (HolderSection(12, 35, 4, 4),))

    mill = assembly('mill', EndMill('flat', diameter=4, flute_length=8, overall_length=30))
    cone = assembly(
        'bevel',
        ChamferMill(
            'cone', diameter=6, flute_length=8, overall_length=30, tip_diameter=0.5, angle=90, center_cutting=False
        ),
    )
    operations = (
        Contour(
            'clear-edge',
            mill,
            region,
            floor_z_mm=-2.5,
            stepdown_mm=1.25,
            clearance_z_mm=4,
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        ),
        Chamfer(
            'bottom-bevel',
            cone,
            region,
            top_z_mm=0,
            width_mm=0.8,
            width_step_mm=0.4,
            clearance_z_mm=4,
            tool_number=2,
            length_register=2,
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            coolant="flood",
        ),
    )
    setup = Setup(
        'bottom',
        StockSpec((-18, -12, -3), (18, 12, 0), 0.2),
        (mill, cone),
        operations,
        (0, 0, 4),
        sources=(CadSource.from_shape('part', '1', part),),
        verification=StreamChecks(stock_contact=StockContactOptions()),
    )
    return Job('underside', '1', (setup,))


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
