"""Synthetic CAM regression input."""

from camlib.postprocessor import GCodeDialect


def target():
    import cadquery as cq

    part = cq.Workplane('XY').box(18, 14, 8, centered=False).edges('|Z').fillet(2).edges('>Z').fillet(1)
    part = part.cut(cq.Workplane('XY').center(12, 7).circle(2.2).extrude(8))
    part = part.cut(cq.Workplane('XY').center(12, 7).circle(3).extrude(3).translate((0, 0, 5)))
    recess = (
        cq.Workplane('XZ').moveTo(4, 2).bezier([(4, 5), (7, 5)], includeCurrent=True).lineTo(7, 2).close().extrude(-1.5)
    )
    return part.cut(recess)


def build_job(*, cad_contacts=False, cad_inner_patches=False):
    from camlib.cadquery import export_surface_mesh, export_contact_witnesses, export_inner_patches
    from camlib.jobs import CadSource, Job, Setup
    from camlib.operations import Parallel
    from camlib.simulation import StockSpec
    from camlib.tools import BallEndMill, ToolAssembly, HolderSection
    from camlib.verification import StreamChecks, StockContactOptions, MaterialCheck
    from camlib.material import SurfaceTarget, FinishingScope

    part = target()
    surface = export_surface_mesh(part, source={'revision': '1'})
    tool = ToolAssembly(
        'ball',
        BallEndMill('bit', diameter=3, flute_length=20, overall_length=40),
        25,
        50,
        (HolderSection(25, 50, 4, 4),),
    )
    operation = Parallel(
        'finish',
        tool,
        surface,
        6,
        0.5,
        11,
        feed_mm_min=600,
        plunge_mm_min=150,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant="flood",
    )
    witnesses = export_contact_witnesses(part, min_z_mm=6) if cad_contacts else None
    patches = export_inner_patches(part) if cad_inner_patches else ()
    setup = Setup(
        'top',
        StockSpec((0, 0, 0), (18, 14, 8.5), 0.25),
        (tool,),
        (operation,),
        (0, 0, 11),
        sources=(CadSource.from_shape('rounded-solid', '1', part),),
        verification=StreamChecks(
            stock_contact=StockContactOptions(),
            material=(
                MaterialCheck(
                    SurfaceTarget(
                        'curved',
                        '1',
                        surface,
                        0,
                        finishing=FinishingScope.from_operation(operation),
                        contact_witnesses=witnesses,
                        inner_patches=patches,
                    ),
                    after_operation=operation.id,
                ),
            ),
        ),
    )
    return Job('curved-solid', '1', (setup,))


def build_machining_job(*, rest_clearing=None):
    """One retained curved solid, offset roughing, smaller rest cutter, ball finish.

    rest_clearing accepts an ordinary RasterClearing or OffsetClearing instance.

    The rest pass also reduces radial allowance from .3 to .1 mm. Top finishing
    stops at Z6; the underside and side recess require separate setups.
    """
    from camlib.cadquery import export_surface_mesh
    from camlib.jobs import CadSource, Job, Setup
    from camlib.operations import (
        Roughing,
        RestRoughing,
        Parallel,
        OffsetClearing,
        RasterClearing,
        StockEdgeEntry,
        HelicalEntry,
    )
    from camlib.simulation import StockSpec
    from camlib.tools import EndMill, BallEndMill, ToolAssembly, HolderSection
    from camlib.verification import StreamChecks, StockContactOptions, MaterialCheck
    from camlib.material import SurfaceTarget

    part = target()
    surface = export_surface_mesh(part, source={'revision': '1'})

    def assembly(name, cutter):
        return ToolAssembly(name, cutter, 25, 50, (HolderSection(25, 50, 4, 4),))

    rougher = assembly('rougher', EndMill('rough-bit', diameter=2, flute_length=20, overall_length=40))
    smaller = assembly('smaller', EndMill('rest-bit', diameter=1, flute_length=20, overall_length=40))
    ball = assembly('ball', BallEndMill('ball-bit', diameter=3, flute_length=20, overall_length=40))
    entry = StockEdgeEntry(helix=HelicalEntry(radius_mm=0.5))
    rough = Roughing(
        'rough',
        rougher,
        surface,
        10,
        2,
        2,
        12,
        OffsetClearing(1.0),
        radial_allowance_mm=0.3,
        bottom_allowance_mm=0.2,
        entry=entry,
        omit_empty_cuts=True,
        feed_mm_min=600,
        plunge_mm_min=150,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant="flood",
    )
    rest = RestRoughing(
        'rest',
        smaller,
        surface,
        10,
        2,
        2,
        12,
        rest_clearing if rest_clearing is not None else RasterClearing(0.4),
        radial_allowance_mm=0.1,
        bottom_allowance_mm=0.2,
        entry=entry,
        tool_number=2,
        length_register=2,
        feed_mm_min=600,
        plunge_mm_min=150,
        spindle_rpm=12000,
        coolant="flood",
    )
    finish = Parallel(
        'finish',
        ball,
        surface,
        6,
        0.5,
        12,
        tool_number=3,
        length_register=3,
        feed_mm_min=600,
        plunge_mm_min=150,
        spindle_rpm=12000,
        coolant="flood",
    )
    material = SurfaceTarget('curved', '1', surface, min(p[2] for t in surface.triangles for p in t))
    setup = Setup(
        'top',
        StockSpec((-2, -2, 0), (20, 16, 10), 0.25),
        (rougher, smaller, ball),
        (rough, rest, finish),
        (0, 0, 12),
        sources=(CadSource.from_shape('rounded-solid', '1', part),),
        verification=StreamChecks(
            stock_contact=StockContactOptions(),
            material=tuple(MaterialCheck(material, after_operation=op.id) for op in (rough, rest, finish)),
        ),
    )
    return Job('curved-machining', '1', (setup,))


def machine_for(job):
    from camlib.post import Machine, ToolSetting

    header = job.compile().setup('top').program_header()
    return Machine(
        'Example mill',
        tuple(
            ToolSetting.for_tool(tool, number=i, length_register=i, position_after_mm=header.initial_position_mm)
            for i, tool in enumerate(header.tools, 1)
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
