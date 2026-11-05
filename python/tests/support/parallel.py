"""Synthetic CAM regression input."""

from camlib.postprocessor import GCodeDialect

from camlib.cadquery import export_surface_mesh
from camlib.operations import Parallel
from camlib.simulation import StockSpec
from camlib.tools import BallEndMill, HolderSection, ToolAssembly


def build_finishing_job(*, revision="1", finish="parallel", rough_stepdown=1):
    """Rough and ball-finish one CAD solid using the same retained stock."""
    from camlib.jobs import CadSource, Job, Setup
    from camlib.operations import Roughing, RasterClearing, Waterline
    from camlib.tools import EndMill
    from camlib.verification import MaterialCheck, StockContactOptions, StreamChecks
    from camlib.material import Options, SurfaceTarget

    if finish not in ("parallel", "waterline", "combined"):
        raise ValueError("finish must be parallel, waterline or combined")
    solid = target()
    surface = export_surface_mesh(solid, source=dict(model="roof", revision=revision))
    rough_tool = ToolAssembly(
        "rough4",
        EndMill("rough-mill", diameter=4, flute_length=20, overall_length=50),
        30,
        60,
        (HolderSection(30, 60, 4, 4),),
    )
    ball = ToolAssembly(
        "ball4",
        BallEndMill("finish-mill", diameter=4, flute_length=10, overall_length=50),
        30,
        60,
        (HolderSection(30, 60, 4, 4),),
    )
    rough = Roughing(
        "rough",
        rough_tool,
        surface,
        12,
        0,
        rough_stepdown,
        14,
        RasterClearing(1.5),
        radial_allowance_mm=0.2,
        bottom_allowance_mm=0.2,
        tool_number=1,
        length_register=1,
        feed_mm_min=600,
        plunge_mm_min=150,
        spindle_rpm=12000,
        coolant="flood",
    )
    parallel = Parallel(
        "parallel",
        ball,
        surface,
        0,
        0.5,
        14,
        tool_number=2,
        length_register=2,
        feed_mm_min=600,
        plunge_mm_min=150,
        spindle_rpm=12000,
        coolant="flood",
    )
    waterline = Waterline(
        "waterline",
        ball,
        surface,
        0,
        8.5,
        0.006,
        0.5,
        14,
        tool_number=2,
        length_register=2,
        feed_mm_min=600,
        plunge_mm_min=150,
        spindle_rpm=12000,
        coolant="flood",
    )
    finishing = (
        (parallel,) if finish == "parallel" else (waterline,) if finish == "waterline" else (waterline, parallel)
    )
    setup = Setup(
        "top",
        StockSpec((0, 0, 0), (24, 16, 12), 0.25),
        (rough_tool, ball),
        (rough, *finishing),
        (0, 0, 14),
        sources=(CadSource.from_shape("roof", revision, solid),),
        verification=StreamChecks(
            stock_contact=StockContactOptions(),
            material=tuple(
                MaterialCheck(SurfaceTarget("roof", revision, surface, 0, "offset"), stage, Options(max_findings=4096))
                for stage in ("rough", None)
            ),
        ),
    )
    return Job("rough-finish", revision, (setup,))


def finishing_machine(job):
    """Example machine whose tool changer preserves compensated tip position."""
    from camlib.post import Machine, ToolSetting

    header = job.compile().setup("top").program_header()
    return Machine(
        "Three-axis finishing mill",
        tuple(
            ToolSetting.for_tool(tool, number=i, length_register=i, position_after_mm=header.initial_position_mm)
            for i, tool in enumerate(header.tools, 1)
        ),
        tool_change_position_mode="preserve_tip",
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


def finishing_verifier(job):
    """Compare retained stock to the planar CAD roof's bounded surface target."""
    from camlib.material import Options, SurfaceTarget, Verifier

    surface = job.setups[0].operations[-1].surface
    return Verifier(
        SurfaceTarget("roof", job.revision, surface, 0, source_enclosure="offset"), Options(max_findings=4096)
    )


def target():
    import cadquery as cq

    return cq.Workplane("XZ").polyline([(0, 0), (24, 0), (24, 4), (12, 8), (0, 4)]).close().extrude(-16)


def parallel_job():
    surface = export_surface_mesh(target(), source=dict(model="CadQuery roof", revision="parallel-1"))
    tool = ToolAssembly(
        "ball4",
        BallEndMill("be4", diameter=4, flute_length=20, overall_length=50),
        30,
        60,
        (HolderSection(30, 60, 4, 4),),
    )
    op = Parallel(
        "finish-roof",
        tool,
        surface,
        0,
        0.5,
        12,
        feed_mm_min=600,
        plunge_mm_min=150,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant="flood",
    )
    return dict(
        stock=StockSpec((0, 0, 0), (24, 16, 10), 0.25),
        tools=(tool,),
        operations=(op,),
        revision="parallel-1",
        initial_position_mm=(0, 0, 12),
    )
