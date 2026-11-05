"""Synthetic CAM regression input."""

from dataclasses import asdict, replace
import hashlib
import inspect
import math
from pathlib import Path
import sys
import time

from camlib.material import Options as MaterialOptions, Target
from camlib.motion import Program
from camlib.planar import Component, PlanarRegion
from camlib.post import GCodeDialect, GCodePost, LoweringOptions, Machine, ToolSetting, post_events
from camlib.simulation import OperationUpdate, Simulation, VerificationUpdate
from camlib.verification import MaterialCheck, Prism, Scene, StockContactOptions, StreamChecks
from support.pocket import cadquery_job

EXAMPLES = Path(__file__).resolve().parents[3] / 'examples'

# The test machine (camlib assumes nothing about a machine): LinuxCNC, fine
# precision so the posted stock matches the source exactly.
LINUXCNC = dict(
    dialect=GCodeDialect.linuxcnc(),
    output_units='mm',
    decimals=6,
    tolerance_mm=0.001,
    feed_tolerance_mm_min=0.0001,
    max_feed_mm_min=10000,
    max_spindle_rpm=30000,
    rapid_mm_min=6000,
    planes=('xy', 'xz', 'yz'),
    arcs=True,
    helices=True,
    max_sweep_rad=math.tau,
    max_tool_number=99999,
    max_length_register=99999,
    tool_changes=True,
    flood=True,
    mist=True,
    tool_change_position_mode='table',
)


def grbl_post():
    sys.path.insert(0, str(EXAMPLES))
    try:
        import grbl_post
    finally:
        sys.path.remove(str(EXAMPLES))
    return grbl_post


def run(
    nc_path,
    *,
    profile='linuxcnc',
    lowering=False,
    radial_allowance_mm=0,
    post_type=None,
    machine_options=None,
    machine_builder=None,
    decoder_type=None,
):
    """profile: 'linuxcnc' (or 'python', with machine_builder/machine_options)
    posts through a LinuxCNC Machine; 'grbl' through examples/grbl_post.py."""
    if profile not in ('linuxcnc', 'python', 'grbl'):
        raise ValueError('profile is linuxcnc, python or grbl')
    config = cadquery_job()
    config['operations'] = (replace(config['operations'][0], radial_allowance_mm=radial_allowance_mm),)
    tool = config['tools'][0]
    operation = config['operations'][0]
    header = Program(
        'posted-cadquery-pocket',
        config['revision'],
        config['tools'],
        config['initial_position_mm'],
        (),
        tolerance_mm=1e-5,
    )
    setting = ToolSetting.for_tool(tool, number=1, length_register=1, position_after_mm=config['initial_position_mm'])
    if profile == 'grbl':
        grbl = grbl_post()
        machine = grbl.define_machine(
            'GRBL pocket router',
            setting,
            work_offset_machine_mm=(0, 0, 0),
            initial_machine_mm=config['initial_position_mm'],
            output_units='mm',
            decimals=6,
            tolerance_mm=0.001,
            max_feed_mm_min=10000,
            max_spindle_rpm=30000,
            rapid_mm_min=6000,
            flood=True,
            mist=False,
        )
        post_type = post_type or grbl.GrblPost
        decoder_type = decoder_type or grbl.GrblDecoder
        lowering = True
    elif machine_builder is not None:
        machine = machine_builder(header, (setting,))
    else:
        machine = Machine('Python defined pocket machine', (setting,), **{**LINUXCNC, **(machine_options or {})})
    island = PlanarRegion(
        'kept-island',
        (Component(operation.region.components[0].holes[0]),),
        boundary_error_mm=operation.region.boundary_error_mm,
        source=operation.region.source,
    )
    scene = Scene(
        'posted-island',
        config['revision'],
        (Prism('kept-island', island, operation.floor_z_mm, operation.top_z_mm, 'protected_target'),),
    )
    checks = StreamChecks(
        scene,
        stock_contact=StockContactOptions(),
        material=(
            MaterialCheck(
                Target('finished-part', config['revision'], operation.region, operation.floor_z_mm, 1e-7),
                options=MaterialOptions(max_findings=8),
            ),
        ),
    )
    posted_config = {k: v for k, v in config.items() if k != 'operations'}
    posted_config.update(revision=config['revision'] + '-posted', verification=checks)
    first = None
    definition = None
    max_deviation = 0.0
    lines = motions = chunks = source_motions = 0
    counts = dict(clear=0, violation=0, uncertain=0)
    material = None
    issue_samples = []
    temporary = nc_path.with_suffix(nc_path.suffix + '.part')
    if nc_path.exists() or temporary.exists():
        raise ValueError('preserve output: choose a new NC path')
    nc_path.parent.mkdir(parents=True, exist_ok=True)
    began = time.perf_counter()
    with Simulation(**config) as source, Simulation(**posted_config) as posted, temporary.open('w') as output:

        def events():
            for update in source.run_operations():
                if isinstance(update, OperationUpdate):
                    yield from update.events
                else:
                    yield update.motion

        for chunk in post_events(
            header,
            events(),
            machine=machine,
            lowering=LoweringOptions(chord_tolerance_mm=min(0.0025, machine.tolerance_mm / 4)) if lowering else None,
            post=post_type or GCodePost,
            decoder=decoder_type,
        ):
            if definition is None:
                definition = chunk.data.get('definition')
            chunks += 1
            output.write(chunk.text)
            lines += len(chunk.data['lines'])
            if chunk.motions and first is None:
                first = dict(
                    ms=(time.perf_counter() - began) * 1000,
                    source_finished=source.state.finished,
                    source_moves=source.state.completed,
                    source_native_exited=source._process.poll() is not None,
                )
            maps = chunk.data.get('mappings')
            if maps is not None:
                source_motions += sum(
                    m['source_fraction_interval'] is not None and m['source_fraction_interval'][1] == 1 for m in maps
                )
            else:
                source_motions += len(chunk.motions)
            for check in chunk.data['checked']:
                if check['deviation_bound_mm'] is not None:
                    max_deviation = max(max_deviation, check['deviation_bound_mm'])
            for update in posted.motions(chunk.motions):
                if isinstance(update, VerificationUpdate):
                    counts[update.status] += 1
                    if update.status != 'clear' and len(issue_samples) < 8:
                        report = next(
                            r
                            for r in (update.check.get('report'), update.check.get('stock_contact'))
                            if r and r['status'] != 'clear'
                        )
                        posted_id = report['motion_id']
                        mapping = next((m for m in (maps or ()) if m['posted_motion_id'] == posted_id), None)
                        issue_samples.append(
                            dict(posted_motion_id=posted_id, source_mapping=mapping, findings=report['findings'][:2])
                        )
            motions += len(chunk.motions)
        for update in posted.finish_updates():
            if update.check['kind'] == 'material':
                material = update.check['report']
        assert source.state.finished and posted.state.finished
        assert (
            first is not None
            and not first['source_finished']
            and not first['source_native_exited']
            and source_motions == source.state.completed
            and motions == posted.state.completed
        )
        result = dict(
            scope='Actual native cadquery pocket source -> attributed checked controller words (profile below) -> independent subset decoding -> actual stock/contact/material checks of decoded motions. External tables/M6 route, controller runtime, physical machine and browser remain unchecked.',
            revision=config['revision'],
            machine=asdict(machine),
            definition=definition,
            radial_allowance_mm=radial_allowance_mm,
            issue_samples=issue_samples,
            first_posted_motion=first,
            motions=motions,
            source_motions=source_motions,
            profile=profile,
            lowered=lowering,
            lines=lines,
            chunks=chunks,
            max_continuous_deviation_bound_mm=max_deviation,
            posted_scene_stock_counts=counts,
            posted_material=material,
            source_stock_volume=source.state.volume(),
            posted_stock_volume=posted.state.volume(),
            source_heights_sha256=hashlib.sha256(source.state.heights).hexdigest(),
            posted_heights_sha256=hashlib.sha256(posted.state.heights).hexdigest(),
            total_ms=(time.perf_counter() - began) * 1000,
        )
    temporary.replace(nc_path)
    result['nc_sha256'] = hashlib.sha256(nc_path.read_bytes()).hexdigest()
    result['nc_bytes'] = nc_path.stat().st_size
    root = Path(__file__).resolve().parents[3]
    names = {
        'src/post.rs',
        'src/post_decode.rs',
        'src/post_intent.rs',
        'src/post_lowering.rs',
        'src/grbl.rs',
        'src/motion.rs',
        'bindings/python/src/lib.rs',
        'python/camlib/post.py',
        'python/camlib/postprocessor.py',
        'python/tests/support/post_pocket.py',
    }
    for cls in (post_type, decoder_type):
        if cls is not None:
            try:
                source = inspect.getsourcefile(cls)
            except TypeError:
                source = None
            if source and Path(source).is_file() and Path(source).resolve().is_relative_to(root):
                names.add(Path(source).resolve().relative_to(root).as_posix())
    result['source_sha256'] = {name: hashlib.sha256((root / name).read_bytes()).hexdigest() for name in sorted(names)}
    return result
