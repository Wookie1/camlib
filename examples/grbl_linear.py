"""Review a complete authored linear GRBL program and its decoded stock removal.

Posted with examples/grbl_post.py (GRBL 1.1h, lines only). Work zero, the tool
and the initial machine state are declared, not read from hardware. No machine
connection. This example authors only lines; posting a native operation to GRBL
uses the post engine's checked arc lowering (LoweringOptions).
"""

import hashlib
from pathlib import Path
import sys
import time
from itertools import chain

from camlib.motion import Program, WorkOffset, ToolChange, Spindle, Coolant, Motion, Line
from camlib.post import ToolSetting, post_program
from camlib.planar import Boundary, Component, PlanarRegion
from camlib.simulation import Simulation, StockSpec, VerificationUpdate
from camlib.verification import Prism, Scene, StreamChecks, StockContactOptions
from camlib.tools import EndMill, HolderSection, ToolAssembly

sys.path.insert(0, str(Path(__file__).resolve().parent))
from grbl_post import GrblDecoder, GrblPost, define_machine  # noqa: E402


def inputs():
    tool = ToolAssembly(
        't1', EndMill('em4', diameter=4, flute_length=30, overall_length=50), 35, 60, (HolderSection(35, 60, 4, 4),)
    )
    current = (2.0, 2.0, 12.0)
    events = [WorkOffset(1), ToolChange('t1', 1, 1, current), Spindle(12000), Coolant('flood')]
    moves = 0

    def move(end, feed, kind):
        nonlocal current, moves
        moves += 1
        events.append(Motion(moves, 'authored-linear-clearing', 't1', Line(current, end), feed, kind))
        current = end

    move((2.0, 2.0, 11.0), None, 'rapid')
    move((2.0, 2.0, 5.0), 100.0, 'plunge')
    for i, y in enumerate(range(2, 19, 2)):
        if current[1] != y:
            move((current[0], float(y), 5.0), 600.0, 'link')
        move((18.123456 if i % 2 == 0 else 2.0, float(y), 5.0), 600.0, 'cut')
    move((current[0], current[1], 12.0), 900.0, 'retract')
    events.extend([Coolant(), Spindle(0)])
    program = Program('grbl-linear-example', 'grbl-linear-1', (tool,), (2.0, 2.0, 12.0), tuple(events))
    # The machine is the caller's to declare: a small GRBL 1.1h router in mm.
    machine = define_machine(
        "GRBL router",
        ToolSetting.for_tool(tool, number=1, length_register=1, position_after_mm=program.initial_position_mm),
        work_offset_machine_mm=(1000.0, -2000.0, 100.0),
        initial_machine_mm=(1002.0, -1998.0, 112.0),
        output_units="mm",
        decimals=6,
        tolerance_mm=0.001,
        max_feed_mm_min=10000,
        max_spindle_rpm=30000,
        rapid_mm_min=5000,
        flood=True,
        mist=False,
    )
    return program, machine


def run(nc):
    program, machine = inputs()
    stock = StockSpec((0, 0, 0), (20, 20, 10), 0.25)
    clamp = PlanarRegion('vice', (Component(Boundary.polygon(((24, 0), (28, 0), (28, 20), (24, 20)))),))
    checks = StreamChecks(Scene('grbl-vice', '1', (Prism('vice', clamp, 0, 10),)), stock_contact=StockContactOptions())
    temporary = nc.with_suffix(nc.suffix + '.part')
    if nc.exists() or temporary.exists():
        raise ValueError('preserve artifacts: choose a new NC path')
    nc.parent.mkdir(parents=True, exist_ok=True)
    counts = dict(clear=0, violation=0, uncertain=0)
    max_error = 0.0
    decoded_count = 0
    line_count = 0
    first = None
    config = dict(
        stock=stock, tools=program.tools, revision=program.revision, initial_position_mm=program.initial_position_mm
    )
    started = time.perf_counter()
    chunks = post_program(program, machine=machine, post=GrblPost, decoder=GrblDecoder)
    begin = next(chunks)
    posted_config = dict(
        config,
        revision=program.revision + '-posted',
        verification=checks,
        initial_position_mm=tuple(begin.data['decoded_initial_position_mm']),
    )
    with Simulation(**config) as original, Simulation(**posted_config) as posted, temporary.open('w') as output:
        for _ in original.motions(e for e in program.events if isinstance(e, Motion)):
            pass
        original.finish()
        for chunk in chain((begin,), chunks):
            output.write(chunk.text)
            line_count += len(chunk.data['lines'])
            if chunk.motions and first is None:
                first = (time.perf_counter() - started) * 1000
            for check in chunk.data['checked']:
                if check['deviation_bound_mm'] is not None:
                    max_error = max(max_error, check['deviation_bound_mm'])
            for update in posted.motions(chunk.motions):
                if isinstance(update, VerificationUpdate):
                    counts[update.status] += 1
            decoded_count += len(chunk.motions)
        posted.finish()
        assert posted.state.finished and original.state.finished and decoded_count == original.state.completed
        result = dict(
            scope='Complete explicitly authored linear program; GRBL binary32 nominal words/WCS decoding and actual native stock/contact/fixture checks. No circular source dropped. Preloaded physical tool/WCS/state and firmware/stepper/servo behavior remain external premises.',
            machine=machine.name,
            program=program.to_dict(),
            motions=decoded_count,
            lines=line_count,
            max_continuous_deviation_bound_mm=max_error,
            posted_checks=counts,
            first_decoded_ms=first,
            source_volume=original.state.volume(),
            posted_volume=posted.state.volume(),
            source_heights_sha256=hashlib.sha256(original.state.heights).hexdigest(),
            posted_heights_sha256=hashlib.sha256(posted.state.heights).hexdigest(),
            total_ms=(time.perf_counter() - started) * 1000,
        )
    temporary.replace(nc)
    result.update(nc_sha256=hashlib.sha256(nc.read_bytes()).hexdigest(), nc_bytes=nc.stat().st_size)
    return result
