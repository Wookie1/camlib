"""Synthetic CAM regression input."""

from dataclasses import replace
import hashlib
from pathlib import Path
import time

from camlib.operations import RestPocket
from camlib.simulation import OperationUpdate, Simulation, curve_length
from support.pocket import cadquery_job

ROOT = Path(__file__).resolve().parents[3]


def jobs():
    config = cadquery_job()
    fine = config['tools'][0]
    large = replace(fine, id='rough8', cutter=replace(fine.cutter, diameter_mm=8, shank_diameter_mm=8))
    operation = config['operations'][0]
    rough = replace(operation, id='rough-eight', tool=large, stepover_mm=3)
    full = replace(operation, id='fine-four', tool=fine, tool_number=2, length_register=2)
    rest = RestPocket(**{k: getattr(full, k) for k in full.__dataclass_fields__ if k != 'rest_tolerance_mm'})
    config.update(stock=replace(config['stock'], cell_size_mm=0.5), tools=(large, fine))
    return dict(config, operations=(rough, full)), dict(config, operations=(rough, rest))


def run(config):
    began = time.perf_counter()
    first = None
    length = 0
    report = None
    count = 0
    nominal = 0.0
    fine_nominal = 0.0
    kinds = {}
    stages = {}
    with Simulation(**config) as sim:
        for update in sim.run_operations():
            if isinstance(update, OperationUpdate):
                now = (time.perf_counter() - began) * 1000
                if update.stage == 'begin':
                    stages[update.operation] = dict(begin_receipt_ms=now)
                else:
                    stages[update.operation]['active_to_end_receipt_ms'] = (
                        now - stages[update.operation]['begin_receipt_ms']
                    )
                if update.stage == 'end' and update.operation == 'fine-four':
                    report = update.report
            else:
                count += 1
                kind = update.motion['kind']
                kinds[kind] = kinds.get(kind, 0) + 1
                distance = curve_length(update.motion['curve'])
                rate = update.motion['feed_mm_min'] or 6000
                seconds = distance / rate * 60
                nominal += seconds
                if update.motion['operation'] == 'fine-four':
                    fine_nominal += seconds
                if first is None and update.changed_count:
                    first = (time.perf_counter() - began) * 1000
                    assert not sim.state.finished
                if update.motion['operation'] == 'fine-four' and update.motion['kind'] == 'cut':
                    length += distance
        elapsed = (time.perf_counter() - began) * 1000
        assert sim.state.finished and first is not None
        return dict(
            whole_ms=elapsed,
            first_stock_ms=first,
            motions=count,
            fine_cut_length_mm=length,
            nominal_seconds=nominal,
            fine_nominal_seconds=fine_nominal,
            motion_kinds=kinds,
            operation_receipt_ms=stages,
            report=report,
            volume=sim.state.volume(),
            stock_sha256=hashlib.sha256(sim.state.heights).hexdigest(),
        )


def stats(values):
    v = sorted(values)
    return dict(samples=len(v), p50_ms=v[(len(v) - 1) // 2], p95_ms=v[(95 * len(v) + 99) // 100 - 1], max_ms=v[-1])
