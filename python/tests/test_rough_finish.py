"""One CAD target through roughing, both finishers and actual stock/post checks."""

from dataclasses import replace
from functools import lru_cache
import importlib.util
import os
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from camlib.jobs import RegenerationCache
from camlib.playback import Cursor, Player
from camlib.post import LoweringOptions
from camlib.simulation import OperationUpdate, Update, VerificationUpdate


@lru_cache(None)
def example():
    path = Path(__file__).resolve().parents[2] / "python/tests/support/parallel.py"
    spec = importlib.util.spec_from_file_location("rough_finish_example", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def engine():
    return os.environ.get("CAMLIB_TEST_ENGINE")


def collect(job, cache=None):
    ends, volumes, counts, states, statuses = {}, {}, {}, {}, []
    xyz = 0
    materials = []
    with job.regenerate(cache, executable=engine()) as run:
        for event in run:
            update = event.update
            state = run._simulation.state
            if isinstance(update, OperationUpdate) and update.stage == "end":
                ends[update.operation] = update.report
                volumes[update.operation] = state.volume()["removed_lower_mm3"]
                counts[update.operation] = state.completed
                states[update.operation] = state.clone()
            elif isinstance(update, VerificationUpdate):
                if update.status:
                    statuses.append(update.status)
                if update.kind == "material":
                    materials.append(update.check)
            elif isinstance(update, Update) and update.motion["operation"] == "parallel":
                c = update.motion["curve"]
                xyz += update.motion["kind"] == "cut" and abs(c["end"][2] - c["start"][2]) > 0.001
        assert run.finished
        return dict(
            ends=ends,
            volumes=volumes,
            counts=counts,
            states=states,
            statuses=statuses,
            stats=run.statistics,
            xyz=xyz,
            materials=materials,
        )


class RoughFinishTest(unittest.TestCase):
    def assert_roof_stock(self, state):
        # Exact CAD roof, independent of the exported triangles/planner.
        # Whole-cell height intervals must enclose the target; their upper
        # remaining allowance includes the explicit .25 mm stock-grid extent.
        for y in range(state.ny):
            for x in range(state.nx):
                x0, x1 = x * state.dx, (x + 1) * state.dx

                def roof(u):
                    return 4 + min(u, 24 - u) / 3

                lo, hi = min(roof(x0), roof(x1)), max(roof(x0), roof(x1))
                if x0 <= 12 <= x1:
                    hi = 8
                upper = state.heights[2 * (y * state.nx + x) + 1]
                self.assertGreaterEqual(upper, lo)
                self.assertLessEqual(upper, hi + 0.15)
        volume = state.volume()
        self.assertLessEqual(volume["remaining_lower_mm3"], 2304)
        self.assertGreaterEqual(volume["remaining_upper_mm3"], 2304)

    def test_shared_cad_rough_waterline_parallel_stock_and_cache(self):
        with TemporaryDirectory() as directory:
            cache = RegenerationCache(directory)
            job = example().build_finishing_job(finish="combined")
            cold = collect(job, cache)
            self.assertGreater(cold["xyz"], 10)
            self.assertEqual(set(cold["statuses"]), {"clear"})
            previous = "rough"
            for op in ("waterline", "parallel"):
                top = cold["ends"][op]["stock_top"]
                self.assertEqual(top["original_top_z_mm"], 12)
                self.assertLess(top["maximum_upper_z_mm"], 10)
                self.assertEqual(top["generation"], cold["counts"][previous])
                self.assertGreater(cold["volumes"][op], cold["volumes"][previous])
                previous = op
            self.assert_roof_stock(cold["states"]["parallel"])
            warm = collect(replace(job, revision="2"), cache)
            self.assertTrue(all(s["cache_hit"] for s in warm["stats"].values()))
            for op in cold["states"]:
                self.assertEqual(cold["states"][op].heights.tobytes(), warm["states"][op].heights.tobytes())
            changed = example().build_finishing_job(revision="3", finish="combined", rough_stepdown=0.5)
            result = collect(changed, cache)
            self.assertFalse(result["stats"][("top", "rough")]["cache_hit"])
            self.assertFalse(result["stats"][("top", "waterline")]["cache_hit"])
            self.assert_roof_stock(result["states"]["parallel"])

    def test_actual_short_flute_job_posts_tool_changes_and_retains_stock(self):
        job = example().build_finishing_job()
        for lowering in (None, LoweringOptions()):
            with self.subTest(lowering=lowering):
                with job.post(
                    "top",
                    machine=example().finishing_machine(job),
                    executable=engine(),
                    checkpoint_every=64,
                    lowering=lowering,
                ) as run:
                    run.wait(timeout=120)
                    self.assertTrue(run.posting_status()["finished"])
                    tool_changes, arcs, start = 0, 0, 1
                    while page := run.nc_lines(start, limit=256):
                        tool_changes += sum(row["text"] == "M6" for row in page)
                        arcs += sum(row["text"].startswith(("G2 ", "G3 ")) for row in page)
                        start = page[-1]["number"] + 1
                    self.assertEqual(tool_changes, 2)
                    if lowering is None:
                        self.assertGreater(arcs, 0)
                    materials, record_start = [], 0
                    while records := list(run.verification_records(record_start, limit=256)):
                        materials.extend(r for r in records if r.kind == "material")
                        record_start += len(records)
                    self.assertEqual(len(materials), 2)
                    self.assertEqual(materials[0].check["after_operation"], "rough")
                    self.assertIsNone(materials[-1].check["after_operation"])
                    self.assertLess(materials[0].completed, materials[-1].completed)
                    self.assertEqual(materials[-1].completed, run.status()["available"])
                    for record in materials:
                        self.assertEqual(record.completed, record.check["report"]["context"]["generation"])
                    player = Player(run)
                    end = Cursor(run.status()["available"])
                    player.seek(end)
                    self.assert_roof_stock(player.state)
                    final = player.state.heights.tobytes()
                    player.seek(Cursor(end.completed // 2, 0.5))
                    player.seek(end)
                    self.assertEqual(player.state.heights.tobytes(), final)
