from dataclasses import replace
from array import array
import json
import unittest
from unittest.mock import patch

from camlib.material import Options, Target, Verifier
from camlib.planar import Boundary, Component, PlanarRegion
from camlib.playback import Cursor, Player
from camlib.simulation import ProtocolError, Simulation
from test_operations import pocket
from test_simulation import simulation, tool
from test_sparse_preview import run


def target():
    region = PlanarRegion("all", (Component(Boundary.polygon(((-10, -10), (30, -10), (30, 30), (-10, 30)))),))
    return Target("part", "cad-1", region, 4)


class MaterialTest(unittest.TestCase):
    def test_raw_readonly_bounds_have_no_run_claim_and_bad_format_shape_reject(self):
        from camlib.simulation import StockSpec

        spec = StockSpec((0, 0, 0), (4, 4, 10), 1)
        heights = array("d", [10.0]) * 32
        verifier = Verifier(target(), Options(max_findings=1))
        result = verifier.check_bounds(memoryview(heights).toreadonly(), spec, generation=3, fraction=1)
        self.assertIsNone(result["context"]["run"])
        self.assertEqual(result["context"]["generation"], 3)
        self.assertEqual(result["context"]["fraction"], 1)
        with self.assertRaises((BufferError, TypeError, ValueError)):
            verifier.check_bounds(array("f", [10.0]) * 32, spec)
        with self.assertRaisesRegex(ValueError, "one-dimensional"):
            verifier.check_bounds(memoryview(heights).cast("B").cast("d", shape=(16, 2)), spec)
        with self.assertRaises(ValueError):
            verifier.check_bounds(heights[:-1], spec)

    def test_native_preparation_borrows_stock_and_reports_actual_run_cursor_data(self):
        with simulation() as sim:
            verifier = Verifier(target(), Options(max_findings=1))
            before = sim.state.heights.tobytes()
            result = verifier.check(sim.state)
            self.assertTrue(result["complete"])
            self.assertEqual(result["residual"]["assessment"], "definite_excess")
            self.assertEqual(result["gouge"]["assessment"], "within_tolerance")
            self.assertLessEqual(result["residual"]["volume_mm3"][0], 20 * 20 * 6)
            self.assertGreaterEqual(result["residual"]["volume_mm3"][1], 20 * 20 * 6)
            self.assertEqual(result["context"]["run"]["run"], sim.state.manifest["run"])
            self.assertEqual(result["context"]["run"]["manifest_sha256"], sim.state._digest.hex())
            self.assertEqual(result["context"]["generation"], 0)
            self.assertEqual(len(result["stock_sha256"]), 64)
            self.assertTrue(result["stats"]["findings_truncated"])
            self.assertEqual(sim.state.heights.tobytes(), before)
            self.assertLess(len(json.dumps(result)), 10000)
            sim.finish()

    def test_compact_line_arc_helix_and_replayed_cursor_reports_equal_dense_without_allocation(self):
        with run() as recording:
            recording.wait()
            compact, dense = Player(recording, compact=True), Player(recording)
            verifier = Verifier(target(), Options(max_findings=3))
            prior = None
            for cursor in (Cursor(0, 0.3), Cursor(1, 0.7), Cursor(2, 0.4), Cursor(3, 0.6), Cursor(4), Cursor(1, 0.2)):
                compact.seek(cursor)
                dense.seek(cursor)
                expected = verifier.check(dense.state)
                with patch("camlib.simulation._height_storage", side_effect=AssertionError("dense allocation")):
                    actual = verifier.check(compact.state)
                self.assertEqual(actual, expected)
                self.assertEqual(actual["context"]["fraction"], cursor.fraction)
                if prior:
                    self.assertNotEqual(actual["stock_sha256"], prior)
                prior = actual["stock_sha256"]
            compact.seek(Cursor(0, 0.5))
            borrowed = compact.state
            borrowed._base.accept(recording.read_chunk(0))
            with self.assertRaisesRegex(ProtocolError, "base changed"):
                verifier.check(borrowed)

    def test_work_cap_is_incomplete_and_late_nan_still_fails_without_mutation(self):
        with simulation() as sim:
            verifier = Verifier(target(), Options(max_classified_cells=1))
            result = verifier.check(sim.state)
            self.assertFalse(result["complete"])
            self.assertEqual(result["residual"]["assessment"], "incomplete")
            self.assertEqual(result["stats"]["classified_cells"], 1)
            old = sim.state._heights[-1]
            sim.state._heights[-1] = float("nan")
            with self.assertRaisesRegex(ValueError, "height bounds"):
                verifier.check(sim.state)
            sim.state._heights[-1] = old
            self.assertFalse(verifier.check(sim.state)["complete"])
            sim.finish()

    def test_target_replacement_is_transactional_and_options_change_identity(self):
        with simulation() as sim:
            verifier = Verifier(target(), Options(max_findings=1))
            digest = verifier.digest
            with self.assertRaises(ValueError):
                verifier.prepare(replace(target(), floor_error_mm=-1))
            self.assertEqual(verifier.digest, digest)
            self.assertTrue(verifier.check(sim.state)["complete"])
            verifier.prepare(target(), Options(max_findings=2))
            self.assertNotEqual(verifier.digest, digest)
            old = verifier.check(sim.state)["target_sha256"]
            verifier.prepare(replace(target(), floor_z_mm=3))
            self.assertNotEqual(verifier.check(sim.state)["target_sha256"], old)
            sim.finish()

    def test_real_cadquery_finished_stream_has_actual_residual_report_and_not_job_clearance(self):
        from camlib.simulation import StockSpec

        op = pocket()
        verifier = Verifier(
            Target("finished", op.region.source["revision"], op.region, op.floor_z_mm), Options(max_findings=8)
        )
        with Simulation(
            StockSpec((0, 0, 0), (80, 60, 20), 0.2),
            (tool(),),
            revision=op.region.source["revision"],
            initial_position_mm=(0, 0, 25),
            operations=(op,),
        ) as sim:
            for _ in sim.run_operations():
                pass
            result = verifier.check(sim.state)
            self.assertTrue(result["complete"])
            self.assertEqual(result["context"]["generation"], sim.state.completed)
            self.assertTrue(sim.state.finished)
            self.assertGreater(result["residual"]["volume_mm3"][0], 0)
            self.assertEqual(result["gouge"]["definite_cells"], 0)
            self.assertGreater(result["stats"]["mixed_cells"], 0)
            self.assertLess(result["stats"]["geometry_queries"], result["stats"]["classified_cells"])


if __name__ == "__main__":
    unittest.main()
