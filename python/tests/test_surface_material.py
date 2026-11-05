from array import array
from dataclasses import replace
import math
import unittest
from unittest.mock import patch

from camlib.material import SurfaceTarget, Verifier, Options
from camlib.surface import SurfaceMesh
from camlib.playback import Cursor, Player
from camlib.simulation import Simulation, StockSpec
from test_sparse_preview import run
from test_rough_finish import example, collect
from test_curved_surfaces import curved_config


class SurfaceMaterialTest(unittest.TestCase):
    def test_compact_overlay_matches_dense_and_source_premises_change_identity(self):
        mesh = SurfaceMesh((((-10, -10, 4), (30, -10, 4), (30, 30, 4)), ((-10, -10, 4), (30, 30, 4), (-10, 30, 4))))
        target = SurfaceTarget('surface', '1', mesh, 0)
        v = Verifier(target, Options(max_findings=3))
        with run() as recording:
            recording.wait()
            compact = Player(recording, compact=True)
            dense = Player(recording)
            for cursor in (Cursor(0, 0.3), Cursor(2, 0.4), Cursor(4), Cursor(1, 0.6)):
                compact.seek(cursor)
                dense.seek(cursor)
                expected = v.check(dense.state)
                with patch('camlib.simulation._height_storage', side_effect=AssertionError('dense allocation')):
                    actual = v.check(compact.state)
                self.assertEqual(actual, expected)
                self.assertEqual(actual['context']['fraction'], cursor.fraction)
            old = v.digest
            with self.assertRaises(ValueError):
                v.prepare(replace(target, source_enclosure='invented'))
            self.assertEqual(v.digest, old)
            v.prepare(replace(target, source_enclosure='inscribed'))
            self.assertNotEqual(v.digest, old)
        limited = Verifier(target, Options(max_classified_cells=1))
        heights = array('d', [10.0]) * 32
        spec = StockSpec((0, 0, 0), (4, 4, 10), 1)
        self.assertFalse(limited.check_bounds(heights, spec)['complete'])
        heights[-1] = math.nan
        with self.assertRaisesRegex(ValueError, 'height bounds'):
            limited.check_bounds(heights, spec)

    def test_real_rough_finish_stock_has_whole_cell_roof_enclosures(self):
        job = example().build_finishing_job()
        result = collect(job)
        surface = job.setups[0].operations[-1].surface
        # Planar CAD faces have a topology-error offset enclosure. This is an
        # explicit source premise, not inferred from arbitrary mesh metadata.
        v = Verifier(SurfaceTarget('roof', '1', surface, 0, 'offset'), Options(max_findings=4096))
        report = v.check(result['states']['parallel'])
        self.assertTrue(report['complete'])
        self.assertEqual(report['gouge']['definite_cells'], 0)
        self.assertEqual(report['context']['generation'], result['counts']['parallel'])
        self.assertEqual(len(result["materials"]), 2)
        self.assertEqual(result["materials"][0]["after_operation"], "rough")
        self.assertEqual(result["materials"][-1]["report"], report)
        for f in report['findings']:
            x0, _, x1, _ = f['xy_mm']

            def roof(x):
                return 4 + min(x, 24 - x) / 3

            lo, hi = min(roof(x0), roof(x1)), max(roof(x0), roof(x1))
            if x0 <= 12 <= x1:
                hi = 8
            self.assertLessEqual(f['target_height_mm'][0], lo)
            self.assertGreaterEqual(f['target_height_mm'][1], hi)
        changed = Verifier(replace(v.target, revision='2'))
        self.assertNotEqual(v.digest, changed.digest)

    def test_real_curved_finish_report_encloses_independent_spherical_surface(self):
        config = curved_config()
        # Radial facets lie inside this analytic hemisphere. Topology/normal
        # source uncertainty is carried by the declared offset enclosure.
        target = SurfaceTarget('hemisphere', '1', config['operations'][0].surface, 2, 'offset')
        verifier = Verifier(target, Options(max_findings=4096))
        with Simulation(**config) as sim:
            for _ in sim.run_operations():
                pass
            report = verifier.check(sim.state)
        self.assertTrue(report['complete'])
        self.assertEqual(report['gouge']['definite_cells'], 0)
        self.assertGreater(report['residual']['volume_mm3'][1], 0)
        for f in report['findings']:
            x0, y0, x1, y1 = f['xy_mm']
            dmin = math.hypot(max(x0 - 8, 0, 8 - x1), max(y0 - 8, 0, 8 - y1))
            dmax = max(math.hypot(x - 8, y - 8) for x in (x0, x1) for y in (y0, y1))
            hi = 2 + math.sqrt(max(0, 16 - dmin * dmin))
            lo = 2 + math.sqrt(max(0, 16 - dmax * dmax))
            self.assertLessEqual(f['target_height_mm'][0], lo)
            self.assertGreaterEqual(f['target_height_mm'][1], hi)
