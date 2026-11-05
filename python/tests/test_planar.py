import json
import math
from pathlib import Path
import unittest

from camlib import Engine
from camlib.planar import Boundary, Component, PlanarRegion


class PlanarTest(unittest.TestCase):
    def test_native_offsets_booleans_and_cached_source(self):
        region = PlanarRegion("pocket", (Component(Boundary.polygon([(0, 0), (20, 0), (20, 10), (0, 10)])),))
        other = PlanarRegion("clip", (Component(Boundary.polygon([(10, -5), (30, -5), (30, 15), (10, 15)])),))
        with Engine() as engine:
            prepared = engine.prepare_region(region)
            self.assertEqual(prepared["summary"]["area_mm2"], 200)
            offset = engine.offset_region(2)
            cutter = engine.cutter_region(2, 0.2)
            self.assertAlmostEqual(cutter["applied_offset_mm"], 2.20001)
            boolean = engine.boolean_region(other, "intersection")
            self.assertEqual(boolean["after_components"], 1)
            # Constructions do not replace the prepared source implicitly.
            self.assertEqual(engine.stats()["planar_preparations"], 1)
            self.assertEqual(engine.stats()["planar_constructions"], 3)
            self.assertEqual(engine.query_region([[1, 5]])[0]["verdict"], "allowed")
            with self.assertRaises(ValueError):
                engine.prepare_region(PlanarRegion("invalid", (Component(Boundary.polygon([(0, 0), (0, 0)])),)))
            self.assertEqual(engine.query_region([[1, 5]])[0]["verdict"], "allowed")
            engine.prepare_region(PlanarRegion.from_dict(offset["region"]))
            self.assertEqual(
                [r["verdict"] for r in engine.query_region([[1, 5], [2, 5], [10, 5]])],
                ["blocked", "uncertain", "allowed"],
            )

    def test_circle_keeps_analytic_arcs_and_reports_collapse(self):
        region = PlanarRegion("circle", (Component(Boundary.circle((0, 0), 10)),))
        with Engine() as engine:
            summary = engine.prepare_region(region)["summary"]
            self.assertEqual(summary["arc_segments"], 2)
            self.assertAlmostEqual(summary["area_mm2"], math.pi * 100)
            result = engine.offset_region(3)
            self.assertEqual(len(result["region"]["components"][0]["outer"]["vertices"]), 2)
            self.assertEqual(result["chord_error_mm"], 0)
            self.assertEqual(engine.offset_region(11)["after_components"], 0)

    def test_existing_cadquery_fixture_is_a_programmable_region(self):
        path = Path(__file__).resolve().parents[2] / "fixtures/cadquery-pocket.json"
        fixture = json.loads(path.read_text())
        with Engine() as engine:
            engine.prepare_region(PlanarRegion.from_fixture(fixture))
            result = engine.cutter_region(3)
            self.assertEqual(result["region"]["source"], fixture["source"])
            self.assertEqual(result["after_holes"], 1)


if __name__ == "__main__":
    unittest.main()
