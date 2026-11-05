import json
from pathlib import Path
import unittest

from camlib import Engine


class EngineTest(unittest.TestCase):
    def test_programmable_session_and_invalidation(self):
        fixture = json.loads((Path(__file__).resolve().parents[2] / "fixtures/cadquery-pocket.json").read_text())
        with Engine() as engine:
            engine.load(fixture)
            engine.probe([[30, 20], [40, 30]])
            self.assertEqual([r["verdict"] for r in engine.classify(1)], ["allowed", "blocked"])
            engine.set_feed(1200)
            engine.classify(2)
            self.assertEqual(engine.stats()["geometry_preparations"], 1)
            self.assertEqual(engine.stats()["clearance_batches"], 1)
            with self.assertRaises(ValueError):
                engine.set_feed(-1)
            with self.assertRaises(ValueError):
                engine.probe([[float("nan"), 0]])
            # NaN is rejected before writing a partial request into the stream.
            self.assertEqual(len(engine.classify(1)), 2)
            engine.load(fixture)
            with self.assertRaises(ValueError):
                engine.classify(1)


if __name__ == "__main__":
    unittest.main()
