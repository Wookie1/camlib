"""Turning previews must never masquerade as committed stock/checkpoints."""

from pathlib import Path
import tempfile
import unittest

from camlib.simulation import ProtocolError, Simulation, Update
from test_turning import job


class TurnStateTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        setup = job().job.compile().setup("lathe")
        with Simulation(**setup.simulation_options()) as sim:
            before = sim.state.clone()
            for update in sim.run_operations():
                if isinstance(update, Update):
                    removed = sim.state.volume()["removed_lower_mm3"] - before.volume()["removed_lower_mm3"]
                    if removed > 1:
                        cls.before, cls.frame, cls.after = before, update.frame, sim.state.clone()
                        break
                before = sim.state.clone()
            else:
                raise AssertionError("expected a stock-removing lathe move")

    def test_preview_is_read_only_including_endpoint_fractions(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "stock.checkpoint"
            path.write_bytes(b"existing checkpoint")
            for fraction in (0.0, 0.5, 1.0):
                with self.subTest(fraction=fraction):
                    shown = self.before.preview(self.frame, fraction)
                    stock = shown.stock()
                    with self.assertRaises(ProtocolError):
                        shown.accept(self.frame)
                    with self.assertRaises(ProtocolError):
                        shown.write_checkpoint(path)
                    with self.assertRaises(ProtocolError):
                        shown.material()
                    self.assertEqual(shown.stock(), stock)
                    self.assertEqual(path.read_bytes(), b"existing checkpoint")
            # Refusing the view does not affect the committed receiver.
            state = self.before.clone()
            state.accept(self.frame)
            self.assertEqual(state.stock(), self.after.stock())

    def test_restoring_a_checkpoint_discards_the_partial_display_stock(self):
        for compressed in (False, True):
            with self.subTest(compressed=compressed), tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / "stock.checkpoint"
                self.after.write_checkpoint(path, compressed=compressed)
                shown = self.before.preview(self.frame, 0.5)
                self.assertNotEqual(shown.stock(), self.after.stock())
                shown.restore_checkpoint(path, compressed=compressed)
                self.assertEqual(shown.fraction, 0.0)
                self.assertEqual(shown.completed, self.after.completed)
                self.assertEqual(shown.position, self.after.position)
                self.assertEqual(shown.stock(), self.after.stock())
                self.assertEqual(shown.volume(), self.after.volume())


if __name__ == "__main__":
    unittest.main()
