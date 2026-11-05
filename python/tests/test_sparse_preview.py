from array import array
import gc
import struct
import unittest
from unittest.mock import patch
import weakref

from camlib._buffers import native
from camlib.playback import Cursor, Player, RecordedRun
from camlib.simulation import PartialStockView, ProtocolError, StockSpec
from stock_grid import StockGrid as DisplayGrid, RECORD
from test_simulation import tool, moves


def run():
    return RecordedRun(
        StockSpec((0, 0, 0), (20, 20, 10), 0.2),
        (tool(),),
        moves(),
        revision="sparse-preview",
        initial_position_mm=(3, 5, 6),
        checkpoint_every=2,
        rapid_mm_min=6000,
    )


class SparsePreviewTest(unittest.TestCase):
    def test_compact_prefix_and_incremental_display_match_dense_line_arc_helix_seek(self):
        with run() as recording:
            recording.wait()
            dense, compact = Player(recording), Player(recording, compact=True)
            grid = DisplayGrid(compact.state, 32)
            grid.update(compact)
            for cursor in (
                Cursor(0, 0.4),
                Cursor(1, 0.3),
                Cursor(1, 0.8),
                Cursor(1),
                Cursor(2, 0.37),
                Cursor(3, 0.6),
                Cursor(4),
                Cursor(),
                Cursor(2, 0.2),
            ):
                dense.seek(cursor)
                compact.seek(cursor)
                grid.update(compact)
                self.assertEqual(compact.state.heights.tobytes(), dense.state.heights.tobytes(), str(cursor))
                self.assertEqual(compact.state.position, dense.state.position)
                self.assertEqual(compact.state.volume(), dense.state.volume())
                self.assertEqual(compact.state.completed, dense.state.completed)
                self.assertEqual(compact.state.fraction, dense.state.fraction)
                expected = native.display_patch(dense.state.heights, dense.state.nx, (grid.nx, grid.ny), None, (0, 10))
                values = array("f", (h for _, h in RECORD.iter_unpack(expected)))
                self.assertEqual(grid.heights.tobytes(), values.tobytes(), str(cursor))
                if cursor.fraction:
                    self.assertIsInstance(compact.state, PartialStockView)
                    with self.assertRaises(ProtocolError):
                        compact.state.write_checkpoint("unused")
                    with self.assertRaises(AttributeError):
                        compact.state.position = (0, 0, 0)
                else:
                    self.assertNotIsInstance(compact.state, PartialStockView)

    def test_compact_seek_and_display_do_not_allocate_a_full_height_buffer(self):
        with run() as recording:
            recording.wait()
            player = Player(recording, compact=True)
            grid = DisplayGrid(player.state, 32)
            grid.update(player)
            initial = player._base.heights.tobytes()
            with patch("camlib.simulation._height_storage", side_effect=AssertionError("dense allocation")):
                player.seek(Cursor(0, 0.35))
                grid.update(player)
                player.seek(Cursor(0, 0.7))
                grid.update(player)
                self.assertGreater(player.state.fraction, 0)
            self.assertEqual(player._base.heights.tobytes(), initial)
            self.assertTrue(any(h < 9 for h in grid.heights))
            with self.assertRaises(TypeError):
                PartialStockView()

    def test_borrowed_base_lifetime_and_mutation_are_explicit(self):
        with run() as recording:
            recording.wait()
            player = Player(recording, compact=True)
            player.seek(Cursor(0, 0.4))
            view = player.state
            base = view._base
            ref = weakref.ref(base)
            expected = view.heights.tobytes()
            player.seek(Cursor(4))
            del base
            gc.collect()
            self.assertIsNotNone(ref())
            self.assertEqual(view.heights.tobytes(), expected)
            view._base.accept(recording.read_chunk(0))
            with self.assertRaisesRegex(ProtocolError, "base changed"):
                view.display_patch((16, 16), None)
            with self.assertRaisesRegex(ProtocolError, "base changed"):
                view.clone()

    def test_forward_seek_beyond_one_cache_span_restores_exact_base(self):
        from camlib.simulation import preview_prefix

        with run() as recording:
            recording.wait()
            player = Player(recording, compact=True)
            original = recording.cache_span
            # A valid bounded span that stops before the requested cursor:
            # exercise the same fallback as a history exceeding8MiB/4096frames.
            with patch.object(
                recording, "cache_span", side_effect=lambda start, target: original(start, min(start + 1, target))
            ):
                player.seek(Cursor(3, 0.4))
            self.assertEqual(player._prefix.replayed_chunks, 0)
            self.assertEqual(player._prefix.checkpoint_loads, 1)
            expected, _ = preview_prefix(player._base, player.motion, 0.4)
            self.assertEqual(player.state.heights.tobytes(), expected.heights.tobytes())
            player.seek(Cursor(2, 0.3))
            player.seek(Cursor(3, 0.6))
            expected, _ = preview_prefix(player._base, player.motion, 0.6)
            self.assertEqual(player.state.heights.tobytes(), expected.heights.tobytes())
            self.assertEqual(player._prefix.replayed_chunks, 1)

    def test_wrong_fraction_and_valid_crc_late_corruption_preserve_the_player(self):
        from camlib.simulation import Frame
        from test_simulation import checksum

        with run() as recording:
            recording.wait()
            player = Player(recording, compact=True)
            player.seek(Cursor(0, 0.2))
            grid = DisplayGrid(player.state, 32)
            grid.update(player)
            old = player.state
            before = grid.heights.tobytes()
            with patch.object(player._prefix, "_next", return_value=old._frame):
                with self.assertRaisesRegex(ProtocolError, "requested motion/fraction"):
                    player.seek(Cursor(0, 0.7))
            self.assertIs(player.state, old)
            self.assertEqual(player.cursor, Cursor(0, 0.2))
            valid = player._prefix._next()  # Drain the actual .7 reply hidden by injection.
            from camlib.simulation import _Decoder

            decoder = _Decoder(valid.payload)
            player._base._identity(decoder)
            decoder.number("<Q")
            wrong_motion = bytearray(valid.raw)
            at = 20 + decoder.offset
            struct.pack_into("<Q", wrong_motion, at, struct.unpack_from("<Q", wrong_motion, at)[0] + 1)
            with patch.object(player._prefix, "_next", return_value=Frame.parse(checksum(wrong_motion))):
                with self.assertRaisesRegex(ProtocolError, "requested motion/fraction"):
                    player.seek(Cursor(0, 0.7))
            self.assertIs(player.state, old)
            self.assertEqual(player.cursor, Cursor(0, 0.2))
            valid = player._prefix._next()
            damaged = bytearray(valid.raw)
            struct.pack_into("<d", damaged, len(damaged) - 32, 9.0)
            invalid = Frame.parse(checksum(damaged))
            with patch.object(player._prefix, "_next", return_value=invalid):
                with self.assertRaisesRegex(ProtocolError, "old state"):
                    player.seek(Cursor(0, 0.7))
            self.assertIs(player.state, old)
            self.assertEqual(player.cursor, Cursor(0, 0.2))
            grid.update(player)
            self.assertEqual(grid.heights.tobytes(), before)

    def test_cancelled_checkpoint_keeps_existing_file_and_cleans_temporaries(self):
        import itertools
        from pathlib import Path
        import tempfile
        from test_simulation import simulation
        from camlib.simulation import SimulationError

        with simulation() as sim, tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "stock.bin"
            sim.state.write_checkpoint(path)
            old = path.read_bytes()
            before = sim.state.heights.tobytes()
            polls = itertools.count()
            with self.assertRaisesRegex(SimulationError, "checkpoint cancelled"):
                sim.state.write_checkpoint(path, compressed=True, cancel=lambda: next(polls) >= 3)
            self.assertEqual(path.read_bytes(), old)
            self.assertEqual([p.name for p in Path(temp).iterdir()], ["stock.bin"])
            self.assertEqual(sim.state.heights.tobytes(), before)
            sim.finish()

    def test_readonly_overlay_validates_late_old_values_order_bounds_before_returning(self):
        base = array("d", [10, 10] * 35)
        raw = bytearray(b"prefix")
        changes = [(3, 10, 10, 4.2, 4.3), (17, 10, 10, 1.1, 1.2), (34, 10, 10, 6.2, 6.3)]
        for record in changes:
            raw.extend(struct.pack("<Idddd", *record))
        before = base.tobytes()
        rect = native.validate_delta(memoryview(base).toreadonly(), bytes(raw), 6, 3, 7, (0, 10))
        self.assertEqual(rect, (3, 0, 6, 4))
        patch = native.display_overlay_patch(base, bytes(raw), (6, 3), (7, 3, 2), None, (0, 10))
        expected = list(base)
        for i, _, _, lo, hi in changes:
            expected[2 * i : 2 * i + 2] = [lo, hi]
        for index, height in RECORD.iter_unpack(patch):
            x, y = index % 3, index // 3
            exact = max(
                expected[2 * (sy * 7 + sx) + 1]
                for sy in range(y * 5 // 2, (y + 1) * 5 // 2)
                for sx in range(x * 7 // 3, (x + 1) * 7 // 3)
            )
            self.assertGreaterEqual(height, exact)
            self.assertLess(height - exact, 1e-6)
        self.assertEqual(base.tobytes(), before)
        for relative, value in ((4, 9.0), (20, 11.0), (28, float('nan'))):
            bad = bytearray(raw)
            struct.pack_into("<d", bad, 6 + 2 * 36 + relative, value)
            with self.assertRaises(ValueError):
                native.validate_delta(base, bytes(bad), 6, 3, 7, (0, 10))
            with self.assertRaises(ValueError):
                native.display_overlay_patch(base, bytes(bad), (6, 3), (7, 3, 2), None, (0, 10))
            self.assertEqual(base.tobytes(), before)
        duplicate = bytearray(raw)
        struct.pack_into("<I", duplicate, 6 + 2 * 36, 17)
        with self.assertRaises(ValueError):
            native.display_overlay_patch(base, bytes(duplicate), (6, 3), (7, 3, 2), None, (0, 10))
        self.assertEqual(base.tobytes(), before)
        self.assertEqual(
            native.display_overlay_patch(base, b"", (0, 0), (7, 3, 2), None, (0, 10)),
            native.display_patch(base, 7, (3, 2), None, (0, 10)),
        )


if __name__ == "__main__":
    unittest.main()
