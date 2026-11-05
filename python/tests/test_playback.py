import math
import threading
import struct
import unittest

from camlib.motion import Arc, Line, Motion
from camlib.playback import Cursor, NotReady, Player, RecordedRun
from camlib.simulation import ProtocolError, SimulationError, StockSpec
from test_simulation import moves, tool


def run(motions=None, **kwargs):
    return RecordedRun(
        StockSpec((0, 0, 0), (20, 20, 10), 0.2),
        (tool(),),
        moves() if motions is None else motions,
        revision="playback-cad-1",
        initial_position_mm=(3, 5, 6),
        **{'rapid_mm_min': 6000, **kwargs},
    )


class PlaybackTest(unittest.TestCase):
    def test_full_circle_stock_and_partial_arc_seek_share_authoritative_sweep(self):
        for sweep in (math.tau, -math.tau):
            circle = Motion(1, "circle", "t1", Arc((13, 10, 6), (13, 10, 6), (10, 10, 6), sweep), 1000)
            with RecordedRun(
                StockSpec((0, 0, 0), (20, 20, 10), 0.2),
                (tool(),),
                (circle,),
                revision="analytic-circle-1",
                initial_position_mm=(13, 10, 6),
                checkpoint_every=1,
                rapid_mm_min=6000,
            ) as recording:
                recording.wait()
                player = Player(recording)
                player.pause()
                player.seek(Cursor(0, 0.25))
                partial = player.state.heights.tobytes()
                self.assertGreater(player.state.volume()["removed_lower_mm3"], 1)
                self.assertAlmostEqual(player.state.position[0], 10)
                self.assertAlmostEqual(player.state.position[1], 13 if sweep > 0 else 7)
                # The opposite quadrant is future material at the quarter turn.
                index = int(10 / player.state.dy) * player.state.nx + int(7 / player.state.dx)
                self.assertEqual(tuple(player.state.heights[2 * index : 2 * index + 2]), (10.0, 10.0))
                player.seek(Cursor(1))
                self.assertTrue(player.state.finished)
                self.assertLess(player.state.heights[2 * index + 1], 6.001)
                complete = player.state.heights.tobytes()
                player.seek(Cursor())
                self.assertEqual(player.state.volume()["removed_upper_mm3"], 0)
                player.seek(Cursor(0, 0.25))
                self.assertEqual(player.state.heights.tobytes(), partial)
                player.seek(Cursor(1))
                self.assertEqual(player.state.heights.tobytes(), complete)

    def test_pause_does_not_stop_production_and_seek_replays_actual_stock(self):
        gate = threading.Event()

        def source():
            yield from moves()
            gate.wait(3)
            yield Motion(14, "second-pocket", "t1", Line((14, 18, 4), (3, 18, 4)), 500)
            yield Motion(15, "second-pocket", "t1", Line((3, 18, 4), (3, 3, 4)), 500)

        with run(source(), batch_size=4, checkpoint_every=2) as recording:
            try:
                player = Player(recording)
                recording.wait_available(4)
                player.seek(Cursor(1))
                paused = player.state.heights.tobytes()
                player.pause()
                gate.set()
                recording.wait()
                self.assertEqual(recording.status()["available"], 6)
                self.assertEqual(player.cursor, Cursor(1))
                self.assertEqual(player.state.heights.tobytes(), paused)
                player.tick(100)
                self.assertEqual(player.cursor, Cursor(1))
                player.seek(Cursor(6))
                self.assertNotEqual(player.state.heights.tobytes(), paused)
                final = player.state.heights.tobytes()
                self.assertTrue(player.state.finished)
                player.seek(Cursor())
                self.assertEqual(player.state.volume()["removed_upper_mm3"], 0)
                player.seek(Cursor(6))
                self.assertEqual(player.state.heights.tobytes(), final)
                self.assertLessEqual(recording.status()["largest_frame_bytes"], 8 * 1024 * 1024)
                self.assertLess(recording.status()["retained_bytes"], recording.retained_bytes_limit)
            finally:
                gate.set()

    def test_partial_clock_speed_and_checkpoint_paths_are_deterministic(self):
        with run(checkpoint_every=1) as recording:
            recording.wait()
            player = Player(recording)
            player.seek(Cursor(1, 0.371))
            partial = player.state.heights.tobytes()
            position = player.state.position
            query_pid = player._prefix._process.pid
            loads = player._prefix.checkpoint_loads
            player.seek(Cursor(1, 0.6))
            self.assertEqual(player._prefix._process.pid, query_pid)
            self.assertEqual(player._prefix.checkpoint_loads, loads)
            player.seek(Cursor())
            player.seek(Cursor(1, 0.371))
            self.assertEqual(player.state.heights.tobytes(), partial)
            self.assertEqual(player.state.position, position)
            self.assertEqual(player.motion["operation"], "pocket")
            player.seek(Cursor())
            player.set_speed(2)
            player.play()
            player.tick(0.05)
            self.assertAlmostEqual(player.seconds, 0.1)
            self.assertEqual(player.cursor.completed, 0)
            self.assertGreater(player.cursor.fraction, 0)
            self.assertGreater(player.state.volume()["removed_lower_mm3"], 0)
            player.pause()
            cursor = player.cursor
            player.tick(1)
            self.assertEqual(player.cursor, cursor)
            player.seek(Cursor(1, 1))
            self.assertEqual(player.cursor, Cursor(2))
            self.assertEqual(player.last_seek["checkpoint"], 2)
            with self.assertRaises(ValueError):
                player.set_speed(0)
            with self.assertRaises(NotReady):
                player.seek(Cursor(100))
        self.assertIsNotNone(player._prefix._process.poll())

    def test_missing_or_swapped_history_does_not_modify_display(self):
        with run(checkpoint_every=128) as recording:
            recording.wait()
            player = Player(recording)
            initial = player.state.heights.tobytes()
            index = recording.directory / "motions.idx"
            original = index.read_bytes()
            changed = bytearray(original)
            changed[32:48] = original[:16]
            index.write_bytes(changed)
            with self.assertRaisesRegex(ProtocolError, "sequence"):
                player.seek(Cursor(3))
            self.assertEqual(player.cursor, Cursor())
            self.assertEqual(player.state.heights.tobytes(), initial)
            changed = bytearray(original)
            struct.pack_into("<Q", changed, 32, (recording.directory / "motions.bin").stat().st_size + 100)
            index.write_bytes(changed)
            with self.assertRaisesRegex(ProtocolError, "missing"):
                player.seek(Cursor(3))
            self.assertEqual(player.state.heights.tobytes(), initial)
            # An intact final checkpoint explicitly recovers the later cursor.
            player.seek(Cursor(4))
            self.assertEqual(player.last_seek["checkpoint"], 4)
            self.assertEqual(player.last_seek["replayed"], 0)
            self.assertTrue(player.state.finished)

    def test_storage_budget_and_cancel_never_claim_completion(self):
        with run(retained_bytes_limit=2048) as recording:
            with self.assertRaisesRegex(SimulationError, "byte budget"):
                recording.wait()
            self.assertFalse(recording.status()["finished"])
            self.assertLessEqual(recording.status()["retained_bytes"], 2048)

        def source():
            previous = (3, 5, 6)
            for i in range(10_000):
                end = (17, 5, 6) if i % 2 == 0 else (3, 5, 6)
                yield Motion(i, "long-job", "t1", Line(previous, end), 1000)
                previous = end

        with run(source(), checkpoint_every=128) as recording:
            recording.wait_available(1)
            recording.cancel()
            recording._thread.join(3)
            self.assertFalse(recording._thread.is_alive())
            status = recording.status()
            self.assertTrue(status["cancelled"])
            self.assertFalse(status["finished"])
            with self.assertRaisesRegex(SimulationError, "cancelled"):
                recording.wait()


if __name__ == "__main__":
    unittest.main()
