from io import BytesIO
import math
from pathlib import Path
import struct
import tempfile
import unittest
import zlib

from camlib.motion import Arc, Helix, Line, Motion
from camlib.simulation import (
    FRAME_LIMIT,
    Frame,
    ProtocolError,
    Simulation,
    SimulationError,
    StockSpec,
    StockState,
    PrefixSession,
    preview_prefix,
)
from camlib.tools import EndMill, HolderSection, ToolAssembly


def tool():
    return ToolAssembly(
        "t1", EndMill("em4", diameter=4, flute_length=30, overall_length=50), 35, 60, (HolderSection(35, 60, 4, 4),)
    )


def moves():
    return [
        Motion(10, "pocket", "t1", Line((3, 5, 6), (17, 5, 6)), 1000),
        Motion(11, "pocket", "t1", Line((17, 5, 6), (17, 15, 5)), 800),
        Motion(12, "pocket", "t1", Arc((17, 15, 5), (14, 18, 5), (14, 15, 5), math.pi / 2), 1000),
        Motion(13, "pocket", "t1", Helix((14, 18, 5), (14, 18, 4), (14, 15, 5), math.tau), 300),
    ]


def simulation(**kwargs):
    return Simulation(
        StockSpec((0, 0, 0), (20, 20, 10), 0.2),
        (tool(),),
        revision="cad-revision-1",
        initial_position_mm=(3, 5, 6),
        **kwargs,
    )


def checksum(raw):
    raw[16:20] = struct.pack("<I", zlib.crc32(memoryview(raw)[20:], zlib.crc32(memoryview(raw)[:16])))
    return bytes(raw)


class SimulationTest(unittest.TestCase):
    def test_buffered_handoff_commits_only_the_motion_being_yielded(self):
        with simulation() as producer, tempfile.TemporaryDirectory() as directory:
            frames = [u.frame for u in producer.motions(moves())]
            raw = b"".join(f.raw for f in frames)
            # Force all frames into one buffer, independent of pipe scheduling.
            receiver = object.__new__(Simulation)
            receiver.state = StockState(producer.manifest_frame)
            expected = StockState(producer.manifest_frame)
            offset, items = receiver.state.accept_many(raw, 0)
            self.assertEqual(offset, len(raw))
            self.assertEqual(receiver.state.completed, 0)
            handed = receiver._handed_out(items)
            for index, frame in enumerate(frames, 1):
                update = next(handed)
                expected.accept(frame)
                self.assertEqual(update.completed, index)
                self.assertEqual(receiver.state.completed, index)
                self.assertEqual(receiver.state.position, expected.position)
                self.assertEqual(receiver.state.heights.tobytes(), expected.heights.tobytes())
                self.assertEqual(receiver.buffered_motion(), index < len(frames))
                path = Path(directory) / "prefix.bin"
                receiver.state.write_checkpoint(path)
                restored = StockState(producer.manifest_frame)
                restored.restore_checkpoint(path)
                self.assertEqual(restored.completed, index)
                self.assertEqual(restored.heights.tobytes(), expected.heights.tobytes())
            self.assertEqual(list(handed), [])

    def test_retained_query_worker_reuses_move_start_and_cannot_commit(self):
        with simulation() as sim:
            next(sim.motions(moves()[:1]))
            before = sim.state.heights.tobytes()
            with PrefixSession(sim.state.canonical_manifest) as query:
                pid = query._process.pid
                first, _ = query.preview(sim.state, moves()[1], 0.25)
                second, _ = query.preview(sim.state, moves()[1], 0.75)
                self.assertEqual(query._process.pid, pid)
                self.assertEqual(query.checkpoint_loads, 1)
                self.assertEqual(query.queries, 2)
                self.assertNotEqual(first.heights.tobytes(), second.heights.tobytes())
                self.assertEqual(sim.state.heights.tobytes(), before)
                next(sim.motions(moves()[1:2]))
                query.preview(sim.state, moves()[2], 0.5)
                self.assertEqual(query.checkpoint_loads, 2)
                self.assertEqual(query._process.pid, pid)
                committed = sim.state.heights.tobytes()
                query._send({"command": "motions", "motions": [moves()[2].to_dict()["motion"]]})
                with self.assertRaisesRegex(SimulationError, "read-only"):
                    query._next()
                self.assertEqual(sim.state.heights.tobytes(), committed)
                self.assertIsNone(sim._process.poll())
            sim.finish()

    def test_python_authoring_streams_actual_removal_and_explicit_finish(self):
        with simulation() as sim:
            first = next(sim.motions(moves()[:1]))
            self.assertEqual(first.motion["operation"], "pocket")
            self.assertEqual(first.motion["curve"]["kind"], "line")
            self.assertGreater(first.changed_count, 0)
            self.assertGreater(sim.state.volume()["removed_lower_mm3"], 0)
            # The middle is cut even though it is far from both endpoint disks.
            index = 25 * sim.state.nx + 50
            self.assertLess(sim.state.heights[2 * index + 1], 6.0001)
            self.assertIsNone(sim._process.poll())
            self.assertFalse(sim.state.finished)
            updates = list(sim.run(moves()[1:], batch_size=2))
            self.assertEqual([u.motion["curve"]["kind"] for u in updates], ["line", "arc", "helix"])
            self.assertTrue(sim.state.finished)
            self.assertEqual(sim.state.completed, 4)
            self.assertEqual(sim.state.position, (14, 18, 4))
            self.assertEqual(sim._process.poll(), 0)
            with self.assertRaises(TypeError):
                sim.state.heights[0] = 0

    def test_python_disk_checkpoints_restore_and_native_partial_query_interoperate(self):
        with simulation() as sim, tempfile.TemporaryDirectory() as temp:
            first = next(sim.motions(moves()[:1]))
            before = sim.state.heights.tobytes()
            path = Path(temp) / "base.bin"
            sim.state.write_checkpoint(path)
            restored = StockState(sim.manifest_frame)
            restored.restore_checkpoint(path)
            self.assertEqual(restored.heights.tobytes(), before)
            self.assertEqual(restored.completed, 1)
            self.assertEqual(restored.position, sim.state.position)
            shown, update = preview_prefix(restored, moves()[1], 0.371, directory=temp)
            self.assertEqual(update.fraction, 0.371)
            self.assertEqual(shown.completed, 1)
            self.assertAlmostEqual(shown.position[1], 8.71)
            self.assertEqual(restored.heights.tobytes(), before)
            self.assertNotEqual(shown.heights.tobytes(), before)
            repeated, _ = preview_prefix(restored, moves()[1], 0.371, directory=temp)
            self.assertEqual(repeated.heights.tobytes(), shown.heights.tobytes())
            whole, _ = preview_prefix(restored, moves()[1], 1, directory=temp)
            second = next(sim.motions(moves()[1:2]))
            self.assertEqual(whole.heights.tobytes(), sim.state.heights.tobytes())
            restored.accept(second.frame)
            self.assertEqual(restored.heights.tobytes(), sim.state.heights.tobytes())
            # Invalid snapshot contents with a valid CRC still cannot alter state.
            raw = bytearray(path.read_bytes())
            raw[-8:] = struct.pack("<d", math.nan)
            path.write_bytes(checksum(raw))
            current = restored.heights.tobytes()
            with self.assertRaisesRegex(ProtocolError, "height bounds"):
                restored.restore_checkpoint(path)
            self.assertEqual(restored.heights.tobytes(), current)
            sim.finish()
            self.assertGreater(len(first.frame.raw), 20)

    def test_lost_repeated_stale_and_corrupt_chunks_preserve_state(self):
        with simulation(run_id="source") as sim:
            updates = list(sim.motions(moves()[:2]))
            receiver = StockState(sim.manifest_frame)
            baseline = receiver.heights.tobytes()
            with self.assertRaisesRegex(ProtocolError, "out-of-order"):
                receiver.accept(updates[1].frame)
            self.assertEqual(receiver.heights.tobytes(), baseline)
            raw = bytearray(updates[0].frame.raw)
            raw[-1] ^= 128
            with self.assertRaisesRegex(ProtocolError, "checksum"):
                receiver.accept(raw)
            self.assertEqual(receiver.heights.tobytes(), baseline)
            # Forge a late before-value and repair CRC: transaction must reject
            # it before even the earlier valid cells change.
            raw = bytearray(updates[0].frame.raw)
            offset = len(raw) - updates[0].changed_count * 36 + 36 + 4
            struct.pack_into("<d", raw, offset, 9.0)
            with self.assertRaisesRegex(ProtocolError, "old state"):
                receiver.accept(checksum(raw))
            self.assertEqual(receiver.heights.tobytes(), baseline)
            receiver.accept(updates[0].frame)
            once = receiver.heights.tobytes()
            with self.assertRaisesRegex(ProtocolError, "repeated"):
                receiver.accept(updates[0].frame)
            self.assertEqual(receiver.heights.tobytes(), once)
            end = sim.finish()
            with self.assertRaisesRegex(ProtocolError, "finish"):
                receiver.accept(end)
            self.assertFalse(receiver.finished)
            with simulation(run_id="different-run") as other:
                fresh = other.state.heights.tobytes()
                with self.assertRaisesRegex(ProtocolError, "stale/foreign"):
                    other.state.accept(updates[0].frame)
                self.assertEqual(other.state.heights.tobytes(), fresh)

    def test_binary_reader_checks_lengths_before_allocating_and_distinguishes_truncation(self):
        with simulation() as sim:
            frame = sim.manifest_frame
            self.assertIsNone(Frame.read(BytesIO()))
            for count in [1, 19, len(frame.raw) - 1]:
                with self.assertRaisesRegex(ProtocolError, "truncated"):
                    Frame.read(BytesIO(frame.raw[:count]))
            oversized = bytearray(frame.raw[:20])
            oversized[12:16] = struct.pack("<I", 0xFFFFFFFF)
            # A claimed length is never preallocated: a lying header reads
            # incrementally and reports truncation; an explicit limit refuses it.
            with self.assertRaisesRegex(ProtocolError, "truncated"):
                Frame.read(BytesIO(oversized))
            with self.assertRaisesRegex(ProtocolError, "budget"):
                Frame.read(BytesIO(oversized), limit=8 << 20)
            self.assertLess(len(frame.raw), FRAME_LIMIT)
            with self.assertRaisesRegex(ProtocolError, "length"):
                Frame.parse(frame.raw + b"extra")

    def test_batch_must_be_drained_and_cancel_keeps_prefix_partial(self):
        with simulation() as sim:
            iterator = sim.motions(moves())
            next(iterator)
            with self.assertRaisesRegex(SimulationError, "drain"):
                sim.finish()
            self.assertEqual(len(list(iterator)), 3)
            sim.finish()
        sim = simulation()
        initial = sim.state.heights.tobytes()
        # Deliberately don't drain stdout: the bounded native pipe applies
        # backpressure. Cancellation must terminate this owned producer too.
        sim._send({"command": "motions", "motions": [moves()[0].to_dict()["motion"]]})
        sim.cancel()
        self.assertTrue(sim.cancelled)
        self.assertIsNotNone(sim._process.poll())
        self.assertFalse(sim.state.finished)
        self.assertEqual(sim.state.heights.tobytes(), initial)

    def test_native_input_errors_are_reported_without_leaking_a_process(self):
        with self.assertRaisesRegex(SimulationError, "stock"):
            Simulation(
                StockSpec((0, 0, 0), (20, 20, 10), -0.1), (tool(),), revision="invalid", initial_position_mm=(3, 5, 6)
            )


if __name__ == "__main__":
    unittest.main()
