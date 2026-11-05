from dataclasses import replace
import json
import struct
import unittest
import zlib

from camlib.material import Target, Options as MaterialOptions
from camlib.planar import Boundary, Component, PlanarRegion
from camlib.playback import RecordedRun, Player, Cursor
from camlib.simulation import (
    Simulation,
    StockSpec,
    Frame,
    VerificationUpdate,
    VerificationStream,
    OperationUpdate,
    ProtocolError,
    SimulationError,
    _Decoder,
)
from camlib.verification import StreamChecks, MaterialCheck, Scene, MachineLimits
from test_operations import small
from test_simulation import tool, moves, simulation


def checks(*, material=True, scene=True, after_operation=None, capped=False):
    region = PlanarRegion("target", (Component(Boundary.polygon(((-1, -1), (21, -1), (21, 21), (-1, 21)))),))
    return StreamChecks(
        Scene("limits", "1", machine=MachineLimits((-100, -100, -100), (100, 100, 100), 500)) if scene else None,
        material=(
            MaterialCheck(
                Target("finished", "1", region, 5),
                after_operation,
                MaterialOptions(max_findings=8, max_classified_cells=1 if capped else 16_777_216),
            ),
        )
        if material
        else (),
    )


def modified(frame, edit):
    decoder = _Decoder(frame.payload)
    decoder.string(128)
    decoder.string(512)
    decoder.take(32)
    prefix = frame.payload[: decoder.offset].tobytes()
    data = json.loads(frame.payload[decoder.offset :].tobytes())
    edit(data)
    body = prefix + json.dumps(data, separators=(",", ":"), allow_nan=False).encode()
    header = struct.pack("<8sHHI", b"CAMLSTRM", 1, 8, len(body))
    return Frame.parse(header + struct.pack("<I", zlib.crc32(body, zlib.crc32(header))) + body)


class StreamVerificationTest(unittest.TestCase):
    def test_normal_acceleration_scope_cannot_be_removed_from_a_bound_report(self):
        config = checks(material=False)
        config = replace(
            config,
            scene=replace(
                config.scene, machine=replace(config.scene.machine, max_normal_accel_mm_s2=500, max_feed_mm_min=1000)
            ),
        )
        with simulation(verification=config) as sim:
            motion, report = list(sim.motions(moves()[:1]))
            self.assertTrue(report.check["report"]["coverage"]["nominal_normal_accel_checked"])
            reader = VerificationStream(sim.manifest_frame)
            bad = modified(
                report.frame, lambda d: d["check"]["report"]["coverage"].update(nominal_normal_accel_checked=False)
            )
            with self.assertRaises(ProtocolError):
                reader.read(bad, 1, motion.motion)
            self.assertTrue(
                reader.read(report.frame, 1, motion.motion).check["report"]["coverage"]["nominal_normal_accel_checked"]
            )
            sim.finish()

    def test_authored_stream_reports_and_final_material_are_bound_to_actual_cursor(self):
        with simulation(verification=checks()) as sim:
            updates = sim.run(moves(), batch_size=2)
            first = next(updates)
            self.assertGreater(first.changed_count, 0)
            self.assertIsNone(sim.verification_update)  # No initial full stock scan.
            self.assertFalse(sim.state.finished)
            remaining = list(updates)
            reports = [u for u in remaining if isinstance(u, VerificationUpdate)]
            # The natively read kind and status agree with the check JSON.
            parsed = [replace(u, _kind_status=None) for u in reports]
            self.assertEqual([(u.kind, u.status) for u in reports], [(u.kind, u.status) for u in parsed])
            self.assertEqual([u.kind for u in reports], ["motion"] * 4 + ["material_started", "material"])
            self.assertEqual([u.completed for u in reports], [1, 2, 3, 4, 4, 4])
            self.assertEqual([u.progress["records"] for u in reports], list(range(1, 7)))
            self.assertEqual(reports[-1].progress["violation"], 3)
            material = reports[-1].check["report"]
            self.assertEqual(material["context"]["generation"], sim.state.completed)
            self.assertEqual(material["context"]["run"]["manifest_sha256"], sim.state._digest.hex())
            self.assertTrue(material["complete"])
            self.assertGreater(material["residual"]["definite_cells"], 0)
            self.assertTrue(sim.state.finished)

    def test_bad_identity_progress_source_scope_and_crc_preserve_receipt_state(self):
        with simulation(verification=checks(material=False)) as sim:
            updates = list(sim.motions(moves()[:1]))
            motion, report = updates
            reader = VerificationStream(sim.manifest_frame)
            baseline = sim.state.heights.tobytes()
            mutations = [
                lambda d: d["progress"].update(records=2),
                lambda d: d["check"]["report"].update(scene_sha256="0" * 64),
                lambda d: d["check"]["report"]["coverage"].update(evolving_stock_contact=True),
                lambda d: d.update(completed=2),
                lambda d: d["check"]["report"].update(tool_sha256="0" * 64),
            ]
            for mutation in mutations:
                with self.assertRaises(ProtocolError):
                    reader.read(modified(report.frame, mutation), 1, motion.motion)
            with self.assertRaises(ProtocolError):
                reader.read(report.frame, 0, motion.motion)
            foreign = dict(motion.motion, feed_mm_min=499)
            with self.assertRaises(ProtocolError):
                reader.read(report.frame, 1, foreign)
            good = reader.read(report.frame, 1, motion.motion)
            self.assertEqual(good.progress["violation"], 1)
            with self.assertRaises(ProtocolError):
                reader.read(report.frame, 1, motion.motion)
            with simulation(verification=checks(material=False), run_id="other-run") as other:
                with self.assertRaises(ProtocolError):
                    other._verification.read(report.frame, 1, motion.motion)
            self.assertEqual(sim.state.heights.tobytes(), baseline)
            sim.finish()

    def test_undrained_checks_block_commands_and_material_only_run_has_no_motion_coverage(self):
        with simulation(verification=checks(material=False)) as sim:
            stream = sim.motions(moves()[:1])
            next(stream)
            with self.assertRaises(SimulationError):
                sim.finish()
            self.assertIsInstance(next(stream), VerificationUpdate)
            sim.finish()
        with simulation(verification=checks(scene=False, capped=True)) as sim:
            updates = list(sim.run(moves()))
            reports = [u for u in updates if isinstance(u, VerificationUpdate)]
            self.assertEqual(len(reports), 2)
            self.assertEqual(reports[-1].progress["through_completed"], 0)
            self.assertFalse(reports[-1].check["report"]["complete"])
            self.assertEqual(reports[-1].check["report"]["residual"]["assessment"], "incomplete")
            self.assertTrue(sim.state.finished)  # Generation is separate from checking.

    def test_unexamined_machine_checks_survive_finding_cap_without_clear_claim(self):
        config = checks(material=False)
        config = replace(config, options=replace(config.options, max_findings=1))
        with simulation(verification=config) as sim:
            report = list(sim.motions(moves()[:1]))[-1].check["report"]
            self.assertEqual(report["status"], "violation")
            self.assertFalse(report["coverage"]["machine_tip_travel_examined"])
            self.assertEqual(report["stats"]["unexamined_pairs"], 1)
            sim.finish()

    def test_native_multi_operation_material_stages_are_checked_in_sequence(self):
        first = replace(small(), floor_z_mm=8)
        second = replace(small(), id="deeper", floor_z_mm=4)
        config = checks(after_operation=first.id)
        final = replace(config.material[0], after_operation=None)
        config = replace(config, material=(*config.material, final))
        with Simulation(
            StockSpec((0, 0, 0), (20, 20, 10), 0.5),
            (tool(),),
            operations=(first, second),
            revision="two",
            initial_position_mm=(0, 0, 12),
            verification=config,
        ) as sim:
            material = []
            motion_count = 0
            ends = []
            for update in sim.run_operations():
                if isinstance(update, OperationUpdate):
                    if update.stage == "end":
                        ends.append((update.operation, update.completed))
                elif isinstance(update, VerificationUpdate):
                    if update.kind == "material":
                        material.append(update)
                else:
                    motion_count += 1
            self.assertEqual(len(material), 2)
            self.assertEqual(material[0].completed, ends[0][1])
            self.assertEqual(material[0].check["after_operation"], first.id)
            self.assertEqual(material[-1].completed, sim.state.completed)
            self.assertEqual(material[-1].progress["through_completed"], motion_count)
            self.assertEqual(material[-1].progress["material_reports"], 2)

    def test_retained_pages_cursor_coverage_fractional_replay_and_disk_corruption(self):
        with RecordedRun(
            StockSpec((0, 0, 0), (20, 20, 10), 0.2),
            (tool(),),
            moves(),
            revision="history",
            initial_position_mm=(3, 5, 6),
            verification=checks(),
            checkpoint_every=2,
            rapid_mm_min=6000,
        ) as run:
            run.wait()
            player = Player(run, compact=True)
            self.assertEqual(player.verification["progress"]["through_completed"], 0)
            full = run.verification_status()
            self.assertEqual(full["progress"]["through_completed"], 4)
            self.assertEqual(len(list(run.verification_records(limit=2))), 2)
            self.assertEqual(len(list(run.verification_records(2, limit=4))), 4)
            for cursor in (Cursor(1, 0.4), Cursor(4), Cursor(2, 0.6), Cursor(), Cursor(1, 0.4)):
                player.seek(cursor)
                result = player.verification
                self.assertLessEqual(result["progress"]["through_completed"], cursor.completed)
                self.assertTrue(all(f["completed"] <= cursor.completed for f in result["samples"]))
                if cursor.completed < 4:
                    self.assertIsNone(result["material"])
            player.seek(Cursor(4))
            self.assertTrue(player.state.finished)
            self.assertEqual(player.verification["material"]["status"], "complete")
            self.assertEqual((run.directory / "verification.idx").stat().st_size, 6 * 48)
            with (run.directory / "verification.bin").open("r+b") as f:
                f.seek(-1, 2)
                f.write(b"!")
            with self.assertRaises(ProtocolError):
                list(run.verification_records(5))


if __name__ == "__main__":
    unittest.main()
