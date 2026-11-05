from dataclasses import replace
import unittest

from camlib.motion import Line, Motion
from camlib.playback import RecordedRun, Player, Cursor
from camlib.simulation import Simulation, StockSpec, VerificationUpdate, VerificationStream, ProtocolError
from camlib.verification import StreamChecks, StockContactOptions
from test_simulation import tool
from test_operations import small
from test_stream_verification import checks, modified


def config(**kwargs):
    return dict(
        stock=StockSpec((0, 0, 0), (20, 20, 10), 0.25),
        tools=(tool(),),
        revision="stock-contact-1",
        initial_position_mm=(-5, 5, 6),
        verification=StreamChecks(stock_contact=StockContactOptions(**kwargs)),
    )


def rapids():
    return [
        Motion(1, "rapid-test", "t1", Line((-5, 5, 6), (25, 5, 6)), kind="rapid"),
        Motion(2, "rapid-test", "t1", Line((25, 5, 6), (25, 5, 12)), kind="rapid"),
        Motion(3, "rapid-test", "t1", Line((25, 5, 12), (-5, 5, 12)), kind="rapid"),
    ]


class StockContactTest(unittest.TestCase):
    def test_stock_only_stream_catches_rapid_without_removing_material(self):
        with Simulation(**config()) as sim:
            before = sim.state.heights.tobytes()
            updates = list(sim.run(rapids()))
            reports = [u for u in updates if isinstance(u, VerificationUpdate)]
            self.assertEqual([u.status for u in reports], ["violation", "clear", "clear"])
            self.assertEqual(reports[-1].progress["through_completed"], 3)
            self.assertEqual(reports[-1].progress["violation"], 1)
            self.assertTrue(all(u.check["report"] is None for u in reports))
            self.assertEqual([u.check["stock_contact"]["pre_generation"] for u in reports], [0, 1, 2])
            first = reports[0].check["stock_contact"]
            self.assertEqual(first["scope"], "rapid_assembly")
            finding = list(reports[0].findings())[0]
            self.assertEqual(finding["source"], "stock")
            self.assertEqual(finding["part"], "cutter")
            self.assertIsNotNone(finding["contact_point_mm"])
            self.assertEqual(sim.state.heights.tobytes(), before)
            self.assertTrue(sim.state.finished)

    def test_stock_report_source_cursor_scope_and_witness_corruption_are_rejected(self):
        with Simulation(**config()) as sim:
            motion, report = list(sim.motions(rapids()[:1]))
            reader = VerificationStream(sim.manifest_frame)
            for mutation in [
                lambda d: d["check"]["stock_contact"].update(pre_generation=1),
                lambda d: d["check"]["stock_contact"].update(options_sha256="0" * 64),
                lambda d: d["check"]["stock_contact"].update(scope="fed_noncutting"),
                lambda d: d["check"]["stock_contact"]["findings"][0]["finding"].update(witness=None),
                lambda d: d["check"].update(stock_contact=None),
            ]:
                with self.assertRaises(ProtocolError):
                    reader.read(modified(report.frame, mutation), 1, motion.motion)
            good = reader.read(report.frame, 1, motion.motion)
            self.assertEqual(good.status, "violation")
            sim.finish()

    def test_retained_contact_findings_are_seekable_and_do_not_leak_into_earlier_cursor(self):
        with RecordedRun(**{'rapid_mm_min': 6000, **config()}, motions=rapids(), checkpoint_every=1) as run:
            run.wait()
            player = Player(run, compact=True)
            self.assertTrue(player.verification["coverage"]["evolving_stock_contact"])
            self.assertEqual(player.verification["progress"]["through_completed"], 0)
            full = run.verification_status()
            self.assertEqual(full["progress"]["violation"], 1)
            self.assertEqual(full["samples"][0]["source"], "stock")
            self.assertIn("contact_point_mm", full["samples"][0])
            player.seek(Cursor(0, 0.5))
            self.assertEqual(player.verification["samples"], [])
            player.seek(Cursor(3))
            self.assertEqual(player.verification["progress"]["violation"], 1)
            sample = player.verification["samples"][0]
            player.seek_seconds(sample["seconds"])
            self.assertGreater(player.cursor.fraction, 0)
            self.assertEqual(player.cursor.completed, 0)
            self.assertEqual(player.state.volume()["removed_upper_mm3"], 0)
            player.seek(Cursor(2, 0.5))
            self.assertEqual(player.verification["progress"]["through_completed"], 2)

    def test_native_pocket_checks_actual_stock_before_each_generated_move(self):
        operation = small()
        verification = replace(checks(), stock_contact=StockContactOptions())
        with Simulation(
            StockSpec((0, 0, 0), (20, 20, 10), 0.25),
            (tool(),),
            revision="pocket-contact",
            initial_position_mm=(0, 0, 12),
            operations=(operation,),
            verification=verification,
        ) as sim:
            count = 0
            for update in sim.run_operations():
                if isinstance(update, VerificationUpdate) and update.kind == "motion":
                    count += 1
                    report = update.check["stock_contact"]
                    self.assertEqual(report["pre_generation"], update.completed - 1)
                    self.assertEqual(report["status"], "clear")
                    self.assertTrue(report["fully_examined"])
                    self.assertIn(report["scope"], ("rapid_assembly", "fed_noncutting"))
            self.assertEqual(count, sim.state.completed)
            self.assertGreater(sim.state.volume()["removed_lower_mm3"], 1000)

    def test_stock_contact_budgets_remain_uncertain_and_invalid_options_reject(self):
        with Simulation(**config(max_nodes=1)) as sim:
            updates = list(sim.motions(rapids()[:1]))
            report = updates[-1].check["stock_contact"]
            self.assertNotEqual(report["status"], "clear")
            self.assertFalse(report["fully_examined"])
            self.assertTrue(report["stats"]["work_exhausted"])
            sim.finish()
        from camlib.simulation import SimulationError

        with self.assertRaises(SimulationError):
            Simulation(**config(max_nodes=0))


if __name__ == "__main__":
    unittest.main()
