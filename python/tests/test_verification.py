from camlib.postprocessor import GCodeDialect
from dataclasses import replace
import itertools
import unittest

from camlib.motion import Arc, Helix, Line, Motion, WorkFrame
from camlib.planar import Boundary, Component, PlanarRegion
from camlib.verification import MachineLimits, Options, Prism, Scene, Verifier
from test_simulation import tool


def scene():
    region = PlanarRegion("clamp", (Component(Boundary.polygon(((-0.2, -0.2), (0.2, -0.2), (0.2, 0.2), (-0.2, 0.2)))),))
    return Scene("scene", "cad-1", (Prism("clamp", region, 1, 2),))


class VerificationTest(unittest.TestCase):
    def test_python_machine_normal_acceleration_limit_and_explicit_nominal_scope(self):
        import math
        from camlib.postprocessor import Machine, ToolSetting

        machine = Machine(
            "curve-limits",
            (ToolSetting("t1", 1, 1, 60, (0, 0, 20)),),
            tip_travel_mm=((-100, -100, -100), (100, 100, 100)),
            max_normal_accel_mm_s2=40,
            dialect=GCodeDialect.linuxcnc(),
            output_units='mm',
            decimals=3,
            max_feed_mm_min=10000,
            max_spindle_rpm=30000,
            planes=('xy', 'xz', 'yz'),
            arcs=True,
            helices=True,
            max_sweep_rad=6.283185307179586,
            max_tool_number=99999,
            max_length_register=99999,
            tool_changes=True,
            flood=True,
            mist=True,
            tool_change_position_mode='table',
            rapid_mm_min=6000,
        )
        limits = machine.verification_limits()
        self.assertEqual(limits.max_normal_accel_mm_s2, 40)
        s = Scene("curve-limits", "1", machine=limits)
        arc = Motion(1, "curve", "t1", Arc((2, 0, 0), (2, 0, 0), (0, 0, 0), math.tau), 600)
        helix = replace(arc, curve=Helix((2, 0, 0), (2, 0, 2 * math.tau), (0, 0, 0), math.tau))
        with Verifier(s) as verifier:
            report = verifier.check(tool(), arc)
            self.assertEqual(report["status"], "violation")
            self.assertTrue(report["coverage"]["nominal_normal_accel_checked"])
            self.assertEqual(report["findings"][0]["kind"], "normal_acceleration_limit")
            self.assertIn("excludes junctions", report["findings"][0]["detail"])
            self.assertEqual(verifier.check(tool(), helix)["status"], "clear")
            with self.assertRaises(ValueError):
                verifier.check(tool(), replace(arc, kind="rapid", feed_mm_min=None))
            rapid = verifier.check(
                tool(), replace(arc, kind="rapid", feed_mm_min=None, curve=Line((0, 0, 0), (1, 0, 0)))
            )
            self.assertFalse(rapid["coverage"].get("nominal_normal_accel_checked", False))
        self.assertNotIn("max_normal_accel_mm_s2", replace(limits, max_normal_accel_mm_s2=None).to_dict())
        for value in (0, -1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                replace(machine, max_normal_accel_mm_s2=value)

    def test_prepared_python_motion_stream_reports_localized_independent_findings(self):
        motions = (Motion(i, "rough", "t1", Line((-10, y, 0), (10, y, 0)), 600) for i, y in enumerate((0, 10, 0)))
        with Verifier(scene()) as verifier:
            results = list(verifier.check_motions(tool(), motions))
            self.assertEqual([r["status"] for r in results], ["violation", "clear", "violation"])
            hit = results[0]
            self.assertEqual(hit["operation"], "rough")
            self.assertEqual(hit["findings"][0]["object_id"], "clamp")
            self.assertEqual(hit["findings"][0]["witness"]["fraction"], 0.5)
            self.assertEqual(hit["findings"][0]["part"], "cutter")
            self.assertEqual(hit["scene_sha256"], results[1]["scene_sha256"])
            self.assertEqual(len(hit["tool_sha256"]), 64)
            self.assertNotEqual(hit["motion_sha256"], results[1]["motion_sha256"])
            self.assertFalse(hit["coverage"]["remaining_material"])
            self.assertFalse(hit["coverage"]["evolving_stock_contact"])

    def test_scene_geometry_digest_is_checked_even_with_same_id_and_revision(self):
        with Verifier(scene()) as verifier:
            identity = dict(verifier._identity)
            changed = replace(scene(), bodies=(replace(scene().bodies[0], z_min_mm=100, z_max_mm=101),))
            verifier.prepare(changed)
            self.assertNotEqual(identity["scene_sha256"], verifier._identity["scene_sha256"])
            m = Motion(1, "rough", "t1", Line((-10, 0, 0), (10, 0, 0)), 600)
            with self.assertRaisesRegex(ValueError, "identity/revision"):
                verifier._engine._request(
                    "verify_motion", **identity, tool=tool().to_dict(), motion=m.to_dict()["motion"]
                )
            self.assertEqual(verifier.check(tool(), m)["status"], "clear")
            with self.assertRaises(ValueError):
                verifier.prepare(replace(scene(), bodies=(replace(scene().bodies[0], z_min_mm=3, z_max_mm=2),)))
            self.assertEqual(verifier.check(tool(), m)["status"], "clear")

    def test_machine_frame_feed_and_rapid_scope_are_explicit(self):
        limits = MachineLimits((90, -10, -10), (110, 10, 100), 500)
        s = Scene("travel", "1", machine=limits, work_to_machine=WorkFrame(origin_mm=(100, 0, 0)))
        m = Motion(1, "travel", "t1", Line((-1, 0, 0), (1, 0, 0)), 600)
        with Verifier(s) as verifier:
            result = verifier.check(tool(), m)
            self.assertEqual(result["findings"][0]["kind"], "feed_limit")
            rapid = replace(m, kind="rapid", feed_mm_min=None)
            result = verifier.check(tool(), rapid)
            self.assertEqual(result["status"], "clear")
            self.assertFalse(result["coverage"]["declared_fed_rate_checked"])
            self.assertTrue(result["coverage"]["machine_tip_travel_examined"])

    def test_authored_target_floor_tolerance_is_reported_and_does_not_relax_rapid(self):
        floor = replace(scene().bodies[0], role="protected_target", z_min_mm=0, z_max_mm=3)
        target = Scene("machined-floor", "1", (floor,))
        motion = Motion(1, "floor", "t1", Line((-0.1, 0, 3), (0.1, 0, 3)), 600)
        with Verifier(target) as strict:
            self.assertEqual(strict.check(tool(), motion)["status"], "uncertain")
            strict_identity = strict._identity["scene_sha256"]
        with Verifier(target, Options(target_floor_tolerance_mm=0.01)) as verifier:
            self.assertNotEqual(verifier._identity["scene_sha256"], strict_identity)
            report = verifier.check(tool(), motion)
            self.assertEqual(report["status"], "clear")
            self.assertEqual(report["target_floor_tolerance_mm"], 0.01)
            rapid = replace(motion, kind="rapid", feed_mm_min=None)
            self.assertEqual(verifier.check(tool(), rapid)["status"], "uncertain")
            gouge = replace(motion, curve=Line((-0.1, 0, 2.98), (0.1, 0, 2.98)))
            self.assertEqual(verifier.check(tool(), gouge)["status"], "violation")

    def test_work_and_input_bounds_and_uncertainty_are_not_clear(self):
        # Any number of bodies, but an endless iterator is refused unread.
        with self.assertRaisesRegex(TypeError, "finite collection"):
            Scene("huge", "1", itertools.repeat(scene().bodies[0]))
        with Verifier(scene(), Options(max_nodes=1)) as verifier:
            m = Motion(1, "rough", "t1", Line((-10, 0, 0), (1, 0, 0)), 600)
            result = verifier.check(tool(), m)
            self.assertEqual(result["status"], "uncertain")
            self.assertTrue(result["stats"]["budget_exhausted"])
            self.assertGreater(result["stats"]["unexamined_pairs"], 0)


if __name__ == "__main__":
    unittest.main()
