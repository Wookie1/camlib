from camlib.postprocessor import GCodeDialect
from collections import Counter
from functools import lru_cache
import importlib.util
import math
from pathlib import Path
import unittest

from camlib.cadquery import export_hemisphere_surface


@lru_cache(None)
def example():
    path = Path(__file__).resolve().parents[2] / "python/tests/support/curved_finishing.py"
    spec = importlib.util.spec_from_file_location("curved_example", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@lru_cache(None)
def curved_config(strategy="parallel"):
    return example().curved_job(strategy)


def hemisphere_face(**kwargs):
    import cadquery as cq

    shape = cq.Solid.makeSphere(4, cq.Vector(8, 8, 2), **kwargs)
    return next(f for f in shape.Faces() if f.geomType() == "SPHERE")


class CurvedSurfaceAdapterTest(unittest.TestCase):
    def test_actual_cad_all_facet_error_conforming_coverage_and_volume(self):
        config = curved_config()
        surface = config["operations"][0].surface
        self.assertEqual(surface.source["adapter"], "analytic_upper_hemisphere_facets_v1")
        self.assertLessEqual(surface.surface_error_mm, 0.005)
        self.assertAlmostEqual(surface.surface_error_mm, sum(surface.source["error_components_mm"].values()))
        counts = Counter()
        points = set()
        worst = 0.0
        volume = 0.0
        for tri in surface.triangles:
            vectors = [tuple(p[k] - (8, 8, 2)[k] for k in range(3)) for p in tri]
            for p, v in zip(tri, vectors):
                self.assertAlmostEqual(math.sqrt(sum(x * x for x in v)), 4, places=11)
                self.assertGreaterEqual(p[2], 2)
                points.add(p)
            a, b, c = vectors
            ab = tuple(b[k] - a[k] for k in range(3))
            ac = tuple(c[k] - a[k] for k in range(3))
            n = (ab[1] * ac[2] - ab[2] * ac[1], ab[2] * ac[0] - ab[0] * ac[2], ab[0] * ac[1] - ab[1] * ac[0])
            signed = sum(n[k] * a[k] for k in range(3))
            self.assertGreater(signed, 0)
            worst = max(worst, 4 - signed / math.sqrt(sum(x * x for x in n)))
            bc = (b[1] * c[2] - b[2] * c[1], b[2] * c[0] - b[0] * c[2], b[0] * c[1] - b[1] * c[0])
            volume += sum(a[k] * bc[k] for k in range(3)) / 6
            for p, q in zip(tri, (*tri[1:], tri[0])):
                counts[tuple(sorted((p, q)))] += 1
        self.assertAlmostEqual(worst, surface.source["error_components_mm"]["facets"], places=11)
        self.assertEqual(len(points) - len(counts) + len(surface.triangles), 1)
        boundary = [e for e, n in counts.items() if n == 1]
        self.assertTrue(all(n in (1, 2) for n in counts.values()))
        self.assertEqual(len(boundary), 4 * 2 ** surface.source["subdivision_level"])
        self.assertTrue(all(a[2] == b[2] == 2 for a, b in boundary))
        degree = Counter(p for e in boundary for p in e)
        self.assertTrue(all(n == 2 for n in degree.values()))
        true = 2 * math.pi * 4**3 / 3
        self.assertLess(volume, true)
        self.assertLess(true - volume, 2 * math.pi * 16 * surface.surface_error_mm)
        shape = example().target().val()
        self.assertTrue(shape.isValid())
        self.assertAlmostEqual(shape.Volume(), 512 + true, places=7)

    def test_budgets_transforms_and_rejection_of_incomplete_or_reversed_faces(self):
        import cadquery as cq
        from OCP.TopoDS import TopoDS

        face = hemisphere_face()
        coarse = export_hemisphere_surface(face, max_error_mm=0.02)
        fine = curved_config()["operations"][0].surface
        self.assertLess(len(coarse.triangles), len(fine.triangles))
        self.assertLessEqual(coarse.surface_error_mm, 0.02)
        transformed = face.rotate((0, 0, 0), (0, 0, 1), 37).translate((1, 2, 3))
        moved = export_hemisphere_surface(transformed, max_error_mm=0.02)
        expected = transformed.Center()  # sphere surface centroid is above its equator
        self.assertAlmostEqual(moved.source["base_z_mm"], 5)
        self.assertAlmostEqual(moved.source["radius_mm"], 4)
        self.assertAlmostEqual(expected.z, 7, places=7)
        for bad in [
            hemisphere_face(angleDegrees1=-90, angleDegrees2=90),
            hemisphere_face(angleDegrees2=60),
            hemisphere_face(angleDegrees3=180),
            face.rotate((0, 0, 0), (1, 0, 0), 10),
            cq.Face(TopoDS.Face(face.wrapped.Reversed())),
        ]:
            with self.assertRaises(ValueError):
                export_hemisphere_surface(bad)
        for kwargs in [
            dict(max_error_mm=float("nan")),
            dict(max_error_mm=1e-10),
            dict(max_facets=16),
            dict(max_facets=3),
            dict(max_facets=4.5),
        ]:
            with self.assertRaises(ValueError):
                export_hemisphere_surface(face, **kwargs)

    def test_hole_with_full_sphere_uv_bounds_is_not_accepted_as_complete(self):
        import cadquery as cq

        sphere = cq.Solid.makeSphere(4, cq.Vector(8, 8, 2))
        cutter = cq.Workplane("XY").center(9.5, 8).circle(0.25).extrude(10).val()
        cut = sphere.cut(cutter)
        faces = [f for f in cut.Faces() if f.geomType() == "SPHERE"]
        self.assertTrue(faces)
        for f in faces:
            with self.assertRaisesRegex(ValueError, "unholed|trim|rectangle"):
                export_hemisphere_surface(f)


class CurvedSurfacePlaybackTest(unittest.TestCase):
    def test_waterline_actual_cad_stream_checked_post_and_partial_stock_seek(self):
        from camlib.motion import Program
        from camlib.playback import Cursor, Player
        from camlib.post import Machine, ToolSetting
        from camlib.posted import OperationEvents, PostedRun
        from camlib.simulation import Simulation

        config = curved_config("waterline")
        tool = config["tools"][0]
        header = Program(
            "curved-waterline-post", config["revision"], config["tools"], config["initial_position_mm"], ()
        )
        machine = Machine(
            "Python curved finishing",
            (ToolSetting.for_tool(tool, number=1, length_register=1, position_after_mm=header.initial_position_mm),),
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
        source = Simulation(**config)
        with PostedRun(
            header, OperationEvents(source), machine=machine, stock=config["stock"], checkpoint_every=16
        ) as run:
            player = Player(run)
            player.pause()
            run.wait_available(1, timeout=30)
            self.assertFalse(run.status()["finished"])
            run.wait(timeout=60)
            status = run.status()
            self.assertTrue(status["posting"]["finished"])
            self.assertEqual(status["posting"]["checked_motions"], status["available"])
            self.assertLessEqual(status["largest_frame_bytes"], 8 * 1024 * 1024)
            self.assertLess(status["retained_bytes"], run.retained_bytes_limit)
            self.assertIsNotNone(status["first_stock_seconds"])
            player.seek(Cursor(status["available"]))
            final = player.state.heights.tobytes()
            self.assertGreater(player.state.volume()["removed_lower_mm3"], 10)
            self.assertEqual(player.state._manifest["run"]["revision"], config["revision"])
            partial = None
            for i in range(status["available"]):
                player.seek(Cursor(i))
                before = player.state.heights.tobytes()
                player.seek(Cursor(i, 0.5))
                if player.motion["kind"] == "cut" and player.state.heights.tobytes() != before:
                    partial = Cursor(i, 0.5)
                    break
            self.assertIsNotNone(partial)
            saved = player.state.heights.tobytes()
            player.tick(1)
            self.assertEqual(player.cursor, partial)
            player.seek(Cursor(status["available"]))
            self.assertEqual(player.state.heights.tobytes(), final)
            player.seek(partial)
            self.assertEqual(player.state.heights.tobytes(), saved)
            player.set_speed(2)
            player.play()
            player.tick(0.01)
            self.assertGreater(player.seconds, 0)
            player.seek(Cursor())
            self.assertEqual(player.state.volume()["removed_upper_mm3"], 0)
        self.assertIsNotNone(source._process.poll())

    def test_parallel_actual_removal_live_prefix_pause_speed_seek_and_revision(self):
        from camlib.playback import Cursor, Player, RecordedRun

        config = curved_config()
        with RecordedRun(**{'rapid_mm_min': 6000, **config}, checkpoint_every=32) as run:
            player = Player(run)
            player.pause()
            run.wait_available(16, timeout=30)
            self.assertFalse(player.state.finished)
            self.assertEqual(player.cursor, Cursor())
            run.wait(timeout=60)
            status = run.status()
            self.assertTrue(status["finished"])
            self.assertLessEqual(status["largest_frame_bytes"], 8 * 1024 * 1024)
            self.assertLess(status["retained_bytes"], run.retained_bytes_limit)
            self.assertIsNotNone(status["first_stock_seconds"])
            player.seek(Cursor(status["available"]))
            final = player.state.heights.tobytes()
            self.assertEqual(player.state._manifest["run"]["revision"], config["revision"])
            self.assertGreater(player.state.volume()["removed_lower_mm3"], 1300)
            self.assertLessEqual(player.state.volume()["removed_lower_mm3"], 2048 - example().target().val().Volume())
            # Find a committed cut that has a real removal prefix, then compare
            # partial replay across backward/final seeks using this one recording.
            partial = None
            for i in range(status["available"]):
                player.seek(Cursor(i))
                before = player.state.heights.tobytes()
                player.seek(Cursor(i, 0.5))
                if player.motion["kind"] == "cut" and player.state.heights.tobytes() != before:
                    partial = Cursor(i, 0.5)
                    break
            self.assertIsNotNone(partial)
            saved = player.state.heights.tobytes()
            player.pause()
            player.tick(1)
            self.assertEqual(player.cursor, partial)
            player.seek(Cursor(status["available"]))
            self.assertEqual(player.state.heights.tobytes(), final)
            player.seek(partial)
            self.assertEqual(player.state.heights.tobytes(), saved)
            player.set_speed(2)
            player.play()
            player.tick(0.01)
            self.assertGreater(player.seconds, 0)
            player.seek(Cursor())
            self.assertEqual(player.state.volume()["removed_upper_mm3"], 0)
