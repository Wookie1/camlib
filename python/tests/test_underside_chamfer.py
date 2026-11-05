"""Refixtured convex CAD: full conical section clearance and posted coverage."""

import importlib.util
import math
import os
from pathlib import Path
import unittest
import cadquery as cq
from camlib.playback import Player, Cursor
from camlib.simulation import Simulation, Update


def example():
    path = Path(__file__).resolve().parents[2] / 'python/tests/support/underside_chamfer.py'
    spec = importlib.util.spec_from_file_location('underside_example', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class UndersideChamferTest(unittest.TestCase):
    def test_refixtured_full_cone_sweeps_and_posted_loop_stock_replay(self):
        module = example()
        job = module.build_job()
        part, face, _ = module.geometry()
        # Verify source's entire convex bevel family, not only endpoint samples.
        area = face.Area()
        perimeter = face.outerWire().Length()
        width = 0.8
        self.assertAlmostEqual(area, 36 * 24 - (4 - math.pi) * 9)
        self.assertAlmostEqual(perimeter, 96 + 6 * math.pi)
        desired_bevel = perimeter * width * width / 2 - math.pi * width**3 / 3
        self.assertAlmostEqual(area * 12 - part.Volume(), desired_bevel, places=7)
        ideal = (36 * 24 - area) * 2.5 + desired_bevel
        reference = face.outerWire().rotate((0, 0, 0), (1, 0, 0), 180).translate((0, 0, 0.8))

        def prove(motions):
            from OCP.Geom import Geom_Circle
            from OCP.gp import gp_Ax2, gp_Pnt, gp_Dir
            from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge

            final = []
            for m in motions:
                c = m['curve']
                a = c['start']
                b = c['end']
                tip = min(a[2], b[2])
                if tip >= 0:
                    continue
                if m['kind'] == 'rapid':
                    # Only a proven straight-up retract out of the cut.
                    self.assertEqual(a[:2], b[:2])
                    self.assertGreater(b[2], a[2])
                    continue
                aa = (a[0], a[1], 0)
                bb = (b[0], b[1], 0)
                if c['kind'] == 'line':
                    edge = cq.Vertex.makeVertex(*aa) if math.dist(aa, bb) < 1e-12 else cq.Edge.makeLine(aa, bb)
                else:
                    self.assertEqual(c['kind'], 'arc')
                    self.assertEqual(c['plane'], 'xy')
                    o = c['center']
                    r = math.dist(a[:2], o[:2])
                    theta = math.atan2(a[1] - o[1], a[0] - o[0])
                    lo, hi = sorted((theta, theta + c['sweep_rad']))
                    edge = cq.Edge(
                        BRepBuilderAPI_MakeEdge(
                            Geom_Circle(gp_Ax2(gp_Pnt(o[0], o[1], 0), gp_Dir(0, 0, 1)), r), lo, hi
                        ).Edge()
                    )
                d = edge.distance(reference)
                q = (abs(a[0]) - 15, abs(a[1]) - 9)
                self.assertGreater(math.hypot(max(q[0], 0), max(q[1], 0)) + min(max(q), 0) - 3, 0)
                if m['tool'] == 'mill':
                    self.assertGreaterEqual(d, 2 - 0.005)
                else:
                    # r(z)=.25+z-tip, inner offset=max(0,.8+z).
                    # Minimum across both height intervals is at z=-.8 or0.
                    self.assertGreaterEqual(min(d - 0.25, d + tip + width - 0.25), -0.005)
                    if m['kind'] == 'cut' and abs(tip + 1.3) < 1e-7:
                        final.append(c)
            self.assertEqual(len(final), 8)
            for i, c in enumerate(final):
                self.assertLess(math.dist(c['end'], final[(i + 1) % 8]['start']), 1e-7)
            for c in final:
                a, b = c['start'], c['end']
                if c['kind'] == 'arc':
                    o = c['center']
                    self.assertAlmostEqual(abs(o[0]), 15)
                    self.assertAlmostEqual(abs(o[1]), 9)
                    # Quarter-circle corners (climb milling by default runs them clockwise).
                    self.assertAlmostEqual(abs(c['sweep_rad']), math.pi / 2)
                    offset = math.dist(a[:2], o[:2]) - 3
                elif abs(a[0] - b[0]) < 1e-7:
                    self.assertAlmostEqual(abs(a[1] - b[1]), 18)
                    offset = abs(a[0]) - 18
                else:
                    self.assertAlmostEqual(a[1], b[1])
                    self.assertAlmostEqual(abs(a[0] - b[0]), 30)
                    offset = abs(a[1]) - 12
                self.assertGreaterEqual(1.55 - offset, width - 0.01)
                self.assertLessEqual(1.55 - offset, width + 0.005)

        config = job.compile().setup('bottom').simulation_options()
        config['executable'] = os.environ.get('CAMLIB_TEST_ENGINE')
        with Simulation(**config) as sim:
            native = [u.motion for u in sim.run_operations() if isinstance(u, Update)]
            self.assertTrue(sim.state.finished)
            volume = sim.state.volume()
            self.assertLessEqual(volume['removed_lower_mm3'], ideal)
            self.assertGreaterEqual(volume['removed_upper_mm3'], ideal)
        prove(native)
        with job.post(
            'bottom', machine=module.machine_for(job), executable=os.environ.get('CAMLIB_TEST_ENGINE')
        ) as run:
            run.wait(timeout=60)
            player = Player(run)
            last = run.status()['available']
            player.seek(Cursor(last))
            volume = player.state.volume()
            self.assertLessEqual(volume['removed_lower_mm3'], ideal)
            self.assertGreaterEqual(volume['removed_upper_mm3'], ideal)
            prove([run.posted_motion(i)['motion'] for i in range(last)])
            final = player.state.heights.tobytes()
            cursor = Cursor(last - 4, 0.37)
            player.seek(cursor)
            partial = player.state.heights.tobytes()
            player.seek(Cursor(last))
            self.assertEqual(final, player.state.heights.tobytes())
            player.seek(cursor)
            self.assertEqual(partial, player.state.heights.tobytes())
            self.assertTrue(run.posting_status()['finished'])


if __name__ == '__main__':
    unittest.main()
