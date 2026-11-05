import importlib.util
from pathlib import Path
import unittest

import camlib
from camlib.simulation import Simulation, SimulationError

ROOT = Path(__file__).resolve().parents[2]


def example():
    spec = importlib.util.spec_from_file_location("standalone_trace", ROOT / "examples/standalone_job.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def trace(tool, **overrides):
    paths = (
        camlib.TracePath(
            (4, 4),
            (
                camlib.TraceLine((14, 4)),
                camlib.TraceLine((14, 12)),
                camlib.TraceLine((4, 12)),
                camlib.TraceLine((4, 4)),
            ),
        ),
        camlib.TracePath((17, 4), (camlib.TraceLine((20, 4)), camlib.TraceArc((20, 7), -180))),
    )
    values = dict(top_z_mm=6, depth_z_mm=5, stepdown_mm=0.5, clearance_z_mm=8, tool_number=7, length_register=9)
    values.update(overrides)
    return camlib.Trace(
        "engrave",
        tool,
        paths,
        **{
            'feed_mm_min': 600,
            'plunge_mm_min': 150,
            'spindle_rpm': 12000,
            'tool_number': 1,
            'length_register': 1,
            'coolant': "flood",
            **values,
        },
    )


class TraceTest(unittest.TestCase):
    def test_trace_simulates_checks_and_posts_native_arcs(self):
        ex = example()
        tool = ex.end_mill_assembly("mill4", 4)
        setup = camlib.Setup(
            "top",
            stock=camlib.StockSpec((0, 0, 0), (24, 16, 6), 0.5),
            tools=(tool,),
            operations=(camlib.OperationNode(trace(tool, retract_z_mm=7, feed_z_mm=6.5), sources=()),),
            initial_position_mm=(0, 0, 8),
            verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
        )
        job = camlib.Job("trace", "1", (setup,))
        with job.post("top", machine=ex.machine_for(job)) as posted:
            posted.wait()
            player = camlib.Player(posted)
            player.seek(camlib.Cursor(posted.status()["available"]))
            self.assertGreater(player.state.volume()["removed_upper_mm3"], 0)
            nc, start = [], 1
            while page := posted.nc_lines(start, limit=256):
                nc.extend(line["text"] for line in page)
                start = page[-1]["number"] + 1
        text = "\n".join(nc)
        self.assertIn("G2", text)  # the clockwise half circle stays a native arc
        self.assertIn("Z5", text)  # final depth
        self.assertTrue(
            all(
                line.startswith(
                    (
                        "G0",
                        "G1",
                        "G2",
                        "G3",
                        "G17",
                        "G21",
                        "G90",
                        "G91",
                        "G94",
                        "G97",
                        "G40",
                        "G49",
                        "G80",
                        "G92",
                        "G61",
                        "G54",
                        "G43",
                        "T",
                        "M",
                    )
                )
                for line in nc
                if line.strip()
            )
        )

    def test_invalid_trace_is_refused_before_motion(self):
        ex = example()
        tool = ex.end_mill_assembly("mill4", 4)
        config = dict(
            stock=camlib.StockSpec((0, 0, 0), (24, 16, 6), 0.5),
            tools=(tool,),
            operations=(trace(tool, depth_z_mm=6),),
            revision="1",
            initial_position_mm=(0, 0, 8),
        )
        with Simulation(**config) as sim:
            with self.assertRaisesRegex(SimulationError, "invalid trace parameters: depth_z_mm"):
                list(sim.run_operations())
            self.assertEqual(sim.state.completed, 0)


if __name__ == "__main__":
    unittest.main()
