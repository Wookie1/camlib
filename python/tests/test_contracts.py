import json
import math
import unittest

from camlib import Engine
from camlib.tools import (
    EndMill,
    BallEndMill,
    BullEndMill,
    Drill,
    ChamferMill,
    ToolAssembly,
    HolderSection,
)
from camlib.motion import Program, Line, Arc, Helix, Motion, ToolChange, Spindle, WorkOffset, Coolant


def assembly(cutter):
    return ToolAssembly("t1", cutter, 22, 70, (HolderSection(22, 70, 10, 20),))


class ContractsTest(unittest.TestCase):
    def test_python_tool_families_are_prepared_by_native_geometry(self):
        dims = dict(diameter=6, flute_length=18, overall_length=50)
        tools = [
            EndMill("flat", **dims),
            BallEndMill("ball", **dims),
            BullEndMill("bull", corner_radius=1, **dims),
            Drill("drill", point_angle=90, **dims),
            ChamferMill("chamfer", tip_diameter=1, angle=90, **dims),
        ]
        with Engine() as engine:
            for cutter, tip_radius in zip(tools, [3, 0, 2, 0, 0.5]):
                t = assembly(cutter)
                response = engine.inspect_tool(t, [0, 18, 21, 22, 70, 71])
                self.assertEqual(response["tool"], json.loads(json.dumps(t.to_dict())))
                self.assertAlmostEqual(response["profile"][0]["radius_mm"], tip_radius)
                self.assertTrue(response["profile"][1]["cutting"])
                self.assertFalse(response["profile"][2]["cutting"])
                self.assertEqual(response["profile"][3]["radius_mm"], 10)
                self.assertIsNone(response["profile"][-1])
            invalid = assembly(EndMill("bad", diameter=-1, flute_length=18, overall_length=50))
            with self.assertRaisesRegex(ValueError, "diameter_mm"):
                engine.inspect_tool(invalid, [0])

    def test_line_arc_helix_program_native_round_trip(self):
        tool = assembly(EndMill("em6", diameter=6, flute_length=18, overall_length=50))
        p = Program(
            "job",
            "revision-1",
            (tool,),
            (1, 0, 5),
            (
                WorkOffset(1),
                ToolChange("t1", 1, 1, (1, 0, 5)),
                Motion(0, "pocket", "t1", Line((1, 0, 5), (1, 0, 0)), kind="rapid"),
                Spindle(12000),
                Coolant("flood"),
                Motion(1, "pocket", "t1", Arc((1, 0, 0), (0, 1, 0), (0, 0, 0), math.pi / 2), 1200),
                Motion(2, "pocket", "t1", Helix((0, 1, 0), (0, 1, -2), (0, 0, 0), math.tau), 300, "plunge"),
                Spindle(0),
                Coolant(),
            ),
        )
        with Engine() as engine:
            response = engine.validate_program(p)
            self.assertEqual(response["program"], json.loads(json.dumps(p.to_dict())))
            self.assertEqual(response["summary"]["motions"], 3)
            self.assertAlmostEqual(response["summary"]["fed_length_mm"], math.pi / 2 + math.hypot(math.tau, 2))
            self.assertEqual(response["summary"]["final_position_mm"], [0, 1, -2])
            constrained = dict(
                id="linear_only",
                planes=[],
                arcs=False,
                helices=False,
                max_sweep_rad=math.tau,
                max_feed_mm_min=5000,
                max_spindle_rpm=24000,
                max_tool_number=99,
                max_length_register=99,
                max_work_offset=9,
                tool_changes=True,
                flood=True,
                mist=False,
            )
            with self.assertRaisesRegex(ValueError, "circular motion unsupported"):
                engine.validate_program(p, constrained)


if __name__ == "__main__":
    unittest.main()
