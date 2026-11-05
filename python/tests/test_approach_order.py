"""Approach order: a rapid never lowers Z while it moves in XY.

A machine may leave a new tool high (Z home): each operation must travel at
that height and only then descend, never cut diagonally down toward the part.
"""

import importlib.util
import unittest
from dataclasses import replace
from pathlib import Path

from camlib.simulation import ToolChangePosition, Update

ROOT = Path(__file__).resolve().parents[2]


def example():
    spec = importlib.util.spec_from_file_location("standalone_approach", ROOT / "examples/standalone_job.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def motions(job):
    with job.regenerate() as run:
        return [e.update.motion for e in run if isinstance(e.update, Update)]


class ApproachOrderTest(unittest.TestCase):
    def check(self, job, high_z=None):
        planned = motions(job)
        self.assertTrue(planned)
        operation = None
        for motion in planned:
            curve = motion["curve"]
            start, end = curve["start"], curve["end"]
            if motion["kind"] == "rapid":
                moves_xy = (start[0], start[1]) != (end[0], end[1])
                self.assertFalse(moves_xy and end[2] < start[2], f"diagonal descent: {motion}")
            if motion["operation"] != operation:
                operation = motion["operation"]
                # The first move of an operation leaves the tool-change pose.
                if high_z is not None and motion["tool"] != planned[0]["tool"]:
                    self.assertEqual(start[2], high_z, motion)

    def test_default_and_z_home_tool_change(self):
        module = example()
        job = module.build_rest_job()
        self.check(job)
        # The rest tool arrives with its gauge line at Z100 (tip at Z40).
        setup = replace(job.setups[0], tool_change=ToolChangePosition(gauge_z_mm=100))
        self.check(replace(job, setups=(setup,)), high_z=100 - 60)


if __name__ == "__main__":
    unittest.main()
