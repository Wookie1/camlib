"""Per-arc output policy at standard controller precision (3 mm decimals).

Regression for a Ø6 pocket around an r=4 island: on a 0.001 mm grid no
exactly radius-consistent G2/G3 words exist for some of its arcs, so the
default post rejects the job. LoweringOptions(native_arcs=True) keeps every
exactly representable arc and lowers only the others to certified lines, and
the posted run reports how many arcs took each form and why.
"""

from camlib.postprocessor import GCodeDialect

from decimal import Decimal
from pathlib import Path
import tempfile
import re
import unittest

from camlib.jobs import Job, Setup
from camlib.operations import Pocket
from camlib.playback import Cursor, Player, RecordedRun
from camlib.planar import Boundary, Component, PlanarRegion
from camlib.post import LoweringOptions, Machine, ToolSetting
from camlib.simulation import SimulationError, StockSpec
from camlib.tools import EndMill, HolderSection, ToolAssembly
from camlib.verification import StockContactOptions, StreamChecks


def island_job():
    mill = ToolAssembly(
        "m6", EndMill("em6", diameter=6, flute_length=20, overall_length=60), 30, 70, (HolderSection(30, 70, 10, 10),)
    )
    region = PlanarRegion(
        "pocket", (Component(Boundary.polygon(((5, 5), (35, 5), (35, 25), (5, 25))), (Boundary.circle((20, 15), 4),)),)
    )
    pocket = Pocket(
        "pocket",
        mill,
        region,
        top_z_mm=10,
        floor_z_mm=6,
        stepdown_mm=1.5,
        stepover_mm=2,
        clearance_z_mm=15,
        feed_mm_min=600,
        plunge_mm_min=150,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant="flood",
    )
    setup = Setup(
        "top",
        StockSpec((0, 0, 0), (40, 30, 10), 0.5),
        (mill,),
        (pocket,),
        (0, 0, 15),
        verification=StreamChecks(stock_contact=StockContactOptions()),
    )
    job = Job("island", "1", (setup,))
    header = job.compile().setup("top").program_header()
    machine = Machine(
        "Three-axis mill",
        (
            ToolSetting.for_tool(
                header.tools[0], number=1, length_register=1, position_after_mm=header.initial_position_mm
            ),
        ),
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
    return job, machine


def nc(posted):
    lines, start = [], 1
    while page := posted.nc_lines(start, limit=256):
        lines.extend(line["text"] for line in page)
        start = page[-1]["number"] + 1
    return lines


class ArcOutputPolicyTest(unittest.TestCase):
    def test_default_precision_rejects_and_native_arcs_policy_is_exact_and_reported(self):
        job, machine = island_job()
        self.assertEqual(machine.decimals, 3)
        with self.assertRaisesRegex(SimulationError, "native_arcs=True"):
            with job.post("top", machine=machine) as posted:
                posted.wait()
        with job.post("top", machine=machine, lowering=LoweringOptions(native_arcs=True)) as posted:
            posted.wait()
            lines = nc(posted)
            player = Player(posted)
            player.seek(Cursor(posted.status()["available"]))
            self.assertEqual(player.verification["progress"]["violation"], 0)
            self.assertEqual(player.verification["progress"]["uncertain"], 0)
            # Reported, not silent: every one of the 12 source arcs is counted.
            self.assertEqual(
                posted.status()["posting"]["arc_output"],
                dict(native=0, lines_policy=0, lines_machine=0, lines_precision=12),
            )
            with tempfile.TemporaryDirectory() as directory:
                posted.save(Path(directory) / "saved")
                with RecordedRun.open(Path(directory) / "saved") as saved:
                    self.assertEqual(saved.status()["posting"]["arc_output"]["lines_precision"], 12)
        # At 3 decimals none of this island's arcs has exact words: all lowered.
        self.assertFalse([line for line in lines if line.startswith(("G2 ", "G3 "))])
        # Where exact words exist (4 decimals here), every source arc stays native.
        from dataclasses import replace

        with job.post(
            "top", machine=replace(machine, decimals=4), lowering=LoweringOptions(native_arcs=True)
        ) as posted:
            posted.wait()
            precise = nc(posted)
            self.assertEqual(
                posted.status()["posting"]["arc_output"],
                dict(native=12, lines_policy=0, lines_machine=0, lines_precision=0),
            )
        self.assertEqual(sum(line.startswith(("G2 ", "G3 ")) for line in precise), 12)
        self.check_exact_circles(precise)

    def check_exact_circles(self, lines):
        # Every emitted circle's start/end radii, recomputed from its decimal
        # words independently of the engine, agree within 5e-6 mm.
        position = None
        for line in lines:
            words = dict(re.findall(r"([XYZIJ])(-?[0-9.]+)", line))
            if line.startswith(("G0 ", "G1 ", "G2 ", "G3 ")) and position is None:
                position = [Decimal(words.get(k, "0")) for k in "XYZ"]
                continue
            if position is None:
                continue
            target = [Decimal(words[k]) if k in words else position[i] for i, k in enumerate("XYZ")]
            if line.startswith(("G2 ", "G3 ")):
                cx, cy = position[0] + Decimal(words["I"]), position[1] + Decimal(words["J"])
                r0 = ((position[0] - cx) ** 2 + (position[1] - cy) ** 2).sqrt()
                r1 = ((target[0] - cx) ** 2 + (target[1] - cy) ** 2).sqrt()
                # Engine consistency tolerance for emitted circles (5e-6 mm).
                self.assertLessEqual(abs(r0 - r1), Decimal("0.000005"), line)
            if line.startswith(("G0 ", "G1 ", "G2 ", "G3 ")):
                position = target


if __name__ == "__main__":
    unittest.main()
