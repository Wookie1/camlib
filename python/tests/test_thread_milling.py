from camlib.postprocessor import GCodeDialect
import re
import unittest

import camlib
from camlib.post import Machine, ToolSetting
from camlib.simulation import Simulation, SimulationError
from camlib.tools import HolderSection, ThreadMill, ToolAssembly


def tools():
    holder = (HolderSection(35, 60, 4, 4),)
    mill = ToolAssembly("mill4", camlib.EndMill("em4", diameter=4, flute_length=30, overall_length=50), 35, 60, holder)
    thread = ToolAssembly(
        "thread8",
        ThreadMill("tm8", diameter=8, tooth_height=1.2, neck_diameter=5, neck_length=10, overall_length=50),
        35,
        60,
        holder,
    )
    return mill, thread


def thread(tool, **overrides):
    values = dict(
        internal=True,
        diameter_mm=14,
        start_diameter_mm=12,
        pitch_mm=1.5,
        top_z_mm=9.5,
        bottom_z_mm=5,
        clearance_z_mm=12,
        radial_passes=2,
        tool_number=2,
        length_register=2,
    )
    values.update(overrides)
    return camlib.ThreadMilling(
        "thread",
        tool,
        ((10, 12),),
        **{
            'feed_mm_min': 300,
            'plunge_mm_min': 300,
            'spindle_rpm': 8000,
            'tool_number': 1,
            'length_register': 1,
            'coolant': "flood",
            **values,
        },
    )


class ThreadMillingTest(unittest.TestCase):
    def test_bored_hole_threads_with_checks_and_posts_helical_g3(self):
        mill, tool = tools()
        bore = camlib.Bore(
            "bore",
            mill,
            ((10, 12),),
            12,
            4,
            2,
            12,
            entry_feed_mm_min=150,
            feed_mm_min=600,
            retract_feed_mm_min=600,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        )
        setup = camlib.Setup(
            "top",
            stock=camlib.StockSpec((0, 0, 0), (40, 24, 10), 0.1),
            tools=(mill, tool),
            operations=tuple(camlib.OperationNode(op, sources=()) for op in (bore, thread(tool))),
            initial_position_mm=(0, 0, 12),
            verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
        )
        job = camlib.Job("thread", "1", (setup,))
        header = job.compile().setup("top").program_header()
        machine = Machine(
            "Three-axis mill",
            tuple(
                ToolSetting.for_tool(t, number=i, length_register=i, position_after_mm=header.initial_position_mm)
                for i, t in enumerate(header.tools, 1)
            ),
            tool_change_position_mode="preserve_tip",
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
            rapid_mm_min=6000,
        )
        with job.post("top", machine=machine) as posted:
            posted.wait()
            nc, start = [], 1
            while page := posted.nc_lines(start, limit=256):
                nc.extend(line["text"] for line in page)
                start = page[-1]["number"] + 1
        text = "\n".join(nc)
        thread_part = text[text.index("T2") :]
        # Counter-clockwise helices one pitch apart, climbing bottom-up.
        helices = [line for line in thread_part.splitlines() if line.startswith("G3") and "Z" in line]
        zs = [float(re.search(r"Z(-?[\d.]+)", line).group(1)) for line in helices]
        self.assertEqual(zs, [6.5, 8, 9.5] * 2)
        self.assertNotIn("G2 ", thread_part)

    def test_undrilled_hole_is_refused_before_the_tool_goes_down(self):
        mill, tool = tools()
        config = dict(
            stock=camlib.StockSpec((0, 0, 0), (40, 24, 10), 0.1),
            tools=(tool,),
            operations=(thread(tool),),
            revision="1",
            initial_position_mm=(0, 0, 12),
        )
        with Simulation(**config) as sim:
            with self.assertRaisesRegex(SimulationError, "not proven open .* drill the hole"):
                list(sim.run_operations())

    def test_dict_and_precise_errors(self):
        mill, tool = tools()
        data = thread(tool).to_dict()
        self.assertEqual(data["kind"], "thread_mill")
        self.assertNotIn("retract_z_mm", data)
        config = dict(
            stock=camlib.StockSpec((0, 0, 0), (40, 24, 10), 0.1),
            tools=(tool,),
            operations=(thread(tool, start_diameter_mm=15),),
            revision="1",
            initial_position_mm=(0, 0, 12),
        )
        with Simulation(**config) as sim:
            with self.assertRaisesRegex(SimulationError, "start_diameter_mm 15 must be smaller than diameter_mm 14"):
                list(sim.run_operations())


if __name__ == "__main__":
    unittest.main()
