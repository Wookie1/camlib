from camlib.postprocessor import GCodeDialect
import importlib.util
from pathlib import Path
import unittest

import camlib
from camlib.post import Machine, ToolSetting
from camlib.simulation import Simulation, SimulationError
from camlib.tools import HolderSection, Tap, ToolAssembly

ROOT = Path(__file__).resolve().parents[2]
HOLES = ((8, 8), (20, 8))


def tools():
    holder = (HolderSection(35, 60, 6, 6),)
    drill = ToolAssembly("drill5", camlib.Drill("d5", diameter=5, flute_length=30, overall_length=60), 35, 60, holder)
    tap = ToolAssembly(
        "m6", Tap("m6-tap", drill_diameter=5, thread_length=12, overall_length=60, shank_diameter=6), 35, 60, holder
    )
    return drill, tap


def job(tap_op, drill_depth=0.5):
    drill, tap = tools()
    ops = (
        camlib.Drilling(
            "drill",
            drill,
            HOLES,
            drill_depth,
            12,
            tool_number=1,
            length_register=1,
            feed_mm_min=150,
            retract_feed_mm_min=600,
            spindle_rpm=6000,
            coolant="flood",
        ),
        tap_op(tap),
    )
    setup = camlib.Setup(
        "top",
        stock=camlib.StockSpec((0, 0, 0), (28, 16, 10), 0.1),
        tools=(drill, tap),
        operations=tuple(camlib.OperationNode(op, sources=()) for op in ops),
        initial_position_mm=(0, 0, 12),
        verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
    )
    return camlib.Job("tap", "1", (setup,))


def tapping(tap, **overrides):
    values = dict(
        pitch_mm=1, depth_z_mm=4, clearance_z_mm=12, spindle_rpm=600, feed_z_mm=11, tool_number=2, length_register=2
    )
    values.update(overrides)
    return camlib.Tapping(
        "tap", tap, HOLES, **{'spindle_rpm': 500, 'tool_number': 1, 'length_register': 1, 'coolant': "flood", **values}
    )


def machine(job, **settings):
    header = job.compile().setup("top").program_header()
    return Machine(
        "Three-axis mill",
        tuple(
            ToolSetting.for_tool(t, number=i, length_register=i, position_after_mm=header.initial_position_mm)
            for i, t in enumerate(header.tools, 1)
        ),
        tool_change_position_mode="preserve_tip",
        **{
            'dialect': GCodeDialect.linuxcnc(),
            'output_units': 'mm',
            'decimals': 3,
            'max_feed_mm_min': 10000,
            'max_spindle_rpm': 30000,
            'planes': ('xy', 'xz', 'yz'),
            'arcs': True,
            'helices': True,
            'max_sweep_rad': 6.283185307179586,
            'max_tool_number': 99999,
            'max_length_register': 99999,
            'tool_changes': True,
            'flood': True,
            'mist': True,
            'rapid_mm_min': 6000,
            **settings,
        },
    )


def posted_text(job, machine, post=None, decoder=None, **options):
    kwargs = dict(options) if post is None else dict(options, post=post, decoder=decoder)
    with job.post("top", machine=machine, **kwargs) as posted:
        posted.wait()
        nc, start = [], 1
        while page := posted.nc_lines(start, limit=256):
            nc.extend(line["text"] for line in page)
            start = page[-1]["number"] + 1
    return nc


class TappingTest(unittest.TestCase):
    def test_linuxcnc_g33_1_per_hole_with_checks(self):
        j = job(tapping)
        nc = posted_text(j, machine(j))
        cycles = [line for line in nc if line.startswith("G33.1")]
        self.assertEqual(cycles, ["G33.1 Z4 K1"] * 2)
        # Each cycle starts at the R plane above its hole.
        for i, line in enumerate(nc):
            if line.startswith("G33.1"):
                self.assertTrue(any("Z11" in nc[k] for k in range(i - 3, i)), nc[i - 3 : i + 1])

    def test_g84_cycle_is_cancelled_after_each_hole(self):
        spec = importlib.util.spec_from_file_location("haas", ROOT / "examples/haas_ngc_post.py")
        haas = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(haas)
        j = job(tapping)
        header = j.compile().setup("top").program_header()
        tools = tuple(
            ToolSetting.for_tool(t, number=i, length_register=i, position_after_mm=header.initial_position_mm)
            for i, t in enumerate(header.tools, 1)
        )
        from camlib.motion import WorkFrame

        m = haas.define_machine(
            "Haas",
            tools,
            traverse_feed_mm_min=5000,
            coordinate_system="fanuc",
            h_t_agreement=True,
            g92_shift_mm=(0, 0, 0),
            rotation_scaling=False,
            work_to_machine=WorkFrame(),
            decimals=3,
            tool_change_position_mode="preserve_tip",
            rigid_tapping=True,
            output_units='mm',
            max_feed_mm_min=10000,
            max_spindle_rpm=30000,
            max_sweep_rad=6.283185307179586,
            tool_changes=True,
            flood=True,
            rapid_mm_min=6000,
        )
        nc = posted_text(j, m, haas.HaasNGCPost, haas.HaasNGCDecoder)
        cycles = [i for i, line in enumerate(nc) if line.startswith("G84")]
        self.assertEqual([nc[i] for i in cycles], ["G84 X8. Y8. Z4. R11. F600.", "G84 X20. Y8. Z4. R11. F600."])
        for i in cycles:
            self.assertEqual(nc[i + 1], "G80")
        without = haas.define_machine(
            "Haas",
            tools,
            traverse_feed_mm_min=5000,
            coordinate_system="fanuc",
            h_t_agreement=True,
            g92_shift_mm=(0, 0, 0),
            rotation_scaling=False,
            work_to_machine=WorkFrame(),
            decimals=3,
            tool_change_position_mode="preserve_tip",
            output_units='mm',
            max_feed_mm_min=10000,
            max_spindle_rpm=30000,
            max_sweep_rad=6.283185307179586,
            tool_changes=True,
            flood=True,
            rapid_mm_min=6000,
        )
        with self.assertRaisesRegex(Exception, "declares no tapping cycle"):
            posted_text(j, without, haas.HaasNGCPost, haas.HaasNGCDecoder)

    def test_shallow_drill_is_refused_before_the_tap_goes_down(self):
        drill, tap = tools()
        config = dict(
            stock=camlib.StockSpec((0, 0, 0), (28, 16, 10), 0.1),
            tools=(drill, tap),
            operations=(
                camlib.Drilling(
                    "drill",
                    drill,
                    HOLES,
                    5,
                    12,
                    tool_number=1,
                    length_register=1,
                    feed_mm_min=150,
                    retract_feed_mm_min=600,
                    spindle_rpm=6000,
                    coolant="flood",
                ),
                tapping(tap),
            ),
            revision="1",
            initial_position_mm=(0, 0, 12),
        )
        with Simulation(**config) as sim:
            with self.assertRaisesRegex(SimulationError, "tapping hole 0: the stock is not proven open"):
                list(sim.run_operations())


class ReamingTest(unittest.TestCase):
    def test_reams_drilled_holes_feed_in_feed_out_and_refuses_undrilled(self):
        holder = (HolderSection(35, 60, 6, 6),)
        drill = ToolAssembly(
            "drill5", camlib.Drill("d5", diameter=5, flute_length=30, overall_length=60), 35, 60, holder
        )
        reamer = ToolAssembly(
            "ream52",
            camlib.EndMill("r52", diameter=5.2, flute_length=20, overall_length=60, center_cutting=False),
            35,
            60,
            holder,
        )
        ream = camlib.Reaming(
            "ream",
            reamer,
            HOLES,
            5,
            4,
            12,
            feed_z_mm=11,
            tool_number=2,
            length_register=2,
            feed_mm_min=100,
            retract_feed_mm_min=100,
            spindle_rpm=1000,
            coolant="flood",
        )
        stock = camlib.StockSpec((0, 0, 0), (28, 16, 10), 0.1)
        drilling = camlib.Drilling(
            "drill",
            drill,
            HOLES,
            0.5,
            12,
            tool_number=1,
            length_register=1,
            feed_mm_min=150,
            retract_feed_mm_min=600,
            spindle_rpm=6000,
            coolant="flood",
        )
        moves = []
        with Simulation(
            stock=stock,
            tools=(drill, reamer),
            operations=(drilling, ream),
            revision="1",
            initial_position_mm=(0, 0, 12),
            verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
        ) as sim:
            for update in sim.run_operations():
                motion = getattr(update, "motion", None)
                if motion is not None and motion["operation"] == "ream" and motion["kind"] != "rapid":
                    moves.append((motion["kind"], motion["curve"]["start"][2], motion["curve"]["end"][2]))
        self.assertEqual(moves, [("plunge", 11, 4), ("retract", 4, 11)] * 2)
        with Simulation(
            stock=stock, tools=(reamer,), operations=(ream,), revision="1", initial_position_mm=(0, 0, 12)
        ) as sim:
            with self.assertRaisesRegex(SimulationError, "reaming hole 0: the stock is not proven open"):
                list(sim.run_operations())


if __name__ == "__main__":
    unittest.main()
