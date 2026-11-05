"""Cutter compensation (wear): the tool-center path posted with G41/G42 on
over the lead-in and G40 before the lead-out, decoded and checked."""

from camlib.postprocessor import GCodeDialect

import unittest

import camlib
from camlib.planar import Boundary, Component, PlanarRegion
from camlib.post import Machine, ToolSetting
from test_tapping import posted_text


def job(**contour):
    holder = (camlib.HolderSection(35, 60, 8, 8),)
    tool = camlib.ToolAssembly(
        "em6", camlib.EndMill("e6", diameter=6, flute_length=20, overall_length=60), 35, 60, holder
    )
    part = PlanarRegion(
        "part",
        (
            Component(
                Boundary.polygon(((10, 10), (30, 10), (30, 30), (10, 30))),
                (Boundary.polygon(((15, 15), (15, 25), (25, 25), (25, 15))),),
            ),
        ),
        1e-4,
    )
    values = dict(boundaries="outer", lead_in_mm=3, lead_out_mm=3, compensation="wear")
    values.update(contour)
    op = camlib.Contour(
        "walls",
        tool,
        part,
        values.pop("floor_z_mm", 8),
        2,
        14,
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
    setup = camlib.Setup(
        "top",
        stock=camlib.StockSpec((0, 0, 0), (40, 40, 10), 0.2),
        tools=(tool,),
        operations=(camlib.OperationNode(op, sources=()),),
        initial_position_mm=(0, 0, 14),
    )
    return camlib.Job("comp", "1", (setup,))


def machine(j, register=7):
    header = j.compile().setup("top").program_header()
    return Machine(
        "mill",
        tuple(
            ToolSetting.for_tool(
                t, number=1, length_register=1, position_after_mm=header.initial_position_mm, diameter_register=register
            )
            for t in header.tools
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


class CutterCompensationTest(unittest.TestCase):
    def sides(self, nc):
        return [line for line in nc if line.split(" ")[0] in ("G41", "G42", "G40")][1:]  # [0]: header G40

    def test_wear_compensation_on_over_lead_in_off_before_lead_out(self):
        j = job()
        nc = posted_text(j, machine(j))
        marks = self.sides(nc)
        # The outer wall: turned on and off once per depth pass.
        self.assertEqual(marks, ["G41 D7", "G40"], marks)  # climb, clockwise spindle: left
        for i, line in enumerate(nc):
            if line.startswith(("G41", "G40")) and i > nc.index("M3 S12000"):
                # The next move is a straight XY move at the cutting depth.
                self.assertTrue(
                    nc[i + 1].startswith("G1 ") and nc[i + 1].split()[3] == nc[i - 1].split()[3], nc[i - 1 : i + 2]
                )

    def test_conventional_milling_is_right(self):
        j = job(cutting_sense="conventional")
        self.assertEqual(self.sides(posted_text(j, machine(j))), ["G42 D7", "G40"])

    def test_refusals(self):
        j = job()
        with self.assertRaisesRegex(Exception, "D register"):
            posted_text(j, machine(j, register=None))
        with self.assertRaisesRegex(Exception, "lead_in_mm and lead_out_mm"):
            posted_text(job(lead_in_mm=0), machine(j))

    def test_centroid_writes_it_and_haas_refuses(self):
        from test_drill_cycles import example
        from camlib.motion import WorkFrame

        j = job()
        tools = machine(j).tools
        centroid = example("centroid_post")
        m = centroid.define_machine(
            "Acorn",
            tools,
            traverse_feed_mm_min=3000,
            work_to_machine=WorkFrame(),
            decimals=3,
            output_units='mm',
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
        # CNC12 documents no arc radius tolerance, so arcs whose 3-decimal words
        # cannot be exactly consistent are posted as checked segments.
        nc = posted_text(
            j, m, centroid.CentroidPost, centroid.CentroidDecoder, lowering=camlib.LoweringOptions(native_arcs=True)
        )
        self.assertEqual(self.sides(nc), ["G41 D7", "G40"])
        haas = example("haas_ngc_post")
        h = haas.define_machine(
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
        with self.assertRaisesRegex(Exception, "does not declare radius_left"):
            posted_text(j, h, haas.HaasNGCPost, haas.HaasNGCDecoder)

    def test_fanuc_and_haas_turn_it_on_and_off_in_the_move_block(self):
        from test_drill_cycles import example
        from camlib.motion import WorkFrame

        j = job()
        tools = machine(j).tools
        haas = example("haas_ngc_post")
        h = haas.define_machine(
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
            cutter_compensation=True,
            output_units='mm',
            max_feed_mm_min=10000,
            max_spindle_rpm=30000,
            max_sweep_rad=6.283185307179586,
            tool_changes=True,
            flood=True,
            rapid_mm_min=6000,
        )
        fanuc = example("fanuc_0imb_post")
        oem = dict(tool_change='M6', spindle_cw='M3', spindle_ccw='M4', spindle_off='M5', coolant_off='M9', flood='M8')
        f = fanuc.define_machine(
            "FANUC",
            tools,
            startup_units="mm",
            startup_standard_modes=True,
            g92_shift_mm=(0, 0, 0),
            m_codes=oem,
            traverse_feed_mm_min=3000,
            work_to_machine=WorkFrame(),
            decimals=3,
            tool_change_position_mode="table",
            output_units='mm',
            max_feed_mm_min=10000,
            max_spindle_rpm=30000,
            max_sweep_rad=6.283185307179586,
            tool_changes=True,
            flood=True,
            rapid_mm_min=6000,
        )
        for m, post, decoder in (
            (h, haas.HaasNGCPost, haas.HaasNGCDecoder),
            (f, fanuc.Fanuc0iMBPost, fanuc.Fanuc0iMBDecoder),
        ):
            nc = posted_text(j, m, post, decoder, lowering=camlib.LoweringOptions())
            on = [line for line in nc if " G41 " in line]
            off = [line for line in nc if " G40 " in line]
            self.assertEqual((len(on), len(off)), (1, 1), nc)  # one depth pass
            for line in on + off:
                # G1, the change, then an XY move in the same block; never alone.
                words = line.split(" ")
                self.assertEqual(words[0], "G1", line)
                self.assertTrue(any(w[0] in "XY" for w in words[2:]), line)
            self.assertTrue(all(" D7 " in line for line in on), on)
            self.assertFalse(any(line in ("G41 D7", "G40") for line in nc[nc.index(on[0]) - 1 :]))
        # FANUC offset memory A shares H and D numbers.
        from dataclasses import replace

        same = tuple(replace(t, diameter_register=t.length_register) for t in tools)
        with self.assertRaisesRegex(ValueError, "must differ from every H"):
            fanuc.define_machine(
                "FANUC",
                same,
                startup_units="mm",
                startup_standard_modes=True,
                g92_shift_mm=(0, 0, 0),
                m_codes=oem,
                traverse_feed_mm_min=3000,
                work_to_machine=WorkFrame(),
                decimals=3,
                tool_change_position_mode="table",
                output_units='mm',
                max_feed_mm_min=10000,
                max_spindle_rpm=30000,
                max_sweep_rad=6.283185307179586,
                tool_changes=True,
                flood=True,
                rapid_mm_min=6000,
            )


class TabsTest(unittest.TestCase):
    def test_triangle_tabs_with_wear_compensation_post_and_check_clear(self):
        from camlib.simulation import VerificationUpdate
        from dataclasses import replace

        j = job(floor_z_mm=0, tabs=camlib.Tabs(width_mm=3, height_mm=2, count=4))
        setup = replace(j.setups[0], verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()))
        j = camlib.Job("comp", "1", (setup,))
        nc = posted_text(j, machine(j))
        # Ramps up to Z2 and back down, four tabs on each of the lower passes.
        self.assertTrue(any(line.startswith(("G1 ", "G2 ", "G3 ")) and " Z2 " in line for line in nc))
        statuses = set()
        with j.regenerate() as run:
            for u in run:
                if isinstance(u.update, VerificationUpdate):
                    statuses.add(u.update.status)
        self.assertEqual(statuses, {"clear"})

    def test_rectangle_tabs_refuse_compensation(self):
        j = job(floor_z_mm=0, tabs=camlib.Tabs(width_mm=3, height_mm=2, count=4, shape="rectangle"))
        with self.assertRaisesRegex(Exception, "triangular"):
            posted_text(j, machine(j))


if __name__ == "__main__":
    unittest.main()
