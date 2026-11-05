"""Rest-pocket links between separate regions: stay down at depth only when
that is faster than leaving at the feed height (automatic), or up to an
explicit max_stay_down_mm."""

import math
import unittest

import camlib
from camlib.simulation import Update, VerificationUpdate
from test_limits import square


def tools():
    holder = (camlib.HolderSection(35, 60, 8, 8),)
    big = camlib.ToolAssembly(
        "em6", camlib.EndMill("e6", diameter=6, flute_length=20, overall_length=60), 35, 60, holder
    )
    small = camlib.ToolAssembly(
        "em2", camlib.EndMill("e2", diameter=2, flute_length=12, overall_length=50), 35, 60, holder
    )
    return big, small


def run(**rest):
    big, small = tools()
    region = square(4, 4, 36, 36)
    ops = (
        camlib.Pocket(
            "rough",
            big,
            region,
            10,
            6,
            2,
            2.5,
            20,
            tool_number=1,
            length_register=1,
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            coolant="flood",
        ),
        camlib.RestPocket(
            "rest",
            small,
            region,
            10,
            6,
            2,
            0.8,
            20,
            tool_number=2,
            length_register=2,
            **{'feed_mm_min': 600, 'plunge_mm_min': 150, 'spindle_rpm': 12000, 'coolant': "flood", **rest},
        ),
    )
    setup = camlib.Setup(
        "top",
        stock=camlib.StockSpec((0, 0, 0), (40, 40, 10), 0.2),
        tools=(big, small),
        operations=tuple(camlib.OperationNode(o) for o in ops),
        initial_position_mm=(0, 0, 20),
        verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
    )
    moves, statuses = [], set()
    with camlib.Job("rest", "1", (setup,)).regenerate() as r:
        for u in r:
            if isinstance(u.update, Update) and u.update.motion["operation"] == "rest":
                moves.append(u.update.motion)
            elif isinstance(u.update, VerificationUpdate):
                statuses.add(u.update.status)
    long_links = [
        m for m in moves if m["kind"] == "link" and math.dist(m["curve"]["start"][:2], m["curve"]["end"][:2]) > 4 * 0.8
    ]
    return moves, long_links, statuses


class StayDownTest(unittest.TestCase):
    def test_stay_down_is_adjustable(self):
        _, automatic, s1 = run()
        _, never, s2 = run(max_stay_down_mm=0)
        _, low_feed, s3 = run(feed_z_mm=10.5)
        self.assertEqual(s1 | s2 | s3, {"clear"})
        self.assertEqual(never, [])
        # A feed height just above the stock makes leaving cheap: fewer
        # long links than when the full clearance had to be crossed.
        self.assertLess(len(low_feed), len(automatic))


if __name__ == "__main__":
    unittest.main()
