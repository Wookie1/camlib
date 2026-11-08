"""Roughing a chamfered plate whose mesh was tessellated face by face (issue #70).

A 100 x 56 x 12 plate with r3 upright corners, a 1 mm top chamfer and a boss,
plus the variants without each feature. The target mesh is made one face at a
time (each face copied first, so neighbouring faces do not share their edge
vertices), as meshers that triangulate faces separately do. Its projection then
has slivers between faces; roughing must still run, keep every layer clear of
the part, and report the slivers it protected.
"""

import unittest

from camlib.material import Options, SurfaceTarget, Verifier
from camlib.operations import Facing, RasterClearing, Roughing
from camlib.simulation import OperationUpdate, Simulation, StockSpec
from camlib.surface import SurfaceMesh
from camlib.tools import EndMill, HolderSection, ToolAssembly

try:
    import cadquery as cq
except ImportError:  # pragma: no cover - CI installs CadQuery
    cq = None

TOOL = ToolAssembly(
    "em12",
    EndMill("em12", diameter=12, flute_length=75, overall_length=120),
    80,
    120,
    (HolderSection(80, 120, 16, 16),),
)
CUT = dict(feed_mm_min=900, plunge_mm_min=250, spindle_rpm=6000, tool_number=1, length_register=1, coolant="flood")
# Plate Z -19..-7, boss to Z-1; stock Z -19..0 (as in the report).
VARIANTS = (
    (True, True, "hexagon"),
    (True, True, "square"),
    (True, False, "hexagon"),
    (False, True, "hexagon"),
    (False, True, "square"),
    (True, True, None),
)


def plate(corners, chamfer, boss):
    solid = cq.Workplane("XY").box(100, 56, 12, centered=(True, True, False))
    if corners:
        solid = solid.edges("|Z").fillet(3)
    if chamfer:
        solid = solid.faces(">Z").edges().chamfer(1)
    if boss == "hexagon":
        solid = solid.union(cq.Workplane("XY").workplane(offset=12).polygon(6, 32).extrude(6))
    elif boss == "square":
        solid = solid.union(cq.Workplane("XY").workplane(offset=12).rect(30, 30).extrude(6))
    return solid.translate((52, 30, -19)).val()


def face_by_face(solid):
    """Each face meshed on its own copy, with alternating deflections."""
    triangles = []
    for index, face in enumerate(solid.Faces()):
        vertices, indices = face.copy().tessellate((0.01, 0.004)[index % 2], 0.2)
        triangles.extend(tuple(vertices[i].toTuple() for i in triangle) for triangle in indices)
    return SurfaceMesh(triangles, 0.01, dict(adapter="face-by-face test mesh"))


@unittest.skipIf(cq is None, "needs CadQuery")
class SeparatelyTessellatedRoughingTest(unittest.TestCase):
    def test_every_variant_roughs_and_keeps_the_part(self):
        for corners, chamfer, boss in VARIANTS:
            with self.subTest(corners=corners, chamfer=chamfer, boss=boss):
                solid = plate(corners, chamfer, boss)
                face = Facing("face", TOOL, floor_z_mm=-1, stepdown_mm=4, stepover_mm=6, clearance_z_mm=10, **CUT)
                rough = Roughing(
                    "rough",
                    TOOL,
                    face_by_face(solid),
                    top_z_mm=0,
                    floor_z_mm=-18.5,
                    stepdown_mm=4,
                    clearance_z_mm=10,
                    clearing=RasterClearing(6, routing="economical"),
                    radial_allowance_mm=0.3,
                    bottom_allowance_mm=0.2,
                    omit_empty_cuts=True,
                    **CUT,
                )
                report = None
                with Simulation(
                    stock=StockSpec((0, 0, -19), (104, 60, 0), 0.5),
                    tools=(TOOL,),
                    operations=(face, rough),
                    revision="1",
                    initial_position_mm=(0, 0, 10),
                ) as sim:
                    for update in sim.run_operations():
                        if (
                            isinstance(update, OperationUpdate)
                            and update.stage == "end"
                            and update.operation == "rough"
                        ):
                            report = update.report
                    self.assertTrue(sim.state.finished)
                    # Independent check against the watertight CAD mesh: no
                    # stock cell is certainly below the part.
                    from camlib.cadquery import export_surface_mesh

                    target = SurfaceTarget("plate", "1", export_surface_mesh(solid), -19)
                    material = Verifier(target, Options(max_findings=1)).check(sim.state)
                self.assertEqual(report["completed_layers"], len(report["layers"]))
                self.assertEqual(material["gouge"]["definite_cells"], 0)
                self.assertTrue(material["complete"])
                if corners and chamfer:
                    # The rounded corners leave slivers between the walls and
                    # the chamfer's cones; each is protected, and tiny.
                    filled = [layer["shadow"]["filled_holes"] for layer in report["layers"]]
                    area = sum(layer["shadow"]["filled_hole_area_mm2"] for layer in report["layers"])
                    self.assertGreater(sum(filled), 0)
                    self.assertLess(area, 1)

    def test_seam_bridge_is_opt_in_and_reported(self):
        solid = plate(True, True, "hexagon")
        common = dict(
            top_z_mm=0,
            floor_z_mm=-18.5,
            stepdown_mm=4,
            clearance_z_mm=10,
            clearing=RasterClearing(6, routing="economical"),
            radial_allowance_mm=0.3,
            bottom_allowance_mm=0.2,
            omit_empty_cuts=True,
            **CUT,
        )
        mesh = face_by_face(solid)
        self.assertNotIn("seam_bridge_mm", Roughing("rough", TOOL, mesh, **common).to_dict())
        rough = Roughing("rough", TOOL, mesh, seam_bridge_mm=0.01, **common)
        self.assertEqual(rough.to_dict()["seam_bridge_mm"], 0.01)
        face = Facing("face", TOOL, floor_z_mm=-1, stepdown_mm=4, stepover_mm=6, clearance_z_mm=10, **CUT)
        report = None
        with Simulation(
            stock=StockSpec((0, 0, -19), (104, 60, 0), 0.5),
            tools=(TOOL,),
            operations=(face, rough),
            revision="1",
            initial_position_mm=(0, 0, 10),
        ) as sim:
            for update in sim.run_operations():
                if isinstance(update, OperationUpdate) and update.stage == "end" and update.operation == "rough":
                    report = update.report
            self.assertTrue(sim.state.finished)
            from camlib.cadquery import export_surface_mesh

            target = SurfaceTarget("plate", "1", export_surface_mesh(solid), -19)
            material = Verifier(target, Options(max_findings=1)).check(sim.state)
        self.assertEqual(material["gouge"]["definite_cells"], 0)
        closings = [layer["shadow"].get("seam_closing") for layer in report["layers"]]
        self.assertTrue(closings and all(c and c["bridged_below_mm"] == 0.02 for c in closings), closings)
        with self.assertRaises(Exception):
            with Simulation(
                stock=StockSpec((0, 0, -19), (104, 60, 0), 0.5),
                tools=(TOOL,),
                operations=(Roughing("rough", TOOL, mesh, seam_bridge_mm=2.0, **common),),
                revision="1",
                initial_position_mm=(0, 0, 10),
            ) as sim:
                for _ in sim.run_operations():
                    pass


if __name__ == "__main__":
    unittest.main()
