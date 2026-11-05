"""Operation-scoped material through ordinary Python, streaming and posted replay."""

from array import array
from dataclasses import replace
import os
import unittest
from camlib.material import FinishingScope, SurfaceTarget, Verifier, Options
from camlib.simulation import Simulation, VerificationUpdate, StockSpec
from camlib.playback import Player, Cursor
from camlib.operations import Waterline
from camlib.tools import EndMill
from test_cad_surfaces import example


class FinishingMaterialTest(unittest.TestCase):
    def test_scope_binds_real_operation_tool_depth_and_identity(self):
        job = example().build_job()
        op = job.setups[0].operations[0]
        scope = FinishingScope.from_operation(op)
        self.assertEqual(scope.tool_radius_mm, 1.5)
        self.assertEqual(scope.floor_z_mm, 6)
        self.assertEqual(FinishingScope.from_operation(replace(op, stock_to_leave_mm=0.2)).floor_z_mm, 6.2)
        self.assertEqual(scope, FinishingScope.from_operation(replace(op, tool=op.tool.id), tool=op.tool))
        with self.assertRaisesRegex(ValueError, 'matching tool'):
            FinishingScope.from_operation(replace(op, tool='unresolved'))
        with self.assertRaisesRegex(ValueError, 'ball cutters'):
            flat = replace(op.tool, cutter=EndMill('flat', diameter=3, flute_length=20, overall_length=40))
            FinishingScope.from_operation(replace(op, tool=flat))
        waterline = Waterline(
            'levels',
            op.tool,
            op.surface,
            0,
            8,
            5,
            0.5,
            11,
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        )
        self.assertEqual(FinishingScope.from_operation(waterline).floor_z_mm, 5)
        target = SurfaceTarget('target', '1', op.surface, 0, finishing=scope)
        v = Verifier(target)
        old = v.digest
        v.prepare(replace(target, finishing=replace(scope, stock_to_leave_mm=0.2)))
        self.assertNotEqual(v.digest, old)
        with self.assertRaises(ValueError):
            v.prepare(replace(target, finishing=replace(scope, tool_radius_mm=0)))

    def test_native_stage_matches_borrowed_stock_and_preserves_raw_gouge(self):
        job = example().build_job()
        setup = job.setups[0]
        target = setup.verification.material[0].target
        material = []
        v = Verifier(target)
        with Simulation(
            setup.stock,
            setup.tools,
            revision='access-1',
            initial_position_mm=setup.initial_position_mm,
            operations=setup.operations,
            verification=setup.verification,
            executable=os.environ.get('CAMLIB_TEST_ENGINE'),
        ) as sim:
            before = v.check(sim.state)
            for u in sim.run_operations():
                if isinstance(u, VerificationUpdate) and u.check['kind'] == 'material':
                    material.append(u.check)
            after = v.check(sim.state)
            self.assertEqual(material[-1]['after_operation'], 'finish')
            self.assertEqual(material[-1]['report'], after)
            self.assertTrue(after['complete'])
            self.assertTrue(after['finishing']['complete'])
            self.assertFalse(after['finishing']['inner_contact_available'])
            self.assertEqual(after['finishing']['within_allowance']['volume_mm3'], [0.0, 0.0])
            uncertainty = after['finishing']['uncertainty']
            self.assertGreater(uncertainty['finding_cells'], 0)
            self.assertEqual(uncertainty['missing_inner_contact_cells'], uncertainty['finding_cells'])
            self.assertTrue(all(f['uncertainty']['missing_inner_contact'] for f in after['finishing']['findings']))
            self.assertEqual(after['finishing']['tool_limited_residual']['definite_cells'], 0)
            self.assertGreater(after['finishing']['within_depth_exclusion']['volume_mm3'][1], 0)
            self.assertGreater(
                before['finishing']['reachable_residual']['volume_mm3'][0],
                after['finishing']['reachable_residual']['volume_mm3'][0],
            )
            raw = Verifier(replace(target, finishing=None)).check(sim.state)
            for field in ('residual', 'gouge', 'stock_sha256', 'context'):
                self.assertEqual(after[field], raw[field])
            self.assertEqual(after['gouge']['definite_cells'], 0)

    def test_posted_stage_and_fractional_overlay_keep_scoped_reports_at_cursor(self):
        job = example().build_job()
        target = job.setups[0].verification.material[0].target
        v = Verifier(target)
        with job.post(
            'top', machine=example().machine_for(job), executable=os.environ.get('CAMLIB_TEST_ENGINE')
        ) as run:
            run.wait()
            last = run.status()['available']
            compact = Player(run, compact=True)
            dense = Player(run)
            compact.seek(Cursor(last))
            dense.seek(Cursor(last))
            checked = v.check(compact.state)
            records = compact.verification['records']
            full = list(run.verification_records(records - 1, limit=1))[0].check['report']
            self.assertEqual(full, checked)
            self.assertEqual(
                compact.verification['material']['finishing'],
                {k: v for k, v in checked['finishing'].items() if k != 'findings'},
            )
            self.assertTrue(checked['finishing']['complete'])
            for cursor in (Cursor(last // 2, 0.37), Cursor(last), Cursor(last // 2, 0.37)):
                compact.seek(cursor)
                dense.seek(cursor)
                a = v.check(compact.state)
                b = v.check(dense.state)
                self.assertEqual(a, b)
                self.assertEqual(a['context']['fraction'], cursor.fraction)
                self.assertEqual(a['finishing']['scope']['floor_z_mm'], 6)
            self.assertEqual(compact.verification['progress']['violation'], 0)

    def test_budget_exhaustion_is_incomplete_without_losing_unknown_volume(self):
        from camlib.surface import SurfaceMesh

        surface = SurfaceMesh((((-2, -2, 2), (4, -2, 2), (4, 4, 2)), ((-2, -2, 2), (4, 4, 2), (-2, 4, 2))))
        target = SurfaceTarget('budget', '1', surface, 0, finishing=FinishingScope(3, 1, max_access_work=1))
        report = Verifier(target).check_bounds(array('d', [6.0]) * 8, StockSpec((0, 0, 0), (2, 2, 6), 1))
        self.assertFalse(report['complete'])
        self.assertFalse(report['finishing']['complete'])
        self.assertEqual(report['finishing']['unexamined_cells'], 4)
        self.assertEqual(report['finishing']['within_allowance']['volume_mm3'], [0.0, 0.0])
        self.assertEqual(report['finishing']['within_allowance']['assessment'], 'within_tolerance')
        self.assertEqual(report['finishing']['uncertainty']['finding_cells'], 0)
        self.assertGreaterEqual(report['finishing']['reachable_residual']['volume_mm3'][1], 24)

        corrupt = array('d', [6.0]) * 8
        corrupt[-1] = float('nan')
        with self.assertRaisesRegex(ValueError, 'height bounds'):
            Verifier(target).check_bounds(corrupt, StockSpec((0, 0, 0), (2, 2, 6), 1))

    def test_real_cad_allowance_below_analysis_stock_and_truncated_diagnostics(self):
        import cadquery as cq
        from camlib.cadquery import export_surface_mesh

        shape = cq.Workplane('XY').box(4, 4, 2, centered=(True, True, False)).val()
        surface = export_surface_mesh(shape, max_error_mm=0.005)
        target = SurfaceTarget('slab', '1', surface, 0, finishing=FinishingScope(5, 1, stock_to_leave_mm=0.5))
        report = Verifier(target, Options(max_findings=1)).check_bounds(
            array('d', [6.0]) * 8, StockSpec((-0.1, -0.1, 5), (0.1, 0.1, 6), 0.1)
        )
        f = report['finishing']
        self.assertTrue(report['complete'])
        self.assertTrue(f['complete'])
        # This CAD box ends at Z2. Its .5mm normal offset cannot enter Z[5,6].
        self.assertLess(f['within_allowance']['volume_mm3'][1], 1e-8)
        self.assertLessEqual(f['in_scope_residual']['volume_mm3'][0], 0.04)
        self.assertGreaterEqual(f['in_scope_residual']['volume_mm3'][1], 0.04)
        self.assertGreater(f['reachable_residual']['volume_mm3'][0], 0.039)
        self.assertEqual(f['uncertainty']['finding_cells'], 4)
        self.assertEqual(len(f['findings']), 1)
        self.assertTrue(f['findings_truncated'])
        self.assertEqual(f['uncertainty']['missing_inner_contact_cells'], 4)
        self.assertEqual(f['uncertainty']['stock_height_interval_cells'], 0)
