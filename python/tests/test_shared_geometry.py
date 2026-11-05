"""Large shared planning/material input, immutable identity and retained replay."""

from camlib.postprocessor import GCodeDialect

from dataclasses import replace
from functools import lru_cache
import json
import os
from tempfile import TemporaryDirectory
import unittest

from camlib._geometry import share_geometry, resolve_geometry
from camlib.jobs import Job, Setup, RegenerationCache
from camlib.material import SurfaceTarget, FinishingScope, Verifier
from camlib.operations import Parallel
from camlib.playback import Player, Cursor
from camlib.simulation import Simulation, StockSpec, VerificationUpdate
from camlib.surface import SurfaceMesh
from camlib.tools import BallEndMill, ToolAssembly, HolderSection
from camlib.verification import MaterialCheck, StreamChecks
from camlib.post import Machine, ToolSetting


def encoded(value):
    return json.dumps(value, allow_nan=False, separators=(',', ':')).encode()


@lru_cache(None)
def job():
    n = 180
    points = [(-2 + x / 23.123456789, -2 + y / 23.123456789, 2.0) for y in range(n + 1) for x in range(n + 1)]
    triangles = []
    for y in range(n):
        for x in range(n):
            a = y * (n + 1) + x
            b = a + 1
            c = b + n + 1
            d = a + n + 1
            triangles.extend(((points[a], points[b], points[c]), (points[a], points[c], points[d])))
    surface = SurfaceMesh(triangles, source={'revision': '1'})
    tool = ToolAssembly(
        'ball',
        BallEndMill('bit', diameter=2, flute_length=10, overall_length=20),
        12,
        25,
        (HolderSection(12, 25, 3, 3),),
    )
    op = Parallel(
        'finish',
        tool,
        surface,
        0,
        0.5,
        6,
        feed_mm_min=600,
        plunge_mm_min=150,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant="flood",
    )
    target = SurfaceTarget('target', '1', surface, 0, finishing=FinishingScope.from_operation(op))
    setup = Setup(
        'top',
        StockSpec((-2, -2, 0), (6, 6, 4), 1),
        (tool,),
        (op,),
        (0, 0, 6),
        verification=StreamChecks(material=(MaterialCheck(target, after_operation='finish'),)),
    )
    return Job('shared-surface', '1', (setup,))


class SharedGeometryTest(unittest.TestCase):
    def test_exact_sharing_preserves_inputs_metadata_and_signed_zero(self):
        small = SurfaceMesh((((0.0, 0.0, 1.0), (2.0, 0.0, 1.0), (0.0, 2.0, 1.0)),), source={'revision': '1'})
        a = dict(kind='parallel', surface=small.to_dict())
        b = dict(kind='waterline', surface=small.to_dict())
        original = dict(operations=[a, b])
        before = encoded(original)
        compact = share_geometry(original)
        self.assertEqual(len(compact['geometry']), 1)
        self.assertEqual(encoded(original), before)
        resolved = resolve_geometry(json.loads(encoded(compact)))
        self.assertIs(
            resolved['operations'][0]['surface']['triangles'], resolved['operations'][1]['surface']['triangles']
        )
        for mesh in (
            replace(small, source={'revision': '2'}),
            replace(small, surface_error_mm=0.005),
            replace(small, triangles=(((-0.0, 0.0, 1.0), (2.0, 0.0, 1.0), (0.0, 2.0, 1.0)),)),
        ):
            changed = share_geometry(dict(operations=[a, dict(kind='parallel', surface=mesh.to_dict())]))
            self.assertEqual(len(changed['geometry']), 2 if mesh.triangles[0][0][0].hex().startswith('-') else 1)
            self.assertNotEqual(changed['operations'][0]['surface'], changed['operations'][1]['surface'])

    def test_large_native_posted_material_and_fractional_seek(self):
        j = job()
        setup = j.setups[0]
        target = setup.verification.material[0].target
        raw = dict(operations=[setup.operations[0].to_dict()], verification=setup.verification.to_dict())
        # A >4 MiB duplicated workload; sharing stores the mesh once.
        self.assertGreater(len(encoded(raw)), 4 << 20)
        compact = share_geometry(raw)
        self.assertLess(len(encoded(compact)) * 1.8, len(encoded(raw)))
        self.assertEqual(len(compact['geometry']), 1)
        executable = os.environ.get('CAMLIB_TEST_ENGINE')
        for posted in (False, True):
            with self.subTest(posted=posted):
                if posted:
                    header = j.compile().setup('top').program_header()
                    machine = Machine(
                        'Example mill',
                        (
                            ToolSetting.for_tool(
                                header.tools[0],
                                number=1,
                                length_register=1,
                                position_after_mm=header.initial_position_mm,
                            ),
                        ),
                        tool_change_position_mode='preserve_tip',
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
                    context = j.post('top', machine=machine, executable=executable)
                else:
                    context = j.record('top', executable=executable, checkpoint_every=8, rapid_mm_min=6000)
                with context as run:
                    run.wait()
                    last = run.status()['available']
                    self.assertGreater(last, 0)
                    player = Player(run, compact=True)
                    dense = Player(run)
                    verifier = Verifier(target)
                    for cursor in (Cursor(last), Cursor(last // 2, 0.37), Cursor(last)):
                        player.seek(cursor)
                        dense.seek(cursor)
                        self.assertEqual(player.state.heights.tobytes(), dense.state.heights.tobytes())
                    report = verifier.check(player.state)
                    self.assertTrue(report['complete'])
                    self.assertEqual(report['gouge']['definite_cells'], 0)
                    self.assertGreater(player.state.volume()['removed_lower_mm3'], 1)
                    records = player.verification['records']
                    full = list(run.verification_records(records - 1, limit=1))[0].check['report']
                    self.assertEqual(full, report)
                    wire = json.loads(player.state.canonical_manifest)
                    self.assertEqual(len(wire['geometry']), 1)
                    self.assertLess(len(player.state.canonical_manifest) * 1.8, len(encoded(raw)))
                    if not posted:
                        manifest = player.state.manifest
                        self.assertIs(
                            manifest['operations'][0]['surface']['vertices'],
                            manifest['verification']['material'][0]['target']['surface']['vertices'],
                        )

    def test_shared_geometry_regeneration_reuses_stock_but_binds_new_run(self):
        # The same integration at a small size isolates cache identity from input
        # size; the previous test covers the actual >4MiB duplicated workload.
        base = job()
        setup = base.setups[0]
        op = setup.operations[0]
        small = SurfaceMesh(
            (((-2, -2, 2), (6, -2, 2), (6, 6, 2)), ((-2, -2, 2), (6, 6, 2), (-2, 6, 2))), source={'revision': '1'}
        )
        op = replace(op, surface=small)
        target = replace(setup.verification.material[0].target, surface=small)
        setup = replace(
            setup,
            operations=(op,),
            verification=StreamChecks(material=(MaterialCheck(target, after_operation='finish'),)),
        )
        base = replace(base, setups=(setup,))
        with TemporaryDirectory() as directory:
            cache = RegenerationCache(directory)
            states = []
            for revision in ('1', '2'):
                with replace(base, revision=revision).regenerate(
                    cache, executable=os.environ.get('CAMLIB_TEST_ENGINE')
                ) as run:
                    list(run)
                    states.append(run.states['top'])
                    self.assertEqual(run.statistics[('top', 'finish')]['cache_hit'], revision == '2')
            self.assertEqual(states[0].heights.tobytes(), states[1].heights.tobytes())
            self.assertNotEqual(states[0].canonical_manifest, states[1].canonical_manifest)

    def test_material_base_below_stock_keeps_shared_native_stage(self):
        setup = job().setups[0]
        eps = 2.0**-52
        surface = SurfaceMesh(
            (
                ((-2, -2, 2), (6, -2, 2), (6, 6, 2)),
                ((-2, -2, 2), (6, 6, 2), (-2, 6, 2)),
                ((-2, -2, -eps), (6, -2, -eps), (6, 6, -eps)),
            )
        )
        op = replace(setup.operations[0], surface=surface)
        target = replace(setup.verification.material[0].target, surface=surface, base_z_mm=-eps)
        checks = StreamChecks(material=(MaterialCheck(target, after_operation=op.id),))
        with Simulation(
            setup.stock,
            setup.tools,
            revision='base',
            initial_position_mm=setup.initial_position_mm,
            operations=(op,),
            verification=checks,
            executable=os.environ.get('CAMLIB_TEST_ENGINE'),
        ) as sim:
            reports = [
                u.check['report']
                for u in sim.run_operations()
                if isinstance(u, VerificationUpdate) and u.check['kind'] == 'material'
            ]
            self.assertTrue(sim.state.finished)
            self.assertEqual(reports, [Verifier(target).check(sim.state)])
            self.assertTrue(reports[0]['complete'])
            self.assertEqual(reports[0]['gouge']['definite_cells'], 0)
