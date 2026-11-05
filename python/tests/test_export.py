"""Export acceptance: no machine-ready program with unaccepted check results.

Built on the QA review's reproduction: a fixture prism in the standalone
example's cutting path.
"""

from dataclasses import replace
import importlib.util
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from camlib.export import ExportRefused
from camlib.planar import Boundary, Component, PlanarRegion
from camlib.verification import Prism, Scene, StockContactOptions, StreamChecks

ROOT = Path(__file__).resolve().parents[2]


def example():
    spec = importlib.util.spec_from_file_location("standalone_export", ROOT / "examples/standalone_job.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def with_fixture(job):
    footprint = PlanarRegion("fixture-footprint", (Component(Boundary.polygon(((4, 4), (14, 4), (14, 12), (4, 12)))),))
    fixture = Prism("blocking-fixture", footprint, 0, 10, role="fixture")
    checks = StreamChecks(scene=Scene("export-scene", "1", bodies=(fixture,)), stock_contact=StockContactOptions())
    return replace(job, setups=(replace(job.setups[0], verification=checks),))


class ExportTest(unittest.TestCase):
    def setUp(self):
        self.module = example()

    def test_violations_refuse_text_and_file_and_keep_the_old_file(self):
        job = with_fixture(self.module.build_job())
        machine = self.module.machine_for(job)
        with TemporaryDirectory() as directory:
            target = Path(directory) / "part.nc"
            with self.assertRaises(ExportRefused) as refused:
                job.post_nc("top", machine=machine, path=target)
            self.assertFalse(target.exists())
            self.assertEqual(refused.exception.statuses, ("violation",))
            first = refused.exception.findings[0]["findings"][0]
            self.assertEqual((first["kind"], first["object_id"]), ("fixture_contact", "blocking-fixture"))
            self.assertIn("blocking-fixture", str(refused.exception))
            target.write_text("previous program\n")
            with self.assertRaises(ExportRefused):
                job.post_nc("top", machine=machine, path=target)
            self.assertEqual(target.read_text(), "previous program\n")
            self.assertFalse(list(Path(directory).glob("*.writing")))
        with self.assertRaises(ExportRefused):
            job.post_nc("top", machine=machine)

    def test_explicit_acceptance_releases_and_is_recorded(self):
        job = with_fixture(self.module.build_job())
        with TemporaryDirectory() as directory:
            target = Path(directory) / "part.nc"
            result = job.post_nc("top", machine=self.module.machine_for(job), path=target, accept={"violation"})
            self.assertTrue(target.read_text().endswith("M2\n"))
            self.assertEqual(result.accepted, ("violation",))
            self.assertTrue(all(f["status"] == "violation" for f in result.findings))
        with self.assertRaisesRegex(ValueError, "accept takes"):
            job.post_nc("top", machine=self.module.machine_for(job), accept={"everything"})

    def test_uncertain_stock_checks_require_their_own_explicit_acceptance(self):
        job = self.module.build_job()
        checks = StreamChecks(stock_contact=StockContactOptions(max_nodes=1, max_cell_reads=1))
        job = replace(job, setups=(replace(job.setups[0], verification=checks),))
        machine = self.module.machine_for(job)
        with TemporaryDirectory() as directory:
            target = Path(directory) / "uncertain.nc"
            target.write_text("previous program\n")
            for accept in ((), {"violation"}):
                with self.assertRaises(ExportRefused) as refused:
                    job.post_nc("top", machine=machine, path=target, accept=accept)
                self.assertEqual(refused.exception.statuses, ("uncertain",))
                self.assertEqual(target.read_text(), "previous program\n")
            result = job.post_nc("top", machine=machine, accept={"uncertain"})
            self.assertEqual(result.accepted, ("uncertain",))
            self.assertTrue(result.findings)
            self.assertTrue(all(f["status"] == "uncertain" for f in result.findings))
            with job.post("top", machine=machine) as run:
                with self.assertRaises(ExportRefused):
                    run.export(target, accept={"violation"})
                self.assertEqual(target.read_text(), "previous program\n")
                released = run.export(target, accept={"uncertain"})
                self.assertEqual(released["accepted"], ("uncertain",))
                self.assertEqual(target.read_text(), result.text)

    def test_unconfigured_checks_are_not_misrepresented_as_clear(self):
        job = self.module.build_job()
        job = replace(job, setups=(replace(job.setups[0], verification=None),))
        result = job.post_nc("top", machine=self.module.machine_for(job))
        self.assertEqual(result.unchecked, ("scene", "stock_contact", "material"))

    def test_clean_job_exports_and_names_unconfigured_checks(self):
        job = self.module.build_job()
        result = job.post_nc("top", machine=self.module.machine_for(job))
        self.assertEqual((result.findings, result.accepted), ((), ()))
        self.assertEqual(result.unchecked, ("scene", "material"))
        self.assertTrue(result.text.endswith("M2\n"))

    def test_posted_run_export_applies_the_same_rule(self):
        clean = self.module.build_job()
        bad = with_fixture(clean)
        with TemporaryDirectory() as directory:
            with clean.post("top", machine=self.module.machine_for(clean)) as run:
                written = run.export(Path(directory) / "clean.nc")
                text = Path(written["path"]).read_text()
                self.assertEqual(text.count("\n"), written["lines"])
                self.assertEqual(text, clean.post_nc("top", machine=self.module.machine_for(clean)).text)
            target = Path(directory) / "bad.nc"
            with bad.post("top", machine=self.module.machine_for(bad)) as run:
                with self.assertRaises(ExportRefused):
                    run.export(target)
                self.assertFalse(target.exists())
                released = run.export(target, accept={"violation"})
                self.assertEqual(released["accepted"], ("violation",))
                self.assertTrue(target.is_file())


if __name__ == "__main__":
    unittest.main()
