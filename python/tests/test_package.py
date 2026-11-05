import subprocess
import sys
import unittest

import camlib


class PackageTest(unittest.TestCase):
    def test_top_level_names_resolve_lazily_without_cad(self):
        for name in camlib.__all__:
            self.assertTrue(hasattr(camlib, name), name)
        from camlib import jobs

        self.assertIs(camlib.Setup, jobs.Setup)
        with self.assertRaises(AttributeError):
            camlib.not_a_name
        # A fresh interpreter: importing camlib must not pull in CadQuery.
        code = "import sys, camlib; camlib.Pocket; print('cadquery' in sys.modules)"
        out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, check=True)
        self.assertEqual(out.stdout.strip(), "False")


if __name__ == "__main__":
    unittest.main()
