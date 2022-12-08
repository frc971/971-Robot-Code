import sys
import unittest


class TestPipImports(unittest.TestCase):

    def test_version(self):
        """Validates that we are using the version specified in rules_python."""
        self.assertEqual(sys.version_info[0:3], (3, 9, 15))

    def test_imports(self):
        """Validates that we can import pip packages from pypi.org."""
        import numpy
        import scipy
        import matplotlib

        # Make sure we're sourcing numpy from the expected source. We could
        # pick any of the three we imported above.
        self.assertTrue(
            numpy.__file__.endswith(
                "/upstream_python_test.runfiles/pip_deps_numpy/site-packages/numpy/__init__.py"
            ), numpy.__file__)


if __name__ == "__main__":
    unittest.main()
