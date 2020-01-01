#!/usr/bin/python3
import unittest

import matplotlib
# Use a non-interactive backend so that the test can actually run...
matplotlib.use('Agg')

import frc971.analysis.plot


class PlotterTest(unittest.TestCase):
    def test_plotter(self):
        """Basic test that makes sure that we can run the test without crashing."""
        self.assertEqual(0,
                         frc971.analysis.plot.main([
                             "binary", "--logfile",
                             "external/sample_logfile/file/log.fbs",
                             "--config", "gyro.pb"
                         ]))


if __name__ == '__main__':
    unittest.main()
