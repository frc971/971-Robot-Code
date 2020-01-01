#!/usr/bin/python3
import json
import unittest

from aos.Configuration import Configuration
from frc971.analysis.py_log_reader import LogReader


class LogReaderTest(unittest.TestCase):
    def setUp(self):
        self.reader = LogReader("external/sample_logfile/file/log.fbs")
        # A list of all the channels in the logfile--this is used to confirm that
        # we did indeed read the config correctly.
        self.all_channels = [
            ("/aos", "aos.JoystickState"), ("/aos", "aos.RobotState"),
            ("/aos", "aos.timing.Report"), ("/aos", "frc971.PDPValues"),
            ("/aos",
             "frc971.wpilib.PneumaticsToLog"), ("/autonomous",
                                                "aos.common.actions.Status"),
            ("/autonomous", "frc971.autonomous.AutonomousMode"),
            ("/autonomous", "frc971.autonomous.Goal"), ("/camera",
                                                        "y2019.CameraLog"),
            ("/camera", "y2019.control_loops.drivetrain.CameraFrame"),
            ("/drivetrain",
             "frc971.IMUValues"), ("/drivetrain",
                                   "frc971.control_loops.drivetrain.Goal"),
            ("/drivetrain",
             "frc971.control_loops.drivetrain.LocalizerControl"),
            ("/drivetrain", "frc971.control_loops.drivetrain.Output"),
            ("/drivetrain", "frc971.control_loops.drivetrain.Position"),
            ("/drivetrain", "frc971.control_loops.drivetrain.Status"),
            ("/drivetrain", "frc971.sensors.GyroReading"),
            ("/drivetrain",
             "y2019.control_loops.drivetrain.TargetSelectorHint"),
            ("/superstructure",
             "y2019.StatusLight"), ("/superstructure",
                                    "y2019.control_loops.superstructure.Goal"),
            ("/superstructure", "y2019.control_loops.superstructure.Output"),
            ("/superstructure", "y2019.control_loops.superstructure.Position"),
            ("/superstructure", "y2019.control_loops.superstructure.Status")
        ]
        # A channel that is known to have data on it which we will use for testing.
        self.test_channel = ("/aos", "aos.timing.Report")
        # A non-existent channel
        self.bad_channel = ("/aos", "aos.timing.FooBar")

    def test_do_nothing(self):
        """Tests that we sanely handle doing nothing.

        A previous iteration of the log reader seg faulted when doing this."""
        pass

    def test_read_config(self):
        """Tests that we can read the configuration from the logfile."""
        config_bytes = self.reader.configuration()
        config = Configuration.GetRootAsConfiguration(config_bytes, 0)

        channel_set = set(self.all_channels)
        for ii in range(config.ChannelsLength()):
            channel = config.Channels(ii)
            # Will raise KeyError if the channel does not exist
            channel_set.remove((channel.Name().decode("utf-8"),
                                channel.Type().decode("utf-8")))

        self.assertEqual(0, len(channel_set))

    def test_empty_process(self):
        """Tests running process() without subscribing to anything succeeds."""
        self.reader.process()
        for channel in self.all_channels:
            with self.assertRaises(ValueError) as context:
                self.reader.get_data_for_channel(channel[0], channel[1])

    def test_subscribe(self):
        """Tests that we can subscribe to a channel and get data out."""
        name = self.test_channel[0]
        message_type = self.test_channel[1]
        self.assertTrue(self.reader.subscribe(name, message_type))
        self.reader.process()
        data = self.reader.get_data_for_channel(name, message_type)
        self.assertLess(100, len(data))
        last_monotonic_time = 0
        for entry in data:
            monotonic_time = entry[0]
            realtime_time = entry[1]
            json_data = entry[2].replace('nan', '\"nan\"')
            self.assertLess(last_monotonic_time, monotonic_time)
            # Sanity check that the realtime times are in the correct range.
            self.assertLess(1500000000e9, realtime_time)
            self.assertGreater(2000000000e9, realtime_time)
            parsed_json = json.loads(json_data)
            self.assertIn("name", parsed_json)

            last_monotonic_time = monotonic_time

    def test_bad_subscribe(self):
        """Tests that we return false when subscribing to a non-existent channel."""
        self.assertFalse(
            self.reader.subscribe(self.bad_channel[0], self.bad_channel[1]),
            self.bad_channel)

    def test_subscribe_after_process(self):
        """Tests that an exception is thrown if we subscribe after calling process()."""
        self.reader.process()
        for channel in self.all_channels:
            with self.assertRaises(RuntimeError) as context:
                self.reader.subscribe(channel[0], channel[1])

    def test_get_data_before_processj(self):
        """Tests that an exception is thrown if we retrieve data before calling process()."""
        for channel in self.all_channels:
            with self.assertRaises(RuntimeError) as context:
                self.reader.get_data_for_channel(channel[0], channel[1])


if __name__ == '__main__':
    unittest.main()
