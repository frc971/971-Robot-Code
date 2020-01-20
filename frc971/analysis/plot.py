#!/usr/bin/python3
# Sample usage:
# bazel run -c opt //frc971/analysis:plot  -- --logfile /tmp/log.fbs --config gyro.pb
import argparse
import json
import os.path
from pathlib import Path
import sys

from frc971.analysis.py_log_reader import LogReader
from frc971.analysis.plot_config_pb2 import PlotConfig, Signal, Line
from google.protobuf import text_format

import numpy as np
import matplotlib
from matplotlib import pyplot as plt


class Plotter:
    def __init__(self, plot_config: PlotConfig, reader: LogReader):
        self.config = plot_config
        self.reader = reader
        # Data streams, indexed by alias.
        self.data = {}

    def process_logfile(self):
        aliases = set()
        for channel in self.config.channel:
            if channel.alias in aliases:
                raise ValueError("Duplicate alias " + channel.alias)
            aliases.add(channel.alias)
            if not self.reader.subscribe(channel.name, channel.type):
                raise ValueError("No such channel with name " + channel.name +
                                 " and type " + channel.type)

        self.reader.process()

        for channel in self.config.channel:
            self.data[channel.alias] = []
            for message in self.reader.get_data_for_channel(
                    channel.name, channel.type):
                valid_json = message[2].replace('nan', '"nan"')
                parsed_json = json.loads(valid_json)
                self.data[channel.alias].append((message[0], message[1],
                                                 parsed_json))
        self.calculate_signals()

    def calculate_imu_signals(self):
        if 'IMU' in self.data:
            entries = []
            for entry in self.data['IMU']:
                accel_x = 'accelerometer_x'
                accel_y = 'accelerometer_y'
                accel_z = 'accelerometer_z'
                msg = entry[2]
                new_msg = {}
                if accel_x in msg and accel_y in msg and accel_x in msg:
                    total_acceleration = np.sqrt(
                        msg[accel_x]**2 + msg[accel_y]**2 + msg[accel_z]**2)
                    new_msg['total_acceleration'] = total_acceleration
                timestamp = 'monotonic_timestamp_ns'
                if timestamp in msg:
                    timestamp_sec = msg[timestamp] * 1e-9
                    new_msg['monotonic_timestamp_sec'] = timestamp_sec
                entries.append((entry[0], entry[1], new_msg))
            if 'CalcIMU' in self.data:
                raise RuntimeError('CalcIMU is already a member of data.')
            self.data['CalcIMU'] = entries

    def calculate_signals(self):
        """Calculate any derived signals for plotting.

        See calculate_imu_signals for an example, but the basic idea is that
        for any data that is read in from the logfile, we may want to calculate
        some derived signals--possibly as simple as doing unit conversions,
        or more complicated version where we do some filtering or the such.
        The way this will work is that the calculate_* functions will, if the
        raw data is available, calculate the derived signals and place them into
        fake "messages" with an alias of "Calc*". E.g., we currently calculate
        an overall magnitude for the accelerometer readings, which is helpful
        to understanding how some internal filters work."""
        self.calculate_imu_signals()

    def extract_field(self, message: dict, field: str):
        """Extracts a field with the given name from the message.

        message will be a dictionary with field names as the keys and then
        values, lists, or more dictionaries as the values. field is the full
        path to the field to extract, with periods separating sub-messages."""
        field_path = field.split('.')
        value = message
        for name in field_path:
            # If the value wasn't populated in a given message, fill in
            # NaN rather than crashing.
            if name in value:
                value = value[name]
            else:
                return None
        # Catch NaNs and convert them to floats.
        return float(value)

    def plot_line(self, axes: matplotlib.axes.Axes, line: Line):
        if not line.HasField('y_signal'):
            raise ValueError("No y_channel specified for line.")
        y_signal = line.y_signal
        if not y_signal.channel in self.data:
            raise ValueError("No channel alias " + y_signal.channel)
        x_signal = line.x_signal if line.HasField('x_signal') else None
        if x_signal is not None and not x_signal.channel in self.data:
            raise ValueError("No channel alias " + x_signal.channel)
        y_messages = self.data[y_signal.channel]
        x_messages = self.data[
            x_signal.channel] if x_signal is not None else None
        if x_messages is not None and len(x_messages) != len(y_messages):
            raise ValueError(
                "X and Y signal lengths don't match. X channel is " +
                x_signal.channel + " Y channel is " + y_signal.channel)
        x_data = []
        y_data = []
        for ii in range(len(y_messages)):
            y_entry = y_messages[ii]
            if x_signal is None:
                x_data.append(y_entry[0] * 1e-9)
            else:
                x_entry = x_messages[ii]
                x_data.append(self.extract_field(x_entry[2], x_signal.field))
            y_data.append(self.extract_field(y_entry[2], y_signal.field))
            if x_data[-1] is None and y_data[-1] is not None:
                raise ValueError(
                    "Only one of the x and y signals is present. X " +
                    x_signal.channel + "." + x_signal.field + " Y " +
                    y_signal.channel + "." + y_signal.field)
        label_name = y_signal.channel + "." + y_signal.field
        axes.plot(x_data, y_data, marker='o', label=label_name)

    def plot(self):
        shared_axis = None
        for figure_config in self.config.figure:
            fig = plt.figure()
            num_subplots = len(figure_config.axes)
            for ii in range(num_subplots):
                axes = fig.add_subplot(
                    num_subplots, 1, ii + 1, sharex=shared_axis)
                shared_axis = shared_axis or axes
                axes_config = figure_config.axes[ii]
                for line in axes_config.line:
                    self.plot_line(axes, line)
                # Make the legend transparent so we can see behind it.
                legend = axes.legend(framealpha=0.5)
                axes.set_xlabel("Monotonic Time (sec)")
                axes.grid(True)
                if axes_config.HasField("ylabel"):
                    axes.set_ylabel(axes_config.ylabel)


def main(argv):
    parser = argparse.ArgumentParser(
        description="Plot data from an aos logfile.")
    parser.add_argument(
        "--logfile",
        type=str,
        required=True,
        help="Path to the logfile to parse.")
    parser.add_argument(
        "--config",
        type=str,
        required=True,
        help="Name of the plot config to use.")
    parser.add_argument(
        "--config_dir",
        type=str,
        default="frc971/analysis/plot_configs",
        help="Directory to look for plot configs in.")
    args = parser.parse_args(argv[1:])

    if not os.path.isdir(args.config_dir):
        print(args.config_dir + " is not a directory.")
        return 1
    config_path = os.path.join(args.config_dir, args.config)
    if not os.path.isfile(config_path):
        print(config_path +
              " does not exist or is not a file--available configs are")
        for file_name in os.listdir(args.config_dir):
            print(os.path.basename(file_name))
        return 1

    config = PlotConfig()
    with open(config_path) as config_file:
        text_format.Merge(config_file.read(), config)

    if not os.path.isfile(args.logfile):
        print(args.logfile + " is not a file.")
        return 1

    reader = LogReader(args.logfile)

    plotter = Plotter(config, reader)
    plotter.process_logfile()
    plotter.plot()
    plt.show()

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
