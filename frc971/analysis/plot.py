#!/usr/bin/python3
# Sample usage:
# bazel run -c opt //frc971/analysis:plot  -- --logfile /tmp/log.fbs --config gyro.pb
import argparse
import json
import os.path
from pathlib import Path
import sys

from frc971.analysis.py_log_reader import LogReader
from frc971.analysis.plot_config_pb2 import PlotConfig, Signal
from google.protobuf import text_format

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

    def plot_signal(self, axes: matplotlib.axes.Axes, signal: Signal):
        if not signal.channel in self.data:
            raise ValueError("No channel alias " + signal.channel)
        field_path = signal.field.split('.')
        monotonic_time = []
        signal_data = []
        for entry in self.data[signal.channel]:
            monotonic_time.append(entry[0] * 1e-9)
            value = entry[2]
            for name in field_path:
                value = value[name]
            # Catch NaNs and convert them to floats.
            value = float(value)
            signal_data.append(value)
        label_name = signal.channel + "." + signal.field
        axes.plot(monotonic_time, signal_data, label=label_name)

    def plot(self):
        for figure_config in self.config.figure:
            fig = plt.figure()
            num_subplots = len(figure_config.axes)
            for ii in range(num_subplots):
                axes = fig.add_subplot(num_subplots, 1, ii + 1)
                axes_config = figure_config.axes[ii]
                for signal in axes_config.signal:
                    self.plot_signal(axes, signal)
                axes.legend()
                axes.set_xlabel("Monotonic Time (sec)")
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
