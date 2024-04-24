# Parse log profiling data and produce a graph showing encode times in time bins,
# with a breakdown per message type.

import argparse
import csv
import math
import os
import webbrowser
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
from bokeh.io import output_notebook
from bokeh.models import ColumnDataSource, HoverTool, Legend, LegendItem
from bokeh.palettes import Category20, Viridis256
from bokeh.plotting import figure, show, output_file
from tabulate import tabulate


@dataclass
class SeriesDetail:
    event_loop_times_s: List[float]
    encode_times_ms: List[float]


def parse_csv_file(filepath: Path, max_lines: Optional[int],
                   start_time: Optional[float],
                   end_time: Optional[float]) -> Dict[str, SeriesDetail]:
    """Parses the CSV file to extract needed data, respecting the maximum number of lines if provided."""
    data_by_type: Dict[str, SeriesDetail] = {}

    with open(filepath, 'r') as file:
        reader = csv.reader(file)
        next(reader)  # Skip the header line

        line_count = 0
        for line in reader:
            if max_lines is not None and line_count >= max_lines:
                break

            line_count += 1

            assert len(line) > 0

            # Note that channel_name and encoding_start_time_ns are not yet used.
            # They may be used here in the future, but for now they are helpful when
            # directly looking at the csv file.
            channel_name, channel_type, encode_duration_ns, encoding_start_time_ns, message_time_s = line

            # Convert nanoseconds to milliseconds
            encode_duration_ms = float(encode_duration_ns) * 1e-6
            message_time_s = float(message_time_s)

            if (start_time is not None and message_time_s < start_time):
                continue
            if (end_time is not None and message_time_s > end_time):
                continue

            if channel_type in data_by_type:
                data_by_type[channel_type].encode_times_ms.append(
                    encode_duration_ms)
                data_by_type[channel_type].event_loop_times_s.append(
                    message_time_s)
            else:
                data_by_type[channel_type] = SeriesDetail(
                    encode_times_ms=[encode_duration_ms],
                    event_loop_times_s=[message_time_s])

    return data_by_type


@dataclass
class DataBin:
    bin_range: str
    message_encode_times: Dict[str, float]


@dataclass
class BinnedData:
    bins: List[DataBin] = field(default_factory=list)
    top_type_names: List[str] = field(default_factory=list)


def create_binned_data(data_by_type: Dict[str, SeriesDetail],
                       num_bins: int = 25,
                       top_n: int = 5) -> BinnedData:
    # Calculate total encoding times for each message type across the entire file.
    total_encode_times: Dict[str, float] = {
        message_type: sum(detail.encode_times_ms)
        for message_type, detail in data_by_type.items()
    }

    # Determine the top N message types based on total encoding times.
    top_types: List[Tuple[str, float]] = sorted(total_encode_times.items(),
                                                key=lambda item: item[1],
                                                reverse=True)[:top_n]
    print(f"{top_types=}")
    top_type_names: List[str] = [type_name for type_name, _ in top_types]

    # Find the global minimum and maximum times to establish bin edges.
    min_time: float = min(detail.event_loop_times_s[0]
                          for detail in data_by_type.values())
    max_time: float = max(detail.event_loop_times_s[-1]
                          for detail in data_by_type.values())

    # Create bins.
    bins = np.linspace(min_time, max_time, num_bins + 1)

    # Initialize the list of DataBin instances with the correct number of bins.
    binned_data = BinnedData(top_type_names=top_type_names)
    for i in range(num_bins):
        bin_range = f"{bins[i]:.2f} - {bins[i+1]:.2f}"
        binned_data.bins.append(
            DataBin(bin_range=bin_range,
                    message_encode_times={
                        name: 0
                        for name in top_type_names + ['other']
                    }))

    # Populate binned_data with message encode times.
    for message_type, details in data_by_type.items():
        binned_indices = np.digitize(details.event_loop_times_s, bins) - 1
        # Correcting the bin indices that are out of range by being exactly on the maximum edge.
        binned_indices = np.minimum(binned_indices, num_bins - 1)

        for idx, encode_time in enumerate(details.encode_times_ms):
            bin_index = binned_indices[idx]
            current_bin = binned_data.bins[bin_index]
            if message_type in top_type_names:
                current_bin.message_encode_times[message_type] += encode_time
            else:
                current_bin.message_encode_times['other'] += encode_time

    return binned_data


def print_binned_data(binned_data: BinnedData) -> None:
    # Extend headers for the table by replacing '.' with '\n.' for each message type name and
    # adding 'Total'.
    headers = ['Bin Range'] + [
        key.replace('.', '\n.')
        for key in binned_data.top_type_names + ['other']
    ] + ['Total']

    # Initialize the table data list.
    table_data = []

    # Populate the table data with the values from each DataBin instance and calculate totals.
    for data_bin in binned_data.bins:
        # Initialize a row with the bin range.
        row = [data_bin.bin_range]
        # Add the total message encode times for each message type.
        encode_times = [
            data_bin.message_encode_times[message_type]
            for message_type in binned_data.top_type_names
        ]
        other_time = data_bin.message_encode_times['other']
        row += encode_times + [other_time]
        # Calculate the total encode time for the row and append it.
        total_encode_time = sum(encode_times) + other_time
        row.append(total_encode_time)
        # Append the row to the table data.
        table_data.append(row)

    # Print the table using tabulate with 'grid' format for better structure.
    print(
        tabulate(table_data, headers=headers, tablefmt='grid', floatfmt=".2f"))


def plot_bar(binned_data: BinnedData) -> None:
    filename = "plot.html"
    output_file(filename, title="Message Encode Time Plot Stacked Bar Graph")

    # Adjust width based on bin count for readability.
    plot_width = max(1200, 50 * len(binned_data.bins))

    p = figure(x_range=[bin.bin_range for bin in binned_data.bins],
               title='Message Encode Time by Type over Event Loop Time',
               x_axis_label='Event Loop Time Bins',
               y_axis_label='Total Message Encode Time (ms)',
               width=plot_width,
               height=600,
               tools="")

    source_data = {'bin_edges': [bin.bin_range for bin in binned_data.bins]}
    for message_type in binned_data.top_type_names + ['other']:
        source_data[message_type] = [
            bin.message_encode_times.get(message_type, 0)
            for bin in binned_data.bins
        ]

    source = ColumnDataSource(data=source_data)

    # Calculate totals and sort in descending order.
    totals = {
        message_type: sum(source_data[message_type])
        for message_type in source_data if message_type != 'bin_edges'
    }
    sorted_message_types = sorted(totals, key=totals.get, reverse=True)

    # Reverse the list to place larger segments at the top.
    sorted_message_types.reverse()

    num_types = len(sorted_message_types)
    if num_types > 20:
        raise ValueError(
            f"Number of types ({num_types}) exceeds the number of available colors in Category20."
        )
    colors = Category20[20][:num_types]

    # Apply reversed order to stack rendering.
    renderers = p.vbar_stack(sorted_message_types,
                             x='bin_edges',
                             width=0.7,
                             color=colors,
                             source=source)

    # Change the orientation of the x-axis labels to a 45-degree angle.
    p.xaxis.major_label_orientation = math.pi / 4

    # Create a custom legend, maintaining the reversed order for visual consistency.
    legend_items = [
        LegendItem(label=mt.replace('.', ' '), renderers=[renderers[i]])
        for i, mt in enumerate(sorted_message_types)
    ]
    legend = Legend(items=legend_items, location=(0, -30))
    p.add_layout(legend, 'right')

    p.y_range.start = 0
    p.x_range.range_padding = 0.05
    p.xgrid.grid_line_color = None
    p.axis.minor_tick_line_color = None
    p.outline_line_color = None

    file_path = os.path.realpath(filename)
    print('\n')
    print(f"Plot saved to '{file_path}'")
    webbrowser.open('file://' + file_path)


def main():
    parser = argparse.ArgumentParser(
        description=
        'Process log files to extract and plot message encode times.')
    parser.add_argument('--log_file_path',
                        type=Path,
                        help='Path to the log file',
                        required=True)
    parser.add_argument(
        '--max_lines',
        type=int,
        default=None,
        help='Maximum number of lines to read from the log file')
    parser.add_argument('--num_bins',
                        type=int,
                        default=40,
                        help='Number of bins to use')
    parser.add_argument('--top_n',
                        type=int,
                        default=10,
                        help='Number of top message types to plot')
    parser.add_argument('--start_time',
                        type=float,
                        default=None,
                        help='Start time in seconds')
    parser.add_argument('--end_time',
                        type=float,
                        default=None,
                        help='End time in seconds')

    args = parser.parse_args()

    data_by_type = parse_csv_file(filepath=args.log_file_path,
                                  max_lines=args.max_lines,
                                  start_time=args.start_time,
                                  end_time=args.end_time)
    binned_data = create_binned_data(data_by_type,
                                     num_bins=args.num_bins,
                                     top_n=args.top_n)
    print_binned_data(binned_data)
    plot_bar(binned_data)
    print(f"{os.path.basename(__file__)} Finished.")


if __name__ == '__main__':
    main()
