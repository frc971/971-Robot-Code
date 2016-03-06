#!/usr/bin/python3

import sys
import numpy
from plotter import Plotter
import argparse

def ReadPlotDefinitions(filename):
  """
  Read a file with plotting definitions.

  A plotting definition is a single line that defines what data to search for
  in order to plot it. The following in a file would duplicate the default
  behaviour:

    fridge goal height
    fridge goal angle
    fridge goal velocity
    fridge goal angular_velocity
    fridge output left_arm
    fridge output right_arm
    fridge output left_elevator
    fridge output right_elevator

  Lines are ignored if they start with a hash mark (i.e. '#').

  Lines that end with a "-b X" where X is a number then it designates that line
  as plotting a boolean value. X is the value plotted when the boolean is true.
  When the boolean is false then the values is plotted as zero. For example,
  the following boolean value is drawn to toggle between 2.0 and 0 when the
  boolean is True and False, respectively:

    fridge status zeroed -b 2.0

  Args:
    filename: The name of the file to read the definitions from.

  Returns:
    [[str]]: The definitions in the specified file.
  """
  defs = []
  with open(filename) as fd:
    for line in fd:
      raw_defs = line.split()

      # Only add to the list of definitions if the line's not empty and it
      # doesn't start with a hash.
      if raw_defs and not raw_defs[0].startswith('#'):
        defs.append(raw_defs)

  return defs

def main():
  # Parse all command line arguments.
  arg_parser = argparse.ArgumentParser(description='Log Plotter')
  arg_parser.add_argument('log_file', metavar='LOG_FILE', type=str, \
      help='The file from which to read logs and plot.')
  arg_parser.add_argument('--plot-defs', '-p', action='store', type=str, \
      help='Read the items to plot from this file.')
  arg_parser.add_argument('--no-binary', '-n', action='store_true', \
      help='Don\'t print the binary name in the legend.')

  args = arg_parser.parse_args(sys.argv[1:])

  p = Plotter()

  # If the user defines the list of data to plot in a file, read it from there.
  if args.plot_defs:
    defs = ReadPlotDefinitions(args.plot_defs)
    for definition in defs:
      p.Add(definition[0], definition[1], *definition[2:])

  # Otherwise use a pre-defined set of data to plot.
  else:
    p.Add('fridge', 'goal', 'height')
    p.Add('fridge', 'goal', 'angle')
    p.Add('fridge', 'goal', 'velocity')
    p.Add('fridge', 'goal', 'angular_velocity')

    p.Add('fridge', 'output', 'left_arm')
    p.Add('fridge', 'output', 'right_arm')
    p.Add('fridge', 'output', 'left_elevator')
    p.Add('fridge', 'output', 'right_elevator')

  p.PlotFile(args.log_file, args.no_binary)

if __name__ == '__main__':
  main()
