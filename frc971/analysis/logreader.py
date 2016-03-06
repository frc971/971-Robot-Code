#!/usr/bin/python3

import collections
from logentry import LogEntry

class Dataset(object):
  def __init__(self):
    self.time = []
    self.data = []

  def Add(self, time, data):
    self.time.append(time)
    self.data.append(data)

class CollectingLogReader(object):
  """
  Reads log files and collected requested data.
  """
  def __init__(self):
    self.signal = collections.OrderedDict()

  def Add(self, binary, struct_instance_name, *data_search_path):
    """
    Specifies a specific piece of data to collect

    Args:
      binary: str, The name of the executable that generated the log.
      struct_instance_name: str, The name of the struct instance whose data
                            contents should be collected.
      data_search_path: [str], The path into the struct of the exact piece of
                        data to collect.

    Returns:
      None
    """
    self.signal[(binary, struct_instance_name, data_search_path)] = Dataset()

  def HandleFile(self, f):
    """
    Parses the specified log file.

    Args:
      f: str, The filename of the log whose data to parse.

    Returns:
      None
    """
    with open(f, 'r') as fd:
      for line in fd:
        self.HandleLine(line)

  def HandleLine(self, line):
    """
    Parses a line from a log file and adds the data to the plot data.

    Args:
      line: str, The line from the log file to parse

    Returns:
      None
    """
    pline = LogEntry(line)
    pline_data = None

    for key in self.signal:
      value = self.signal[key]
      binary = key[0]
      struct_instance_name = key[1]
      data_search_path = key[2]
      boolean_multiplier = None

      # If the plot definition line ends with a "-b X" where X is a number then
      # that number gets drawn when the value is True. Zero gets drawn when the
      # value is False.
      if len(data_search_path) >= 2 and data_search_path[-2] == '-b':
        boolean_multiplier = float(data_search_path[-1])
        data_search_path = data_search_path[:-2]

      # Make sure that we're looking at the right binary structure instance.
      if binary == pline.name:
        if pline.msg.startswith(struct_instance_name + ': '):
          # Parse the structure once.
          if pline_data is None:
              _, _, pline_data = pline.ParseStruct()
          # Traverse the structure as specified in `data_search_path`.
          # This lets the user access very deeply nested structures.
          data = pline_data
          for path in data_search_path:
            data = data[path]

          if boolean_multiplier is not None:
            if data == 'T':
              value.Add(pline.time, boolean_multiplier)
            else:
              value.Add(pline.time, 0)
          else:
            value.Add(pline.time, data)
