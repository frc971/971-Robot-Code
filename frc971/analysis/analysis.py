#!/usr/bin/python3
import matplotlib
from matplotlib import pylab
from matplotlib.font_manager import FontProperties

class Dataset(object):
  def __init__(self):
    self.time = []
    self.data = []

  def Add(self, time, data):
    self.time.append(time)
    self.data.append(data)


class Plotter(object):
  def __init__(self):
    self.signal = dict()

  def Add(self, binary, struct_instance_name, *data_search_path):
    """
    Specifies a specific piece of data to plot

    Args:
      binary: str, The name of the executable that generated the log.
      struct_instance_name: str, The name of the struct instance whose data
                            contents should be plotted.
      data_search_path: [str], The path into the struct of the exact piece of
                        data to plot.

    Returns:
      None
    """
    self.signal[(binary, struct_instance_name, data_search_path)] = Dataset()

  def HandleLine(self, line):
    """
    Parses a line from a log file and adds the data to the plot data.

    Args:
      line: str, The line from the log file to parse

    Returns:
      None
    """
    pline = ParseLine(line)
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
          # Parse the structure and traverse it as specified in
          # `data_search_path`. This lets the user access very deeply nested
          # structures.
          _, _, data = pline.ParseStruct()
          for path in data_search_path:
            data = data[path]

          if boolean_multiplier is not None:
            if data == 'T':
              value.Add(pline.time, boolean_multiplier)
            else:
              value.Add(pline.time, 0)
          else:
            value.Add(pline.time, data)

  def Plot(self, no_binary_in_legend):
    """
    Plots all the data after it's parsed.

    This should only be called after `HandleFile` has been called so that there
    is actual data to plot.
    """
    for key in self.signal:
      value = self.signal[key]

      # Create a legend label using the binary name (optional), the structure
      # name and the data search path.
      label = key[1] + '.' + '.'.join(key[2])
      if not no_binary_in_legend:
        label = key[0] + ' ' + label

      pylab.plot(value.time, value.data, label=label)

    # Set legend font size to small and move it to the top center.
    fontP = FontProperties()
    fontP.set_size('small')
    pylab.legend(bbox_to_anchor=(0.5, 1.05), prop=fontP)

    pylab.show()

  def PlotFile(self, f, no_binary_in_legend=False):
    """
    Parses and plots all the data.

    Args:
      f: str, The filename of the log whose data to parse and plot.

    Returns:
      None
    """
    self.HandleFile(f)
    self.Plot(no_binary_in_legend)

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


class LogEntry:
  """This class provides a way to parse log entries."""

  def __init__(self, line):
    """Creates a LogEntry from a line."""
    name_index = line.find('(')
    self.name = line[0:name_index]

    pid_index = line.find(')', name_index + 1)
    self.pid = int(line[name_index + 1:pid_index])

    msg_index_index = line.find(')', pid_index + 1)
    self.msg_index = int(line[pid_index + 2:msg_index_index])

    level_index = line.find(' ', msg_index_index + 3)
    self.level = line[msg_index_index + 3:level_index]

    time_index_start = line.find(' at ', level_index) + 4
    time_index_end = line.find('s:', level_index)
    self.time = float(line[time_index_start:time_index_end])

    filename_end = line.find(':', time_index_end + 3)
    self.filename = line[time_index_end + 3:filename_end]

    linenumber_end = line.find(':', filename_end + 2)
    self.linenumber = int(line[filename_end + 2:linenumber_end])

    self.msg = line[linenumber_end+2:]

  def __str__(self):
    """Formats the data cleanly."""
    return '%s(%d)(%d): %s at %fs: %s: %d: %s' % (self.name, self.pid, self.msg_index, self.level, self.time, self.filename, self.linenumber, self.msg)

  def __JsonizeTokenArray(self, sub_array, tokens, token_index):
    """Parses an array from the provided tokens.

    Args:
      sub_array: list, The list to stick the elements in.
      tokens: list of strings, The list with all the tokens in it.
      token_index: int, Where to start in the token list.

    Returns:
      int, The last token used.
    """
    # Make sure the data starts with a '['
    if tokens[token_index] != '[':
      print(tokens)
      print('Expected [ at beginning, found', tokens[token_index + 1])
      return None

    # Eat the '['
    token_index += 1

    # Loop through the tokens.
    while token_index < len(tokens):
      if tokens[token_index + 1] == ',':
        # Next item is a comma, so we should just add the element.
        sub_array.append(tokens[token_index])
        token_index += 2
      elif tokens[token_index + 1] == ']':
        # Next item is a ']', so we should just add the element and finish.
        sub_array.append(tokens[token_index])
        token_index += 1
        return token_index
      else:
        # Otherwise, it must be a sub-message.
        sub_json = dict()
        token_index = self.JsonizeTokens(sub_json, tokens, token_index + 1)
        sub_array.append(sub_json)
        if tokens[token_index] == ',':
          # Handle there either being another data element.
          token_index += 1
        elif tokens[token_index] == ']':
          # Handle the end of the array.
          return token_index
        else:
          print('Unexpected ', tokens[token_index])
          return None

    print('Unexpected end')
    return None

  def JsonizeTokens(self, json, tokens, token_index):
    """Creates a json-like dictionary from the provided tokens.

    Args:
      json: dict, The dict to stick the elements in.
      tokens: list of strings, The list with all the tokens in it.
      token_index: int, Where to start in the token list.

    Returns:
      int, The last token used.
    """
    # Check that the message starts with a {
    if tokens[token_index] != '{':
      print(tokens)
      print('Expected { at beginning, found', tokens[token_index])
      return None

    # Eat the {
    token_index += 1

    # States and state variable for parsing elements.
    STATE_INIT = 'init'
    STATE_HAS_NAME = 'name'
    STATE_HAS_COLON = 'colon'
    STATE_EXPECTING_SUBMSG = 'submsg'
    STATE_EXPECTING_COMMA = 'comma'
    parser_state = STATE_INIT

    while token_index < len(tokens):
      if tokens[token_index] == '}':
        # Finish if there is a }
        return token_index + 1
      elif tokens[token_index] == '{':
        if parser_state != STATE_EXPECTING_SUBMSG:
          print(tokens)
          print(parser_state)
          print('Bad input, was not expecting {')
          return None
        # Found a submessage, parse it.
        sub_json = dict()
        token_index = self.JsonizeTokens(sub_json, tokens, token_index)
        json[token_name] = sub_json
        parser_state = STATE_EXPECTING_COMMA
      else:
        if parser_state == STATE_INIT:
          # This token is the name.
          token_name = tokens[token_index]
          parser_state = STATE_HAS_NAME
        elif parser_state == STATE_HAS_NAME:
          if tokens[token_index] != ':':
            print(tokens)
            print(parser_state)
            print('Bad input, found', tokens[token_index], 'expected :')
            return None
          # After a name, comes a :
          parser_state = STATE_HAS_COLON
        elif parser_state == STATE_HAS_COLON:
          # After the colon, figure out what is next.
          if tokens[token_index] == '[':
            # Found a sub-array!
            sub_array = []
            token_index = self.__JsonizeTokenArray(sub_array, tokens, token_index)
            json[token_name] = sub_array
            parser_state = STATE_EXPECTING_COMMA
          elif tokens[token_index + 1] == '{':
            # Found a sub-message, trigger parsing it.
            parser_state = STATE_EXPECTING_SUBMSG
          else:
            # This is just an element, move on.
            json[token_name] = tokens[token_index]
            parser_state = STATE_EXPECTING_COMMA
        elif parser_state == STATE_EXPECTING_COMMA:
          # Complain if there isn't a comma here.
          if tokens[token_index] != ',':
            print(tokens)
            print(parser_state)
            print('Bad input, found', tokens[token_index], 'expected ,')
            return None
          parser_state = STATE_INIT
        else:
          print('Bad parser state')
          return None
        token_index += 1

    print('Unexpected end')
    return None

  def ParseStruct(self):
    """Parses the message as a structure.

    Returns:
      struct_name, struct_type, json dict.
    """
    struct_name_index = self.msg.find(':')
    struct_name = self.msg[0:struct_name_index]

    struct_body = self.msg[struct_name_index+2:]
    tokens = []
    this_token = ''
    # For the various deliminators, append what we have found so far to the
    # list and the token.
    for char in struct_body:
      if char == '{':
        if this_token:
          tokens.append(this_token)
          this_token = ''
        tokens.append('{')
      elif char == '}':
        if this_token:
          tokens.append(this_token)
          this_token = ''
        tokens.append('}')
      elif char == '[':
        if this_token:
          tokens.append(this_token)
          this_token = ''
        tokens.append('[')
      elif char == ']':
        if this_token:
          tokens.append(this_token)
          this_token = ''
        tokens.append(']')
      elif char == ':':
        if this_token:
          tokens.append(this_token)
          this_token = ''
        tokens.append(':')
      elif char == ',':
        if this_token:
          tokens.append(this_token)
          this_token = ''
        tokens.append(',')
      elif char == ' ':
        if this_token:
          tokens.append(this_token)
          this_token = ''
      else:
        this_token += char
    if this_token:
      tokens.append(this_token)

    struct_type = tokens[0]
    json = dict()
    # Now that we have tokens, parse them.
    self.JsonizeTokens(json, tokens, 1)

    return (struct_name, struct_type, json)


def ParseLine(line):
  return LogEntry(line)

if __name__ == '__main__':
  print('motor_writer(2240)(07421): DEBUG   at 0000000819.99620s: ../../frc971/output/motor_writer.cc: 105: sending: .aos.controls.OutputCheck{pwm_value:221, pulse_length:2.233333}')
  line = ParseLine('motor_writer(2240)(07421): DEBUG   at 0000000819.99620s: ../../frc971/output/motor_writer.cc: 105: sending: .aos.controls.OutputCheck{pwm_value:221, pulse_length:2.233333}')
  if '.aos.controls.OutputCheck' in line.msg:
    print(line)
    print(line.ParseStruct())

  line = ParseLine('claw(2263)(19404): DEBUG   at 0000000820.00000s: ../../aos/common/controls/control_loop-tmpl.h: 104: position: .frc971.control_loops.ClawGroup.Position{top:.frc971.control_loops.HalfClawPosition{position:1.672153, front:.frc971.HallEffectStruct{current:f, posedge_count:0, negedge_count:52}, calibration:.frc971.HallEffectStruct{current:f, posedge_count:6, negedge_count:13}, back:.frc971.HallEffectStruct{current:f, posedge_count:0, negedge_count:62}, posedge_value:0.642681, negedge_value:0.922207}, bottom:.frc971.control_loops.HalfClawPosition{position:1.353539, front:.frc971.HallEffectStruct{current:f, posedge_count:2, negedge_count:150}, calibration:.frc971.HallEffectStruct{current:f, posedge_count:8, negedge_count:18}, back:.frc971.HallEffectStruct{current:f, posedge_count:0, negedge_count:6}, posedge_value:0.434514, negedge_value:0.759491}}')
  print(line.ParseStruct())

  line = ParseLine('joystick_proxy(2255)(39560): DEBUG   at 0000000820.00730s: ../../aos/prime/input/joystick_input.cc: 61: sending: .aos.RobotState{joysticks:[.aos.Joystick{buttons:0, axis:[0.000000, 1.000000, 1.000000, 0.000000]}, .aos.Joystick{buttons:0, axis:[-0.401575, 1.000000, -1.007874, 0.000000]}, .aos.Joystick{buttons:0, axis:[0.007874, 0.000000, 1.000000, -1.007874]}, .aos.Joystick{buttons:0, axis:[0.000000, 0.000000, 0.000000, 0.000000]}], test_mode:f, fms_attached:f, enabled:T, autonomous:f, team_id:971, fake:f}')
  print(line.ParseStruct())
