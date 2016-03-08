#!/usr/bin/python3

import re

"""
A regular expression to match the envelope part of the log entry.
Parsing of the JSON msg is handled elsewhere.
"""
LOG_RE = re.compile("""
  (.*?)              # 1 name
  \((\d+)\)          # 2 pid
  \((\d+)\)          # 3 message_index
  :\s
  (\w+?)             # 4 level
  \s+at\s+
  (\d+\.\d+)s        # 5 time
  :\s
  ([A-Za-z0-9_./-]+) # 6 filename
  :\s
  (\d+)              # 7 linenumber
  :\s
  (.*)               # 8 msg
  """, re.VERBOSE)

class LogEntry:
  """
  This class provides a way to parse log entries.
  The header portion of the log entry is parsed eagerly.
  The structured portion of a log entry is parsed on demand.
  """

  def __init__(self, line):
    """Populates a LogEntry from a line."""
    self.line = line
    m = LOG_RE.match(line)
    if m is None:
        print("LOG_RE failed on", line)
        return
    self.name = m.group(1)
    self.pid_index = int(m.group(2))
    self.msg_index = int(m.group(3))
    self.level = m.group(4)
    self.time = float(m.group(5))
    self.filename = m.group(6)
    self.linenumber = m.group(7)
    self.msg = m.group(8)
    self.struct_name = None

  def __str__(self):
    """Formats the data cleanly."""
    return '%s(%d)(%d): %s at %fs: %s: %d: %s' % (
        self.name, self.pid, self.msg_index, self.level, self.time, self.filename, self.linenumber, self.msg)

  def ParseStruct(self):
    """Parses the message as a structure.

    Returns:
      struct_name, struct_type, json dict.
    """
    if self.struct_name:
        # We've already parsed the structural part. Return the cached result
        return (self.struct_name, self.struct_type, self.struct_json)

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

    # Cache the result to avoid having to reparse.
    self.struct_name = struct_name
    self.struct_type = struct_type
    self.struct_json = json

    return (struct_name, struct_type, json)

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


if __name__ == '__main__':
  def ParseLine(line):
    return LogEntry(line)

  print('motor_writer(2240)(07421): DEBUG   at 0000000819.99620s: ../../frc971/output/motor_writer.cc: 105: sending: .aos.controls.OutputCheck{pwm_value:221, pulse_length:2.233333}')
  line = ParseLine('motor_writer(2240)(07421): DEBUG   at 0000000819.99620s: ../../frc971/output/motor_writer.cc: 105: sending: .aos.controls.OutputCheck{pwm_value:221, pulse_length:2.233333}')
  if '.aos.controls.OutputCheck' in line.msg:
    print(line)
    print(line.ParseStruct())

  line = ParseLine('claw(2263)(19404): DEBUG   at 0000000820.00000s: ../../aos/common/controls/control_loop-tmpl.h: 104: position: .frc971.control_loops.ClawGroup.Position{top:.frc971.control_loops.HalfClawPosition{position:1.672153, front:.frc971.HallEffectStruct{current:f, posedge_count:0, negedge_count:52}, calibration:.frc971.HallEffectStruct{current:f, posedge_count:6, negedge_count:13}, back:.frc971.HallEffectStruct{current:f, posedge_count:0, negedge_count:62}, posedge_value:0.642681, negedge_value:0.922207}, bottom:.frc971.control_loops.HalfClawPosition{position:1.353539, front:.frc971.HallEffectStruct{current:f, posedge_count:2, negedge_count:150}, calibration:.frc971.HallEffectStruct{current:f, posedge_count:8, negedge_count:18}, back:.frc971.HallEffectStruct{current:f, posedge_count:0, negedge_count:6}, posedge_value:0.434514, negedge_value:0.759491}}')
  print(line.ParseStruct())

  line = ParseLine('joystick_proxy(2255)(39560): DEBUG   at 0000000820.00730s: ../../aos/prime/input/joystick_input.cc: 61: sending: .aos.RobotState{joysticks:[.aos.Joystick{buttons:0, axis:[0.000000, 1.000000, 1.000000, 0.000000]}, .aos.Joystick{buttons:0, axis:[-0.401575, 1.000000, -1.007874, 0.000000]}, .aos.Joystick{buttons:0, axis:[0.007874, 0.000000, 1.000000, -1.007874]}, .aos.Joystick{buttons:0, axis:[0.000000, 0.000000, 0.000000, 0.000000]}], test_mode:f, fms_attached:f, enabled:T, autonomous:f, team_id:971, fake:f}')
  print(line.ParseStruct())
