#!/usr/bin/python3

# This is a program to parse output from the ITM and DWT.
# The "Debug ITM and DWT Packet Protocol" section of the ARMv7-M Architecture
# Reference Manual is a good reference.
#
# This seems like it might be a poster child for using coroutines, but those
# look scary so we're going to stick with generators.

import io
import os
import sys

def open_file_for_bytes(path):
  '''Returns a file-like object which reads bytes without buffering.'''
  # Not using `open` because it's unclear from the docs how (if it's possible at
  # all) to get something that will only do one read call and return what that
  # gets on a fifo.
  try:
    return io.FileIO(path, 'r')
  except FileNotFoundError:
    # If it wasn't found, try (once) to create it and then open again.
    try:
      os.mkfifo(path)
    except FileExistsError:
      pass
    return io.FileIO(path, 'r')

def read_bytes(path):
  '''Reads bytes from a file. This is appropriate both for regular files and
  fifos.
  Args:
    path: A path-like object to open.
  Yields:
    Individual bytes from the file, until hitting EOF.
  '''
  with open_file_for_bytes(path) as f:
    while True:
      buf = f.read(1024)
      if not buf:
        return
      for byte in buf:
        yield byte

def parse_packets(source):
  '''Parses a stream of bytes into packets.
  Args:
    source: A generator of individual bytes.
  Generates:
    Packets as bytes objects.
  '''
  try:
    while True:
      header = next(source)
      if header == 0:
        # Synchronization packets consist of a bunch of 0 bits (not necessarily
        # a whole number of bytes), followed by a 128 byte. This is for hardware
        # to synchronize on, but we're not in a position to do that, so
        # presumably those should get filtered out before getting here?
        raise 'Not sure how to handle synchronization packets'
      packet = bytearray()
      packet.append(header)
      header_size = header & 3
      if header_size == 0:
        while packet[-1] & 128 and len(packet) < 7:
          packet.append(next(source))
      else:
        if header_size == 3:
          header_size = 4
        for _ in range(header_size):
          packet.append(next(source))
      yield bytes(packet)
  except StopIteration:
    return

class PacketParser(object):
  def __init__(self):
    self.stimulus_handlers = {}

  def register_stimulus_handler(self, port_number, handler):
    '''Registers a function to call on packets to the specified port.'''
    self.stimulus_handlers[port_number] = handler

  def process(self, path):
    for packet in parse_packets(read_bytes(path)):
      header = packet[0]
      header_size = header & 3
      if header_size == 0:
        # TODO(Brian): At least handle overflow packets here.
        pass
      else:
        port_number = header >> 3
        if port_number in self.stimulus_handlers:
          self.stimulus_handlers[port_number](packet[1:])
        else:
          print('Warning: unhandled stimulus port %d' % port_number,
                file=sys.stderr)
          self.stimulus_handlers[port_number] = lambda _: None

if __name__ == '__main__':
  parser = PacketParser()
  def print_byte(payload):
    sys.stdout.write(payload.decode('ascii'))
  parser.register_stimulus_handler(0, print_byte)

  for path in sys.argv[1:]:
    parser.process(path)
