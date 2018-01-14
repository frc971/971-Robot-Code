#!/usr/bin/python

import sys
from matplotlib import pylab

# TODO(austin): Plot flag.

def main(argv):
  if len(argv) < 4:
    print 'Args: input output.cc struct_name'
    return 1
  data_sum = [0.0] * 4096
  data_count = [0] * 4096
  data_list_absolute = []
  data_list_current = []

  with open(argv[1], 'r') as fd:
    for line in fd:
      if line.startswith('reading'):
        split_line = line.split()
        data_absolute = int(split_line[1])
        data_index = int(split_line[3][2:])
        data_current = int(split_line[2]) / 10000.0
        data_sum[data_index] += data_current
        data_count[data_index] += 1
        data_list_absolute.append(data_absolute)
        data_list_current.append(data_current)
  data = [0.0] * 4096
  min_zero = 4096
  max_zero = 0
  for i in xrange(0, 4096):
    if data_count[i] == 0:
      min_zero = min(i, min_zero)
      max_zero = max(i, min_zero)

  for i in xrange(0, 4096):
    if data_count[i] != 0:
      data[i] = data_sum[i] / data_count[i]
  if min_zero == 0 and max_zero == 4095:
    for i in xrange(0, 4096):
      if data_count[i] != 0:
        while i > 0:
          data[i - 1] = data[i]
          i -= 1
        break;

    for i in reversed(xrange(0, 4096)):
      if data_count[i] != 0:
        while i < 4095:
          data[i + 1] = data[i]
          i += 1
        break;
  else:
    for i in xrange(0, 4096):
      if data_count[i] == 0:
        if i < (min_zero + max_zero) / 2:
          data[i] = data[min_zero - 1]
        else:
          data[i] = data[max_zero + 1]

  pylab.plot(range(0, 4096), data)
  pylab.figure()
  pylab.plot(data_list_absolute, data_list_current)
  pylab.show()
  with open(argv[2], 'w') as out_fd:
    out_fd.write('extern const float %s[4096];\n' % argv[3])
    out_fd.write('const float %s[4096] = {\n' % argv[3])
    for datapoint in data:
      out_fd.write('    %ff,\n' % datapoint)
    out_fd.write('};')

  return 0

if __name__ == '__main__':
  sys.exit(main(sys.argv))
