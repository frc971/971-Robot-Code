#!/usr/bin/python3

# Pipe the binary data in and give the CSV output filename as an argument.

import struct
import sys
import select

DatapointStruct = struct.Struct('<11h')
DATAPOINTS = 5000
TOTAL_SIZE = DatapointStruct.size * DATAPOINTS

data = bytes()
while len(data) < TOTAL_SIZE:
  read_now = sys.stdin.buffer.read(TOTAL_SIZE - len(data))
  if not read_now:
    print('EOF before data finished', file=sys.stderr)
    sys.exit(1)
  data += read_now
print('%s' % len(data))

readable, _, _ = select.select([sys.stdin.buffer], [], [], 1)
if readable:
  print('Extra bytes', file=sys.stderr)
  sys.exit(1)

decoded = []
for i in range(DATAPOINTS):
  datapoint = DatapointStruct.unpack_from(data, i * DatapointStruct.size)
  decoded.append(datapoint)

def current(reading, ref):
  reading_voltage = reading / 4096 * 3.3  / 1.47 * (0.768 + 1.47)
  reading_voltage = reading / 4096 * 3.3  / 18.0 * (18.0 + 10.0)
  #reading_ref = ref / 4096 * 3.3
  reading_ref = 2.5
  #reading_ref = 0
  #return (reading_voltage - reading_ref) / 50 / 0.0003
  return (reading_voltage - reading_ref) / 0.195

with open(sys.argv[1], 'w') as out:
  out.write('balanced0,balanced1,balanced2,current0.0,current1.0,current2.0,current0.1,current1.1,current2.1,count\n')
  #for point in decoded[2000:7200]:
  for point in decoded:
    out.write(','.join(str(d) for d in (
        current(point[0], point[6]),
        current(point[1], point[6]),
        current(point[2], point[6]),
        current(point[3], point[6]),
        current(point[4], point[6]),
        current(point[5], point[6]),
        current(point[6], point[6]),
        current(point[7], point[6]),
        current(point[8], point[6]),
        #point[6] / 100.0,
        #point[7] / 100.0,
        #point[8] / 100.0,
        point[9] / 100.0,
        point[10] / 100.0,
        )) + '\n')

print('all done')
