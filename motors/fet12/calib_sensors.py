#!/usr/bin/python3
import sys
import numpy as np

# Note on associated data files:
# calib_data_60*.csv has each output channel set at a constant value of 60.
# calib_data_6030*.csv actuates two channels.

def calibrate(fnames):
  """Do fitting to calibrate ADC data given csv files.

  CSVs should be of format:
  command_a, command_b, command_c, reading0, reading1, reading2
  The command columns are the on-time for each timer in FTM ticks.
  The reading columns in this case are the 3 samples taken from the
  ADC (with each pair corresponding to the same measurement pre-averaged). We
  only care about the averaged samples because otherwise the solution matrix
  can't be solved for in a stable manner.
  """
  data = np.zeros((1, 6))
  for fname in fnames:
    data = np.vstack((data, np.genfromtxt(fname, delimiter=',')))
  data = data[1:, :]

  data = data[:, :6]

  b = data[:, 0:3]
  b = b - np.tile(np.mean(b, axis=1), (3, 1)).T
  # Vcc / 3000 / R
  # 3000 converts duty cycle in FTM ticks to fraction of full.
  b *= 20.9 / 3000.0 / 0.0079
  A = data[:, 3:]

  return np.linalg.lstsq(A, b[:])[0].T

if __name__ == "__main__":
  if len(sys.argv) < 2:
    print("Need filenames for data")
    sys.exit(1)
  print(calibrate(sys.argv[1:]))
