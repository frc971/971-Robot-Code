#!/usr/bin/python3
import sys
import numpy as np

# Note on associated data files:
# calib_data_60*.csv has each output channel set at a constant value of 60.
# calib_data_6030.csv actuates two channels.

def calibrate(fnames):
  """Do fitting to calibrate ADC data given csv files.

  CSVs should be of format:
  command_a, command_b, command_c, adc0, adc0, adc1, adc2, adc1, adc2
  Where The adc columns in this case are the 6 samples taken from the
  ADC where each pair of columns with the same name correspond with
  the same measurement (we average samples that are of the same value
  because otherwise the solution matrix can't be solved for in a stable
  manner).
  """
  data = np.zeros((1, 9))
  for fname in fnames:
    data = np.vstack((data, np.genfromtxt(fname, delimiter=',')))
  data = data[1:, :]

  if data.shape[1] == 9:
    data[:, 3] = (data[:, 3] + data[:, 4]) / 2.0
    data[:, 4] = (data[:, 5] + data[:, 7]) / 2.0
    data[:, 5] = (data[:, 6] + data[:, 8]) / 2.0
  data = data[:, :6]

  b = data[:, 0:3]
  b = b - np.tile(np.mean(b, axis=1), (3, 1)).T
  # Vcc / 3000 / R
  b *= 30.8 / 3000.0 / 0.0084
  A = data[:, 3:]

  return np.linalg.lstsq(A, b[:])[0].T

if __name__ == "__main__":
  if len(sys.argv) < 2:
    print("Need filenames for data")
    sys.exit(1)
  print(calibrate(sys.argv[1:]))
