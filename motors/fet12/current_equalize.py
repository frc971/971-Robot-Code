#!/usr/bin/python3

import numpy
import sys
import calib_sensors

def manual_calibrate():
    #  38  27 -84
    #  36 -64  39
    # -74  21  35
    Is0 = numpy.matrix([[38.0, 27.0, -84.0]]).T
    Is1 = numpy.matrix([[36.0, -64.0, 39.0]]).T
    Is2 = numpy.matrix([[-74.0, 21.0, 35.0]]).T
    Is = numpy.matrix(numpy.hstack((Is0, Is1, Is2)))

    current = 46.0
    I = numpy.matrix([[current, -current / 2.0, -current / 2.0],
                      [-current / 2.0, current, -current / 2.0],
                      [-current / 2.0, -current / 2.0, current]])
    transform = I * numpy.linalg.inv(Is)
    return transform

def main():
    transform = manual_calibrate()

    if len(sys.argv) > 1:
      transform = calib_sensors.calibrate(sys.argv[1:])

    print("#ifndef MOTORS_FET12_CURRENT_MATRIX_")
    print("#define MOTORS_FET12_CURRENT_MATRIX_")
    print("")
    print("#include <array>")
    print("")
    print("namespace frc971 {")
    print("namespace motors {")
    print("")
    print(
        "inline ::std::array<float, 3> DecoupleCurrents(int16_t currents[3]) {")
    print("  ::std::array<float, 3> ans;")

    for i in range(3):
        print("  ans[%d] = %ff * static_cast<float>(currents[0]) +" %
              (i, transform[i, 0]))
        print("      %ff * static_cast<float>(currents[1]) +" %
              transform[i, 1])
        print("      %ff * static_cast<float>(currents[2]);" % transform[i, 2])

    print("  return ans;")
    print("}")
    print("")
    print("}  // namespace motors")
    print("}  // namespace frc971")
    print("#endif  // MOTORS_FET12_CURRENT_MATRIX_")

    return 0

if __name__ == '__main__':
    sys.exit(main())
