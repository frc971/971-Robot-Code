#ifndef FRC971_CONTROL_LOOPS_VOLTAGE_CAP_VOLTAGE_CAP_H_
#define FRC971_CONTROL_LOOPS_VOLTAGE_CAP_VOLTAGE_CAP_H_

#include <stdio.h>

namespace frc971 {
namespace control_loops {

// This function maintains the difference of power between two voltages passed in
//   that are outside of our range of possible voltage output.
// This is because we figured that maintaining the difference rather than the ratio
//   between the voltages would get us to our goal as fast as possible.
//
//
// The 'box' is a box formed on a graph by the maximum and minimun voltages of
//   voltage_one and voltage_two with
//   voltage_one on the y-axis and voltage_two on the x-axis.
// If a line with a slope of one(45degrees) is plotted from the point formed
//   by graphing voltage_one(y) and voltage_two(x), the first intersecting point of
//   the box is the maximum voltage that we can output to get to the goal as
//   fast as possible.
// If the line does not intersect the box, then we use the closest corner of
//   the box to the line in either quadrant two or quadrant four of the graph.
void VoltageCap(double max_voltage, double voltage_one, double voltage_two,
                double *out_voltage_one, double *out_voltage_two);
  // Defaults to 12v if no voltage is specified.
void VoltageCap(double voltage_one, double voltage_two, double *out_voltage_one,
                double *out_voltage_two);

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_VOLTAGE_CAP_VOLTAGE_CAP_H_
