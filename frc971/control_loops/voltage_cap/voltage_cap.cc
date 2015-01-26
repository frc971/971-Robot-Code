#include "voltage_cap.h"

#include <limits>

namespace frc971 {
namespace control_loops {

void VoltageCap(double max_voltage, double voltage_one, double voltage_two,
                double *out_voltage_one, double *out_voltage_two) {
  *out_voltage_one = *out_voltage_two =
      ::std::numeric_limits<double>::quiet_NaN();
  if (voltage_one <= max_voltage && voltage_two <= max_voltage &&
      voltage_one >= -max_voltage && voltage_two >= -max_voltage) {
    *out_voltage_one = voltage_one;
    *out_voltage_two = voltage_two;
  } else if (voltage_one - voltage_two > 2 * max_voltage) {
    //(voltage_one is larger)
    // if the difference between the two voltages is more than 24 we know a
    // 45degree slope will not intersect the 'box'
    *out_voltage_one = max_voltage;
    *out_voltage_two = -max_voltage;
    // top left corner of 'box'
  } else if (voltage_one - voltage_two < -2 * max_voltage) {
    //(voltage_two is larger)
    // if the difference between the two voltages is less than -24 we know a
    //  45degree slope will not intersect the 'box'
    *out_voltage_one = -max_voltage;
    *out_voltage_two = max_voltage;
    // bottom right corner of 'box'
  } else {
    if (voltage_one >= 0) {
      if (voltage_two >= 0) {
        // Quadrant 1
        if (voltage_one > voltage_two) {
          // Above box
          double difference = voltage_one - voltage_two;
          *out_voltage_one = max_voltage;
          *out_voltage_two = max_voltage - difference;
        } else {
          // Right of box
          double difference = voltage_two - voltage_one;
          *out_voltage_one = max_voltage - difference;
          *out_voltage_two = max_voltage;
        }
      } else {  // voltage_two < 0
        // Quadrant 2
        double difference = voltage_one - voltage_two;
        if (-voltage_two > voltage_one) {
          // Left of box
          *out_voltage_one = -max_voltage + difference;
          *out_voltage_two = -max_voltage;
        } else {
          // Above box
          *out_voltage_one = max_voltage;
          *out_voltage_two = max_voltage - difference;
        }
      }
    } else {  // voltage_one < 0
      if (voltage_two <= 0) {
        // Quadrant 3
        if (voltage_one < voltage_two) {
          // Left of Box
          double difference = voltage_two - voltage_one;
          *out_voltage_one = -max_voltage;
          *out_voltage_two = -max_voltage + difference;
        } else {
          // Under box
          double difference = voltage_one - voltage_two;
          *out_voltage_one = -max_voltage + difference;
          *out_voltage_two = -max_voltage;
        }
      } else {  // voltage_two > 0
        // Quadrant 4
        double difference = voltage_two - voltage_one;
        if (-voltage_one > voltage_two) {
          // Right of box
          *out_voltage_one = -max_voltage;
          *out_voltage_two = -max_voltage + difference;
        } else {
          // Under box
          *out_voltage_one = max_voltage - difference;
          *out_voltage_two = max_voltage;
        }
      }
    }
  }
}

void VoltageCap(double voltage_one, double voltage_two, double *out_voltage_one,
                double *out_voltage_two) {
  VoltageCap(12.0, voltage_one, voltage_two, out_voltage_one, out_voltage_two);
}

}  // namespace control_loops
}  // namespace frc971
