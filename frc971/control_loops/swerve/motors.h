#ifndef FRC971_CONTROL_LOOPS_SWERVE_MOTORS_
#define FRC971_CONTROL_LOOPS_SWERVE_MOTORS_

#include <numbers>

namespace frc971::control_loops::swerve {

// Class holding the physical parameters for a motor.
struct Motor {
  constexpr Motor(double stall_torque, double stall_current, double free_speed,
                  double free_current, double motor_inertia)
      : stall_torque(stall_torque),
        stall_current(stall_current),
        free_speed(free_speed),
        free_current(free_current),
        resistance(12 / stall_current),
        Kv(free_speed / (12 - resistance * free_current)),
        Kt(stall_torque / stall_current),
        motor_inertia(motor_inertia) {}
  // Stall Torque in Nm
  double stall_torque;
  // Stall Current in Amps
  double stall_current;
  // Free Speed in rad / sec
  double free_speed;
  // Free Current in Amps
  double free_current;
  // Resistance of the motor, divided by 2 to account for the 2 motors
  double resistance;
  // Motor velocity constant
  double Kv;
  // Torque constant
  double Kt;
  // Motor inertia in kg m^2
  // Diameter of 1.9", weight of: 100 grams
  // TODO(Filip/Justin): Update motor inertia for Kraken, currently using Falcon
  // motor inertia
  double motor_inertia;
};

// Struct representing the WCP Kraken X60 motor using
// Field Oriented Controls (FOC) communication.
//
// All numbers based on data from
// https://wcproducts.com/products/kraken.
constexpr Motor KrakenFOC() {
  return Motor{9.37, 483.0, 5800.0 / 60.0 * 2.0 * std::numbers::pi, 2.0,
               0.1 * (0.95 * 0.0254) * (0.95 * 0.0254)};
};
}  // namespace frc971::control_loops::swerve

#endif  // FRC971_CONTROL_LOOPS_SWERVE_MOTORS_
