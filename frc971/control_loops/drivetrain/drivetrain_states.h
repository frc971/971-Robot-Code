#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_STATES_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_STATES_H_

namespace frc971::control_loops ::drivetrain {

enum KalmanState {
  kLeftPosition = 0,
  kLeftVelocity = 1,
  kRightPosition = 2,
  kRightVelocity = 3,
  kLeftError = 4,
  kRightError = 5,
  kAngularError = 6
};

enum OutputState { kLeftVoltage = 0, kRightVoltage = 1 };

}  // namespace frc971::control_loops::drivetrain

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_STATES_H_
