#ifndef FRC971_CONTROL_LOOPS_SHOOTER_SHOOTER_MOTOR_PLANT_H_
#define FRC971_CONTROL_LOOPS_SHOOTER_SHOOTER_MOTOR_PLANT_H_

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {
static constexpr double kMaxExtension = 0.323850;

static constexpr double kSpringConstant = 2800.000000;


StateFeedbackPlantCoefficients<3, 1, 1> MakeSprungShooterPlantCoefficients();

StateFeedbackController<3, 1, 1> MakeSprungShooterController();

StateFeedbackPlantCoefficients<3, 1, 1> MakeShooterPlantCoefficients();

StateFeedbackController<3, 1, 1> MakeShooterController();

StateFeedbackPlant<3, 1, 1> MakeShooterPlant();

StateFeedbackLoop<3, 1, 1> MakeShooterLoop();

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_SHOOTER_SHOOTER_MOTOR_PLANT_H_
