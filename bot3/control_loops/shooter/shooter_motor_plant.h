#ifndef BOT3_CONTROL_LOOPS_SHOOTER_SHOOTER_MOTOR_PLANT_H_
#define BOT3_CONTROL_LOOPS_SHOOTER_SHOOTER_MOTOR_PLANT_H_

#include "frc971/control_loops/state_feedback_loop.h"

namespace bot3 {
namespace control_loops {

StateFeedbackPlantCoefficients<2, 1, 1> MakeShooterPlantCoefficients();

StateFeedbackController<2, 1, 1> MakeShooterController();

StateFeedbackPlant<2, 1, 1> MakeShooterPlant();

StateFeedbackLoop<2, 1, 1> MakeShooterLoop();

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_SHOOTER_SHOOTER_MOTOR_PLANT_H_
