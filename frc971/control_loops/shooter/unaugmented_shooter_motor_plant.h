#ifndef FRC971_CONTROL_LOOPS_SHOOTER_UNAUGMENTED_SHOOTER_MOTOR_PLANT_H_
#define FRC971_CONTROL_LOOPS_SHOOTER_UNAUGMENTED_SHOOTER_MOTOR_PLANT_H_

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<2, 1, 1> MakeRawShooterPlantCoefficients();

StateFeedbackController<2, 1, 1> MakeRawShooterController();

StateFeedbackPlant<2, 1, 1> MakeRawShooterPlant();

StateFeedbackLoop<2, 1, 1> MakeRawShooterLoop();

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_SHOOTER_UNAUGMENTED_SHOOTER_MOTOR_PLANT_H_
