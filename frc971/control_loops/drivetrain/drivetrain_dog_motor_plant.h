#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_DOG_MOTOR_PLANT_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_DOG_MOTOR_PLANT_H_

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<4, 2, 2> MakeDogDrivetrainPlantCoefficients();

StateFeedbackController<4, 2, 2> MakeDogDrivetrainController();

StateFeedbackPlant<4, 2, 2> MakeDogDrivetrainPlant();

StateFeedbackLoop<4, 2, 2> MakeDogDrivetrainLoop();

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_DOG_MOTOR_PLANT_H_
