#ifndef BOT3_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_MOTOR_PLANT_H_
#define BOT3_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_MOTOR_PLANT_H_

#include "frc971/control_loops/state_feedback_loop.h"

namespace bot3 {
namespace control_loops {

StateFeedbackPlantCoefficients<4, 2, 2> MakeDrivetrainPlantCoefficients();

StateFeedbackController<4, 2, 2> MakeDrivetrainController();

StateFeedbackPlant<4, 2, 2> MakeDrivetrainPlant();

StateFeedbackLoop<4, 2, 2> MakeDrivetrainLoop();

}  // namespace control_loops
}  // namespace bot3

#endif  // BOT3_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_MOTOR_PLANT_H_
