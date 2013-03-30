#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_MOTOR_PLANT_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_MOTOR_PLANT_H_

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlant<4, 2, 2> MakeDrivetrainPlant();

StateFeedbackLoop<4, 2, 2> MakeDrivetrainLoop();

}  // namespace frc971
}  // namespace control_loops

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_MOTOR_PLANT_H_
