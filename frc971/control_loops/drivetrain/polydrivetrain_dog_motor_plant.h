#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_POLYDRIVETRAIN_DOG_MOTOR_PLANT_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_POLYDRIVETRAIN_DOG_MOTOR_PLANT_H_

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<2, 2, 2> MakeDogVelocityDrivetrainLowLowPlantCoefficients();

StateFeedbackController<2, 2, 2> MakeDogVelocityDrivetrainLowLowController();

StateFeedbackPlantCoefficients<2, 2, 2> MakeDogVelocityDrivetrainLowHighPlantCoefficients();

StateFeedbackController<2, 2, 2> MakeDogVelocityDrivetrainLowHighController();

StateFeedbackPlantCoefficients<2, 2, 2> MakeDogVelocityDrivetrainHighLowPlantCoefficients();

StateFeedbackController<2, 2, 2> MakeDogVelocityDrivetrainHighLowController();

StateFeedbackPlantCoefficients<2, 2, 2> MakeDogVelocityDrivetrainHighHighPlantCoefficients();

StateFeedbackController<2, 2, 2> MakeDogVelocityDrivetrainHighHighController();

StateFeedbackPlant<2, 2, 2> MakeVDogDrivetrainPlant();

StateFeedbackLoop<2, 2, 2> MakeVDogDrivetrainLoop();

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_POLYDRIVETRAIN_DOG_MOTOR_PLANT_H_
