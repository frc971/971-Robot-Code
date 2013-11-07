#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_POLYDRIVETRAIN_CLUTCH_MOTOR_PLANT_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_POLYDRIVETRAIN_CLUTCH_MOTOR_PLANT_H_

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<2, 2, 2> MakeClutchVelocityDrivetrainLowLowPlantCoefficients();

StateFeedbackController<2, 2, 2> MakeClutchVelocityDrivetrainLowLowController();

StateFeedbackPlantCoefficients<2, 2, 2> MakeClutchVelocityDrivetrainLowHighPlantCoefficients();

StateFeedbackController<2, 2, 2> MakeClutchVelocityDrivetrainLowHighController();

StateFeedbackPlantCoefficients<2, 2, 2> MakeClutchVelocityDrivetrainHighLowPlantCoefficients();

StateFeedbackController<2, 2, 2> MakeClutchVelocityDrivetrainHighLowController();

StateFeedbackPlantCoefficients<2, 2, 2> MakeClutchVelocityDrivetrainHighHighPlantCoefficients();

StateFeedbackController<2, 2, 2> MakeClutchVelocityDrivetrainHighHighController();

StateFeedbackPlant<2, 2, 2> MakeVClutchDrivetrainPlant();

StateFeedbackLoop<2, 2, 2> MakeVClutchDrivetrainLoop();

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_POLYDRIVETRAIN_CLUTCH_MOTOR_PLANT_H_
