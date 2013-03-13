#ifndef FRC971_CONTROL_LOOPS_INDEX_INDEX_MOTOR_PLANT_H_
#define FRC971_CONTROL_LOOPS_INDEX_INDEX_MOTOR_PLANT_H_

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<2, 1, 1> MakeIndex0DiscPlantCoefficients();

StateFeedbackController<2, 1, 1> MakeIndex0DiscController();

StateFeedbackPlantCoefficients<2, 1, 1> MakeIndex1DiscPlantCoefficients();

StateFeedbackController<2, 1, 1> MakeIndex1DiscController();

StateFeedbackPlantCoefficients<2, 1, 1> MakeIndex2DiscPlantCoefficients();

StateFeedbackController<2, 1, 1> MakeIndex2DiscController();

StateFeedbackPlantCoefficients<2, 1, 1> MakeIndex3DiscPlantCoefficients();

StateFeedbackController<2, 1, 1> MakeIndex3DiscController();

StateFeedbackPlantCoefficients<2, 1, 1> MakeIndex4DiscPlantCoefficients();

StateFeedbackController<2, 1, 1> MakeIndex4DiscController();

StateFeedbackPlant<2, 1, 1> MakeIndexPlant();

StateFeedbackLoop<2, 1, 1> MakeIndexLoop();

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_INDEX_INDEX_MOTOR_PLANT_H_
