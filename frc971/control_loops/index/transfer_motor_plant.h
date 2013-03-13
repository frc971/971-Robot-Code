#ifndef FRC971_CONTROL_LOOPS_INDEX_TRANSFER_MOTOR_PLANT_H_
#define FRC971_CONTROL_LOOPS_INDEX_TRANSFER_MOTOR_PLANT_H_

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<2, 1, 1> MakeTransferPlantCoefficients();

StateFeedbackController<2, 1, 1> MakeTransferController();

StateFeedbackPlant<2, 1, 1> MakeTransferPlant();

StateFeedbackLoop<2, 1, 1> MakeTransferLoop();

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_INDEX_TRANSFER_MOTOR_PLANT_H_
