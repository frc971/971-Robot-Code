#ifndef FRC971_CONTROL_LOOPS_INDEX_MOTOR_PLANT_H_
#define FRC971_CONTROL_LOOPS_INDEX_MOTOR_PLANT_H_

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlant<2, 1, 1> MakeIndexPlant();

StateFeedbackLoop<2, 1, 1> MakeIndexLoop();

}  // namespace frc971
}  // namespace control_loops

#endif  // FRC971_CONTROL_LOOPS_INDEX_MOTOR_PLANT_H_
