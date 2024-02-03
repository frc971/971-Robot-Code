#ifndef FRC971_CONTROL_LOOPS_DOUBLE_JOINTED_ARM_DEMO_PATH_H_
#define FRC971_CONTROL_LOOPS_DOUBLE_JOINTED_ARM_DEMO_PATH_H_

#include <memory>

#include "frc971/control_loops/double_jointed_arm/trajectory.h"

namespace frc971::control_loops::arm {

::std::unique_ptr<Path> MakeDemoPath();
::std::unique_ptr<Path> MakeReversedDemoPath();

}  // namespace frc971::control_loops::arm

#endif  // FRC971_CONTROL_LOOPS_DOUBLE_JOINTED_ARM_DEMO_PATH_H_
