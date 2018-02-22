#ifndef Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_ARM_DEMO_PATH_H_
#define Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_ARM_DEMO_PATH_H_

#include <memory>

#include "y2018/control_loops/superstructure/arm/trajectory.h"

namespace y2018 {
namespace control_loops {
namespace superstructure {
namespace arm {

::std::unique_ptr<Path> MakeDemoPath();
::std::unique_ptr<Path> MakeReversedDemoPath();

}  // namespace arm
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2018

#endif  // Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_ARM_DEMO_PATH_H_
