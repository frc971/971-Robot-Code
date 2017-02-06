#ifndef Y2017_CONTROL_LOOPS_SUPERSTRUCTURE_HOOD_HOOD_H_
#define Y2017_CONTROL_LOOPS_SUPERSTRUCTURE_HOOD_HOOD_H_

#include "frc971/control_loops/profiled_subsystem.h"
#include "y2017/control_loops/superstructure/superstructure.q.h"

namespace y2017 {
namespace control_loops {
namespace superstructure {
namespace hood {

class Hood {
 public:
   Hood();
   double goal(int row, int col) const {
     return profiled_subsystem_.goal(row, col);
   }

   // The zeroing and operating voltages.
   static constexpr double kZeroingVoltage = 2.5;
   static constexpr double kOperatingVoltage = 12.0;

   void Iterate(const control_loops::HoodGoal *unsafe_goal,
                const ::frc971::PotAndIndexPosition *position, double *output,
                ::frc971::control_loops::ProfiledJointStatus *status);

   void Reset();

   enum class State : int32_t{
     UNINITIALIZED,
     DISABLED_INITIALIZED,
     ZEROING,
     RUNNING,
     ESTOP,
   };

   State state() const { return state_; }

  private:
   State state_;

   ::frc971::control_loops::SingleDOFProfiledSubsystem<> profiled_subsystem_;
};

}  // namespace hood
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2017

#endif  // Y2017_CONTROL_LOOPS_SUPERSTRUCTURE_HOOD_HOOD_H_
