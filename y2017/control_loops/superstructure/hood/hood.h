#ifndef Y2017_CONTROL_LOOPS_SUPERSTRUCTURE_HOOD_HOOD_H_
#define Y2017_CONTROL_LOOPS_SUPERSTRUCTURE_HOOD_HOOD_H_

#include "frc971/control_loops/profiled_subsystem.h"
#include "y2017/control_loops/superstructure/superstructure.q.h"

namespace y2017 {
namespace control_loops {
namespace superstructure {
namespace hood {

// Profiled subsystem class with significantly relaxed limits while zeroing.  We
// need relaxed limits, because if you start at the top of the range, you need
// to go to -range, and if you start at the bottom of the range, you need to go
// to +range.  The standard subsystem doesn't support that.
class IndexPulseProfiledSubsystem
    : public ::frc971::control_loops::SingleDOFProfiledSubsystem<
          ::frc971::zeroing::PulseIndexZeroingEstimator> {
 public:
  IndexPulseProfiledSubsystem();

 private:
  void CapGoal(const char *name, Eigen::Matrix<double, 3, 1> *goal) override;
};

class Hood {
 public:
  Hood();
  double goal(int row, int col) const {
    return profiled_subsystem_.goal(row, col);
  }

  // The zeroing and operating voltages.
  static constexpr double kZeroingVoltage = 2.0;
  static constexpr double kOperatingVoltage = 3.0;

  void Iterate(const control_loops::HoodGoal *unsafe_goal,
               const ::frc971::IndexPosition *position, double *output,
               ::frc971::control_loops::IndexProfiledJointStatus *status);

  void Reset();

  enum class State : int32_t {
    UNINITIALIZED,
    DISABLED_INITIALIZED,
    ZEROING,
    RUNNING,
    ESTOP,
  };

  State state() const { return state_; }

 private:
  State state_;

  IndexPulseProfiledSubsystem profiled_subsystem_;
};

}  // namespace hood
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2017

#endif  // Y2017_CONTROL_LOOPS_SUPERSTRUCTURE_HOOD_HOOD_H_
