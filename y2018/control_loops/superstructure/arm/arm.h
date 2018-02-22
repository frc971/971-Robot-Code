#ifndef Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_ARM_ARM_H_
#define Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_ARM_ARM_H_

#include "frc971/zeroing/zeroing.h"
#include "y2018/constants.h"
#include "y2018/control_loops/superstructure/arm/dynamics.h"
#include "y2018/control_loops/superstructure/arm/ekf.h"
#include "y2018/control_loops/superstructure/arm/graph.h"
#include "y2018/control_loops/superstructure/arm/trajectory.h"
#include "y2018/control_loops/superstructure/superstructure.q.h"

namespace y2018 {
namespace control_loops {
namespace superstructure {
namespace arm {

struct TrajectoryPair {
  TrajectoryPair(::std::unique_ptr<Path> forwards_path,
                 ::std::unique_ptr<Path> backwards_path, double step_size)
      : forwards(::std::move(forwards_path), step_size),
        backwards(::std::move(backwards_path), step_size) {}

  Trajectory forwards;
  Trajectory backwards;
};

class Arm {
 public:
  Arm();

  // The operating voltage.
  static constexpr double kOperatingVoltage() { return 12.0; }
  static constexpr double kDt() { return 0.00505; }
  static constexpr double kAlpha0Max() { return 25.0; }
  static constexpr double kAlpha1Max() { return 25.0; }
  static constexpr double kVMax() { return 11.0; }
  static constexpr double kPathlessVMax() { return 4.0; }

  void Iterate(const uint32_t *unsafe_goal,
               const control_loops::ArmPosition *position,
               double *proximal_output, double *distal_output,
               bool *release_arm_brake,
               control_loops::ArmStatus *status);

  void Reset();

  enum class State : int32_t {
    UNINITIALIZED,
    ZEROING,
    RUNNING,
    ESTOP,
  };

  State state() const { return state_; }

 private:
  State state_ = State::UNINITIALIZED;

  ::aos::monotonic_clock::time_point last_time_ =
      ::aos::monotonic_clock::min_time;

  ::frc971::zeroing::PotAndAbsEncoderZeroingEstimator proximal_zeroing_estimator_;
  ::frc971::zeroing::PotAndAbsEncoderZeroingEstimator distal_zeroing_estimator_;

  double proximal_offset_ = 0.0;
  double distal_offset_ = 0.0;

  const ::Eigen::Matrix<double, 2, 2> alpha_unitizer_;

  ::std::vector<TrajectoryPair> trajectories_;
  SearchGraph search_graph_;

  bool close_enough_for_full_power_ = false;

  EKF arm_ekf_;
  TrajectoryFollower follower_;

  // Start at the 0th index.
  uint32_t current_node_ = 0;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

}  // namespace arm
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2018

#endif  // Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_ARM_ARM_H_
