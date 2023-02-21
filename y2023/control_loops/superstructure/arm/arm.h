#ifndef Y2023_CONTROL_LOOPS_SUPERSTRUCTURE_ARM_ARM_H_
#define Y2023_CONTROL_LOOPS_SUPERSTRUCTURE_ARM_ARM_H_

#include "aos/time/time.h"
#include "frc971/control_loops/double_jointed_arm/dynamics.h"
#include "frc971/control_loops/double_jointed_arm/ekf.h"
#include "frc971/control_loops/double_jointed_arm/graph.h"
#include "frc971/control_loops/double_jointed_arm/trajectory.h"
#include "frc971/zeroing/zeroing.h"
#include "y2023/constants.h"
#include "y2023/control_loops/superstructure/arm/generated_graph.h"
#include "y2023/control_loops/superstructure/superstructure_position_generated.h"
#include "y2023/control_loops/superstructure/superstructure_status_generated.h"

using frc971::control_loops::arm::EKF;
using frc971::control_loops::arm::TrajectoryFollower;

namespace y2023 {
namespace control_loops {
namespace superstructure {
namespace arm {

class Arm {
 public:
  Arm(std::shared_ptr<const constants::Values> values);

  // if true, tune down all the constants for testing.
  static constexpr bool kGrannyMode() { return false; }

  // the operating voltage.
  static constexpr double kOperatingVoltage() {
    return kGrannyMode() ? 5.0 : 12.0;
  }
  static constexpr double kDt() { return 0.00505; }
  static constexpr double kAlpha0Max() { return kGrannyMode() ? 5.0 : 15.0; }
  static constexpr double kAlpha1Max() { return kGrannyMode() ? 5.0 : 15.0; }

  static constexpr double kVMax() { return kGrannyMode() ? 5.0 : 11.5; }
  static constexpr double kPathlessVMax() { return 5.0; }
  static constexpr double kGotoPathVMax() { return 6.0; }

  flatbuffers::Offset<superstructure::ArmStatus> Iterate(
      const ::aos::monotonic_clock::time_point /*monotonic_now*/,
      const uint32_t *unsafe_goal, const superstructure::ArmPosition *position,
      bool trajectory_override, double *proximal_output, double *distal_output,
      bool /*intake*/, bool /*spit*/, flatbuffers::FlatBufferBuilder *fbb);

  void Reset();

  ArmState state() const { return state_; }

  bool estopped() const { return state_ == ArmState::ESTOP; }
  bool zeroed() const {
    return (proximal_zeroing_estimator_.zeroed() &&
            distal_zeroing_estimator_.zeroed());
  }

  // Returns the maximum position for the intake.  This is used to override the
  // intake position to release the box when the state machine is lifting.
  double max_intake_override() const { return max_intake_override_; }

  uint32_t current_node() const { return current_node_; }

  double path_distance_to_go() { return follower_.path_distance_to_go(); }

 private:
  bool AtState(uint32_t state) const { return current_node_ == state; }
  bool NearEnd(double threshold = 0.03) const {
    return ::std::abs(arm_ekf_.X_hat(0) - follower_.theta(0)) <= threshold &&
           ::std::abs(arm_ekf_.X_hat(2) - follower_.theta(1)) <= threshold &&
           follower_.path_distance_to_go() < 1e-3;
  }

  std::shared_ptr<const constants::Values> values_;

  ArmState state_;

  ::frc971::zeroing::PotAndAbsoluteEncoderZeroingEstimator
      proximal_zeroing_estimator_;
  ::frc971::zeroing::PotAndAbsoluteEncoderZeroingEstimator
      distal_zeroing_estimator_;

  double proximal_offset_;
  double distal_offset_;

  double max_intake_override_;

  const ::Eigen::Matrix<double, 2, 2> alpha_unitizer_;

  double vmax_ = kVMax();

  frc971::control_loops::arm::Dynamics dynamics_;

  ::std::vector<TrajectoryAndParams> trajectories_;
  SearchGraph search_graph_;

  bool close_enough_for_full_power_;

  size_t brownout_count_;

  EKF arm_ekf_;
  TrajectoryFollower follower_;

  const ::std::vector<::Eigen::Matrix<double, 2, 1>> points_;

  // Start at the 0th index.
  uint32_t current_node_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

}  // namespace arm
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2023

#endif  // Y2023_CONTROL_LOOPS_SUPERSTRUCTURE_ARM_ARM_H_
