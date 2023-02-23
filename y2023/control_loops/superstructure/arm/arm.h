#ifndef Y2023_CONTROL_LOOPS_SUPERSTRUCTURE_ARM_ARM_H_
#define Y2023_CONTROL_LOOPS_SUPERSTRUCTURE_ARM_ARM_H_

#include "aos/time/time.h"
#include "frc971/control_loops/double_jointed_arm/dynamics.h"
#include "frc971/control_loops/double_jointed_arm/ekf.h"
#include "frc971/control_loops/double_jointed_arm/graph.h"
#include "frc971/zeroing/zeroing.h"
#include "y2023/constants.h"
#include "y2023/control_loops/superstructure/arm/generated_graph.h"
#include "y2023/control_loops/superstructure/arm/trajectory.h"
#include "y2023/control_loops/superstructure/superstructure_position_generated.h"
#include "y2023/control_loops/superstructure/superstructure_status_generated.h"

using frc971::control_loops::arm::EKF;

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
  static constexpr std::chrono::nanoseconds kDtDuration() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(kDt()));
  }
  static constexpr double kAlpha0Max() { return kGrannyMode() ? 5.0 : 15.0; }
  static constexpr double kAlpha1Max() { return kGrannyMode() ? 5.0 : 15.0; }
  static constexpr double kAlpha2Max() { return kGrannyMode() ? 5.0 : 15.0; }

  static constexpr double kVMax() { return kGrannyMode() ? 5.0 : 11.5; }
  static constexpr double kPathlessVMax() { return 5.0; }
  static constexpr double kGotoPathVMax() { return 6.0; }

  flatbuffers::Offset<superstructure::ArmStatus> Iterate(
      const ::aos::monotonic_clock::time_point /*monotonic_now*/,
      const uint32_t *unsafe_goal, const superstructure::ArmPosition *position,
      bool trajectory_override, double *proximal_output, double *distal_output,
      double *roll_joint_output, flatbuffers::FlatBufferBuilder *fbb);

  void Reset();

  ArmState state() const { return state_; }

  bool estopped() const { return state_ == ArmState::ESTOP; }
  bool zeroed() const {
    return (proximal_zeroing_estimator_.zeroed() &&
            distal_zeroing_estimator_.zeroed() &&
            roll_joint_zeroing_estimator_.zeroed());
  }

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
  ::frc971::zeroing::PotAndAbsoluteEncoderZeroingEstimator
      roll_joint_zeroing_estimator_;

  double proximal_offset_;
  double distal_offset_;
  double roll_joint_offset_;

  const ::Eigen::DiagonalMatrix<double, 3> alpha_unitizer_;

  double vmax_ = kVMax();

  frc971::control_loops::arm::Dynamics dynamics_;

  ::std::vector<TrajectoryAndParams> trajectories_;

  bool close_enough_for_full_power_;

  size_t brownout_count_;

  StateFeedbackLoop<3, 1, 1, double, StateFeedbackPlant<3, 1, 1>,
                    StateFeedbackObserver<3, 1, 1>>
      roll_joint_loop_;
  const StateFeedbackLoop<3, 1, 1, double, StateFeedbackHybridPlant<3, 1, 1>,
                          HybridKalman<3, 1, 1>>
      hybrid_roll_joint_loop_;
  EKF arm_ekf_;
  SearchGraph search_graph_;
  TrajectoryFollower follower_;

  const ::std::vector<::Eigen::Matrix<double, 3, 1>> points_;

  // Start at the 0th index.
  uint32_t current_node_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

}  // namespace arm
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2023

#endif  // Y2023_CONTROL_LOOPS_SUPERSTRUCTURE_ARM_ARM_H_
