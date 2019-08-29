#ifndef Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_ARM_ARM_H_
#define Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_ARM_ARM_H_

#include "aos/time/time.h"
#include "frc971/zeroing/zeroing.h"
#include "y2018/constants.h"
#include "y2018/control_loops/superstructure/arm/dynamics.h"
#include "y2018/control_loops/superstructure/arm/ekf.h"
#include "y2018/control_loops/superstructure/arm/generated_graph.h"
#include "y2018/control_loops/superstructure/arm/graph.h"
#include "y2018/control_loops/superstructure/arm/trajectory.h"
#include "y2018/control_loops/superstructure/superstructure_position_generated.h"
#include "y2018/control_loops/superstructure/superstructure_status_generated.h"

namespace y2018 {
namespace control_loops {
namespace superstructure {
namespace arm {

class Arm {
 public:
  Arm();

  // If true, tune down all the constants for testing.
  static constexpr bool kGrannyMode() { return false; }

  // The operating voltage.
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
      const ::aos::monotonic_clock::time_point monotonic_now,
      const uint32_t *unsafe_goal, bool grab_box, bool open_claw,
      bool close_claw, const superstructure::ArmPosition *position,
      const bool claw_beambreak_triggered,
      const bool box_back_beambreak_triggered, const bool intake_clear_of_box,
      bool suicide, bool trajectory_override, double *proximal_output,
      double *distal_output, bool *release_arm_brake, bool *claw_closed,
      flatbuffers::FlatBufferBuilder *fbb);

  void Reset();

  enum class State : int32_t {
    UNINITIALIZED,
    ZEROING,
    DISABLED,
    GOTO_PATH,
    RUNNING,
    PREP_CLIMB,
    ESTOP,
  };

  enum class GrabState : int32_t {
    NORMAL = 0,
    WAIT_FOR_BOX = 1,
    TALL_BOX = 2,
    SHORT_BOX = 3,
    CLAW_CLOSE = 4,
    OPEN_INTAKE = 5,
  };

  State state() const { return state_; }
  GrabState grab_state() const { return grab_state_; }

  bool estopped() const { return state_ == State::ESTOP; }
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

  State state_ = State::UNINITIALIZED;

  GrabState grab_state_ = GrabState::NORMAL;

  ::aos::monotonic_clock::time_point claw_close_start_time_ =
      ::aos::monotonic_clock::min_time;

  ::frc971::zeroing::PotAndAbsoluteEncoderZeroingEstimator
      proximal_zeroing_estimator_;
  ::frc971::zeroing::PotAndAbsoluteEncoderZeroingEstimator
      distal_zeroing_estimator_;

  double proximal_offset_ = 0.0;
  double distal_offset_ = 0.0;

  bool claw_closed_ = true;
  double max_intake_override_ = 1000.0;

  const ::Eigen::Matrix<double, 2, 2> alpha_unitizer_;

  double vmax_ = kVMax();

  ::std::vector<TrajectoryAndParams> trajectories_;
  SearchGraph search_graph_;

  bool close_enough_for_full_power_ = false;

  int32_t climb_count_ = 0;
  int32_t brownout_count_ = 0;

  EKF arm_ekf_;
  TrajectoryFollower follower_;

  const ::std::vector<::Eigen::Matrix<double, 2, 1>> points_;

  // Start at the 0th index.
  uint32_t current_node_ = 0;

  uint32_t claw_closed_count_ = 0;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

}  // namespace arm
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2018

#endif  // Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_ARM_ARM_H_
