#include "y2023/control_loops/superstructure/arm/arm.h"

#include "y2023/control_loops/superstructure/roll/integral_hybrid_roll_plant.h"
#include "y2023/control_loops/superstructure/roll/integral_roll_plant.h"

namespace y2023 {
namespace control_loops {
namespace superstructure {
namespace arm {
namespace {

namespace chrono = ::std::chrono;
using ::aos::monotonic_clock;

constexpr int kMaxBrownoutCount = 4;

}  // namespace

Arm::Arm(std::shared_ptr<const constants::Values> values)
    : values_(values),
      state_(ArmState::UNINITIALIZED),
      proximal_zeroing_estimator_(values_->arm_proximal.zeroing),
      distal_zeroing_estimator_(values_->arm_distal.zeroing),
      roll_joint_zeroing_estimator_(values_->roll_joint.zeroing),
      proximal_offset_(0.0),
      distal_offset_(0.0),
      roll_joint_offset_(0.0),
      alpha_unitizer_((::Eigen::DiagonalMatrix<double, 3>().diagonal()
                           << (1.0 / kAlpha0Max()),
                       (1.0 / kAlpha1Max()), (1.0 / kAlpha2Max()))
                          .finished()),
      dynamics_(kArmConstants),
      close_enough_for_full_power_(false),
      brownout_count_(0),
      roll_joint_loop_(roll::MakeIntegralRollLoop()),
      hybrid_roll_joint_loop_(roll::MakeIntegralHybridRollLoop()),
      arm_ekf_(&dynamics_),
      search_graph_(MakeSearchGraph(&dynamics_, &trajectories_, alpha_unitizer_,
                                    kVMax(), &hybrid_roll_joint_loop_)),
      // Go to the start of the first trajectory.
      follower_(&dynamics_, &hybrid_roll_joint_loop_, NeutralPoint()),
      points_(PointList()),
      current_node_(0) {
  int i = 0;
  for (const auto &trajectory : trajectories_) {
    AOS_LOG(INFO, "trajectory length for edge node %d: %f\n", i,
            trajectory.trajectory.path().length());
    ++i;
  }
}

void Arm::Reset() { state_ = ArmState::UNINITIALIZED; }

namespace {

// Proximal joint center in xy space
constexpr std::pair<double, double> kJointCenter = {-0.203, 0.787};

std::tuple<double, double, int> ArmThetasToXY(double theta_proximal,
                                              double theta_distal) {
  double theta_proximal_shifted = M_PI / 2.0 - theta_proximal;
  double theta_distal_shifted = M_PI / 2.0 - theta_distal;

  double x = std::cos(theta_proximal_shifted) * kArmConstants.l0 +
             std::cos(theta_distal_shifted) * kArmConstants.l1 +
             kJointCenter.first;
  double y = std::sin(theta_proximal_shifted) * kArmConstants.l0 +
             std::sin(theta_distal_shifted) * kArmConstants.l1 +
             kJointCenter.second;

  int circular_index =
      std::floor((theta_distal_shifted - theta_proximal_shifted) / M_PI);

  return std::make_tuple(x, y, circular_index);
}

}  // namespace

flatbuffers::Offset<superstructure::ArmStatus> Arm::Iterate(
    const ::aos::monotonic_clock::time_point /*monotonic_now*/,
    const uint32_t *unsafe_goal, const superstructure::ArmPosition *position,
    bool trajectory_override, double *proximal_output, double *distal_output,
    double *roll_joint_output, flatbuffers::FlatBufferBuilder *fbb) {
  ::Eigen::Matrix<double, 2, 1> Y;
  const bool outputs_disabled =
      ((proximal_output == nullptr) || (distal_output == nullptr) ||
       (roll_joint_output == nullptr));
  if (outputs_disabled) {
    ++brownout_count_;
  } else {
    brownout_count_ = 0;
  }

  // TODO(milind): should we default to the closest position?
  uint32_t filtered_goal = arm::NeutralIndex();
  if (unsafe_goal != nullptr) {
    filtered_goal = *unsafe_goal;
  }

  ::Eigen::Matrix<double, 2, 1> Y_arm;
  Y_arm << position->proximal()->encoder() + proximal_offset_,
      position->distal()->encoder() + distal_offset_;
  ::Eigen::Matrix<double, 1, 1> Y_roll_joint;
  Y_roll_joint << position->roll_joint()->encoder() + roll_joint_offset_;

  proximal_zeroing_estimator_.UpdateEstimate(*position->proximal());
  distal_zeroing_estimator_.UpdateEstimate(*position->distal());
  roll_joint_zeroing_estimator_.UpdateEstimate(*position->roll_joint());

  if (proximal_output != nullptr) {
    *proximal_output = 0.0;
  }
  if (distal_output != nullptr) {
    *distal_output = 0.0;
  }
  if (roll_joint_output != nullptr) {
    *roll_joint_output = 0.0;
  }

  arm_ekf_.Correct(Y_arm, kDt());
  roll_joint_loop_.Correct(Y_roll_joint);

  if (::std::abs(arm_ekf_.X_hat(0) - follower_.theta(0)) <= 0.05 &&
      ::std::abs(arm_ekf_.X_hat(2) - follower_.theta(1)) <= 0.05 &&
      ::std::abs(roll_joint_loop_.X_hat(0) - follower_.theta(2)) <= 0.05) {
    close_enough_for_full_power_ = true;
  }
  if (::std::abs(arm_ekf_.X_hat(0) - follower_.theta(0)) >= 1.10 ||
      ::std::abs(arm_ekf_.X_hat(2) - follower_.theta(1)) >= 1.10 ||
      ::std::abs(roll_joint_loop_.X_hat(0) - follower_.theta(2)) >= 0.50) {
    close_enough_for_full_power_ = false;
  }

  switch (state_) {
    case ArmState::UNINITIALIZED:
      // Wait in the uninitialized state until the intake is initialized.
      AOS_LOG(DEBUG, "Uninitialized, waiting for intake\n");
      state_ = ArmState::ZEROING;
      proximal_zeroing_estimator_.Reset();
      distal_zeroing_estimator_.Reset();
      roll_joint_zeroing_estimator_.Reset();
      break;

    case ArmState::ZEROING:
      // Zero by not moving.
      if (zeroed()) {
        state_ = ArmState::DISABLED;

        proximal_offset_ = proximal_zeroing_estimator_.offset();
        distal_offset_ = distal_zeroing_estimator_.offset();
        roll_joint_offset_ = roll_joint_zeroing_estimator_.offset();

        Y_arm << position->proximal()->encoder() + proximal_offset_,
            position->distal()->encoder() + distal_offset_;
        Y_roll_joint << position->roll_joint()->encoder() + roll_joint_offset_;

        // TODO(austin): Offset ekf rather than reset it.  Since we aren't
        // moving at this point, it's pretty safe to do this.
        ::Eigen::Matrix<double, 4, 1> X_arm;
        X_arm << Y_arm(0), 0.0, Y_arm(1), 0.0;
        arm_ekf_.Reset(X_arm);

        ::Eigen::Matrix<double, 3, 1> X_roll_joint;
        X_roll_joint << Y_roll_joint(0), 0.0, 0.0;
        roll_joint_loop_.mutable_X_hat() = X_roll_joint;
      } else {
        break;
      }
      [[fallthrough]];

    case ArmState::DISABLED: {
      follower_.SwitchTrajectory(nullptr);
      close_enough_for_full_power_ = false;

      const ::Eigen::Matrix<double, 3, 1> current_theta =
          (::Eigen::Matrix<double, 3, 1>() << arm_ekf_.X_hat(0),
           arm_ekf_.X_hat(2), roll_joint_loop_.X_hat(0))
              .finished();
      uint32_t best_index = 0;
      double best_distance = (points_[0] - current_theta).norm();
      uint32_t current_index = 0;
      for (const ::Eigen::Matrix<double, 3, 1> &point : points_) {
        const double new_distance = (point - current_theta).norm();
        if (new_distance < best_distance) {
          best_distance = new_distance;
          best_index = current_index;
        }
        ++current_index;
      }
      follower_.set_theta(points_[best_index]);
      current_node_ = best_index;

      if (!outputs_disabled) {
        state_ = ArmState::GOTO_PATH;
      } else {
        break;
      }
    }
      [[fallthrough]];

    case ArmState::GOTO_PATH:
      if (outputs_disabled) {
        state_ = ArmState::DISABLED;
      } else if (trajectory_override) {
        follower_.SwitchTrajectory(nullptr);
        current_node_ = filtered_goal;
        follower_.set_theta(points_[current_node_]);
        state_ = ArmState::GOTO_PATH;
      } else if (close_enough_for_full_power_) {
        state_ = ArmState::RUNNING;
      }
      break;

    case ArmState::RUNNING:
      // ESTOP if we hit the hard limits.
      // TODO(austin): Pick some sane limits.
      if (proximal_zeroing_estimator_.error() ||
          distal_zeroing_estimator_.error() ||
          roll_joint_zeroing_estimator_.error()) {
        AOS_LOG(ERROR, "Zeroing error ESTOP\n");
        state_ = ArmState::ESTOP;
      } else if (outputs_disabled && brownout_count_ > kMaxBrownoutCount) {
        state_ = ArmState::DISABLED;
      } else if (trajectory_override) {
        follower_.SwitchTrajectory(nullptr);
        current_node_ = filtered_goal;
        follower_.set_theta(points_[current_node_]);
        state_ = ArmState::GOTO_PATH;
      }
      break;

    case ArmState::ESTOP:
      AOS_LOG(ERROR, "Estop\n");
      break;
  }

  const bool disable = outputs_disabled || (state_ != ArmState::RUNNING &&
                                            state_ != ArmState::GOTO_PATH);
  if (disable) {
    close_enough_for_full_power_ = false;
  }

  if (state_ == ArmState::RUNNING && unsafe_goal != nullptr) {
    if (current_node_ != filtered_goal) {
      AOS_LOG(INFO, "Goal is different\n");
      if (filtered_goal >= search_graph_.num_vertexes()) {
        AOS_LOG(ERROR, "goal node out of range ESTOP\n");
        state_ = ArmState::ESTOP;
      } else if (follower_.path_distance_to_go() > 1e-3) {
        // Still on the old path segment.  Can't change yet.
      } else {
        search_graph_.SetGoal(filtered_goal);

        size_t min_edge = 0;
        double min_cost = ::std::numeric_limits<double>::infinity();
        for (const SearchGraph::HalfEdge &edge :
             search_graph_.Neighbors(current_node_)) {
          const double cost = search_graph_.GetCostToGoal(edge.dest);
          if (cost < min_cost) {
            min_edge = edge.edge_id;
            min_cost = cost;
          }
        }
        // Ok, now we know which edge we are on.  Figure out the path and
        // trajectory.
        const SearchGraph::Edge &next_edge = search_graph_.edges()[min_edge];
        AOS_LOG(INFO, "Switching from node %d to %d along edge %d\n",
                static_cast<int>(current_node_),
                static_cast<int>(next_edge.end), static_cast<int>(min_edge));
        vmax_ = trajectories_[min_edge].vmax;
        follower_.SwitchTrajectory(&trajectories_[min_edge].trajectory);
        current_node_ = next_edge.end;
      }
    }
  }

  const double max_operating_voltage =
      close_enough_for_full_power_
          ? kOperatingVoltage()
          : (state_ == ArmState::GOTO_PATH ? kGotoPathVMax() : kPathlessVMax());
  ::Eigen::Matrix<double, 9, 1> X_hat;
  X_hat.block<6, 1>(0, 0) = arm_ekf_.X_hat();
  X_hat.block<3, 1>(6, 0) = roll_joint_loop_.X_hat();

  follower_.Update(X_hat, disable, kDt(), vmax_, max_operating_voltage);

  arm_ekf_.Predict(follower_.U().head<2>(), kDt());
  roll_joint_loop_.UpdateObserver(follower_.U().tail<1>(), kDtDuration());

  flatbuffers::Offset<frc971::PotAndAbsoluteEncoderEstimatorState>
      proximal_estimator_state_offset =
          proximal_zeroing_estimator_.GetEstimatorState(fbb);
  flatbuffers::Offset<frc971::PotAndAbsoluteEncoderEstimatorState>
      distal_estimator_state_offset =
          distal_zeroing_estimator_.GetEstimatorState(fbb);
  flatbuffers::Offset<frc971::PotAndAbsoluteEncoderEstimatorState>
      roll_joint_estimator_state_offset =
          roll_joint_zeroing_estimator_.GetEstimatorState(fbb);

  const auto [arm_x, arm_y, arm_circular_index] =
      ArmThetasToXY(arm_ekf_.X_hat(0), arm_ekf_.X_hat(2));

  superstructure::ArmStatus::Builder status_builder(*fbb);
  status_builder.add_proximal_estimator_state(proximal_estimator_state_offset);
  status_builder.add_distal_estimator_state(distal_estimator_state_offset);
  status_builder.add_roll_joint_estimator_state(
      roll_joint_estimator_state_offset);

  status_builder.add_goal_theta0(follower_.theta(0));
  status_builder.add_goal_theta1(follower_.theta(1));
  status_builder.add_goal_theta2(follower_.theta(2));
  status_builder.add_goal_omega0(follower_.omega(0));
  status_builder.add_goal_omega1(follower_.omega(1));
  status_builder.add_goal_omega2(follower_.omega(2));

  status_builder.add_theta0(arm_ekf_.X_hat(0));
  status_builder.add_theta1(arm_ekf_.X_hat(2));
  status_builder.add_theta2(roll_joint_loop_.X_hat(0));
  status_builder.add_omega0(arm_ekf_.X_hat(1));
  status_builder.add_omega1(arm_ekf_.X_hat(3));
  status_builder.add_omega2(roll_joint_loop_.X_hat(1));
  status_builder.add_voltage_error0(arm_ekf_.X_hat(4));
  status_builder.add_voltage_error1(arm_ekf_.X_hat(5));
  status_builder.add_voltage_error2(roll_joint_loop_.X_hat(2));

  status_builder.add_arm_x(arm_x);
  status_builder.add_arm_y(arm_y);
  status_builder.add_arm_circular_index(arm_circular_index);

  if (!disable) {
    *proximal_output = ::std::max(
        -kOperatingVoltage(), ::std::min(kOperatingVoltage(), follower_.U(0)));
    *distal_output = ::std::max(
        -kOperatingVoltage(), ::std::min(kOperatingVoltage(), follower_.U(1)));
    *roll_joint_output = ::std::max(
        -kOperatingVoltage(), ::std::min(kOperatingVoltage(), follower_.U(2)));
  }

  status_builder.add_path_distance_to_go(follower_.path_distance_to_go());
  status_builder.add_current_node(current_node_);

  status_builder.add_zeroed(zeroed());
  status_builder.add_estopped(estopped());
  status_builder.add_state(state_);
  status_builder.add_failed_solutions(follower_.failed_solutions());

  return status_builder.Finish();
}

}  // namespace arm
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2023
