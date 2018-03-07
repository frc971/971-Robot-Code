#include "y2018/control_loops/superstructure/arm/arm.h"

#include <chrono>
#include <iostream>

#include "aos/common/logging/logging.h"
#include "aos/common/logging/queue_logging.h"
#include "y2018/constants.h"
#include "y2018/control_loops/superstructure/arm/demo_path.h"
#include "y2018/control_loops/superstructure/arm/dynamics.h"
#include "y2018/control_loops/superstructure/arm/generated_graph.h"

namespace y2018 {
namespace control_loops {
namespace superstructure {
namespace arm {

namespace chrono = ::std::chrono;
using ::aos::monotonic_clock;

Arm::Arm()
    : proximal_zeroing_estimator_(constants::GetValues().arm_proximal.zeroing),
      distal_zeroing_estimator_(constants::GetValues().arm_distal.zeroing),
      alpha_unitizer_((::Eigen::Matrix<double, 2, 2>() << 1.0 / kAlpha0Max(),
                       0.0, 0.0, 1.0 / kAlpha1Max())
                          .finished()),
      search_graph_(MakeSearchGraph(&trajectories_, alpha_unitizer_, kVMax())),
      // Go to the start of the first trajectory.
      follower_(ReadyAboveBoxPoint()) {
  int i = 0;
  for (const auto &trajectory : trajectories_) {
    LOG(INFO, "trajectory length for edge node %d: %f\n", i,
        trajectory.path().length());
    ++i;
  }
}

void Arm::Reset() { state_ = State::UNINITIALIZED; }

void Arm::Iterate(const uint32_t *unsafe_goal,
                  const control_loops::ArmPosition *position,
                  double *proximal_output, double *distal_output,
                  bool *release_arm_brake, control_loops::ArmStatus *status) {
  ::Eigen::Matrix<double, 2, 1> Y;

  Y << position->proximal.encoder + proximal_offset_,
      position->distal.encoder + distal_offset_;

  proximal_zeroing_estimator_.UpdateEstimate(position->proximal);
  distal_zeroing_estimator_.UpdateEstimate(position->distal);

  if (proximal_output != nullptr) {
    *proximal_output = 0.0;
  }
  if (distal_output != nullptr) {
    *distal_output = 0.0;
  }

  arm_ekf_.Correct(Y, kDt());

  if (::std::abs(arm_ekf_.X_hat(0) - follower_.theta(0)) <= 0.05 &&
      ::std::abs(arm_ekf_.X_hat(2) - follower_.theta(1)) <= 0.05) {
    close_enough_for_full_power_ = true;
  }
  if (::std::abs(arm_ekf_.X_hat(0) - follower_.theta(0)) >= 1.10 ||
      ::std::abs(arm_ekf_.X_hat(2) - follower_.theta(1)) >= 1.10) {
    close_enough_for_full_power_ = false;
  }

  switch (state_) {
    case State::UNINITIALIZED:
      // Wait in the uninitialized state until the intake is initialized.
      LOG(DEBUG, "Uninitialized, waiting for intake\n");
      state_ = State::ZEROING;
      proximal_zeroing_estimator_.Reset();
      distal_zeroing_estimator_.Reset();
      // TODO(austin): Go to the nearest node.  For now, we are always going to
      // go to node 0.
      break;

    case State::ZEROING:
      // Zero by not moving.
      if (proximal_zeroing_estimator_.zeroed() &&
          distal_zeroing_estimator_.zeroed()) {
        state_ = State::RUNNING;

        proximal_offset_ = proximal_zeroing_estimator_.offset();
        distal_offset_ = distal_zeroing_estimator_.offset();

        Y << position->proximal.encoder + proximal_offset_,
            position->distal.encoder + distal_offset_;

        // TODO(austin): Offset ekf rather than reset it.  Since we aren't
        // moving at this point, it's pretty safe to do this.
        ::Eigen::Matrix<double, 4, 1> X;
        X << Y(0), 0.0, Y(1), 0.0;
        arm_ekf_.Reset(X);
      } else {
        break;
      }

    case State::RUNNING:
      // ESTOP if we hit the hard limits.
      // TODO(austin): Pick some sane limits.
      if (proximal_zeroing_estimator_.error() ||
          distal_zeroing_estimator_.error()) {
        LOG(ERROR, "Zeroing error ESTOP\n");
        state_ = State::ESTOP;
      } else if (unsafe_goal != nullptr) {
        if (!follower_.has_path()) {
          // TODO(austin): Nearest point at the end of Initialize.
          // So, get a vector of all the points, loop through them, and find the
          // closest one.

          // If we don't have a path and are far from the goal, don't update the
          // path.
          // TODO(austin): Once we get close to our first setpoint, crank the
          // power back up. Otherwise we'll be weak at startup.
          LOG(INFO, "No path.\n");
          if (!close_enough_for_full_power_) {
            break;
          }
        }
        if (current_node_ != *unsafe_goal) {
          LOG(INFO, "Goal is different\n");
          if (*unsafe_goal >= search_graph_.num_vertexes()) {
            LOG(ERROR, "goal out of range ESTOP\n");
            state_ = State::ESTOP;
            break;
          }

          if (follower_.path_distance_to_go() > 1e-3) {
            LOG(INFO, " Distance to go %f\n", follower_.path_distance_to_go());
            // Still on the old path segment.  Can't change yet.
            break;
          }

          search_graph_.SetGoal(*unsafe_goal);

          size_t min_edge = 0;
          double min_cost = -1.0;
          for (const SearchGraph::HalfEdge &edge :
               search_graph_.Neighbors(current_node_)) {
            const double cost = search_graph_.GetCostToGoal(edge.dest);
            if (min_cost == -1 || cost < min_cost) {
              min_edge = edge.edge_id;
              min_cost = cost;
            }
          }
          // Ok, now we know which edge we are on.  Figure out the path and
          // trajectory.
          const SearchGraph::Edge &next_edge = search_graph_.edges()[min_edge];
          LOG(INFO, "Switching from node %d to %d along edge %d\n",
              static_cast<int>(current_node_), static_cast<int>(next_edge.end),
              static_cast<int>(min_edge));
          follower_.SwitchTrajectory(&trajectories_[min_edge]);
          current_node_ = next_edge.end;
        }
      }
      break;

    case State::ESTOP:
      LOG(ERROR, "Estop\n");
      break;
  }

  const bool disable =
      ((proximal_output == nullptr) || (distal_output == nullptr) ||
       (release_arm_brake == nullptr)) ||
      state_ != State::RUNNING;
  if (disable) {
    close_enough_for_full_power_ = false;
  }

  follower_.Update(
      arm_ekf_.X_hat(), disable, kDt(), kVMax(),
      close_enough_for_full_power_ ? kOperatingVoltage() : kPathlessVMax());
  LOG(INFO, "Max voltage: %f\n",
      close_enough_for_full_power_ ? kOperatingVoltage() : kPathlessVMax());
  status->goal_theta0 = follower_.theta(0);
  status->goal_theta1 = follower_.theta(1);
  status->goal_omega0 = follower_.omega(0);
  status->goal_omega1 = follower_.omega(1);

  status->theta0 = arm_ekf_.X_hat(0);
  status->theta1 = arm_ekf_.X_hat(2);
  status->omega0 = arm_ekf_.X_hat(1);
  status->omega1 = arm_ekf_.X_hat(3);
  status->voltage_error0 = arm_ekf_.X_hat(4);
  status->voltage_error1 = arm_ekf_.X_hat(5);

  if (!disable) {
    *proximal_output = ::std::max(
        -kOperatingVoltage(), ::std::min(kOperatingVoltage(), follower_.U(0)));
    *distal_output = ::std::max(
        -kOperatingVoltage(), ::std::min(kOperatingVoltage(), follower_.U(1)));
    *release_arm_brake = true;
  }

  status->proximal_estimator_state =
      proximal_zeroing_estimator_.GetEstimatorState();
  status->distal_estimator_state =
      distal_zeroing_estimator_.GetEstimatorState();

  status->path_distance_to_go = follower_.path_distance_to_go();
  status->current_node = current_node_;

  status->zeroed = (proximal_zeroing_estimator_.zeroed() &&
                    distal_zeroing_estimator_.zeroed());
  status->estopped = (state_ == State::ESTOP);
  status->state = static_cast<int32_t>(state_);
  status->failed_solutions = follower_.failed_solutions();

  arm_ekf_.Predict(follower_.U(), kDt());
}

}  // namespace arm
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2018
