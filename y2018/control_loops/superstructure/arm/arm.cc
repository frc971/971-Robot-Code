#include "y2018/control_loops/superstructure/arm/arm.h"

#include <chrono>
#include <iostream>

#include "aos/logging/logging.h"
#include "y2018/constants.h"
#include "y2018/control_loops/superstructure/arm/demo_path.h"
#include "y2018/control_loops/superstructure/arm/dynamics.h"
#include "y2018/control_loops/superstructure/arm/generated_graph.h"

namespace y2018 {
namespace control_loops {
namespace superstructure {
namespace arm {

namespace {

namespace chrono = ::std::chrono;
using ::aos::monotonic_clock;

constexpr int kMaxBrownoutCount = 4;

}  // namespace

Arm::Arm()
    : proximal_zeroing_estimator_(constants::GetValues().arm_proximal.zeroing),
      distal_zeroing_estimator_(constants::GetValues().arm_distal.zeroing),
      alpha_unitizer_((::Eigen::Matrix<double, 2, 2>() << 1.0 / kAlpha0Max(),
                       0.0, 0.0, 1.0 / kAlpha1Max())
                          .finished()),
      search_graph_(MakeSearchGraph(&trajectories_, alpha_unitizer_, kVMax())),
      // Go to the start of the first trajectory.
      follower_(ReadyAboveBoxPoint()),
      points_(PointList()) {
  int i = 0;
  for (const auto &trajectory : trajectories_) {
    AOS_LOG(INFO, "trajectory length for edge node %d: %f\n", i,
            trajectory.trajectory.path().length());
    ++i;
  }
}

void Arm::Reset() { state_ = State::UNINITIALIZED; }

flatbuffers::Offset<superstructure::ArmStatus> Arm::Iterate(
    const ::aos::monotonic_clock::time_point monotonic_now,
    const uint32_t *unsafe_goal, bool grab_box, bool open_claw, bool close_claw,
    const superstructure::ArmPosition *position,
    const bool claw_beambreak_triggered,
    const bool box_back_beambreak_triggered, const bool intake_clear_of_box,
    bool suicide, bool trajectory_override, double *proximal_output,
    double *distal_output, bool *release_arm_brake, bool *claw_closed,
    flatbuffers::FlatBufferBuilder *fbb) {
  ::Eigen::Matrix<double, 2, 1> Y;
  const bool outputs_disabled =
      ((proximal_output == nullptr) || (distal_output == nullptr) ||
       (release_arm_brake == nullptr) || (claw_closed == nullptr));
  if (outputs_disabled) {
    ++brownout_count_;
  } else {
    brownout_count_ = 0;
  }

  uint32_t filtered_goal = 0;
  if (unsafe_goal != nullptr) {
    filtered_goal = *unsafe_goal;
  }

  if (open_claw) {
    claw_closed_ = false;
  }
  if (close_claw) {
    claw_closed_ = true;
  }
  if (outputs_disabled) {
    if (claw_closed_count_ == 0) {
      claw_closed_ = true;
    } else {
      --claw_closed_count_;
    }
  } else {
    // Wait this many iterations before closing the claw.  That prevents
    // brownouts from closing the claw.
    claw_closed_count_ = 50;
  }

  Y << position->proximal()->encoder() + proximal_offset_,
      position->distal()->encoder() + distal_offset_;

  proximal_zeroing_estimator_.UpdateEstimate(*position->proximal());
  distal_zeroing_estimator_.UpdateEstimate(*position->distal());

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
      AOS_LOG(DEBUG, "Uninitialized, waiting for intake\n");
      state_ = State::ZEROING;
      proximal_zeroing_estimator_.Reset();
      distal_zeroing_estimator_.Reset();
      break;

    case State::ZEROING:
      // Zero by not moving.
      if (proximal_zeroing_estimator_.zeroed() &&
          distal_zeroing_estimator_.zeroed()) {
        state_ = State::DISABLED;

        proximal_offset_ = proximal_zeroing_estimator_.offset();
        distal_offset_ = distal_zeroing_estimator_.offset();

        Y << position->proximal()->encoder() + proximal_offset_,
            position->distal()->encoder() + distal_offset_;

        // TODO(austin): Offset ekf rather than reset it.  Since we aren't
        // moving at this point, it's pretty safe to do this.
        ::Eigen::Matrix<double, 4, 1> X;
        X << Y(0), 0.0, Y(1), 0.0;
        arm_ekf_.Reset(X);
      } else {
        break;
      }
      [[fallthrough]];

    case State::DISABLED: {
      follower_.SwitchTrajectory(nullptr);
      close_enough_for_full_power_ = false;

      const ::Eigen::Matrix<double, 2, 1> current_theta =
          (::Eigen::Matrix<double, 2, 1>() << arm_ekf_.X_hat(0),
           arm_ekf_.X_hat(2))
              .finished();
      uint32_t best_index = 0;
      double best_distance = (points_[0] - current_theta).norm();
      uint32_t current_index = 0;
      for (const ::Eigen::Matrix<double, 2, 1> &point : points_) {
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
        state_ = State::GOTO_PATH;
      } else {
        break;
      }
    }
    [[fallthrough]];

    case State::GOTO_PATH:
      if (outputs_disabled) {
        state_ = State::DISABLED;
      } else if (trajectory_override) {
        follower_.SwitchTrajectory(nullptr);
        current_node_ = filtered_goal;
        follower_.set_theta(points_[current_node_]);
        state_ = State::GOTO_PATH;
      } else if (close_enough_for_full_power_) {
        state_ = State::RUNNING;
        grab_state_ = GrabState::NORMAL;
      }
      break;

    case State::RUNNING:
      // ESTOP if we hit the hard limits.
      // TODO(austin): Pick some sane limits.
      if (proximal_zeroing_estimator_.error() ||
          distal_zeroing_estimator_.error()) {
        AOS_LOG(ERROR, "Zeroing error ESTOP\n");
        state_ = State::ESTOP;
      } else if (outputs_disabled && brownout_count_ > kMaxBrownoutCount) {
        state_ = State::DISABLED;
      } else if (trajectory_override) {
        follower_.SwitchTrajectory(nullptr);
        current_node_ = filtered_goal;
        follower_.set_theta(points_[current_node_]);
        state_ = State::GOTO_PATH;
      } else if (suicide) {
        state_ = State::PREP_CLIMB;
        climb_count_ = 50;
      }
      break;

    case State::PREP_CLIMB:
      --climb_count_;
      if (climb_count_ <= 0) {
        state_ = State::ESTOP;
      } else if (!suicide) {
        state_ = State::RUNNING;
      }
      break;

    case State::ESTOP:
      AOS_LOG(ERROR, "Estop\n");
      break;
  }

  const bool disable = outputs_disabled || (state_ != State::RUNNING &&
                                            state_ != State::GOTO_PATH &&
                                            state_ != State::PREP_CLIMB);
  if (disable) {
    close_enough_for_full_power_ = false;
  }

  // TODO(austin): Do we need to debounce box_back_beambreak_triggered ?
  if (claw_closed_) {
    if ((filtered_goal == ReadyAboveBoxIndex()) ||
        (filtered_goal == TallBoxGrabIndex()) ||
        (filtered_goal == ShortBoxGrabIndex())) {
      filtered_goal = NeutralIndex();
    }
  }

  // TODO(austin): Do we need to debounce box_back_beambreak_triggered ?
  switch (grab_state_) {
    case GrabState::NORMAL:
      if (grab_box && !claw_closed_) {
        grab_state_ = GrabState::WAIT_FOR_BOX;
      } else {
        break;
      }
    case GrabState::WAIT_FOR_BOX:
      if (!grab_box) {
        grab_state_ = GrabState::NORMAL;
      } else {
        if (AtState(ReadyAboveBoxIndex()) && NearEnd()) {
          // We are being asked to grab the box, and the claw is near the box.
          if (box_back_beambreak_triggered) {
            // And we now see the box!  Try for a tall box.
            grab_state_ = GrabState::TALL_BOX;
          }
        }
      }
      break;
    case GrabState::TALL_BOX:
      if (!grab_box) {
        grab_state_ = GrabState::NORMAL;
      } else if (AtState(TallBoxGrabIndex()) && NearEnd()) {
        // We are being asked to grab the box, and the claw is near the box.
        if (claw_beambreak_triggered) {
          grab_state_ = GrabState::CLAW_CLOSE;
          // Snap time for the delay here.
          claw_close_start_time_ = monotonic_now;
        } else {
          grab_state_ = GrabState::SHORT_BOX;
        }
      }
      break;
    case GrabState::SHORT_BOX:
      if (!grab_box) {
        grab_state_ = GrabState::NORMAL;
      } else if (AtState(ShortBoxGrabIndex()) && NearEnd()) {
        // We are being asked to grab the box, and the claw is near the box.
        if (claw_beambreak_triggered) {
          grab_state_ = GrabState::CLAW_CLOSE;
          // Snap time for the delay here.
          claw_close_start_time_ = monotonic_now;
        } else {
          grab_state_ = GrabState::WAIT_FOR_BOX;
        }
      }
      break;
    case GrabState::CLAW_CLOSE:
      if (monotonic_now >
          claw_close_start_time_ + ::std::chrono::milliseconds(300)) {
        grab_state_ = GrabState::OPEN_INTAKE;
      }
      break;
    case GrabState::OPEN_INTAKE:
      if (intake_clear_of_box) {
        grab_state_ = GrabState::NORMAL;
      }
      break;
  }

  // Now, based out our current state, go to the right state.
  switch (grab_state_) {
    case GrabState::NORMAL:
      // Don't let the intake close fully with the claw closed.
      // TODO(austin): If we want to transfer the box from the claw to the
      // intake, we'll need to change this.
      if (claw_closed_) {
        max_intake_override_ = -0.5;
      } else {
        max_intake_override_ = 1000.0;
      }
      break;
    case GrabState::WAIT_FOR_BOX:
      filtered_goal = ReadyAboveBoxIndex();
      claw_closed_ = false;
      max_intake_override_ = 1000.0;
      break;
    case GrabState::TALL_BOX:
      filtered_goal = TallBoxGrabIndex();
      claw_closed_ = false;
      max_intake_override_ = 1000.0;
      break;
    case GrabState::SHORT_BOX:
      filtered_goal = ShortBoxGrabIndex();
      claw_closed_ = false;
      max_intake_override_ = 1000.0;
      break;
    case GrabState::CLAW_CLOSE:
      // Don't move.
      filtered_goal = current_node_;
      claw_closed_ = true;
      max_intake_override_ = 1000.0;
      break;
    case GrabState::OPEN_INTAKE:
      // Don't move.
      filtered_goal = current_node_;
      claw_closed_ = true;
      max_intake_override_ = -0.5;
      break;
  }

  if (state_ == State::RUNNING && unsafe_goal != nullptr) {
    if (current_node_ != filtered_goal) {
      AOS_LOG(INFO, "Goal is different\n");
      if (filtered_goal >= search_graph_.num_vertexes()) {
        AOS_LOG(ERROR, "goal node out of range ESTOP\n");
        state_ = State::ESTOP;
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
          : (state_ == State::GOTO_PATH ? kGotoPathVMax() : kPathlessVMax());
  follower_.Update(arm_ekf_.X_hat(), disable, kDt(), vmax_,
                   max_operating_voltage);
  AOS_LOG(INFO, "Max voltage: %f\n", max_operating_voltage);

  flatbuffers::Offset<frc971::PotAndAbsoluteEncoderEstimatorState>
      proximal_estimator_state_offset =
          proximal_zeroing_estimator_.GetEstimatorState(fbb);
  flatbuffers::Offset<frc971::PotAndAbsoluteEncoderEstimatorState>
      distal_estimator_state_offset =
          distal_zeroing_estimator_.GetEstimatorState(fbb);

  superstructure::ArmStatus::Builder status_builder(*fbb);
  status_builder.add_proximal_estimator_state(proximal_estimator_state_offset);
  status_builder.add_distal_estimator_state(distal_estimator_state_offset);

  status_builder.add_goal_theta0(follower_.theta(0));
  status_builder.add_goal_theta1(follower_.theta(1));
  status_builder.add_goal_omega0(follower_.omega(0));
  status_builder.add_goal_omega1(follower_.omega(1));

  status_builder.add_theta0(arm_ekf_.X_hat(0));
  status_builder.add_theta1(arm_ekf_.X_hat(2));
  status_builder.add_omega0(arm_ekf_.X_hat(1));
  status_builder.add_omega1(arm_ekf_.X_hat(3));
  status_builder.add_voltage_error0(arm_ekf_.X_hat(4));
  status_builder.add_voltage_error1(arm_ekf_.X_hat(5));

  if (!disable) {
    *proximal_output = ::std::max(
        -kOperatingVoltage(), ::std::min(kOperatingVoltage(), follower_.U(0)));
    *distal_output = ::std::max(
        -kOperatingVoltage(), ::std::min(kOperatingVoltage(), follower_.U(1)));
    if (state_ != State::PREP_CLIMB) {
      *release_arm_brake = true;
    } else {
      *release_arm_brake = false;
    }
    *claw_closed = claw_closed_;
  }

  status_builder.add_path_distance_to_go(follower_.path_distance_to_go());
  status_builder.add_current_node(current_node_);

  status_builder.add_zeroed(zeroed());
  status_builder.add_estopped(estopped());
  status_builder.add_state(static_cast<int32_t>(state_));
  status_builder.add_grab_state(static_cast<int32_t>(grab_state_));
  status_builder.add_failed_solutions(follower_.failed_solutions());

  arm_ekf_.Predict(follower_.U(), kDt());
  return status_builder.Finish();
}

}  // namespace arm
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2018
