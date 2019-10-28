#include "y2018/control_loops/superstructure/arm/trajectory.h"

#include "Eigen/Dense"
#include "aos/logging/logging.h"
#include "frc971/control_loops/dlqr.h"
#include "frc971/control_loops/jacobian.h"
#include "gflags/gflags.h"
#include "y2018/control_loops/superstructure/arm/dynamics.h"

DEFINE_double(lqr_proximal_pos, 0.15, "Position LQR gain");
DEFINE_double(lqr_proximal_vel, 4.0, "Velocity LQR gain");
DEFINE_double(lqr_distal_pos, 0.20, "Position LQR gain");
DEFINE_double(lqr_distal_vel, 4.0, "Velocity LQR gain");

namespace y2018 {
namespace control_loops {
namespace superstructure {
namespace arm {

Path Path::Reversed(const Path &p) {
  ::std::vector<::std::array<double, 6>> list(p.distances().size());
  for (ssize_t i = list.size() - 1; i >= 0; --i) {
    list[i][0] = p.thetas_[list.size() - 1 - i][0];
    list[i][1] = p.thetas_[list.size() - 1 - i][1];

    list[i][2] = -p.omegas_[list.size() - 1 - i][0];
    list[i][3] = -p.omegas_[list.size() - 1 - i][1];

    list[i][4] = p.alphas_[list.size() - 1 - i][0];
    list[i][5] = p.alphas_[list.size() - 1 - i][1];
  }
  return Path(list);
}

::std::unique_ptr<Path> Path::Reversed(::std::unique_ptr<Path> p) {
  return ::std::unique_ptr<Path>(new Path(Path::Reversed(*p)));
}

Path::Path(::std::vector<::std::array<double, 6>> list) {
  distances_.reserve(list.size());
  ::Eigen::Matrix<double, 2, 1> last_theta;
  bool first = true;

  // Pull apart the initializer list into points, and compute the distance
  // lookup for efficient path lookups.
  for (const auto &e : list) {
    thetas_.push_back(
        (::Eigen::Matrix<double, 2, 1>() << e[0], e[1]).finished());
    omegas_.push_back(
        (::Eigen::Matrix<double, 2, 1>() << e[2], e[3]).finished());
    alphas_.push_back(
        (::Eigen::Matrix<double, 2, 1>() << e[4], e[5]).finished());

    if (first) {
      distances_.push_back(0.0);
    } else {
      distances_.push_back((thetas_.back() - last_theta).norm() +
                           distances_.back());
    }
    first = false;
    last_theta = thetas_.back();
  }
}

Path::Path(::std::initializer_list<::std::array<double, 6>> list) {
  distances_.reserve(list.size());
  ::Eigen::Matrix<double, 2, 1> last_theta;
  bool first = true;

  // Pull apart the initializer list into points, and compute the distance
  // lookup for efficient path lookups.
  for (const auto &e : list) {
    thetas_.push_back(
        (::Eigen::Matrix<double, 2, 1>() << e[0], e[1]).finished());
    omegas_.push_back(
        (::Eigen::Matrix<double, 2, 1>() << e[2], e[3]).finished());
    alphas_.push_back(
        (::Eigen::Matrix<double, 2, 1>() << e[4], e[5]).finished());

    if (first) {
      distances_.push_back(0.0);
    } else {
      distances_.push_back((thetas_.back() - last_theta).norm() +
                           distances_.back());
    }
    first = false;
    last_theta = thetas_.back();
  }
}

::std::vector<double> Trajectory::CurvatureOptimizationPass(
    const ::Eigen::Matrix<double, 2, 2> &alpha_unitizer, double plan_vmax) {
  ::std::vector<double> max_dvelocity_unfiltered;
  max_dvelocity_unfiltered.reserve(num_plan_points_);
  for (size_t i = 0; i < num_plan_points_; ++i) {
    const double distance = DistanceForIndex(i);

    // TODO(austin): We are looking up the index in the path 3 times here.
    const ::Eigen::Matrix<double, 2, 1> theta = path().Theta(distance);
    const ::Eigen::Matrix<double, 2, 1> omega = path().Omega(distance);
    const ::Eigen::Matrix<double, 2, 1> alpha = path().Alpha(distance);
    const ::Eigen::Matrix<double, 4, 1> X =
        (::Eigen::Matrix<double, 4, 1>() << theta(0, 0), 0.0, theta(1, 0), 0.0)
            .finished();
    ::Eigen::Matrix<double, 2, 2> K1;
    ::Eigen::Matrix<double, 2, 2> K2;

    const ::Eigen::Matrix<double, 2, 1> gravity_volts =
        Dynamics::K3_inverse * Dynamics::GravityTorque(X);

    Dynamics::NormilizedMatriciesForState(X, &K1, &K2);

    const ::Eigen::Matrix<double, 2, 2> omega_square =
        (::Eigen::Matrix<double, 2, 2>() << omega(0, 0), 0.0, 0.0, omega(1, 0))
            .finished();

    // Here, we can say that
    //   d^2/dt^2 theta = d^2/dd^2 theta(d) * (d d/dt)^2
    // Normalize so that the max accel is 1, and take magnitudes. This
    // gives us the max velocity we can be at each point due to
    // curvature.
    double min_good_ddot =
        ::std::sqrt(1.0 / ::std::max(0.001, (alpha_unitizer * alpha).norm()));

    const ::Eigen::Matrix<double, 2, 1> vk1 =
        Dynamics::K3_inverse * (K1 * alpha + K2 * omega_square * omega);
    const ::Eigen::Matrix<double, 2, 1> vk2 =
        Dynamics::K3_inverse * Dynamics::K4 * omega;

    // Loop through all the various vmin, plan_vmax combinations.
    for (const double c : {-plan_vmax, plan_vmax}) {
      // Also loop through saturating theta0 and theta1
      for (const ::std::tuple<double, double, double> abgravity :
           {::std::tuple<double, double, double>{vk1(0), vk2(0),
                                                gravity_volts(0)},
            ::std::tuple<double, double, double>{vk1(1), vk2(1),
                                                gravity_volts(1)}}) {
        const double a = ::std::get<0>(abgravity);
        const double b = ::std::get<1>(abgravity);
        const double gravity = ::std::get<2>(abgravity);
        const double sqrt_number = b * b - 4.0 * a * (c - gravity);
        // Throw out imaginary solutions to the quadratic.
        if (sqrt_number > 0) {
          const double sqrt_result = ::std::sqrt(sqrt_number);
          const double ddot1 = (-b + sqrt_result) / (2.0 * a);
          const double ddot2 = (-b - sqrt_result) / (2.0 * a);
          // Loop through both solutions.
          for (const double ddot : {ddot1, ddot2}) {
            const ::Eigen::Matrix<double, 2, 1> U =
                vk1 * ddot * ddot + vk2 * ddot - gravity_volts;

            // Finally, make sure the velocity is positive and valid.
            if ((U.array().abs() <= plan_vmax + 1e-6).all() && ddot > 0.0) {
              min_good_ddot = ::std::min(min_good_ddot, ddot);
            }
          }
        }
      }
    }

    max_dvelocity_unfiltered.push_back(min_good_ddot);
  }
  return max_dvelocity_unfiltered;
}

double Trajectory::FeasableForwardsAcceleration(
    double goal_distance, double goal_velocity, double plan_vmax,
    const ::Eigen::Matrix<double, 2, 2> &alpha_unitizer) const {
  // TODO(austin): We are looking up the index in the path 3 times here.
  const ::Eigen::Matrix<double, 2, 1> theta = path().Theta(goal_distance);
  const ::Eigen::Matrix<double, 2, 1> omega = path().Omega(goal_distance);
  const ::Eigen::Matrix<double, 2, 1> alpha = path().Alpha(goal_distance);

  const ::Eigen::Matrix<double, 4, 1> X =
      (::Eigen::Matrix<double, 4, 1>() << theta(0, 0), 0.0, theta(1, 0), 0.0)
          .finished();

  ::Eigen::Matrix<double, 2, 2> K1;
  ::Eigen::Matrix<double, 2, 2> K2;

  Dynamics::NormilizedMatriciesForState(X, &K1, &K2);

  const ::Eigen::Matrix<double, 2, 2> omega_square =
      (::Eigen::Matrix<double, 2, 2>() << omega(0, 0), 0.0, 0.0, omega(1, 0))
          .finished();

  const ::Eigen::Matrix<double, 2, 1> k_constant =
      Dynamics::K3_inverse *
      ((K1 * alpha + K2 * omega_square * omega) * goal_velocity *
           goal_velocity +
       Dynamics::K4 * omega * goal_velocity - Dynamics::GravityTorque(X));
  const ::Eigen::Matrix<double, 2, 1> k_scalar =
      Dynamics::K3_inverse * K1 * omega;

  const double constraint_goal_acceleration =
      ::std::sqrt(
          ::std::max(0.0, 1.0 - ::std::pow((alpha_unitizer * alpha).norm() *
                                               goal_velocity * goal_velocity,
                                           2))) /
      (alpha_unitizer * omega).norm();

  double goal_acceleration = -::std::numeric_limits<double>::infinity();
  for (double c : {-plan_vmax, plan_vmax}) {
    for (const ::std::pair<double, double> ab :
         {::std::pair<double, double>{k_constant(0, 0), k_scalar(0, 0)},
          ::std::pair<double, double>{k_constant(1, 0), k_scalar(1, 0)}}) {
      const double a = ab.first;
      const double b = ab.second;
      const double voltage_accel = (c - a) / b;
      const ::Eigen::Matrix<double, 2, 1> U =
          k_constant + k_scalar * voltage_accel;

      if ((U.array().abs() <= plan_vmax + 1e-6).all()) {
        goal_acceleration = ::std::max(voltage_accel, goal_acceleration);
      }
    }
  }
  if (goal_acceleration == -::std::numeric_limits<double>::infinity()) {
    // TODO(austin): The math above doesn't always give a valid solution.
    // Figure out why.  It should be an affine transformation.
    return constraint_goal_acceleration;
  }

  return ::std::min(constraint_goal_acceleration, goal_acceleration);
}

double Trajectory::FeasableBackwardsAcceleration(
    double goal_distance, double goal_velocity, double plan_vmax,
    const ::Eigen::Matrix<double, 2, 2> &alpha_unitizer) const {
  const ::Eigen::Matrix<double, 2, 1> theta = path().Theta(goal_distance);
  const ::Eigen::Matrix<double, 2, 1> omega = path().Omega(goal_distance);
  const ::Eigen::Matrix<double, 2, 1> alpha = path().Alpha(goal_distance);
  const ::Eigen::Matrix<double, 4, 1> X =
      (::Eigen::Matrix<double, 4, 1>() << theta(0, 0), 0.0, theta(1, 0), 0.0)
          .finished();
  ::Eigen::Matrix<double, 2, 2> K1;
  ::Eigen::Matrix<double, 2, 2> K2;

  Dynamics::NormilizedMatriciesForState(X, &K1, &K2);

  const ::Eigen::Matrix<double, 2, 2> omega_square =
      (::Eigen::Matrix<double, 2, 2>() << omega(0, 0), 0.0, 0.0, omega(1, 0))
          .finished();

  const ::Eigen::Matrix<double, 2, 1> k_constant =
      Dynamics::K3_inverse *
      ((K1 * alpha + K2 * omega_square * omega) * goal_velocity *
           goal_velocity +
       Dynamics::K4 * omega * goal_velocity - Dynamics::GravityTorque(X));
  const ::Eigen::Matrix<double, 2, 1> k_scalar =
      Dynamics::K3_inverse * K1 * omega;

  const double constraint_goal_acceleration =
      ::std::sqrt(
          ::std::max(0.0, 1.0 - ::std::pow(((alpha_unitizer * alpha).norm() *
                                            goal_velocity * goal_velocity),
                                           2))) /
      (alpha_unitizer * omega).norm();

  double goal_acceleration = -::std::numeric_limits<double>::infinity();
  for (double c : {-plan_vmax, plan_vmax}) {
    for (const ::std::pair<double, double> ab :
         {::std::pair<double, double>{k_constant(0, 0), k_scalar(0, 0)},
          ::std::pair<double, double>{k_constant(1, 0), k_scalar(1, 0)}}) {
      const double a = ab.first;
      const double b = ab.second;
      // This time, we are doing the other pass.  So, find all the
      // decelerations (and flip them) to find the prior velocity.
      const double voltage_accel = (c - a) / b;

      const ::Eigen::Matrix<double, 2, 1> U =
          k_constant + k_scalar * voltage_accel;

      // TODO(austin): This doesn't always give a valid solution.  It really
      // should.  Figure out why.
      if ((U.array().abs() <= plan_vmax + 1e-6).all()) {
        goal_acceleration = ::std::max(-voltage_accel, goal_acceleration);
      }
    }
  }
  if (goal_acceleration == -::std::numeric_limits<double>::infinity()) {
    return constraint_goal_acceleration;
  }

  return ::std::min(goal_acceleration, constraint_goal_acceleration);
}

::std::vector<double> Trajectory::BackwardsOptimizationPass(
    const ::std::vector<double> &max_dvelocity,
    const ::Eigen::Matrix<double, 2, 2> &alpha_unitizer, double plan_vmax) {
  ::std::vector<double> max_dvelocity_back_pass = max_dvelocity;

  // Now, iterate over the list of velocities and constrain the acceleration.
  for (int i = num_plan_points_ - 2; i >= 0; --i) {
    const double previous_velocity = max_dvelocity_back_pass[i + 1];
    const double prev_distance = DistanceForIndex(i + 1);

    // Now, integrate with respect to distance (not time like normal).
    // Subdivide the plan step size into kNumSteps steps.
    double integrated_distance = 0.0;
    double integrated_velocity = previous_velocity;
    constexpr int kNumSteps = 50;
    for (int j = 0; j < kNumSteps; ++j) {
      // Compute the acceleration with respect to time.
      const double acceleration = FeasableBackwardsAcceleration(
          prev_distance - integrated_distance, integrated_velocity, plan_vmax,
          alpha_unitizer);

      // And then integrate it with respect to distance.
      const double integration_step_size_distance =
          step_size_ / static_cast<double>(kNumSteps);
      integrated_distance += integration_step_size_distance;
      integrated_velocity =
          ::std::sqrt(2.0 * acceleration * integration_step_size_distance +
                      ::std::pow(integrated_velocity, 2));
    }
    max_dvelocity_back_pass[i] =
        ::std::min(integrated_velocity, max_dvelocity_back_pass[i]);
  }

  return max_dvelocity_back_pass;
}

::std::vector<double> Trajectory::ForwardsOptimizationPass(
    const ::std::vector<double> &max_dvelocity,
    const ::Eigen::Matrix<double, 2, 2> &alpha_unitizer, double plan_vmax) {
  ::std::vector<double> max_dvelocity_forward_pass = max_dvelocity;
  // Now, iterate over the list of velocities and constrain the acceleration.
  for (size_t i = 1; i < num_plan_points_; ++i) {
    const double previous_velocity = max_dvelocity_forward_pass[i - 1];
    const double previous_distance = DistanceForIndex(i - 1);

    // Now, integrate with respect to distance (not time like normal).
    // Subdivide the plan step size into kNumSteps steps.
    double integrated_distance = 0.0;
    double integrated_velocity = previous_velocity;
    constexpr int kNumSteps = 10;
    for (int j = 0; j < kNumSteps; ++j) {
      // Compute the acceleration with respect to time.
      const double acceleration = FeasableForwardsAcceleration(
          previous_distance + integrated_distance, integrated_velocity,
          plan_vmax, alpha_unitizer);

      // And then integrate it with respect to distance.
      const double integration_step_size_distance =
          step_size_ / static_cast<double>(kNumSteps);
      integrated_distance += integration_step_size_distance;
      integrated_velocity =
          ::std::sqrt(2.0 * acceleration * integration_step_size_distance +
                      ::std::pow(integrated_velocity, 2));
    }

    max_dvelocity_forward_pass[i] =
        ::std::min(integrated_velocity, max_dvelocity_forward_pass[i]);
  }

  return max_dvelocity_forward_pass;
}

void TrajectoryFollower::Reset() {
  next_goal_ = last_goal_ = goal_ = ::Eigen::Matrix<double, 2, 1>::Zero();
  U_unsaturated_ = U_ff_ = U_ = ::Eigen::Matrix<double, 2, 1>::Zero();
  goal_acceleration_ = 0.0;
  saturation_fraction_along_path_ = 0.0;
  omega_.setZero();
  if (trajectory_ != nullptr) {
    set_theta(trajectory_->ThetaT(goal_(0)));
  }
}

::Eigen::Matrix<double, 2, 6> TrajectoryFollower::K_at_state(
    const ::Eigen::Matrix<double, 6, 1> &X,
    const ::Eigen::Matrix<double, 2, 1> &U) {
  const double kProximalPos = FLAGS_lqr_proximal_pos;
  const double kProximalVel = FLAGS_lqr_proximal_vel;
  const double kDistalPos = FLAGS_lqr_distal_pos;
  const double kDistalVel = FLAGS_lqr_distal_vel;
  const ::Eigen::DiagonalMatrix<double, 4> Q =
      (::Eigen::DiagonalMatrix<double, 4>().diagonal()
           << 1.0 / ::std::pow(kProximalPos, 2),
       1.0 / ::std::pow(kProximalVel, 2), 1.0 / ::std::pow(kDistalPos, 2),
       1.0 / ::std::pow(kDistalVel, 2))
          .finished()
          .asDiagonal();

  const ::Eigen::DiagonalMatrix<double, 2> R =
      (::Eigen::DiagonalMatrix<double, 2>().diagonal()
           << 1.0 / ::std::pow(12.0, 2),
       1.0 / ::std::pow(12.0, 2))
          .finished()
          .asDiagonal();

  const ::Eigen::Matrix<double, 4, 4> final_A =
      ::frc971::control_loops::NumericalJacobianX<4, 2>(
          Dynamics::UnboundedDiscreteDynamics, X.block<4, 1>(0, 0), U, 0.00505);
  const ::Eigen::Matrix<double, 4, 2> final_B =
      ::frc971::control_loops::NumericalJacobianU<4, 2>(
          Dynamics::UnboundedDiscreteDynamics, X.block<4, 1>(0, 0), U, 0.00505);

  ::Eigen::Matrix<double, 4, 4> S;
  ::Eigen::Matrix<double, 2, 4> sub_K;
  if (::frc971::controls::dlqr<4, 2>(final_A, final_B, Q, R, &sub_K, &S) == 0) {
    ::Eigen::EigenSolver<::Eigen::Matrix<double, 4, 4>> eigensolver(
        final_A - final_B * sub_K);

    ::Eigen::Matrix<double, 2, 6> K;
    K.setZero();
    K.block<2, 4>(0, 0) = sub_K;
    K(0, 4) = 1.0;
    K(1, 5) = 1.0;
    failed_solutions_ = 0;
    last_K_ = K;
  } else {
    ++failed_solutions_;
  }
  return last_K_;
}

void TrajectoryFollower::USaturationSearch(
    double goal_distance, double last_goal_distance, double goal_velocity,
    double last_goal_velocity, double saturation_fraction_along_path,
    const ::Eigen::Matrix<double, 2, 6> &K,
    const ::Eigen::Matrix<double, 6, 1> &X, const Trajectory &trajectory,
    ::Eigen::Matrix<double, 2, 1> *U, double *saturation_goal_velocity,
    double *saturation_goal_acceleration) {
  double saturation_goal_distance =
      ((goal_distance - last_goal_distance) * saturation_fraction_along_path +
       last_goal_distance);

  const ::Eigen::Matrix<double, 2, 1> theta_t =
      trajectory.ThetaT(saturation_goal_distance);
  *saturation_goal_velocity = trajectory.InterpolateVelocity(
      saturation_goal_distance, last_goal_distance, goal_distance,
      last_goal_velocity, goal_velocity);
  *saturation_goal_acceleration = trajectory.InterpolateAcceleration(
      last_goal_distance, goal_distance, last_goal_velocity, goal_velocity);
  const ::Eigen::Matrix<double, 2, 1> omega_t =
      trajectory.OmegaT(saturation_goal_distance, *saturation_goal_velocity);
  const ::Eigen::Matrix<double, 2, 1> alpha_t =
      trajectory.AlphaT(saturation_goal_distance, *saturation_goal_velocity,
                        *saturation_goal_acceleration);
  const ::Eigen::Matrix<double, 6, 1> R = trajectory.R(theta_t, omega_t);
  const ::Eigen::Matrix<double, 2, 1> U_ff =
      Dynamics::FF_U(R.block<4, 1>(0, 0), omega_t, alpha_t);

  *U = U_ff + K * (R - X);
}

::Eigen::Matrix<double, 2, 1> TrajectoryFollower::PlanNextGoal(
    const ::Eigen::Matrix<double, 2, 1> &goal, double plan_vmax, double dt) {
  // Figure out where we would be if we were to accelerate as fast as
  // our constraints allow.
  ::Eigen::Matrix<double, 2, 1> next_goal = ::frc971::control_loops::RungeKutta(
      [this, &plan_vmax](const ::Eigen::Matrix<double, 2, 1> &X) {
        return (::Eigen::Matrix<double, 2, 1>() << X(1),
                trajectory_->FeasableForwardsAcceleration(
                    X(0), X(1), plan_vmax, trajectory_->alpha_unitizer()))
            .finished();
      },
      goal, dt);

  // Min that with the backwards pass velocity in case the backwards pass
  // wants us to slow down.
  const double next_trajectory_velocity =
      trajectory_->GetDVelocity(next_goal(0));
  if (next_trajectory_velocity < next_goal(1)) {
    next_goal(1) = next_trajectory_velocity;
    // And then recompute how far to go with our new trajectory in mind.
    double goal_acceleration = trajectory_->InterpolateAcceleration(
        goal(0), next_goal(0), goal(1), next_goal(1));
    next_goal(0) = (goal(0) + goal(1) * dt + 0.5 * dt * dt * goal_acceleration);
    next_goal(1) = trajectory_->GetDVelocity(next_goal(0));
  }
  return next_goal;
}

void TrajectoryFollower::Update(const ::Eigen::Matrix<double, 6, 1> &X,
                                bool disabled, double dt, double plan_vmax,
                                double voltage_limit) {
  // TODO(austin): Separate voltage limit for shoulder for no path.
  last_goal_ = goal_;

  if (!has_path()) {
    // No path, so go to the last theta (no velocity).
    last_goal_.setZero();
    next_goal_.setZero();
    goal_.setZero();
    goal_acceleration_ = 0.0;
    saturation_fraction_along_path_ = 0.0;

    if (disabled) {
      U_ff_.setZero();
      U_.setZero();
      U_unsaturated_.setZero();
    } else {
      const ::Eigen::Matrix<double, 6, 1> R =
          trajectory_->R(theta_, ::Eigen::Matrix<double, 2, 1>::Zero());

      U_ff_ = Dynamics::FF_U(X.block<4, 1>(0, 0),
                             ::Eigen::Matrix<double, 2, 1>::Zero(),
                             ::Eigen::Matrix<double, 2, 1>::Zero());
      const ::Eigen::Matrix<double, 2, 6> K = K_at_state(X, U_ff_);
      U_ = U_unsaturated_ = U_ff_ + K * (R - X);

      U_ = U_.array().max(-voltage_limit).min(voltage_limit);
    }
    return;
  }

  if (disabled) {
    // If we are disabled, it's likely for a bit of time.  So, lets freeze
    // ourselves on the path (accept the previous motion, but then zero out the
    // velocity).  Set all outputs to 0 as well.
    next_goal_(1) = 0.0;
    goal_ = next_goal_;
    goal_acceleration_ = 0.0;
    U_unsaturated_.setZero();
    U_.setZero();
    U_ff_.setZero();
    saturation_fraction_along_path_ = 1.0;
    theta_ = trajectory_->ThetaT(goal_(0));
    omega_.setZero();
    return;
  }

  // To avoid exposing the new goals before the outer code has a chance to
  // querry the internal state, move to the new goals here.
  goal_ = next_goal_;

  if (::std::abs(goal_(0) - path()->length()) < 1e-2) {
    // If we go backwards along the path near the goal, snap us to the end
    // point or we'll never actually finish.
    if (goal_acceleration_ * dt + goal_(1) < 0.0 ||
        goal_(0) > path()->length()) {
      goal_(0) = path()->length();
      goal_(1) = 0.0;
    }
  }

  if (goal_(0) == path()->length()) {
    next_goal_(0) = goal_(0);
    next_goal_(1) = 0.0;
    goal_acceleration_ = 0.0;
  } else {
    // Figure out where we would be if we were to accelerate as fast as
    // our constraints allow.
    next_goal_ = PlanNextGoal(goal_, plan_vmax, dt);

    goal_acceleration_ = trajectory_->InterpolateAcceleration(
        goal_(0), next_goal_(0), goal_(1), next_goal_(1));
  }

  const ::Eigen::Matrix<double, 2, 1> theta_t = trajectory_->ThetaT(goal_(0));
  const ::Eigen::Matrix<double, 2, 1> omega_t =
      trajectory_->OmegaT(goal_(0), goal_(1));
  const ::Eigen::Matrix<double, 2, 1> alpha_t =
      trajectory_->AlphaT(goal_(0), goal_(1), goal_acceleration_);

  const ::Eigen::Matrix<double, 6, 1> R = trajectory_->R(theta_t, omega_t);

  U_ff_ = Dynamics::FF_U(R.block<4, 1>(0, 0), omega_t, alpha_t);

  const ::Eigen::Matrix<double, 2, 6> K = K_at_state(X, U_ff_);
  U_ = U_unsaturated_ = U_ff_ + K * (R - X);

  // Ok, now we know if we are staturated or not.  If we are, time to search
  // between here and our previous goal either until we find a state where we
  // aren't saturated, or we are really close to our starting point.
  saturation_fraction_along_path_ = 1.0;
  if ((U_.array().abs() > voltage_limit).any()) {
    // Saturated.  Let's do a binary search.
    double step_size;
    if ((goal_(0) - last_goal_(0)) < 1e-8) {
      // print "Not bothering to move"
      // Avoid the divide by 0 when interpolating.  Just don't move since we
      // are saturated.
      saturation_fraction_along_path_ = 0.0;
      step_size = 0.0;
    } else {
      saturation_fraction_along_path_ = 0.5;
      step_size = 0.5;
    }

    // Pull us back to the previous point until we aren't saturated anymore.
    double saturation_goal_velocity;
    double saturation_goal_acceleration;
    while (step_size > 0.01) {
      USaturationSearch(goal_(0), last_goal_(0), goal_(1), last_goal_(1),
                        saturation_fraction_along_path_, K, X, *trajectory_,
                        &U_, &saturation_goal_velocity,
                        &saturation_goal_acceleration);
      step_size = step_size * 0.5;
      if ((U_.array().abs() > voltage_limit).any()) {
        saturation_fraction_along_path_ -= step_size;
      } else {
        saturation_fraction_along_path_ += step_size;
      }
    }

    goal_(0) = ((goal_(0) - last_goal_(0)) * saturation_fraction_along_path_ +
                last_goal_(0));
    goal_(1) = saturation_goal_velocity;

    next_goal_ = PlanNextGoal(goal_, plan_vmax, dt);

    goal_acceleration_ = trajectory_->InterpolateAcceleration(
        goal_(0), next_goal_(0), goal_(1), next_goal_(1));

    U_ = U_.array().max(-voltage_limit).min(voltage_limit);
  }
  theta_ = trajectory_->ThetaT(goal_(0));
  omega_ = trajectory_->OmegaT(goal_(0), goal_(1));
}

}  // namespace arm
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2018
