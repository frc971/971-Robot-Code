#include "y2023/control_loops/superstructure/arm/trajectory.h"

#include "Eigen/Dense"
#include "aos/logging/logging.h"
#include "frc971/control_loops/dlqr.h"
#include "frc971/control_loops/double_jointed_arm/dynamics.h"
#include "frc971/control_loops/jacobian.h"
#include "frc971/control_loops/runge_kutta.h"
#include "gflags/gflags.h"
#include "y2023/control_loops/superstructure/arm/arm_trajectories_generated.h"

DEFINE_double(lqr_proximal_pos, 0.5, "Position LQR gain");
DEFINE_double(lqr_proximal_vel, 5, "Velocity LQR gain");
DEFINE_double(lqr_distal_pos, 0.5, "Position LQR gain");
DEFINE_double(lqr_distal_vel, 5, "Velocity LQR gain");

namespace y2023 {
namespace control_loops {
namespace superstructure {
namespace arm {

::Eigen::Matrix<double, 3, 1> CosSpline::Theta(double alpha) const {
  ::Eigen::Matrix<double, 3, 1> result;
  result.block<2, 1>(0, 0) = spline_.Theta(alpha);

  const std::pair<AlphaTheta, AlphaTheta> roll_points = RollPoints(alpha);
  const double alpha_percent =
      (alpha - roll_points.first.alpha) /
      (roll_points.second.alpha - roll_points.first.alpha);

  result(2) = (0.5 - 0.5 * std::cos(alpha_percent * M_PI)) *
                  (roll_points.second.theta - roll_points.first.theta) +
              roll_points.first.theta;
  return result;
}

::Eigen::Matrix<double, 3, 1> CosSpline::Omega(double alpha) const {
  ::Eigen::Matrix<double, 3, 1> result;
  result.block<2, 1>(0, 0) = spline_.Omega(alpha);

  const std::pair<AlphaTheta, AlphaTheta> roll_points = RollPoints(alpha);
  const double dalpha = (roll_points.second.alpha - roll_points.first.alpha);
  const double alpha_percent = (alpha - roll_points.first.alpha) / dalpha;

  result(2) = 0.5 * std::sin(alpha_percent * M_PI) *
              (roll_points.second.theta - roll_points.first.theta) * M_PI /
              dalpha;
  return result;
}

::Eigen::Matrix<double, 3, 1> CosSpline::Alpha(double alpha) const {
  ::Eigen::Matrix<double, 3, 1> result;
  result.block<2, 1>(0, 0) = spline_.Alpha(alpha);

  const std::pair<AlphaTheta, AlphaTheta> roll_points = RollPoints(alpha);
  const double dalpha = (roll_points.second.alpha - roll_points.first.alpha);
  const double alpha_percent = (alpha - roll_points.first.alpha) / dalpha;

  result(2) = 0.5 * std::cos(alpha_percent * M_PI) *
              (roll_points.second.theta - roll_points.first.theta) * M_PI *
              M_PI / dalpha / dalpha;
  return result;
}

std::pair<CosSpline::AlphaTheta, CosSpline::AlphaTheta> CosSpline::RollPoints(
    double alpha) const {
  if (alpha <= 0.0) {
    return std::make_pair(roll_[0], roll_[1]);
  }
  if (alpha >= 1.0) {
    return std::make_pair(roll_[roll_.size() - 2], roll_[roll_.size() - 1]);
  }

  // Find the distance right below our number using a binary search.
  size_t after = ::std::distance(
      roll_.begin(), ::std::lower_bound(roll_.begin(), roll_.end(), alpha,
                                        [](AlphaTheta at, double alpha) {
                                          return at.alpha < alpha;
                                        }));
  DCHECK_GT(after, 0u);
  size_t before = after - 1;
  DCHECK_LT(before, roll_.size());
  return std::make_pair(roll_[before], roll_[after]);
}

CosSpline CosSpline::Reversed() const {
  std::vector<AlphaTheta> new_roll(roll_.size());
  for (size_t i = 0; i < roll_.size(); ++i) {
    new_roll[roll_.size() - 1 - i] = {1.0 - roll_[i].alpha, roll_[i].theta};
  }

  Eigen::Matrix<double, 2, 4> new_control_points;
  new_control_points = spline_.control_points().rowwise().reverse();
  return CosSpline(NSpline<4, 2>(new_control_points), std::move(new_roll));
}

::Eigen::Matrix<double, 3, 1> Path::Theta(double distance) const {
  const double alpha = DistanceToAlpha(distance);
  return spline_.Theta(alpha);
}

::Eigen::Matrix<double, 3, 1> Path::Omega(double distance) const {
  const double alpha = DistanceToAlpha(distance);
  return spline_.Omega(alpha).normalized();
}

::Eigen::Matrix<double, 3, 1> Path::Alpha(double distance) const {
  const double alpha = DistanceToAlpha(distance);
  const ::Eigen::Matrix<double, 3, 1> dspline_point = spline_.Omega(alpha);
  const ::Eigen::Matrix<double, 3, 1> ddspline_point = spline_.Alpha(alpha);

  const double squared_norm = dspline_point.squaredNorm();

  return ddspline_point / squared_norm -
         dspline_point * (dspline_point.transpose() * ddspline_point) /
             ::std::pow(squared_norm, 2);
}

std::vector<float> Path::BuildDistances(size_t num_alpha) {
  num_alpha = num_alpha ? num_alpha : 100;
  std::vector<float> distances;
  distances.push_back(0.0);

  const double dalpha = 1.0 / static_cast<double>(num_alpha - 1);
  double last_alpha = 0.0;
  for (size_t i = 1; i < num_alpha; ++i) {
    const double alpha = dalpha * i;
    distances.push_back(
        distances.back() +
        GaussianQuadrature5(
            [this](double alpha) { return this->spline_.Omega(alpha).norm(); },
            last_alpha, alpha));
    last_alpha = alpha;
  }
  return distances;
}

double Path::DistanceToAlpha(double distance) const {
  if (distance <= 0.0) {
    return 0.0;
  }
  if (distance >= length()) {
    return 1.0;
  }

  // Find the distance right below our number using a binary search.
  size_t after = ::std::distance(
      distances().begin(),
      ::std::lower_bound(distances().begin(), distances().end(), distance));
  size_t before = after - 1;
  const double distance_step_size =
      (1.0 / static_cast<double>(distances().size() - 1));

  const double alpha = (distance - distances()[before]) /
                           (distances()[after] - distances()[before]) *
                           distance_step_size +
                       static_cast<double>(before) * distance_step_size;
  CHECK_GT(alpha, 0.0);
  CHECK_LE(alpha, 1.0);
  return alpha;
}

Path Path::Reversed() const { return Path(spline_.Reversed()); }

::std::unique_ptr<Path> Path::Reversed(::std::unique_ptr<Path> p) {
  return ::std::make_unique<Path>(p->Reversed());
}

Trajectory::Trajectory(const frc971::control_loops::arm::Dynamics *dynamics,
                       const StateFeedbackHybridPlant<3, 1, 1> *roll,
                       const TrajectoryFbs &trajectory_fbs)
    : dynamics_(dynamics),
      roll_(roll),
      num_plan_points_(trajectory_fbs.num_plan_points()),
      step_size_(trajectory_fbs.step_size()),
      max_dvelocity_unfiltered_(
          trajectory_fbs.max_dvelocity_unfiltered()->data(),
          trajectory_fbs.max_dvelocity_unfiltered()->data() +
              trajectory_fbs.max_dvelocity_unfiltered()->size()),
      max_dvelocity_backwards_voltage_(
          trajectory_fbs.max_dvelocity_backward_voltage()->data(),
          trajectory_fbs.max_dvelocity_backward_voltage()->data() +
              trajectory_fbs.max_dvelocity_backward_voltage()->size()),
      max_dvelocity_backwards_accel_(
          trajectory_fbs.max_dvelocity_backward_accel()->data(),
          trajectory_fbs.max_dvelocity_backward_accel()->data() +
              trajectory_fbs.max_dvelocity_backward_accel()->size()),
      max_dvelocity_forwards_accel_(
          trajectory_fbs.max_dvelocity_forwards_accel()->data(),
          trajectory_fbs.max_dvelocity_forwards_accel()->data() +
              trajectory_fbs.max_dvelocity_forwards_accel()->size()),
      max_dvelocity_forwards_voltage_(
          trajectory_fbs.max_dvelocity_forwards_voltage()->data(),
          trajectory_fbs.max_dvelocity_forwards_voltage()->data() +
              trajectory_fbs.max_dvelocity_forwards_voltage()->size()),
      alpha_unitizer_(trajectory_fbs.alpha_unitizer()->data()) {
  auto control_points = ::Eigen::Matrix<double, 2, 4>(
      trajectory_fbs.path()->spline()->spline()->control_points()->data());
  NSpline<4, 2> spline(control_points);

  std::vector<CosSpline::AlphaTheta> alpha_roll;

  for (const auto &alpha_theta : *trajectory_fbs.path()->spline()->roll()) {
    CosSpline::AlphaTheta atheta = {
        .alpha = alpha_theta->alpha(),
        .theta = alpha_theta->theta(),
    };

    alpha_roll.emplace_back(atheta);
  }

  CosSpline cos_spline(spline, alpha_roll);

  path_ = std::make_unique<Path>(cos_spline,
                                 (trajectory_fbs.path()->distances()->data(),
                                  trajectory_fbs.path()->distances()->size()));
}

double Trajectory::MaxCurvatureSpeed(
    double goal_distance, const ::Eigen::Matrix<double, 3, 3> &alpha_unitizer,
    double plan_vmax) {
  // TODO(austin): We are looking up the index in the path 3 times here.
  const ::Eigen::Matrix<double, 3, 1> theta = path().Theta(goal_distance);
  const ::Eigen::Matrix<double, 3, 1> omega = path().Omega(goal_distance);
  const ::Eigen::Matrix<double, 3, 1> alpha = path().Alpha(goal_distance);

  const ::Eigen::Matrix<double, 4, 1> X =
      (::Eigen::Matrix<double, 4, 1>() << theta(0, 0), 0.0, theta(1, 0), 0.0)
          .finished();

  ::Eigen::Matrix<double, 2, 2> K1;
  ::Eigen::Matrix<double, 2, 2> K2;

  const ::Eigen::Matrix<double, 2, 1> gravity_volts =
      dynamics_->K3_inverse() * dynamics_->GravityTorque(X);

  dynamics_->NormilizedMatriciesForState(X, &K1, &K2);
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

  // Now, solve for the max speed we can follow the path without hitting the
  // vmax limit.  This makes it such that we can always hold vmax and follow
  // the path.  This helps when there are cases where holding constant speed
  // into a reducing radius turn will saturate our voltage limit. Technically,
  // there are cases where faster paths exist, but they are a lot harder to
  // solve due to the nonlinearities.
  const ::Eigen::Matrix<double, 2, 1> vk1 =
      dynamics_->K3_inverse() * (K1 * alpha.block<2, 1>(0, 0) +
                                 K2 * omega_square * omega.block<2, 1>(0, 0));
  const ::Eigen::Matrix<double, 2, 1> vk2 =
      dynamics_->K3_inverse() * dynamics_->K4() * omega.block<2, 1>(0, 0);

  // Loop through all the various vmin, plan_vmax combinations.
  for (const double c : {-plan_vmax, plan_vmax}) {
    // Also loop through saturating theta0 and theta1
    for (const ::std::tuple<double, double, double> &abgravity :
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

  return min_good_ddot;
}

::std::vector<double> Trajectory::CurvatureOptimizationPass(
    const ::Eigen::Matrix<double, 3, 3> &alpha_unitizer, double plan_vmax) {
  ::std::vector<double> max_dvelocity_unfiltered;
  max_dvelocity_unfiltered.reserve(num_plan_points_);
  for (size_t i = 0; i < num_plan_points_; ++i) {
    const double distance = DistanceForIndex(i);

    max_dvelocity_unfiltered.push_back(
        MaxCurvatureSpeed(distance, alpha_unitizer, plan_vmax));
  }
  return max_dvelocity_unfiltered;
}

double Trajectory::FeasableForwardsAcceleration(
    double goal_distance, double goal_velocity, double /*plan_vmax*/,
    const ::Eigen::Matrix<double, 3, 3> &alpha_unitizer) const {
  const ::Eigen::Matrix<double, 3, 1> omega = path().Omega(goal_distance);
  const ::Eigen::Matrix<double, 3, 1> alpha = path().Alpha(goal_distance);

  return ::std::sqrt(
             ::std::max(0.0, 1.0 - ::std::pow((alpha_unitizer * alpha).norm() *
                                                  goal_velocity * goal_velocity,
                                              2))) /
         (alpha_unitizer * omega).norm();
}

double Trajectory::FeasableForwardsVoltage(
    double goal_distance, double goal_velocity, double plan_vmax,
    const ::Eigen::Matrix<double, 3, 3> &alpha_unitizer) const {
  // TODO(austin): We are looking up the index in the path 3 times here.
  const ::Eigen::Matrix<double, 3, 1> theta = path().Theta(goal_distance);
  const ::Eigen::Matrix<double, 3, 1> omega = path().Omega(goal_distance);
  const ::Eigen::Matrix<double, 3, 1> alpha = path().Alpha(goal_distance);

  const ::Eigen::Matrix<double, 4, 1> arm_X =
      (::Eigen::Matrix<double, 4, 1>() << theta(0, 0), 0.0, theta(1, 0), 0.0)
          .finished();

  ::Eigen::Matrix<double, 2, 2> K1;
  ::Eigen::Matrix<double, 2, 2> K2_normalized;

  dynamics_->NormilizedMatriciesForState(arm_X, &K1, &K2_normalized);

  const ::Eigen::Matrix<double, 2, 2> omega_square =
      (::Eigen::Matrix<double, 2, 2>() << omega(0, 0), 0.0, 0.0, omega(1, 0))
          .finished();

  // The derivitives of f(d) wrt t are:
  // theta = f(d)
  // dtheta/dt = d/dd f(d) * dd/dt
  // d^2 theta/dt^2 = d^2/dd^2 f(d) * (dd/dt)^2 + d^2d/dt^2 * d/dd f(d)
  //
  // And, our arm dynamics are:
  // K1 d^2 theta/dt^2 + K2_normalized * diag(dtheta/dt) * dtheta/dt = k3 * V -
  // K4 * dtheta/dt
  //
  // And our roll dynamics are:
  //
  // d^2 theta2/dt^2 = A[1, 1] d theta2/dt + B[1, 0] * V
  //
  // Plug the derivitives in to our dynamics, solve for d^2d/dt^2, and ship it!
  //
  // We want things in the form V = k_constant + k_scalar * d^2d/dt^2

  const ::Eigen::Matrix<double, 3, 1> k_constant =
      (::Eigen::Matrix<double, 3, 1>()
           << (dynamics_->K3_inverse() *
               ((K1 * alpha.block<2, 1>(0, 0) +
                 K2_normalized * omega_square * omega.block<2, 1>(0, 0)) *
                    goal_velocity * goal_velocity +
                dynamics_->K4() * omega.block<2, 1>(0, 0) * goal_velocity -
                dynamics_->GravityTorque(arm_X))),
       ((alpha(2, 0) * goal_velocity * goal_velocity -
         roll_->coefficients().A_continuous(1, 1) * omega(2, 0) *
             goal_velocity) /
        roll_->coefficients().B_continuous(1, 0)))
          .finished();

  const ::Eigen::Matrix<double, 3, 1> k_scalar =
      (::Eigen::Matrix<double, 3, 1>()
           << dynamics_->K3_inverse() * K1 * omega.block<2, 1>(0, 0),
       (omega(2, 0) / roll_->coefficients().B_continuous(1, 0)))
          .finished();

  const double constraint_goal_acceleration =
      ::std::sqrt(
          ::std::max(0.0, 1.0 - ::std::pow((alpha_unitizer * alpha).norm() *
                                               goal_velocity * goal_velocity,
                                           2))) /
      (alpha_unitizer * omega).norm();

  double min_goal_acceleration = ::std::numeric_limits<double>::infinity();
  double max_goal_acceleration = -::std::numeric_limits<double>::infinity();
  for (double c : {-plan_vmax, plan_vmax}) {
    for (const ::std::pair<double, double> &ab :
         {::std::pair<double, double>{k_constant(0, 0), k_scalar(0, 0)},
          ::std::pair<double, double>{k_constant(1, 0), k_scalar(1, 0)},
          ::std::pair<double, double>{k_constant(2, 0), k_scalar(2, 0)}}) {
      const double a = ab.first;
      const double b = ab.second;
      const double voltage_accel = (c - a) / b;
      const ::Eigen::Matrix<double, 3, 1> U =
          k_constant + k_scalar * voltage_accel;

      VLOG(2) << "Trying, U is " << U.transpose() << ", accel "
              << voltage_accel;
      if ((U.array().abs() <= plan_vmax + 1e-6).all()) {
        min_goal_acceleration = std::min(voltage_accel, min_goal_acceleration);
        max_goal_acceleration = std::max(voltage_accel, max_goal_acceleration);
      }
    }
  }
  if (min_goal_acceleration == ::std::numeric_limits<double>::infinity() ||
      max_goal_acceleration == -::std::numeric_limits<double>::infinity()) {
    // TODO(austin): The math above doesn't always give a valid solution.
    // This happens when things get pretty ill-conditioned.  Figure out what to
    // do about it.
    VLOG(1)
        << "Failed to find a valid accel at distance " << goal_distance
        << ", voltage "
        << (k_constant + k_scalar * constraint_goal_acceleration).transpose()
        << ", accel " << constraint_goal_acceleration << " vs vmax "
        << plan_vmax;
    return constraint_goal_acceleration;
  }

  const double goal_acceleration =
      std::abs(max_goal_acceleration - constraint_goal_acceleration) <
              std::abs(min_goal_acceleration - constraint_goal_acceleration)
          ? max_goal_acceleration
          : min_goal_acceleration;

  if (min_goal_acceleration < constraint_goal_acceleration &&
      constraint_goal_acceleration < max_goal_acceleration) {
    VLOG(2)
        << "Solved valid accel at distance " << goal_distance << ", voltage "
        << (k_constant + k_scalar * constraint_goal_acceleration).transpose()
        << ", accel " << constraint_goal_acceleration
        << ", overall accel limited, U limit is " << goal_acceleration
        << " with U of "
        << (k_constant + k_scalar * goal_acceleration).transpose();

    if (!((k_constant + k_scalar * constraint_goal_acceleration)
              .array()
              .abs() <= plan_vmax + 1e-6)
             .all()) {
      LOG(FATAL) << "Accel in range, but constraint voltage out of range.";
    }
    return constraint_goal_acceleration;
  }

  VLOG(2) << "Solved valid accel at distance " << goal_distance << ", voltage "
          << (k_constant + k_scalar * goal_acceleration).transpose()
          << ", decel limits [" << min_goal_acceleration << ", "
          << max_goal_acceleration << "], picked " << goal_acceleration
          << " voltage limited, constraint accel "
          << constraint_goal_acceleration << " constraint voltage "
          << (k_constant + k_scalar * constraint_goal_acceleration).transpose();

  return goal_acceleration;
}

double Trajectory::FeasableBackwardsAcceleration(
    double goal_distance, double goal_velocity, double /*plan_vmax*/,
    const ::Eigen::Matrix<double, 3, 3> &alpha_unitizer) const {
  const ::Eigen::Matrix<double, 3, 1> omega = path().Omega(goal_distance);
  const ::Eigen::Matrix<double, 3, 1> alpha = path().Alpha(goal_distance);

  return ::std::sqrt(
             ::std::max(0.0, 1.0 - ::std::pow(((alpha_unitizer * alpha).norm() *
                                               goal_velocity * goal_velocity),
                                              2))) /
         (alpha_unitizer * omega).norm();
}

double Trajectory::FeasableBackwardsVoltage(
    double goal_distance, double goal_velocity, double plan_vmax,
    const ::Eigen::Matrix<double, 3, 3> &alpha_unitizer) const {
  const ::Eigen::Matrix<double, 3, 1> theta = path().Theta(goal_distance);
  const ::Eigen::Matrix<double, 3, 1> omega = path().Omega(goal_distance);
  const ::Eigen::Matrix<double, 3, 1> alpha = path().Alpha(goal_distance);
  const ::Eigen::Matrix<double, 4, 1> arm_X =
      (::Eigen::Matrix<double, 4, 1>() << theta(0, 0), 0.0, theta(1, 0), 0.0)
          .finished();
  ::Eigen::Matrix<double, 2, 2> K1;
  ::Eigen::Matrix<double, 2, 2> K2_normalized;

  dynamics_->NormilizedMatriciesForState(arm_X, &K1, &K2_normalized);

  const ::Eigen::Matrix<double, 2, 2> omega_square =
      (::Eigen::Matrix<double, 2, 2>() << omega(0, 0), 0.0, 0.0, omega(1, 0))
          .finished();

  const ::Eigen::Matrix<double, 3, 1> k_constant =
      (::Eigen::Matrix<double, 3, 1>()
           << dynamics_->K3_inverse() *
                  ((K1 * alpha.block<2, 1>(0, 0) +
                    K2_normalized * omega_square * omega.block<2, 1>(0, 0)) *
                       goal_velocity * goal_velocity +
                   dynamics_->K4() * omega.block<2, 1>(0, 0) * goal_velocity -
                   dynamics_->GravityTorque(arm_X)),
       ((alpha(2, 0) * goal_velocity * goal_velocity -
         roll_->coefficients().A_continuous(1, 1) * omega(2, 0) *
             goal_velocity) /
        roll_->coefficients().B_continuous(1, 0)))
          .finished();
  const ::Eigen::Matrix<double, 3, 1> k_scalar =
      (::Eigen::Matrix<double, 3, 1>()
           << dynamics_->K3_inverse() * K1 * omega.block<2, 1>(0, 0),
       (omega(2, 0) / roll_->coefficients().B_continuous(1, 0)))
          .finished();

  const double constraint_goal_acceleration =
      ::std::sqrt(
          ::std::max(0.0, 1.0 - ::std::pow(((alpha_unitizer * alpha).norm() *
                                            goal_velocity * goal_velocity),
                                           2))) /
      (alpha_unitizer * omega).norm();

  double min_goal_acceleration = ::std::numeric_limits<double>::infinity();
  double max_goal_acceleration = -::std::numeric_limits<double>::infinity();
  for (double c : {-plan_vmax, plan_vmax}) {
    for (const ::std::pair<double, double> &ab :
         {::std::pair<double, double>{k_constant(0, 0), k_scalar(0, 0)},
          ::std::pair<double, double>{k_constant(1, 0), k_scalar(1, 0)},
          ::std::pair<double, double>{k_constant(2, 0), k_scalar(2, 0)}}) {
      const double a = ab.first;
      const double b = ab.second;
      // This time, we are doing the other pass.  So, find all the
      // decelerations (and flip them) to find the prior velocity.
      const double voltage_accel = (c - a) / b;

      const ::Eigen::Matrix<double, 3, 1> U =
          k_constant + k_scalar * voltage_accel;

      // TODO(austin): This doesn't always give a valid solution.  It really
      // should.  Figure out why.
      VLOG(2) << "Trying, U is " << U.transpose() << ", accel "
              << voltage_accel;
      if ((U.array().abs() <= plan_vmax + 1e-6).all()) {
        min_goal_acceleration = std::min(-voltage_accel, min_goal_acceleration);
        max_goal_acceleration = std::max(-voltage_accel, max_goal_acceleration);
      }
    }
  }

  if (min_goal_acceleration == ::std::numeric_limits<double>::infinity() ||
      max_goal_acceleration == -::std::numeric_limits<double>::infinity()) {
    VLOG(1)
        << "Failed to find a valid decel at distance " << goal_distance
        << ", voltage "
        << (k_constant + k_scalar * constraint_goal_acceleration).transpose()
        << ", accel " << constraint_goal_acceleration << "  vs vmax "
        << plan_vmax;
    return constraint_goal_acceleration;
  }

  if (min_goal_acceleration < constraint_goal_acceleration &&
      constraint_goal_acceleration < max_goal_acceleration) {
    VLOG(2)
        << "Solved valid decel at distance " << goal_distance << ", voltage "
        << (k_constant - k_scalar * constraint_goal_acceleration).transpose()
        << ", decel " << min_goal_acceleration
        << " <= " << constraint_goal_acceleration
        << " <= " << max_goal_acceleration << ", accel limited";

    if (!((k_constant - k_scalar * constraint_goal_acceleration)
              .array()
              .abs() <= plan_vmax + 1e-6)
             .all()) {
      LOG(FATAL) << "Accel in range, but constraint voltage out of range.";
    }

    return constraint_goal_acceleration;
  }

  const double goal_acceleration =
      std::abs(max_goal_acceleration - constraint_goal_acceleration) <
              std::abs(min_goal_acceleration - constraint_goal_acceleration)
          ? max_goal_acceleration
          : min_goal_acceleration;

  VLOG(2) << "Solved valid decel at distance " << goal_distance << ", voltage "
          << (k_constant - k_scalar * goal_acceleration).transpose()
          << ", decel limits [" << min_goal_acceleration << ", "
          << max_goal_acceleration << "], picked " << goal_acceleration
          << " voltage limited, constraint accel "
          << constraint_goal_acceleration << " constraint voltage "
          << (k_constant - k_scalar * constraint_goal_acceleration).transpose();

  return goal_acceleration;
}

template <typename F>
double IntegrateAccelForDistance(const F &fn, double v, double x, double dx) {
  // Use a trick from
  // https://www.johndcook.com/blog/2012/02/21/care-and-treatment-of-singularities/
  const double a0 = fn(x, v);

  return (frc971::control_loops::RungeKuttaSteps(
              [&fn, &a0](double t, double y) {
                // Since we know that a0 == a(0) and that they are asymtotically
                // the same at 0, we know that the limit is 0 at 0.  This is
                // true because when starting from a stop, under sane
                // accelerations, we can assume that we will start with a
                // constant acceleration.  So, hard-code it.
                if (std::abs(y) < 1e-6) {
                  return 0.0;
                }
                return (fn(t, y) - a0) / y;
              },
              v, x, dx, 10) -
          v) +
         std::sqrt(2.0 * a0 * dx + v * v);
}

::std::vector<double> Trajectory::BackwardsOptimizationAccelerationPass(
    const ::std::vector<double> &max_dvelocity,
    const ::Eigen::Matrix<double, 3, 3> &alpha_unitizer, double plan_vmax) {
  ::std::vector<double> max_dvelocity_back_pass = max_dvelocity;

  // Now, iterate over the list of velocities and constrain the acceleration.
  for (int i = num_plan_points_ - 2; i >= 0; --i) {
    const double previous_velocity = max_dvelocity_back_pass[i + 1];
    const double previous_distance = DistanceForIndex(i + 1);

    // Now, integrate with respect to distance (not time like normal).
    const double integrated_velocity = IntegrateAccelForDistance(
        [&](double x, double v) {
          return FeasableBackwardsAcceleration(x, v, plan_vmax, alpha_unitizer);
        },
        previous_velocity, previous_distance, step_size_);
    max_dvelocity_back_pass[i] =
        ::std::min(integrated_velocity, max_dvelocity_back_pass[i]);
  }

  return max_dvelocity_back_pass;
}

::std::vector<double> Trajectory::BackwardsOptimizationVoltagePass(
    const ::std::vector<double> &max_dvelocity,
    const ::Eigen::Matrix<double, 3, 3> &alpha_unitizer, double plan_vmax) {
  ::std::vector<double> max_dvelocity_back_pass = max_dvelocity;

  // Now, iterate over the list of velocities and constrain the acceleration.
  for (int i = num_plan_points_ - 2; i >= 0; --i) {
    const double previous_velocity = max_dvelocity_back_pass[i + 1];
    const double previous_distance = DistanceForIndex(i + 1);

    // Now, integrate with respect to distance (not time like normal).
    const double integrated_velocity = IntegrateAccelForDistance(
        [&](double x, double v) {
          return FeasableBackwardsVoltage(x, v, plan_vmax, alpha_unitizer);
        },
        previous_velocity, previous_distance, step_size_);
    max_dvelocity_back_pass[i] =
        ::std::min(integrated_velocity, max_dvelocity_back_pass[i]);
  }

  return max_dvelocity_back_pass;
}

::std::vector<double> Trajectory::ForwardsOptimizationAccelerationPass(
    const ::std::vector<double> &max_dvelocity,
    const ::Eigen::Matrix<double, 3, 3> &alpha_unitizer, double plan_vmax) {
  ::std::vector<double> max_dvelocity_forwards_pass = max_dvelocity;
  // Now, iterate over the list of velocities and constrain the acceleration.
  for (size_t i = 1; i < num_plan_points_; ++i) {
    const double previous_velocity = max_dvelocity_forwards_pass[i - 1];
    const double previous_distance = DistanceForIndex(i - 1);

    // Now, integrate with respect to distance (not time like normal).
    const double integrated_velocity = IntegrateAccelForDistance(
        [&](double x, double v) {
          return FeasableForwardsAcceleration(x, v, plan_vmax, alpha_unitizer);
        },
        previous_velocity, previous_distance, step_size_);

    max_dvelocity_forwards_pass[i] =
        ::std::min(integrated_velocity, max_dvelocity_forwards_pass[i]);
  }

  return max_dvelocity_forwards_pass;
}

::std::vector<double> Trajectory::ForwardsOptimizationVoltagePass(
    const ::std::vector<double> &max_dvelocity,
    const ::Eigen::Matrix<double, 3, 3> &alpha_unitizer, double plan_vmax) {
  ::std::vector<double> max_dvelocity_forwards_pass = max_dvelocity;
  // Now, iterate over the list of velocities and constrain the acceleration.
  for (size_t i = 1; i < num_plan_points_; ++i) {
    const double previous_velocity = max_dvelocity_forwards_pass[i - 1];
    const double previous_distance = DistanceForIndex(i - 1);

    // Now, integrate with respect to distance (not time like normal).
    const double integrated_velocity = IntegrateAccelForDistance(
        [&](double x, double v) {
          return FeasableForwardsVoltage(x, v, plan_vmax, alpha_unitizer);
        },
        previous_velocity, previous_distance, step_size_);

    max_dvelocity_forwards_pass[i] =
        ::std::min(integrated_velocity, max_dvelocity_forwards_pass[i]);
  }

  return max_dvelocity_forwards_pass;
}

void TrajectoryFollower::Reset() {
  next_goal_ = last_goal_ = goal_ = ::Eigen::Matrix<double, 2, 1>::Zero();
  U_unsaturated_ = U_ff_ = U_ = ::Eigen::Matrix<double, 3, 1>::Zero();
  goal_acceleration_ = 0.0;
  saturation_fraction_along_path_ = 0.0;
  omega_.setZero();
  if (trajectory_ != nullptr) {
    set_theta(trajectory_->ThetaT(goal_(0)));
  }
}

::Eigen::Matrix<double, 2, 6> TrajectoryFollower::ArmK_at_state(
    const Eigen::Ref<const ::Eigen::Matrix<double, 6, 1>> arm_X,
    const Eigen::Ref<const ::Eigen::Matrix<double, 2, 1>> arm_U) {
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

  const auto x_blocked = arm_X.block<4, 1>(0, 0);

  const ::Eigen::Matrix<double, 4, 4> final_A =
      ::frc971::control_loops::NumericalJacobianX<4, 2>(
          [this](const auto &x_blocked, const auto &U, double dt) {
            return this->dynamics_->UnboundedDiscreteDynamics(x_blocked, U, dt);
          },
          x_blocked, arm_U, 0.00505);

  const ::Eigen::Matrix<double, 4, 2> final_B =
      ::frc971::control_loops::NumericalJacobianU<4, 2>(
          [this](const auto &x_blocked, const auto &U, double dt) {
            return this->dynamics_->UnboundedDiscreteDynamics(x_blocked, U, dt);
          },
          x_blocked, arm_U, 0.00505);

  ::Eigen::Matrix<double, 4, 4> S;
  ::Eigen::Matrix<double, 2, 4> sub_K;
  if (::frc971::controls::dlqr<4, 2>(final_A, final_B, Q, R, &sub_K, &S) == 0) {
    if (VLOG_IS_ON(1)) {
      ::Eigen::EigenSolver<::Eigen::Matrix<double, 4, 4>> eigensolver(
          final_A - final_B * sub_K);
      LOG(INFO) << eigensolver.eigenvalues().transpose();
      LOG(INFO) << sub_K;
    }

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
    const ::Eigen::Matrix<double, 2, 6> &arm_K,
    const ::Eigen::Matrix<double, 9, 1> &X, const Trajectory &trajectory,
    ::Eigen::Matrix<double, 3, 1> *U, double *saturation_goal_velocity,
    double *saturation_goal_acceleration) {
  double saturation_goal_distance =
      ((goal_distance - last_goal_distance) * saturation_fraction_along_path +
       last_goal_distance);

  const ::Eigen::Matrix<double, 3, 1> theta_t =
      trajectory.ThetaT(saturation_goal_distance);
  *saturation_goal_velocity = trajectory.InterpolateVelocity(
      saturation_goal_distance, last_goal_distance, goal_distance,
      last_goal_velocity, goal_velocity);
  *saturation_goal_acceleration = trajectory.InterpolateAcceleration(
      last_goal_distance, goal_distance, last_goal_velocity, goal_velocity);
  const ::Eigen::Matrix<double, 3, 1> omega_t =
      trajectory.OmegaT(saturation_goal_distance, *saturation_goal_velocity);
  const ::Eigen::Matrix<double, 3, 1> alpha_t =
      trajectory.AlphaT(saturation_goal_distance, *saturation_goal_velocity,
                        *saturation_goal_acceleration);
  const ::Eigen::Matrix<double, 9, 1> R = trajectory.R(theta_t, omega_t);

  const ::Eigen::Matrix<double, 3, 1> U_ff = ComputeFF_U(R, omega_t, alpha_t);

  *U = U_ff;
  U->block<2, 1>(0, 0) += arm_K * (R.block<6, 1>(0, 0) - X.block<6, 1>(0, 0));
  U->block<1, 1>(2, 0) +=
      roll_->controller().K() * (R.block<3, 1>(6, 0) - X.block<3, 1>(6, 0));
}

::Eigen::Matrix<double, 2, 1> TrajectoryFollower::PlanNextGoal(
    const ::Eigen::Matrix<double, 2, 1> &goal, double plan_vmax, double dt) {
  // Figure out where we would be if we were to accelerate as fast as
  // our constraints allow.
  ::Eigen::Matrix<double, 2, 1> next_goal = ::frc971::control_loops::RungeKutta(
      [this, &plan_vmax](const ::Eigen::Matrix<double, 2, 1> &X) {
        return (::Eigen::Matrix<double, 2, 1>() << X(1),
                trajectory_->FeasableForwardsVoltage(
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

Eigen::Matrix<double, 3, 1> TrajectoryFollower::ComputeFF_U(
    const Eigen::Matrix<double, 9, 1> &X,
    const Eigen::Matrix<double, 3, 1> &omega_t,
    const Eigen::Matrix<double, 3, 1> &alpha_t) const {
  Eigen::Matrix<double, 3, 1> result;
  result.block<2, 1>(0, 0) =
      dynamics_->FF_U(X.block<4, 1>(0, 0), omega_t.block<2, 1>(0, 0),
                      alpha_t.block<2, 1>(0, 0));
  result(2, 0) =
      (alpha_t(2, 0) -
       roll_->plant().coefficients().A_continuous(1, 1) * omega_t(2, 0)) /
      roll_->plant().coefficients().B_continuous(1, 0);

  return result;
}

void TrajectoryFollower::Update(const ::Eigen::Matrix<double, 9, 1> &X,
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
      const ::Eigen::Matrix<double, 9, 1> R =
          Trajectory::R(theta_, ::Eigen::Matrix<double, 3, 1>::Zero());

      U_ff_ = ComputeFF_U(X, ::Eigen::Matrix<double, 3, 1>::Zero(),
                          ::Eigen::Matrix<double, 3, 1>::Zero());

      const ::Eigen::Matrix<double, 2, 6> arm_K =
          ArmK_at_state(X.block<6, 1>(0, 0), U_ff_.block<2, 1>(0, 0));

      U_unsaturated_.block<2, 1>(0, 0) =
          U_ff_.block<2, 1>(0, 0) +
          arm_K * (R.block<6, 1>(0, 0) - X.block<6, 1>(0, 0));
      U_unsaturated_(2, 0) =
          U_ff_(2, 0) +
          roll_->controller().K() * (R.block<3, 1>(6, 0) - X.block<3, 1>(6, 0));

      U_ = U_unsaturated_;

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

  const ::Eigen::Matrix<double, 3, 1> theta_t = trajectory_->ThetaT(goal_(0));
  const ::Eigen::Matrix<double, 3, 1> omega_t =
      trajectory_->OmegaT(goal_(0), goal_(1));
  const ::Eigen::Matrix<double, 3, 1> alpha_t =
      trajectory_->AlphaT(goal_(0), goal_(1), goal_acceleration_);

  const ::Eigen::Matrix<double, 9, 1> R = Trajectory::R(theta_t, omega_t);

  U_ff_ = ComputeFF_U(R, omega_t, alpha_t);

  const ::Eigen::Matrix<double, 2, 6> arm_K =
      ArmK_at_state(X.block<6, 1>(0, 0), U_ff_.block<2, 1>(0, 0));
  U_unsaturated_ = U_ff_;
  U_unsaturated_.block<2, 1>(0, 0) +=
      arm_K * (R.block<6, 1>(0, 0) - X.block<6, 1>(0, 0));
  U_unsaturated_.block<1, 1>(2, 0) +=
      roll_->controller().K() * (R.block<3, 1>(6, 0) - X.block<3, 1>(6, 0));
  U_ = U_unsaturated_;

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
    double saturation_goal_velocity = 0.0;
    double saturation_goal_acceleration = 0.0;
    while (step_size > 0.01) {
      USaturationSearch(goal_(0), last_goal_(0), goal_(1), last_goal_(1),
                        saturation_fraction_along_path_, arm_K, X, *trajectory_,
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
}  // namespace y2023
