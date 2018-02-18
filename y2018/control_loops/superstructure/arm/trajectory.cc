#include "y2018/control_loops/superstructure/arm/trajectory.h"

namespace y2018 {
namespace control_loops {
namespace superstructure {
namespace arm {

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
    const ::Eigen::Matrix<double, 2, 2> &alpha_unitizer, double vmax) {
  ::std::vector<double> max_dvelocity_unfiltered;
  max_dvelocity_unfiltered.reserve(num_plan_points_);
  for (size_t i = 0; i < num_plan_points_; ++i) {
    const double distance = DistanceForIndex(i);

    // TODO(austin): We are looking up the index in the path 3 times here.
    const ::Eigen::Matrix<double, 2, 1> theta = path_->Theta(distance);
    const ::Eigen::Matrix<double, 2, 1> omega = path_->Omega(distance);
    const ::Eigen::Matrix<double, 2, 1> alpha = path_->Alpha(distance);
    const ::Eigen::Matrix<double, 4, 1> X =
        (::Eigen::Matrix<double, 4, 1>() << theta(0, 0), 0.0, theta(1, 0), 0.0)
            .finished();
    ::Eigen::Matrix<double, 2, 2> K1;
    ::Eigen::Matrix<double, 2, 2> K2;

    Dynamics::NormilizedMatriciesForState(X, &K1, &K2);

    const ::Eigen::Matrix<double, 2, 2> omega_square =
        (::Eigen::Matrix<double, 2, 2>() << omega(0, 0), 0.0, 0.0, omega(1, 0))
            .finished();

    // Here, we can say that
    //   d^2/dt^2 theta = d^2/dd^2 theta(d) * (d d/dt)^2
    // Normalize so that the max accel is 1, and take magnitudes. This
    // gives us the max velocity we can be at each point due to
    // curvature.
    const ::Eigen::Matrix<double, 2, 1> vk1 =
        Dynamics::K3_inverse * (K1 * alpha + K2 * omega_square * omega);
    const ::Eigen::Matrix<double, 2, 1> vk2 =
        Dynamics::K3_inverse * Dynamics::K4 * omega;

    double min_good_ddot =
        ::std::sqrt(1.0 / ::std::max(0.001, (alpha_unitizer * alpha).norm()));

    // Loop through all the various vmin, vmax combinations.
    for (const double c : {-vmax, vmax}) {
      // Also loop through saturating theta0 and theta1
      for (const ::std::pair<double, double> ab :
           {::std::pair<double, double>{vk1(0, 0), vk2(0, 0)},
            ::std::pair<double, double>{vk1(1, 0), vk2(1, 0)}}) {
        const double a = ab.first;
        const double b = ab.second;
        const double sqrt_number = b * b - 4.0 * a * c;
        // Throw out imaginary solutions to the quadratic.
        if (sqrt_number > 0) {
          const double sqrt_result = ::std::sqrt(sqrt_number);
          const double ddot1 = (-b + sqrt_result) / (2.0 * a);
          const double ddot2 = (-b - sqrt_result) / (2.0 * a);
          // Loop through both solutions.
          for (const double ddot : {ddot1, ddot2}) {
            const ::Eigen::Matrix<double, 2, 1> U =
                vk1 * ddot * ddot + vk2 * ddot;
            // Finally, make sure the velocity is positive and valid.
            if ((U.array().abs() <= vmax + 1e-6).all() && ddot > 0.0) {
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
    double goal_distance, double goal_velocity, double vmax,
    const ::Eigen::Matrix<double, 2, 2> &alpha_unitizer) {
  // TODO(austin): We are looking up the index in the path 3 times here.
  const ::Eigen::Matrix<double, 2, 1> theta = path_->Theta(goal_distance);
  const ::Eigen::Matrix<double, 2, 1> omega = path_->Omega(goal_distance);
  const ::Eigen::Matrix<double, 2, 1> alpha = path_->Alpha(goal_distance);

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
      Dynamics::K3_inverse * ((K1 * alpha + K2 * omega_square * omega) *
                                  goal_velocity * goal_velocity +
                              Dynamics::K4 * omega * goal_velocity);
  const ::Eigen::Matrix<double, 2, 1> k_scalar =
      Dynamics::K3_inverse * K1 * omega;

  double goal_acceleration =
      ::std::sqrt(
          ::std::max(0.0, 1.0 - ::std::pow((alpha_unitizer * alpha).norm() *
                                               goal_velocity * goal_velocity,
                                           2))) /
      (alpha_unitizer * omega).norm();

  for (double c : {-vmax, vmax}) {
    for (const ::std::pair<double, double> ab :
         {::std::pair<double, double>{k_constant(0, 0), k_scalar(0, 0)},
          ::std::pair<double, double>{k_constant(1, 0), k_scalar(1, 0)}}) {
      const double a = ab.first;
      const double b = ab.second;
      const double voltage_accel = (c - a) / b;
      const ::Eigen::Matrix<double, 2, 1> U =
          k_constant + k_scalar * voltage_accel;

      if (voltage_accel > 0.0 && (U.array().abs() <= vmax + 1e-6).all())
        goal_acceleration = ::std::min(voltage_accel, goal_acceleration);
    }
  }

  return goal_acceleration;
}

double Trajectory::FeasableBackwardsAcceleration(
    double goal_distance, double goal_velocity, double vmax,
    const ::Eigen::Matrix<double, 2, 2> &alpha_unitizer) {
    const ::Eigen::Matrix<double, 2, 1> theta = path_->Theta(goal_distance);
    const ::Eigen::Matrix<double, 2, 1> omega = path_->Omega(goal_distance);
    const ::Eigen::Matrix<double, 2, 1> alpha = path_->Alpha(goal_distance);
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
        ((K1 * alpha + K2 * omega_square * omega) * goal_velocity * goal_velocity +
         Dynamics::K4 * omega * goal_velocity);
    const ::Eigen::Matrix<double, 2, 1> k_scalar =
        Dynamics::K3_inverse * K1 * omega;

    double goal_acceleration =
        ::std::sqrt(
            ::std::max(0.0, 1.0 - ::std::pow(((alpha_unitizer * alpha).norm() *
                                              goal_velocity * goal_velocity),
                                             2))) /
        (alpha_unitizer * omega).norm();
    for (double c : {-vmax, vmax}) {
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

        if (voltage_accel < 0.0 && (U.array().abs() <= vmax + 1e-6).all()) {
          goal_acceleration = ::std::min(-voltage_accel, goal_acceleration);
        }
      }
    }

    return goal_acceleration;
}

::std::vector<double> Trajectory::BackwardsOptimizationPass(
    const ::std::vector<double> &max_dvelocity,
    const ::Eigen::Matrix<double, 2, 2> &alpha_unitizer, double vmax) {
  ::std::vector<double> max_dvelocity_back_pass = max_dvelocity;

  // Now, iterate over the list of velocities and constrain the acceleration.
  for (int i = num_plan_points_ - 2; i >= 0; --i) {
    const double previous_velocity = max_dvelocity_back_pass[i + 1];
    const double prev_distance = DistanceForIndex(i + 1);

    // Now, integrate with respect to distance (not time like normal).
    // Subdivide the plan step size into kNumSteps steps.
    double integrated_distance = 0.0;
    double integrated_velocity = previous_velocity;
    constexpr int kNumSteps = 10;
    for (int j = 0; j < kNumSteps; ++j) {
      // Compute the acceleration with respect to time.
      const double acceleration = FeasableBackwardsAcceleration(
          prev_distance - integrated_distance, integrated_velocity, vmax,
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
    const ::Eigen::Matrix<double, 2, 2> &alpha_unitizer, double vmax) {
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
          previous_distance + integrated_distance, integrated_velocity, vmax,
          alpha_unitizer);

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

}  // namespace arm
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2018
