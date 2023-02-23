#include "aos/init.h"
#include "frc971/analysis/in_process_plotter.h"
#include "frc971/control_loops/binomial.h"
#include "frc971/control_loops/double_jointed_arm/dynamics.h"
#include "frc971/control_loops/double_jointed_arm/ekf.h"
#include "frc971/control_loops/fixed_quadrature.h"
#include "gflags/gflags.h"
#include "y2023/control_loops/superstructure/arm/arm_constants.h"
#include "y2023/control_loops/superstructure/arm/trajectory.h"
#include "y2023/control_loops/superstructure/roll/integral_hybrid_roll_plant.h"
#include "y2023/control_loops/superstructure/roll/integral_roll_plant.h"

DEFINE_bool(forwards, true, "If true, run the forwards simulation.");
DEFINE_bool(plot, true, "If true, plot");
DEFINE_bool(plot_thetas, true, "If true, plot the angles");

namespace y2023 {
namespace control_loops {
namespace superstructure {
namespace arm {
using frc971::control_loops::MatrixGaussianQuadrature5;

void Main() {
  frc971::control_loops::arm::Dynamics dynamics(kArmConstants);
  StateFeedbackLoop<3, 1, 1, double, StateFeedbackHybridPlant<3, 1, 1>,
                    HybridKalman<3, 1, 1>>
      hybrid_roll = superstructure::roll::MakeIntegralHybridRollLoop();

  Eigen::Matrix<double, 3, 4> spline_params;

  spline_params << 0.30426338, 0.42813912, 0.64902386, 0.55127045, -1.73611082,
      -1.64478944, -1.44763868, -1.22624244, 0.1, 0.2, 0.3, 0.5;
  LOG(INFO) << "Spline " << spline_params;
  NSpline<4, 2> spline(spline_params.block<2, 4>(0, 0));
  CosSpline cos_spline(spline,
                       {{0.0, 0.1}, {0.3, 0.1}, {0.7, 0.2}, {1.0, 0.2}});
  Path distance_spline(cos_spline, 100);

  Trajectory trajectory(&dynamics, &hybrid_roll.plant(),
                        std::make_unique<Path>(cos_spline), 0.001);

  constexpr double kAlpha0Max = 20.0;
  constexpr double kAlpha1Max = 30.0;
  constexpr double kAlpha2Max = 40.0;
  constexpr double vmax = 9.0;
  constexpr double sim_dt = 0.00505;

  const ::Eigen::DiagonalMatrix<double, 3> alpha_unitizer(
      (::Eigen::DiagonalMatrix<double, 3>().diagonal() << (1.0 / kAlpha0Max),
       (1.0 / kAlpha1Max), (1.0 / kAlpha2Max))
          .finished());
  trajectory.OptimizeTrajectory(alpha_unitizer, vmax);

  const ::std::vector<double> distance_array = trajectory.DistanceArray();

  ::std::vector<double> theta0_array;
  ::std::vector<double> theta1_array;
  ::std::vector<double> theta2_array;
  ::std::vector<double> omega0_array;
  ::std::vector<double> omega1_array;
  ::std::vector<double> omega2_array;
  ::std::vector<double> alpha0_array;
  ::std::vector<double> alpha1_array;
  ::std::vector<double> alpha2_array;

  ::std::vector<double> integrated_distance;
  ::std::vector<double> integrated_theta0_array;
  ::std::vector<double> integrated_theta1_array;
  ::std::vector<double> integrated_theta2_array;
  ::std::vector<double> integrated_omega0_array;
  ::std::vector<double> integrated_omega1_array;
  ::std::vector<double> integrated_omega2_array;

  ::Eigen::Matrix<double, 3, 1> integrated_theta = distance_spline.Theta(0.0);
  ::Eigen::Matrix<double, 3, 1> integrated_omega = distance_spline.Omega(0.0);

  // Plot the splines and their integrals to check consistency.
  for (const double d : distance_array) {
    const ::Eigen::Matrix<double, 3, 1> theta = distance_spline.Theta(d);
    const ::Eigen::Matrix<double, 3, 1> omega = distance_spline.Omega(d);
    const ::Eigen::Matrix<double, 3, 1> alpha = distance_spline.Alpha(d);
    theta0_array.push_back(theta(0, 0));
    theta1_array.push_back(theta(1, 0));
    theta2_array.push_back(theta(2, 0));
    omega0_array.push_back(omega(0, 0));
    omega1_array.push_back(omega(1, 0));
    omega2_array.push_back(omega(2, 0));
    alpha0_array.push_back(alpha(0, 0));
    alpha1_array.push_back(alpha(1, 0));
    alpha2_array.push_back(alpha(2, 0));
  }

  const double dd = distance_spline.length() / 1000.0;
  for (double d = 0; d <= distance_spline.length(); d += dd) {
    integrated_distance.push_back(d);
    integrated_omega0_array.push_back(integrated_omega(0));
    integrated_omega1_array.push_back(integrated_omega(1));
    integrated_omega2_array.push_back(integrated_omega(2));
    integrated_theta0_array.push_back(integrated_theta(0));
    integrated_theta1_array.push_back(integrated_theta(1));
    integrated_theta2_array.push_back(integrated_theta(2));

    integrated_theta += MatrixGaussianQuadrature5<3>(
        [&](double distance) { return distance_spline.Omega(distance); }, d,
        d + dd);
    integrated_omega += MatrixGaussianQuadrature5<3>(
        [&](double distance) { return distance_spline.Alpha(distance); }, d,
        d + dd);
  }

  // Next step: see what U is as a function of distance for all the passes.
  ::std::vector<double> Uff0_distance_array_curvature;
  ::std::vector<double> Uff1_distance_array_curvature;
  ::std::vector<double> Uff2_distance_array_curvature;
  ::std::vector<double> Uff0_distance_array_backwards_accel_only;
  ::std::vector<double> Uff1_distance_array_backwards_accel_only;
  ::std::vector<double> Uff2_distance_array_backwards_accel_only;
  ::std::vector<double> Uff0_distance_array_forwards_accel_only;
  ::std::vector<double> Uff1_distance_array_forwards_accel_only;
  ::std::vector<double> Uff2_distance_array_forwards_accel_only;
  ::std::vector<double> Uff0_distance_array_backwards_voltage_only;
  ::std::vector<double> Uff1_distance_array_backwards_voltage_only;
  ::std::vector<double> Uff2_distance_array_backwards_voltage_only;
  ::std::vector<double> Uff0_distance_array_forwards_voltage_only;
  ::std::vector<double> Uff1_distance_array_forwards_voltage_only;
  ::std::vector<double> Uff2_distance_array_forwards_voltage_only;

  TrajectoryFollower follower(&dynamics, &hybrid_roll, &trajectory);

  for (const double distance : distance_array) {
    const ::Eigen::Matrix<double, 3, 1> theta_t = trajectory.ThetaT(distance);

    {
      const double goal_velocity = trajectory.GetDVelocity(
          distance, trajectory.max_dvelocity_unfiltered());
      const double goal_acceleration = trajectory.GetDAcceleration(
          distance, trajectory.max_dvelocity_unfiltered());
      const ::Eigen::Matrix<double, 3, 1> omega_t =
          trajectory.OmegaT(distance, goal_velocity);
      const ::Eigen::Matrix<double, 3, 1> alpha_t =
          trajectory.AlphaT(distance, goal_velocity, goal_acceleration);

      const ::Eigen::Matrix<double, 9, 1> R = trajectory.R(theta_t, omega_t);
      const ::Eigen::Matrix<double, 3, 1> U =
          follower.ComputeFF_U(R, omega_t, alpha_t).array().max(-20).min(20);

      Uff0_distance_array_curvature.push_back(U(0));
      Uff1_distance_array_curvature.push_back(U(1));
      Uff2_distance_array_curvature.push_back(U(2));
    }
    {
      const double goal_velocity = trajectory.GetDVelocity(
          distance, trajectory.max_dvelocity_backward_accel());
      const double goal_acceleration = trajectory.GetDAcceleration(
          distance, trajectory.max_dvelocity_backward_accel());
      const ::Eigen::Matrix<double, 3, 1> omega_t =
          trajectory.OmegaT(distance, goal_velocity);
      const ::Eigen::Matrix<double, 3, 1> alpha_t =
          trajectory.AlphaT(distance, goal_velocity, goal_acceleration);

      const ::Eigen::Matrix<double, 9, 1> R = trajectory.R(theta_t, omega_t);
      const ::Eigen::Matrix<double, 3, 1> U =
          follower.ComputeFF_U(R, omega_t, alpha_t).array().max(-20).min(20);

      Uff0_distance_array_backwards_accel_only.push_back(U(0));
      Uff1_distance_array_backwards_accel_only.push_back(U(1));
      Uff2_distance_array_backwards_accel_only.push_back(U(2));
    }
    {
      const double goal_velocity = trajectory.GetDVelocity(
          distance, trajectory.max_dvelocity_forwards_accel());
      const double goal_acceleration = trajectory.GetDAcceleration(
          distance, trajectory.max_dvelocity_forwards_accel());
      const ::Eigen::Matrix<double, 3, 1> omega_t =
          trajectory.OmegaT(distance, goal_velocity);
      const ::Eigen::Matrix<double, 3, 1> alpha_t =
          trajectory.AlphaT(distance, goal_velocity, goal_acceleration);

      const ::Eigen::Matrix<double, 9, 1> R = trajectory.R(theta_t, omega_t);
      const ::Eigen::Matrix<double, 3, 1> U =
          follower.ComputeFF_U(R, omega_t, alpha_t).array().max(-20).min(20);

      Uff0_distance_array_forwards_accel_only.push_back(U(0));
      Uff1_distance_array_forwards_accel_only.push_back(U(1));
      Uff2_distance_array_forwards_accel_only.push_back(U(2));
    }
    {
      const double goal_velocity = trajectory.GetDVelocity(
          distance, trajectory.max_dvelocity_backward_voltage());
      const double goal_acceleration = trajectory.GetDAcceleration(
          distance, trajectory.max_dvelocity_backward_voltage());
      const ::Eigen::Matrix<double, 3, 1> omega_t =
          trajectory.OmegaT(distance, goal_velocity);
      const ::Eigen::Matrix<double, 3, 1> alpha_t =
          trajectory.AlphaT(distance, goal_velocity, goal_acceleration);

      const ::Eigen::Matrix<double, 9, 1> R = trajectory.R(theta_t, omega_t);
      const ::Eigen::Matrix<double, 3, 1> U =
          follower.ComputeFF_U(R, omega_t, alpha_t).array().max(-20).min(20);

      Uff0_distance_array_backwards_voltage_only.push_back(U(0));
      Uff1_distance_array_backwards_voltage_only.push_back(U(1));
      Uff2_distance_array_backwards_voltage_only.push_back(U(2));
    }
    {
      const double goal_velocity = trajectory.GetDVelocity(
          distance, trajectory.max_dvelocity_forwards_voltage());
      const double goal_acceleration = trajectory.GetDAcceleration(
          distance, trajectory.max_dvelocity_forwards_voltage());
      const ::Eigen::Matrix<double, 3, 1> omega_t =
          trajectory.OmegaT(distance, goal_velocity);
      const ::Eigen::Matrix<double, 3, 1> alpha_t =
          trajectory.AlphaT(distance, goal_velocity, goal_acceleration);

      const ::Eigen::Matrix<double, 9, 1> R = trajectory.R(theta_t, omega_t);
      const ::Eigen::Matrix<double, 3, 1> U =
          follower.ComputeFF_U(R, omega_t, alpha_t).array().max(-20).min(20);

      Uff0_distance_array_forwards_voltage_only.push_back(U(0));
      Uff1_distance_array_forwards_voltage_only.push_back(U(1));
      Uff2_distance_array_forwards_voltage_only.push_back(U(2));
    }
  }

  double t = 0;
  ::Eigen::Matrix<double, 4, 1> arm_X;
  ::Eigen::Matrix<double, 2, 1> roll_X;
  {
    ::Eigen::Matrix<double, 3, 1> theta_t = trajectory.ThetaT(0.0);
    arm_X << theta_t(0), 0.0, theta_t(1), 0.0;
    roll_X << theta_t(2), 0.0;
  }

  ::std::vector<double> t_array;
  ::std::vector<double> theta0_goal_t_array;
  ::std::vector<double> theta1_goal_t_array;
  ::std::vector<double> theta2_goal_t_array;
  ::std::vector<double> omega0_goal_t_array;
  ::std::vector<double> omega1_goal_t_array;
  ::std::vector<double> omega2_goal_t_array;
  ::std::vector<double> alpha0_goal_t_array;
  ::std::vector<double> alpha1_goal_t_array;
  ::std::vector<double> alpha2_goal_t_array;
  ::std::vector<double> theta0_t_array;
  ::std::vector<double> omega0_t_array;
  ::std::vector<double> theta1_t_array;
  ::std::vector<double> omega1_t_array;
  ::std::vector<double> theta2_t_array;
  ::std::vector<double> omega2_t_array;
  ::std::vector<double> distance_t_array;
  ::std::vector<double> velocity_t_array;
  ::std::vector<double> acceleration_t_array;
  ::std::vector<double> u0_unsaturated_array;
  ::std::vector<double> u1_unsaturated_array;
  ::std::vector<double> u2_unsaturated_array;
  ::std::vector<double> alpha0_t_array;
  ::std::vector<double> alpha1_t_array;
  ::std::vector<double> alpha2_t_array;
  ::std::vector<double> uff0_array;
  ::std::vector<double> uff1_array;
  ::std::vector<double> uff2_array;
  ::std::vector<double> u0_array;
  ::std::vector<double> u1_array;
  ::std::vector<double> u2_array;
  ::std::vector<double> theta0_hat_t_array;
  ::std::vector<double> omega0_hat_t_array;
  ::std::vector<double> theta1_hat_t_array;
  ::std::vector<double> omega1_hat_t_array;
  ::std::vector<double> theta2_hat_t_array;
  ::std::vector<double> omega2_hat_t_array;
  ::std::vector<double> torque0_hat_t_array;
  ::std::vector<double> torque1_hat_t_array;
  ::std::vector<double> torque2_hat_t_array;

  // Now follow the trajectory.
  frc971::control_loops::arm::EKF arm_ekf(&dynamics);
  arm_ekf.Reset(arm_X);
  StateFeedbackLoop<3, 1, 1, double, StateFeedbackPlant<3, 1, 1>,
                    StateFeedbackObserver<3, 1, 1>>
      roll = superstructure::roll::MakeIntegralRollLoop();
  roll.mutable_X_hat().setZero();
  roll.mutable_X_hat().block<2, 1>(0, 0) = roll_X;

  ::std::cout << "Reset P: " << arm_ekf.P_reset() << ::std::endl;
  ::std::cout << "Stabilized P: " << arm_ekf.P_half_converged() << ::std::endl;
  ::std::cout << "Really stabilized P: " << arm_ekf.P_converged()
              << ::std::endl;

  while (t < 1.2) {
    t_array.push_back(t);
    arm_ekf.Correct(
        (::Eigen::Matrix<double, 2, 1>() << arm_X(0), arm_X(2)).finished(),
        sim_dt);
    roll.Correct((::Eigen::Matrix<double, 1, 1>() << roll_X(0)).finished());
    follower.Update(
        (Eigen::Matrix<double, 9, 1>() << arm_ekf.X_hat(), roll.X_hat())
            .finished(),
        false, sim_dt, vmax, 12.0);

    const ::Eigen::Matrix<double, 3, 1> theta_t =
        trajectory.ThetaT(follower.goal()(0));
    const ::Eigen::Matrix<double, 3, 1> omega_t =
        trajectory.OmegaT(follower.goal()(0), follower.goal()(1));
    const ::Eigen::Matrix<double, 3, 1> alpha_t = trajectory.AlphaT(
        follower.goal()(0), follower.goal()(1), follower.goal_acceleration());

    theta0_goal_t_array.push_back(theta_t(0));
    theta1_goal_t_array.push_back(theta_t(1));
    theta2_goal_t_array.push_back(theta_t(2));
    omega0_goal_t_array.push_back(omega_t(0));
    omega1_goal_t_array.push_back(omega_t(1));
    omega2_goal_t_array.push_back(omega_t(2));
    alpha0_goal_t_array.push_back(alpha_t(0));
    alpha1_goal_t_array.push_back(alpha_t(1));
    alpha2_goal_t_array.push_back(alpha_t(2));
    theta0_t_array.push_back(arm_X(0));
    omega0_t_array.push_back(arm_X(1));
    theta1_t_array.push_back(arm_X(2));
    omega1_t_array.push_back(arm_X(3));
    theta2_t_array.push_back(roll_X(0));
    omega2_t_array.push_back(roll_X(1));
    theta0_hat_t_array.push_back(arm_ekf.X_hat(0));
    omega0_hat_t_array.push_back(arm_ekf.X_hat(1));
    torque0_hat_t_array.push_back(arm_ekf.X_hat(4));
    theta1_hat_t_array.push_back(arm_ekf.X_hat(2));
    omega1_hat_t_array.push_back(arm_ekf.X_hat(3));
    torque1_hat_t_array.push_back(arm_ekf.X_hat(5));

    theta2_hat_t_array.push_back(roll.X_hat(0));
    omega2_hat_t_array.push_back(roll.X_hat(1));
    torque2_hat_t_array.push_back(roll.X_hat(2));

    distance_t_array.push_back(follower.goal()(0));
    velocity_t_array.push_back(follower.goal()(1));
    acceleration_t_array.push_back(follower.goal_acceleration());

    u0_unsaturated_array.push_back(follower.U_unsaturated()(0));
    u1_unsaturated_array.push_back(follower.U_unsaturated()(1));
    u2_unsaturated_array.push_back(follower.U_unsaturated()(2));

    ::Eigen::Matrix<double, 3, 1> actual_U = follower.U();
    // Add in a disturbance force to see how well the arm learns it.
    // actual_U(0) += 1.0;

    const ::Eigen::Matrix<double, 4, 1> arm_xdot =
        dynamics.Acceleration(arm_X, actual_U.block<2, 1>(0, 0));
    const ::Eigen::Matrix<double, 2, 1> roll_xdot =
        hybrid_roll.plant().coefficients().A_continuous.block<2, 2>(0, 0) *
            roll_X +
        hybrid_roll.plant().coefficients().B_continuous.block<2, 1>(0, 0) *
            actual_U.block<1, 1>(2, 0);

    arm_X = dynamics.UnboundedDiscreteDynamics(
        arm_X, actual_U.block<2, 1>(0, 0), sim_dt);
    arm_ekf.Predict(follower.U().block<2, 1>(0, 0), sim_dt);
    roll_X =
        roll.plant()
            .Update((Eigen::Matrix<double, 3, 1>() << roll_X, 0.0).finished(),
                    follower.U().block<1, 1>(2, 0))
            .block<2, 1>(0, 0);
    roll.UpdateObserver(follower.U().block<1, 1>(2, 0),
                        std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::duration<double>(sim_dt)));

    alpha0_t_array.push_back(arm_xdot(1));
    alpha1_t_array.push_back(arm_xdot(3));
    alpha2_t_array.push_back(roll_xdot(1));

    uff0_array.push_back(follower.U_ff()(0));
    uff1_array.push_back(follower.U_ff()(1));
    uff2_array.push_back(follower.U_ff()(2));
    u0_array.push_back(follower.U()(0));
    u1_array.push_back(follower.U()(1));
    u2_array.push_back(follower.U()(2));

    t += sim_dt;
  }

  if (FLAGS_plot) {
    frc971::analysis::Plotter plotter;

    plotter.AddFigure();
    plotter.Title("Input spline");
    plotter.AddLine(distance_array, theta0_array, "theta0");
    plotter.AddLine(distance_array, theta1_array, "theta1");
    plotter.AddLine(distance_array, theta2_array, "theta2");
    plotter.AddLine(distance_array, omega0_array, "omega0");
    plotter.AddLine(distance_array, omega1_array, "omega1");
    plotter.AddLine(distance_array, omega2_array, "omega2");
    plotter.AddLine(distance_array, alpha0_array, "alpha0");
    plotter.AddLine(distance_array, alpha1_array, "alpha1");
    plotter.AddLine(distance_array, alpha2_array, "alpha2");

    plotter.AddLine(integrated_distance, integrated_theta0_array,
                    "integrated theta0");
    plotter.AddLine(integrated_distance, integrated_theta1_array,
                    "integrated theta1");
    plotter.AddLine(integrated_distance, integrated_theta2_array,
                    "integrated theta2");
    plotter.AddLine(integrated_distance, integrated_omega0_array,
                    "integrated omega0");
    plotter.AddLine(integrated_distance, integrated_omega1_array,
                    "integrated omega1");
    plotter.AddLine(integrated_distance, integrated_omega2_array,
                    "integrated omega2");
    plotter.Publish();

    plotter.AddFigure();
    plotter.Title("Trajectory");
    plotter.AddLine(theta0_array, theta1_array, "desired path");
    plotter.AddLine(theta0_t_array, theta1_t_array, "actual path");
    plotter.Publish();

    plotter.AddFigure();
    plotter.Title("Solver passes");
    plotter.AddLine(distance_array, trajectory.max_dvelocity_unfiltered(),
                    "pass0");
    plotter.AddLine(distance_array, trajectory.max_dvelocity_backward_accel(),
                    "passb accel");
    plotter.AddLine(distance_array, trajectory.max_dvelocity_forwards_accel(),
                    "passf accel");
    plotter.AddLine(distance_array, trajectory.max_dvelocity_backward_voltage(),
                    "passb voltage");
    plotter.AddLine(distance_array, trajectory.max_dvelocity_forwards_voltage(),
                    "passf voltage");
    plotter.Publish();

    plotter.AddFigure();
    plotter.Title("Time Goals");
    plotter.AddLine(t_array, alpha0_goal_t_array, "alpha0_t_goal");
    plotter.AddLine(t_array, alpha0_t_array, "alpha0_t");
    plotter.AddLine(t_array, alpha1_goal_t_array, "alpha1_t_goal");
    plotter.AddLine(t_array, alpha1_t_array, "alpha1_t");
    plotter.AddLine(t_array, alpha2_goal_t_array, "alpha2_t_goal");
    plotter.AddLine(t_array, alpha2_t_array, "alpha2_t");
    plotter.AddLine(t_array, distance_t_array, "distance_t");
    plotter.AddLine(t_array, velocity_t_array, "velocity_t");
    plotter.AddLine(t_array, acceleration_t_array, "acceleration_t");
    plotter.Publish();

    plotter.AddFigure();
    plotter.Title("Angular Velocities");
    plotter.AddLine(t_array, omega0_goal_t_array, "omega0_t_goal");
    plotter.AddLine(t_array, omega0_t_array, "omega0_t");
    plotter.AddLine(t_array, omega0_hat_t_array, "omega0_hat_t");

    plotter.AddLine(t_array, omega1_goal_t_array, "omega1_t_goal");
    plotter.AddLine(t_array, omega1_t_array, "omega1_t");
    plotter.AddLine(t_array, omega1_hat_t_array, "omega1_hat_t");

    plotter.AddLine(t_array, omega2_goal_t_array, "omega2_t_goal");
    plotter.AddLine(t_array, omega2_t_array, "omega2_t");
    plotter.AddLine(t_array, omega2_hat_t_array, "omega2_hat_t");
    plotter.Publish();

    plotter.AddFigure();
    plotter.Title("Voltages");
    plotter.AddLine(t_array, u0_unsaturated_array, "u0_full");
    plotter.AddLine(t_array, u0_array, "u0");
    plotter.AddLine(t_array, uff0_array, "uff0");
    plotter.AddLine(t_array, u1_unsaturated_array, "u1_full");
    plotter.AddLine(t_array, u1_array, "u1");
    plotter.AddLine(t_array, uff1_array, "uff1");
    plotter.AddLine(t_array, u2_unsaturated_array, "u2_full");
    plotter.AddLine(t_array, u2_array, "u2");
    plotter.AddLine(t_array, uff2_array, "uff2");
    plotter.AddLine(t_array, torque0_hat_t_array, "torque0_hat");
    plotter.AddLine(t_array, torque1_hat_t_array, "torque1_hat");
    plotter.AddLine(t_array, torque2_hat_t_array, "torque2_hat");
    plotter.Publish();

    if (FLAGS_plot_thetas) {
      plotter.AddFigure();
      plotter.Title("Angles");
      plotter.AddLine(t_array, theta0_goal_t_array, "theta0_t_goal");
      plotter.AddLine(t_array, theta0_t_array, "theta0_t");
      plotter.AddLine(t_array, theta0_hat_t_array, "theta0_hat_t");
      plotter.AddLine(t_array, theta1_goal_t_array, "theta1_t_goal");
      plotter.AddLine(t_array, theta1_t_array, "theta1_t");
      plotter.AddLine(t_array, theta1_hat_t_array, "theta1_hat_t");
      plotter.AddLine(t_array, theta2_goal_t_array, "theta2_t_goal");
      plotter.AddLine(t_array, theta2_t_array, "theta2_t");
      plotter.AddLine(t_array, theta2_hat_t_array, "theta2_hat_t");
      plotter.Publish();
    }

    plotter.AddFigure();
    plotter.Title("ff for distance");
    plotter.AddLine(distance_array, Uff0_distance_array_forwards_voltage_only,
                    "ff0");
    plotter.AddLine(distance_array, Uff1_distance_array_forwards_voltage_only,
                    "ff1");
    plotter.AddLine(distance_array, Uff2_distance_array_forwards_voltage_only,
                    "ff2");

    plotter.AddLine(distance_array, Uff0_distance_array_backwards_voltage_only,
                    "ff0_back voltage");
    plotter.AddLine(distance_array, Uff1_distance_array_backwards_voltage_only,
                    "ff1_back voltage");
    plotter.AddLine(distance_array, Uff2_distance_array_backwards_voltage_only,
                    "ff2_back voltage");

    plotter.AddLine(distance_array, Uff0_distance_array_forwards_accel_only,
                    "ff0_forward accel");
    plotter.AddLine(distance_array, Uff1_distance_array_forwards_accel_only,
                    "ff1_forward accel");
    plotter.AddLine(distance_array, Uff2_distance_array_forwards_accel_only,
                    "ff2_forward accel");

    plotter.AddLine(distance_array, Uff0_distance_array_backwards_accel_only,
                    "ff0_back accel");
    plotter.AddLine(distance_array, Uff1_distance_array_backwards_accel_only,
                    "ff1_back accel");
    plotter.AddLine(distance_array, Uff2_distance_array_backwards_accel_only,
                    "ff2_back accel");

    plotter.AddLine(distance_array, Uff0_distance_array_curvature, "ff0_curve");
    plotter.AddLine(distance_array, Uff1_distance_array_curvature, "ff1_curve");
    plotter.AddLine(distance_array, Uff2_distance_array_curvature, "ff2_curve");

    plotter.Publish();
    plotter.Spin();
  }
}

}  // namespace arm
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2023

int main(int argc, char **argv) {
  ::aos::InitGoogle(&argc, &argv);
  ::y2023::control_loops::superstructure::arm::Main();
  return 0;
}
