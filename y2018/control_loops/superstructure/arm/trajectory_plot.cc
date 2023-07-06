#include "gflags/gflags.h"

#include "aos/init.h"
#include "frc971/analysis/in_process_plotter.h"
#include "frc971/control_loops/double_jointed_arm/dynamics.h"
#include "frc971/control_loops/double_jointed_arm/ekf.h"
#include "frc971/control_loops/double_jointed_arm/trajectory.h"
#include "y2018/control_loops/superstructure/arm/arm_constants.h"
#include "y2018/control_loops/superstructure/arm/generated_graph.h"

DEFINE_bool(forwards, true, "If true, run the forwards simulation.");
DEFINE_bool(plot, true, "If true, plot");
DEFINE_bool(plot_thetas, true, "If true, plot the angles");

namespace y2018 {
namespace control_loops {
namespace superstructure {
namespace arm {

void Main() {
  frc971::control_loops::arm::Dynamics dynamics(kArmConstants);
  frc971::control_loops::arm::Trajectory trajectory(
      &dynamics,
      FLAGS_forwards ? MakeNeutralToFrontHighPath()
                     : Path::Reversed(MakeNeutralToFrontHighPath()),
      0.001);

  constexpr double kAlpha0Max = 30.0;
  constexpr double kAlpha1Max = 50.0;
  constexpr double vmax = 10.5;
  constexpr double sim_dt = 0.00505;

  const ::Eigen::Matrix<double, 2, 2> alpha_unitizer =
      (::Eigen::Matrix<double, 2, 2>() << 1.0 / kAlpha0Max, 0.0, 0.0,
       1.0 / kAlpha1Max)
          .finished();
  trajectory.OptimizeTrajectory(alpha_unitizer, vmax);

  const ::std::vector<double> distance_array = trajectory.DistanceArray();

  ::std::vector<double> theta0_array;
  ::std::vector<double> theta1_array;
  ::std::vector<double> omega0_array;
  ::std::vector<double> omega1_array;
  ::std::vector<double> alpha0_array;
  ::std::vector<double> alpha1_array;

  ::std::vector<double> integrated_distance;
  ::std::vector<double> integrated_theta0_array;
  ::std::vector<double> integrated_theta1_array;
  ::std::vector<double> integrated_omega0_array;
  ::std::vector<double> integrated_omega1_array;

  ::Eigen::Matrix<double, 2, 1> integrated_theta = trajectory.path().Theta(0);
  ::Eigen::Matrix<double, 2, 1> integrated_omega = trajectory.path().Omega(0);

  for (const double d : distance_array) {
    const ::Eigen::Matrix<double, 2, 1> theta = trajectory.path().Theta(d);
    const ::Eigen::Matrix<double, 2, 1> omega = trajectory.path().Omega(d);
    const ::Eigen::Matrix<double, 2, 1> alpha = trajectory.path().Alpha(d);
    theta0_array.push_back(theta(0, 0));
    theta1_array.push_back(theta(1, 0));
    omega0_array.push_back(omega(0, 0));
    omega1_array.push_back(omega(1, 0));
    alpha0_array.push_back(alpha(0, 0));
    alpha1_array.push_back(alpha(1, 0));
  }

  const double dd = trajectory.path().length() / 1000.0;
  for (double d = 0; d <= trajectory.path().length(); d += dd) {
    integrated_distance.push_back(d);
    integrated_omega0_array.push_back(integrated_omega(0));
    integrated_omega1_array.push_back(integrated_omega(1));
    integrated_theta0_array.push_back(integrated_theta(0));
    integrated_theta1_array.push_back(integrated_theta(1));

    const ::Eigen::Matrix<double, 2, 1> alpha = trajectory.path().Alpha(d);
    integrated_theta += integrated_omega * dd;
    integrated_omega += alpha * dd;
  }

  // Next step: see what U is as a function of distance.
  ::std::vector<double> Uff0_distance_array_curvature;
  ::std::vector<double> Uff1_distance_array_curvature;
  ::std::vector<double> Uff0_distance_array_backwards_only;
  ::std::vector<double> Uff1_distance_array_backwards_only;
  ::std::vector<double> Uff0_distance_array;
  ::std::vector<double> Uff1_distance_array;

  for (const double distance : distance_array) {
    const double goal_velocity = trajectory.GetDVelocity(
        distance, trajectory.max_dvelocity_forward_pass());
    const double goal_acceleration = trajectory.GetDAcceleration(
        distance, trajectory.max_dvelocity_forward_pass());
    const ::Eigen::Matrix<double, 2, 1> theta_t = trajectory.ThetaT(distance);
    const ::Eigen::Matrix<double, 2, 1> omega_t =
        trajectory.OmegaT(distance, goal_velocity);
    const ::Eigen::Matrix<double, 2, 1> alpha_t =
        trajectory.AlphaT(distance, goal_velocity, goal_acceleration);

    const ::Eigen::Matrix<double, 6, 1> R = trajectory.R(theta_t, omega_t);
    const ::Eigen::Matrix<double, 2, 1> U =
        dynamics.FF_U(R.block<4, 1>(0, 0), omega_t, alpha_t)
            .array()
            .max(-20)
            .min(20);

    Uff0_distance_array.push_back(U(0));
    Uff1_distance_array.push_back(U(1));
  }

  for (const double distance : distance_array) {
    const double goal_velocity = trajectory.GetDVelocity(
        distance, trajectory.max_dvelocity_unfiltered());
    const double goal_acceleration = trajectory.GetDAcceleration(
        distance, trajectory.max_dvelocity_unfiltered());
    const ::Eigen::Matrix<double, 2, 1> theta_t = trajectory.ThetaT(distance);
    const ::Eigen::Matrix<double, 2, 1> omega_t =
        trajectory.OmegaT(distance, goal_velocity);
    const ::Eigen::Matrix<double, 2, 1> alpha_t =
        trajectory.AlphaT(distance, goal_velocity, goal_acceleration);

    const ::Eigen::Matrix<double, 6, 1> R = trajectory.R(theta_t, omega_t);
    const ::Eigen::Matrix<double, 2, 1> U =
        dynamics.FF_U(R.block<4, 1>(0, 0), omega_t, alpha_t)
            .array()
            .max(-20)
            .min(20);

    Uff0_distance_array_curvature.push_back(U(0));
    Uff1_distance_array_curvature.push_back(U(1));
  }

  for (const double distance : distance_array) {
    const double goal_velocity =
        trajectory.GetDVelocity(distance, trajectory.max_dvelocity());
    const double goal_acceleration =
        trajectory.GetDAcceleration(distance, trajectory.max_dvelocity());
    const ::Eigen::Matrix<double, 2, 1> theta_t = trajectory.ThetaT(distance);
    const ::Eigen::Matrix<double, 2, 1> omega_t =
        trajectory.OmegaT(distance, goal_velocity);
    const ::Eigen::Matrix<double, 2, 1> alpha_t =
        trajectory.AlphaT(distance, goal_velocity, goal_acceleration);

    const ::Eigen::Matrix<double, 6, 1> R = trajectory.R(theta_t, omega_t);
    const ::Eigen::Matrix<double, 2, 1> U =
        dynamics.FF_U(R.block<4, 1>(0, 0), omega_t, alpha_t)
            .array()
            .max(-20)
            .min(20);

    Uff0_distance_array_backwards_only.push_back(U(0));
    Uff1_distance_array_backwards_only.push_back(U(1));
  }

  double t = 0;
  ::Eigen::Matrix<double, 4, 1> X;
  {
    ::Eigen::Matrix<double, 2, 1> theta_t = trajectory.ThetaT(0.0);
    X << theta_t(0), 0.0, theta_t(1), 0.0;
  }

  frc971::control_loops::arm::TrajectoryFollower follower(&dynamics,
                                                          &trajectory);

  ::std::vector<double> t_array;
  ::std::vector<double> theta0_goal_t_array;
  ::std::vector<double> theta1_goal_t_array;
  ::std::vector<double> omega0_goal_t_array;
  ::std::vector<double> omega1_goal_t_array;
  ::std::vector<double> alpha0_goal_t_array;
  ::std::vector<double> alpha1_goal_t_array;
  ::std::vector<double> theta0_t_array;
  ::std::vector<double> omega0_t_array;
  ::std::vector<double> theta1_t_array;
  ::std::vector<double> omega1_t_array;
  ::std::vector<double> distance_t_array;
  ::std::vector<double> velocity_t_array;
  ::std::vector<double> acceleration_t_array;
  ::std::vector<double> u0_unsaturated_array;
  ::std::vector<double> u1_unsaturated_array;
  ::std::vector<double> alpha0_t_array;
  ::std::vector<double> alpha1_t_array;
  ::std::vector<double> uff0_array;
  ::std::vector<double> uff1_array;
  ::std::vector<double> u0_array;
  ::std::vector<double> u1_array;
  ::std::vector<double> theta0_hat_t_array;
  ::std::vector<double> omega0_hat_t_array;
  ::std::vector<double> theta1_hat_t_array;
  ::std::vector<double> omega1_hat_t_array;
  ::std::vector<double> torque0_hat_t_array;
  ::std::vector<double> torque1_hat_t_array;

  frc971::control_loops::arm::EKF arm_ekf(&dynamics);
  arm_ekf.Reset(X);
  ::std::cout << "Reset P: " << arm_ekf.P_reset() << ::std::endl;
  ::std::cout << "Stabilized P: " << arm_ekf.P_half_converged() << ::std::endl;
  ::std::cout << "Really stabilized P: " << arm_ekf.P_converged()
              << ::std::endl;

  while (t < 1.5) {
    t_array.push_back(t);
    arm_ekf.Correct((::Eigen::Matrix<double, 2, 1>() << X(0), X(2)).finished(),
                    sim_dt);
    follower.Update(arm_ekf.X_hat(), false, sim_dt, vmax, 12.0);

    const ::Eigen::Matrix<double, 2, 1> theta_t =
        trajectory.ThetaT(follower.goal()(0));
    const ::Eigen::Matrix<double, 2, 1> omega_t =
        trajectory.OmegaT(follower.goal()(0), follower.goal()(1));
    const ::Eigen::Matrix<double, 2, 1> alpha_t = trajectory.AlphaT(
        follower.goal()(0), follower.goal()(1), follower.goal_acceleration());

    theta0_goal_t_array.push_back(theta_t(0));
    theta1_goal_t_array.push_back(theta_t(1));
    omega0_goal_t_array.push_back(omega_t(0));
    omega1_goal_t_array.push_back(omega_t(1));
    alpha0_goal_t_array.push_back(alpha_t(0));
    alpha1_goal_t_array.push_back(alpha_t(1));
    theta0_t_array.push_back(X(0));
    omega0_t_array.push_back(X(1));
    theta1_t_array.push_back(X(2));
    omega1_t_array.push_back(X(3));
    theta0_hat_t_array.push_back(arm_ekf.X_hat(0));
    omega0_hat_t_array.push_back(arm_ekf.X_hat(1));
    theta1_hat_t_array.push_back(arm_ekf.X_hat(2));
    omega1_hat_t_array.push_back(arm_ekf.X_hat(3));
    torque0_hat_t_array.push_back(arm_ekf.X_hat(4));
    torque1_hat_t_array.push_back(arm_ekf.X_hat(5));

    distance_t_array.push_back(follower.goal()(0));
    velocity_t_array.push_back(follower.goal()(1));
    acceleration_t_array.push_back(follower.goal_acceleration());

    u0_unsaturated_array.push_back(follower.U_unsaturated()(0));
    u1_unsaturated_array.push_back(follower.U_unsaturated()(1));

    ::Eigen::Matrix<double, 2, 1> actual_U = follower.U();
    // Add in a disturbance force to see how well the arm learns it.
    actual_U(0) += 1.0;

    const ::Eigen::Matrix<double, 4, 1> xdot =
        dynamics.Acceleration(X, actual_U);

    X = dynamics.UnboundedDiscreteDynamics(X, actual_U, sim_dt);
    arm_ekf.Predict(follower.U(), sim_dt);

    alpha0_t_array.push_back(xdot(1));
    alpha1_t_array.push_back(xdot(3));

    uff0_array.push_back(follower.U_ff()(0));
    uff1_array.push_back(follower.U_ff()(1));
    u0_array.push_back(follower.U()(0));
    u1_array.push_back(follower.U()(1));

    t += sim_dt;
  }

  if (FLAGS_plot) {
    frc971::analysis::Plotter plotter;

    plotter.AddFigure();
    plotter.Title("Trajectory");
    plotter.AddLine(theta0_array, theta1_array, "desired path");
    plotter.AddLine(theta0_t_array, theta1_t_array, "actual path");
    plotter.Publish();

    plotter.AddFigure();
    plotter.Title("Input spline");
    plotter.AddLine(distance_array, theta0_array, "theta0");
    plotter.AddLine(distance_array, theta1_array, "theta1");
    plotter.AddLine(distance_array, omega0_array, "omega0");
    plotter.AddLine(distance_array, omega1_array, "omega1");
    plotter.AddLine(distance_array, alpha0_array, "alpha0");
    plotter.AddLine(distance_array, alpha1_array, "alpha1");

    plotter.AddLine(integrated_distance, integrated_theta0_array,
                    "integrated theta0");
    plotter.AddLine(integrated_distance, integrated_theta1_array,
                    "integrated theta1");
    plotter.AddLine(integrated_distance, integrated_omega0_array,
                    "integrated omega0");
    plotter.AddLine(integrated_distance, integrated_omega1_array,
                    "integrated omega1");
    plotter.Publish();

    plotter.AddFigure();
    plotter.Title("Solver passes");
    plotter.AddLine(distance_array, trajectory.max_dvelocity_unfiltered(),
                    "pass0");
    plotter.AddLine(distance_array, trajectory.max_dvelocity(), "passb");
    plotter.AddLine(distance_array, trajectory.max_dvelocity_forward_pass(),
                    "passf");
    plotter.Publish();

    plotter.AddFigure();
    plotter.Title("Time Goals");
    plotter.AddLine(t_array, alpha0_goal_t_array, "alpha0_t_goal");
    plotter.AddLine(t_array, alpha0_t_array, "alpha0_t");
    plotter.AddLine(t_array, alpha1_goal_t_array, "alpha1_t_goal");
    plotter.AddLine(t_array, alpha1_t_array, "alpha1_t");
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
    plotter.Publish();

    plotter.AddFigure();
    plotter.Title("Voltages");
    plotter.AddLine(t_array, u0_unsaturated_array, "u0_full");
    plotter.AddLine(t_array, u0_array, "u0");
    plotter.AddLine(t_array, uff0_array, "uff0");
    plotter.AddLine(t_array, u1_unsaturated_array, "u1_full");
    plotter.AddLine(t_array, u1_array, "u1");
    plotter.AddLine(t_array, uff1_array, "uff1");
    plotter.AddLine(t_array, torque0_hat_t_array, "torque0_hat");
    plotter.AddLine(t_array, torque1_hat_t_array, "torque1_hat");
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
      plotter.Publish();
    }

    plotter.AddFigure();
    plotter.Title("ff for distance");
    plotter.AddLine(distance_array, Uff0_distance_array, "ff0");
    plotter.AddLine(distance_array, Uff1_distance_array, "ff1");
    plotter.AddLine(distance_array, Uff0_distance_array_backwards_only,
                    "ff0_back");
    plotter.AddLine(distance_array, Uff1_distance_array_backwards_only,
                    "ff1_back");
    plotter.AddLine(distance_array, Uff0_distance_array_curvature, "ff0_curve");
    plotter.AddLine(distance_array, Uff1_distance_array_curvature, "ff1_curve");

    plotter.Publish();
    plotter.Spin();
  }
}

}  // namespace arm
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2018

int main(int argc, char **argv) {
  ::aos::InitGoogle(&argc, &argv);
  ::y2018::control_loops::superstructure::arm::Main();
  return 0;
}
