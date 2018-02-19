#include "y2018/control_loops/superstructure/arm/trajectory.h"

#include "third_party/matplotlib-cpp/matplotlibcpp.h"
#include "y2018/control_loops/superstructure/arm/demo_path.h"
#include "y2018/control_loops/superstructure/arm/dynamics.h"

namespace y2018 {
namespace control_loops {
namespace superstructure {
namespace arm {

void Main() {
  Path p = MakeDemoPath();
  Trajectory trajectory(&p, 0.001);

  constexpr double kAlpha0Max = 40.0;
  constexpr double kAlpha1Max = 60.0;
  constexpr double vmax = 11.95;
  constexpr double sim_dt = 0.00505;

  const ::Eigen::Matrix<double, 2, 2> alpha_unitizer =
      (::Eigen::Matrix<double, 2, 2>() << 1.0 / kAlpha0Max, 0.0, 0.0,
       1.0 / kAlpha1Max)
          .finished();
  trajectory.OptimizeTrajectory(alpha_unitizer, vmax);

  ::std::vector<double> distance_array = trajectory.DistanceArray();

  ::std::vector<double> theta0_array;
  ::std::vector<double> theta1_array;
  ::std::vector<double> omega0_array;
  ::std::vector<double> omega1_array;
  ::std::vector<double> alpha0_array;
  ::std::vector<double> alpha1_array;

  for (const double d : distance_array) {
    const ::Eigen::Matrix<double, 2, 1> theta = p.Theta(d);
    const ::Eigen::Matrix<double, 2, 1> omega = p.Omega(d);
    const ::Eigen::Matrix<double, 2, 1> alpha = p.Alpha(d);
    theta0_array.push_back(theta(0, 0));
    theta1_array.push_back(theta(1, 0));
    omega0_array.push_back(omega(0, 0));
    omega1_array.push_back(omega(1, 0));
    alpha0_array.push_back(alpha(0, 0));
    alpha1_array.push_back(alpha(1, 0));
  }

  // Next step: see what U is as a function of distance.
  ::std::vector<double> Uff0_distance_array;
  ::std::vector<double> Uff1_distance_array;

  for (const double distance : distance_array) {
    const double goal_velocity = trajectory.GetDVelocity(distance);
    const double goal_acceleration = trajectory.GetDAcceleration(distance);
    const ::Eigen::Matrix<double, 2, 1> theta_t = trajectory.ThetaT(distance);
    const ::Eigen::Matrix<double, 2, 1> omega_t =
        trajectory.OmegaT(distance, goal_velocity);
    const ::Eigen::Matrix<double, 2, 1> alpha_t =
        trajectory.AlphaT(distance, goal_velocity, goal_acceleration);

    const ::Eigen::Matrix<double, 4, 1> R = trajectory.R(theta_t, omega_t);
    const ::Eigen::Matrix<double, 2, 1> U = Dynamics::FF_U(R, omega_t, alpha_t);

    Uff0_distance_array.push_back(U(0));
    Uff1_distance_array.push_back(U(1));
  }

  double t = 0;
  ::Eigen::Matrix<double, 4, 1> X;
  {
    ::Eigen::Matrix<double, 2, 1> theta_t = trajectory.ThetaT(0.0);
    X << theta_t(0), 0.0, theta_t(1), 0.0;
  }

  TrajectoryFollower follower(&p, &trajectory, alpha_unitizer);

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

  while (t < 1.0) {
    t_array.push_back(t);
    follower.Update(X, sim_dt, vmax);

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

    distance_t_array.push_back(follower.goal()(0));
    velocity_t_array.push_back(follower.goal()(1));
    acceleration_t_array.push_back(follower.goal_acceleration());

    u0_unsaturated_array.push_back(follower.U_unsaturated()(0));
    u1_unsaturated_array.push_back(follower.U_unsaturated()(1));

    const ::Eigen::Matrix<double, 4, 1> xdot =
        Dynamics::Acceleration(X, follower.U());

    X = Dynamics::UnboundedDiscreteDynamics(X, follower.U(), sim_dt);

    alpha0_t_array.push_back(xdot(1));
    alpha1_t_array.push_back(xdot(3));

    uff0_array.push_back(follower.U_ff()(0));
    uff1_array.push_back(follower.U_ff()(1));
    u0_array.push_back(follower.U()(0));
    u1_array.push_back(follower.U()(1));

    t += sim_dt;
  }

  matplotlibcpp::figure();
  matplotlibcpp::title("Trajectory");
  matplotlibcpp::plot(theta0_array, theta1_array, {{"label", "desired path"}});
  matplotlibcpp::plot(theta0_t_array, theta1_t_array,
                      {{"label", "actual path"}});
  matplotlibcpp::legend();

  matplotlibcpp::figure();
  matplotlibcpp::plot(distance_array, theta0_array, {{"label", "theta0"}});
  matplotlibcpp::plot(distance_array, theta1_array, {{"label", "theta1"}});
  matplotlibcpp::plot(distance_array, omega0_array, {{"label", "omega0"}});
  matplotlibcpp::plot(distance_array, omega1_array, {{"label", "omega1"}});
  matplotlibcpp::plot(distance_array, alpha0_array, {{"label", "alpha0"}});
  matplotlibcpp::plot(distance_array, alpha1_array, {{"label", "alpha1"}});
  matplotlibcpp::legend();

  matplotlibcpp::figure();
  matplotlibcpp::plot(distance_array, trajectory.max_dvelocity_unfiltered(),
                      {{"label", "pass0"}});
  matplotlibcpp::plot(distance_array, trajectory.max_dvelocity(),
                      {{"label", "passb"}});
  matplotlibcpp::plot(distance_array, trajectory.max_dvelocity_forward_pass(),
                      {{"label", "passf"}});
  matplotlibcpp::legend();

  matplotlibcpp::figure();
  matplotlibcpp::plot(t_array, alpha0_goal_t_array,
                      {{"label", "alpha0_t_goal"}});
  matplotlibcpp::plot(t_array, alpha0_t_array, {{"label", "alpha0_t"}});
  matplotlibcpp::plot(t_array, alpha1_goal_t_array,
                      {{"label", "alpha1_t_goal"}});
  matplotlibcpp::plot(t_array, alpha1_t_array, {{"label", "alpha1_t"}});
  matplotlibcpp::plot(t_array, distance_t_array, {{"label", "distance_t"}});
  matplotlibcpp::plot(t_array, velocity_t_array, {{"label", "velocity_t"}});
  matplotlibcpp::plot(t_array, acceleration_t_array,
                      {{"label", "acceleration_t"}});
  matplotlibcpp::legend();

  matplotlibcpp::figure();
  matplotlibcpp::title("Angular Velocities");
  matplotlibcpp::plot(t_array, omega0_goal_t_array,
                      {{"label", "omega0_t_goal"}});
  matplotlibcpp::plot(t_array, omega0_t_array, {{"label", "omega0_t"}});
  matplotlibcpp::plot(t_array, omega1_goal_t_array,
                      {{"label", "omega1_t_goal"}});
  matplotlibcpp::plot(t_array, omega1_t_array, {{"label", "omega1_t"}});
  matplotlibcpp::legend();

  matplotlibcpp::figure();
  matplotlibcpp::title("Voltages");
  matplotlibcpp::plot(t_array, u0_unsaturated_array, {{"label", "u0_full"}});
  matplotlibcpp::plot(t_array, u0_array, {{"label", "u0"}});
  matplotlibcpp::plot(t_array, uff0_array, {{"label", "uff0"}});
  matplotlibcpp::plot(t_array, u1_unsaturated_array, {{"label", "u1_full"}});
  matplotlibcpp::plot(t_array, u1_array, {{"label", "u1"}});
  matplotlibcpp::plot(t_array, uff1_array, {{"label", "uff1"}});
  matplotlibcpp::legend();

  matplotlibcpp::figure();
  matplotlibcpp::title("Angles");
  matplotlibcpp::plot(t_array, theta0_goal_t_array,
                      {{"label", "theta0_t_goal"}});
  matplotlibcpp::plot(t_array, theta0_t_array, {{"label", "theta0_t"}});
  matplotlibcpp::plot(t_array, theta1_goal_t_array,
                      {{"label", "theta1_t_goal"}});
  matplotlibcpp::plot(t_array, theta1_t_array, {{"label", "theta1_t"}});
  matplotlibcpp::legend();

/*
  matplotlibcpp::figure();
  matplotlibcpp::title("ff for distance");
  matplotlibcpp::plot(distance_array, Uff0_distance_array, {{"label", "ff0"}});
  matplotlibcpp::plot(distance_array, Uff1_distance_array, {{"label", "ff1"}});
  matplotlibcpp::legend();
*/

  matplotlibcpp::show();
}

}  // namespace arm
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2018

int main(int /*argc*/, const char * /*argv*/ []) {
  ::y2018::control_loops::superstructure::arm::Main();
  return 0;
}
