#include "aos/init.h"
#include "frc971/analysis/in_process_plotter.h"
#include "frc971/control_loops/dlqr.h"
#include "frc971/control_loops/double_jointed_arm/dynamics.h"
#include "frc971/control_loops/jacobian.h"
#include "y2023/control_loops/superstructure/arm/arm_constants.h"

DEFINE_double(lqr_proximal_pos, 0.15, "Position LQR gain");
DEFINE_double(lqr_proximal_vel, 4.0, "Velocity LQR gain");
DEFINE_double(lqr_distal_pos, 0.20, "Position LQR gain");
DEFINE_double(lqr_distal_vel, 4.0, "Velocity LQR gain");
DEFINE_double(fx, 0.0, "X force");
DEFINE_double(fy, 0.0, "y force");

DEFINE_double(start0, 0.0, "starting position on proximal");
DEFINE_double(start1, 0.0, "starting position on distal");
DEFINE_double(goal0, 0.0, "goal position on proximal");
DEFINE_double(goal1, 0.0, "goal position on distal");

namespace y2023 {
namespace control_loops {
namespace superstructure {
namespace arm {

int Main() {
  frc971::analysis::Plotter plotter;

  frc971::control_loops::arm::Dynamics dynamics(kArmConstants);

  ::Eigen::Matrix<double, 4, 1> X = ::Eigen::Matrix<double, 4, 1>::Zero();
  ::Eigen::Matrix<double, 4, 1> goal = ::Eigen::Matrix<double, 4, 1>::Zero();
  goal(0, 0) = FLAGS_goal0;
  goal(1, 0) = 0;
  goal(2, 0) = FLAGS_goal1;
  goal(3, 0) = 0;

  X(0, 0) = FLAGS_start0;
  X(1, 0) = 0;
  X(2, 0) = FLAGS_start1;
  X(3, 0) = 0;
  ::Eigen::Matrix<double, 2, 1> U = ::Eigen::Matrix<double, 2, 1>::Zero();

  constexpr double kDt = 0.00505;

  std::vector<double> t;
  std::vector<double> x0;
  std::vector<double> x1;
  std::vector<double> x2;
  std::vector<double> x3;
  std::vector<double> u0;
  std::vector<double> u1;

  std::vector<double> current0;
  std::vector<double> current1;
  std::vector<double> torque0;
  std::vector<double> torque1;

  const double kProximalPosLQR = FLAGS_lqr_proximal_pos;
  const double kProximalVelLQR = FLAGS_lqr_proximal_vel;
  const double kDistalPosLQR = FLAGS_lqr_distal_pos;
  const double kDistalVelLQR = FLAGS_lqr_distal_vel;
  const ::Eigen::DiagonalMatrix<double, 4> Q =
      (::Eigen::DiagonalMatrix<double, 4>().diagonal()
           << 1.0 / ::std::pow(kProximalPosLQR, 2),
       1.0 / ::std::pow(kProximalVelLQR, 2), 1.0 / ::std::pow(kDistalPosLQR, 2),
       1.0 / ::std::pow(kDistalVelLQR, 2))
          .finished()
          .asDiagonal();

  const ::Eigen::DiagonalMatrix<double, 2> R =
      (::Eigen::DiagonalMatrix<double, 2>().diagonal()
           << 1.0 / ::std::pow(12.0, 2),
       1.0 / ::std::pow(12.0, 2))
          .finished()
          .asDiagonal();

  {
    const ::Eigen::Matrix<double, 2, 1> torque =
        dynamics
            .TorqueFromForce(X,
                             ::Eigen::Matrix<double, 2, 1>(FLAGS_fx, FLAGS_fy))
            .transpose();
    LOG(INFO) << "Torque (N m): " << torque.transpose();
    const ::Eigen::Matrix<double, 2, 1> current =
        dynamics.CurrentFromTorque(torque);

    LOG(INFO) << "Current (Amps): " << current.transpose();

    ::Eigen::Matrix<double, 2, 1> battery_current;
    battery_current(0) =
        current(0) * current(0) * kArmConstants.resistance / 12.0;
    battery_current(1) =
        current(1) * current(1) * kArmConstants.resistance / 12.0;

    LOG(INFO) << "Battery current (Amps): " << battery_current.transpose();
  }

  ::Eigen::Matrix<double, 2, 4> last_K = ::Eigen::Matrix<double, 2, 4>::Zero();
  for (int i = 0; i < 400; ++i) {
    t.push_back(i * kDt);
    x0.push_back(X(0));
    x1.push_back(X(1));
    x2.push_back(X(2));
    x3.push_back(X(3));

    const auto x_blocked = X.block<4, 1>(0, 0);

    const ::Eigen::Matrix<double, 4, 4> final_A =
        ::frc971::control_loops::NumericalJacobianX<4, 2>(
            [dynamics](const auto &x_blocked, const auto &U, double kDt) {
              return dynamics.UnboundedDiscreteDynamics(x_blocked, U, kDt);
            },
            x_blocked, U, 0.00505);
    const ::Eigen::Matrix<double, 4, 2> final_B =
        ::frc971::control_loops::NumericalJacobianU<4, 2>(
            [dynamics](const auto &x_blocked, const auto &U, double kDt) {
              return dynamics.UnboundedDiscreteDynamics(x_blocked, U, kDt);
            },
            x_blocked, U, 0.00505);

    ::Eigen::Matrix<double, 2, 1> U_ff =
        dynamics.FF_U(x_blocked, ::Eigen::Matrix<double, 2, 1>::Zero(),
                      ::Eigen::Matrix<double, 2, 1>::Zero());

    ::Eigen::Matrix<double, 4, 4> S;
    ::Eigen::Matrix<double, 2, 4> K;
    if (::frc971::controls::dlqr<4, 2>(final_A, final_B, Q, R, &K, &S) == 0) {
      ::Eigen::EigenSolver<::Eigen::Matrix<double, 4, 4>> eigensolver(
          final_A - final_B * K);

      last_K = K;
    } else {
      LOG(INFO) << "Failed to solve for K at " << i;
    }
    U = U_ff + last_K * (goal - X);
    if (std::abs(U(0)) > 12.0) {
      U /= std::abs(U(0)) / 12.0;
    }
    if (std::abs(U(1)) > 12.0) {
      U /= std::abs(U(1)) / 12.0;
    }

    const ::Eigen::Matrix<double, 2, 1> torque =
        dynamics.TorqueFromCommand(X, U);
    const ::Eigen::Matrix<double, 2, 1> current_per_motor =
        dynamics.CurrentFromCommand(X, U);

    current0.push_back(current_per_motor(0));
    current1.push_back(current_per_motor(1));
    torque0.push_back(torque(0));
    torque1.push_back(torque(1));

    u0.push_back(U(0));
    u1.push_back(U(1));

    X = dynamics.UnboundedDiscreteDynamics(X, U, kDt);
  }

  plotter.Title("Arm motion");
  plotter.AddFigure("State");
  plotter.AddLine(t, x0, "X 0");
  plotter.AddLine(t, x1, "X 1");
  plotter.AddLine(t, x2, "X 2");
  plotter.AddLine(t, x3, "X 3");

  plotter.AddLine(t, u0, "U 0");
  plotter.AddLine(t, u1, "U 1");
  plotter.Publish();

  plotter.AddFigure("Command");
  plotter.AddLine(t, u0, "U 0");
  plotter.AddLine(t, u1, "U 1");

  plotter.AddLine(t, current0, "current 0");
  plotter.AddLine(t, current1, "current 1");
  plotter.AddLine(t, torque0, "torque 0");
  plotter.AddLine(t, torque1, "torque 1");
  plotter.Publish();

  plotter.Spin();

  return 0;
}

}  // namespace arm
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2023

int main(int argc, char **argv) {
  ::aos::InitGoogle(&argc, &argv);
  return y2023::control_loops::superstructure::arm::Main();
}
