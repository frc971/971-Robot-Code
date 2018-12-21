#include "frc971/control_loops/drivetrain/spline.h"

#include <vector>

#include "gflags/gflags.h"
#include "gtest/gtest.h"
#include "third_party/matplotlib-cpp/matplotlibcpp.h"

DEFINE_bool(plot, false, "If true, plot");

namespace frc971 {
namespace control_loops {
namespace drivetrain {
namespace testing {

// Test fixture with a spline from 0, 0 to 1, 1
class SplineTest : public ::testing::Test {
 protected:
  SplineTest()
      : spline_((::Eigen::Matrix<double, 2, 4>() << 0.0, 0.5, 0.5, 1.0, 0.0,
                 0.0, 1.0, 1.0)
                    .finished()) {}
  Spline spline_;
};

// Tests that the derivitives of xy integrate back up to the position.
TEST_F(SplineTest, XYIntegral) {
  ::std::vector<double> alphas_plot;
  ::std::vector<double> x_plot;
  ::std::vector<double> y_plot;
  ::std::vector<double> ix_plot;
  ::std::vector<double> iy_plot;
  ::std::vector<double> dx_plot;
  ::std::vector<double> dy_plot;
  ::std::vector<double> idx_plot;
  ::std::vector<double> idy_plot;

  const int num_points = 10000;
  ::Eigen::Matrix<double, 2, 1> point = spline_.Point(0.0);
  ::Eigen::Matrix<double, 2, 1> dpoint = spline_.DPoint(0.0);
  ::Eigen::Matrix<double, 2, 1> ddpoint = spline_.DDPoint(0.0);

  const double dalpha = 1.0 / static_cast<double>(num_points - 1);
  for (int i = 0; i < num_points; ++i) {
    const double alpha =
        1.0 * static_cast<double>(i) / static_cast<double>(num_points - 1);
    const ::Eigen::Matrix<double, 2, 1> expected_point = spline_.Point(alpha);
    const ::Eigen::Matrix<double, 2, 1> expected_dpoint = spline_.DPoint(alpha);
    const ::Eigen::Matrix<double, 2, 1> expected_ddpoint =
        spline_.DDPoint(alpha);

    alphas_plot.push_back(alpha);
    x_plot.push_back(expected_point(0));
    y_plot.push_back(expected_point(1));
    ix_plot.push_back(point(0));
    iy_plot.push_back(point(1));
    dx_plot.push_back(expected_dpoint(0));
    dy_plot.push_back(expected_dpoint(1));
    idx_plot.push_back(dpoint(0));
    idy_plot.push_back(dpoint(1));

    EXPECT_LT((point - expected_point).norm(), 1e-2) << ": At alpha " << alpha;
    EXPECT_LT((dpoint - expected_dpoint).norm(), 1e-2) << ": At alpha "
                                                       << alpha;
    EXPECT_LT((ddpoint - expected_ddpoint).norm(), 1e-2) << ": At alpha "
                                                         << alpha;

    // We need to record the starting state without integrating.
    if (i == 0) {
      continue;
    }

    point += dpoint * dalpha;
    dpoint += ddpoint * dalpha;
    ddpoint += spline_.DDDPoint(alpha) * dalpha;
  }

  // Conditionally plot the functions and their integrals to aid debugging.
  if (FLAGS_plot) {
    matplotlibcpp::figure();
    matplotlibcpp::plot(alphas_plot, x_plot, {{"label", "x"}});
    matplotlibcpp::plot(alphas_plot, ix_plot, {{"label", "ix"}});
    matplotlibcpp::plot(alphas_plot, y_plot, {{"label", "y"}});
    matplotlibcpp::plot(alphas_plot, iy_plot, {{"label", "iy"}});
    matplotlibcpp::plot(alphas_plot, dx_plot, {{"label", "dx"}});
    matplotlibcpp::plot(alphas_plot, idx_plot, {{"label", "idx"}});
    matplotlibcpp::plot(alphas_plot, dy_plot, {{"label", "dy"}});
    matplotlibcpp::plot(alphas_plot, idy_plot, {{"label", "idy"}});
    matplotlibcpp::legend();

    matplotlibcpp::show();
  }
}

// Tests that the derivitives of theta integrate back up to the angle.
TEST_F(SplineTest, ThetaIntegral) {
  ::std::vector<double> alphas_plot;
  ::std::vector<double> theta_plot;
  ::std::vector<double> itheta_plot;
  ::std::vector<double> dtheta_plot;
  ::std::vector<double> idtheta_plot;

  const int num_points = 10000;
  double theta = spline_.Theta(0.0);
  double dtheta = spline_.DTheta(0.0);

  const double dalpha = 1.0 / static_cast<double>(num_points - 1);
  for (int i = 0; i < num_points; ++i) {
    const double alpha =
        1.0 * static_cast<double>(i) / static_cast<double>(num_points - 1);
    const double expected_theta = spline_.Theta(alpha);
    const double expected_dtheta = spline_.DTheta(alpha);

    alphas_plot.push_back(alpha);
    theta_plot.push_back(expected_theta);
    itheta_plot.push_back(theta);
    dtheta_plot.push_back(expected_dtheta);
    idtheta_plot.push_back(dtheta);

    EXPECT_NEAR(expected_theta, theta, 1e-2) << ": At alpha " << alpha;
    EXPECT_NEAR(expected_dtheta, dtheta, 1e-2) << ": At alpha " << alpha;

    // We need to record the starting state without integrating.
    if (i == 0) {
      continue;
    }

    theta += dtheta * dalpha;
    dtheta += spline_.DDTheta(alpha) * dalpha;
  }

  // Conditionally plot the functions and their integrals to aid debugging.
  if (FLAGS_plot) {
    matplotlibcpp::figure();
    matplotlibcpp::plot(alphas_plot, theta_plot, {{"label", "theta"}});
    matplotlibcpp::plot(alphas_plot, itheta_plot, {{"label", "itheta"}});
    matplotlibcpp::plot(alphas_plot, dtheta_plot, {{"label", "dtheta"}});
    matplotlibcpp::plot(alphas_plot, idtheta_plot, {{"label", "idtheta"}});
    matplotlibcpp::legend();

    matplotlibcpp::show();
  }
}

}  // namespace testing
}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
