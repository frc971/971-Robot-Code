#include "frc971/control_loops/drivetrain/distance_spline.h"

#include <vector>

#include "aos/testing/test_shm.h"
#include "gflags/gflags.h"
#include "gtest/gtest.h"
#if defined(SUPPORT_PLOT)
#include "third_party/matplotlib-cpp/matplotlibcpp.h"
#endif

DEFINE_bool(plot, false, "If true, plot");

namespace frc971 {
namespace control_loops {
namespace drivetrain {
namespace testing {

// Test fixture with a spline from 0, 0 to 1, 1
class ParameterizedDistanceSplineTest
    : public ::testing::TestWithParam<::std::vector<Spline>> {
 protected:
  ParameterizedDistanceSplineTest()
      : distance_spline_(::std::vector<Spline>(GetParam())) {}
  ::aos::testing::TestSharedMemory shm_;
  DistanceSpline distance_spline_;
};

// Tests that the derivitives of xy integrate back up to the position.
TEST_P(ParameterizedDistanceSplineTest, XYIntegral) {
  ::std::vector<double> distances_plot;
  ::std::vector<double> x_plot;
  ::std::vector<double> y_plot;
  ::std::vector<double> ix_plot;
  ::std::vector<double> iy_plot;
  ::std::vector<double> dx_plot;
  ::std::vector<double> dy_plot;
  ::std::vector<double> idx_plot;
  ::std::vector<double> idy_plot;

  const int num_points = 10000;
  ::Eigen::Matrix<double, 2, 1> point = distance_spline_.XY(0.0);
  ::Eigen::Matrix<double, 2, 1> dpoint = distance_spline_.DXY(0.0);

  const double ddistance =
      distance_spline_.length() / static_cast<double>(num_points - 1);
  for (int i = 0; i < num_points; ++i) {
    const double distance = ddistance * static_cast<double>(i);
    const ::Eigen::Matrix<double, 2, 1> expected_point =
        distance_spline_.XY(distance);
    const ::Eigen::Matrix<double, 2, 1> expected_dpoint =
        distance_spline_.DXY(distance);

    distances_plot.push_back(distance);
    x_plot.push_back(expected_point(0));
    y_plot.push_back(expected_point(1));
    ix_plot.push_back(point(0));
    iy_plot.push_back(point(1));
    dx_plot.push_back(expected_dpoint(0));
    dy_plot.push_back(expected_dpoint(1));
    idx_plot.push_back(dpoint(0));
    idy_plot.push_back(dpoint(1));

    EXPECT_LT((point - expected_point).norm(), 1e-2) << ": At distance "
                                                     << distance;
    EXPECT_LT((dpoint - expected_dpoint).norm(), 1e-2) << ": At distance "
                                                       << distance;

    // We need to record the starting state without integrating.
    if (i == 0) {
      continue;
    }

    point += dpoint * ddistance;
    dpoint += distance_spline_.DDXY(distance) * ddistance;
    EXPECT_FLOAT_EQ(distance_spline_.DDXY(distance).norm(),
                    ::std::abs(distance_spline_.DTheta(distance)));
  }

#if defined(SUPPORT_PLOT)
  // Conditionally plot the functions and their integrals to aid debugging.
  if (FLAGS_plot) {
    matplotlibcpp::figure();
    matplotlibcpp::plot(distances_plot, x_plot, {{"label", "x"}});
    matplotlibcpp::plot(distances_plot, ix_plot, {{"label", "ix"}});
    matplotlibcpp::plot(distances_plot, y_plot, {{"label", "y"}});
    matplotlibcpp::plot(distances_plot, iy_plot, {{"label", "iy"}});
    matplotlibcpp::plot(distances_plot, dx_plot, {{"label", "dx"}});
    matplotlibcpp::plot(distances_plot, idx_plot, {{"label", "idx"}});
    matplotlibcpp::plot(distances_plot, dy_plot, {{"label", "dy"}});
    matplotlibcpp::plot(distances_plot, idy_plot, {{"label", "idy"}});
    matplotlibcpp::legend();

    matplotlibcpp::figure();
    matplotlibcpp::plot(x_plot, y_plot, {{"label", "spline"}});
    matplotlibcpp::legend();

    matplotlibcpp::show();
  }
#endif
}

// Tests that the derivitives of xy integrate back up to the position.
TEST_P(ParameterizedDistanceSplineTest, ThetaIntegral) {
  ::std::vector<double> distances_plot;
  ::std::vector<double> theta_plot;
  ::std::vector<double> itheta_plot;
  ::std::vector<double> dtheta_plot;
  ::std::vector<double> idtheta_plot;

  const int num_points = 10000;
  double theta = distance_spline_.Theta(0.0);
  double dtheta = distance_spline_.DTheta(0.0);

  const double ddistance =
      distance_spline_.length() / static_cast<double>(num_points - 1);
  for (int i = 0; i < num_points; ++i) {
    const double distance = ddistance * static_cast<double>(i);
    const double expected_theta = distance_spline_.Theta(distance);
    const double expected_dtheta = distance_spline_.DTheta(distance);

    distances_plot.push_back(distance);
    theta_plot.push_back(expected_theta);
    itheta_plot.push_back(theta);
    dtheta_plot.push_back(expected_dtheta);
    idtheta_plot.push_back(dtheta);

    EXPECT_NEAR(expected_theta, theta, 1e-2) << ": At distance " << distance;
    EXPECT_NEAR(expected_dtheta, dtheta, 1e-2) << ": At distance " << distance;

    // We need to record the starting state without integrating.
    if (i == 0) {
      continue;
    }

    theta += dtheta * ddistance;
    dtheta += distance_spline_.DDTheta(distance) * ddistance;
  }

#if defined(SUPPORT_PLOT)
  // Conditionally plot the functions and their integrals to aid debugging.
  if (FLAGS_plot) {
    matplotlibcpp::figure();
    matplotlibcpp::plot(distances_plot, theta_plot, {{"label", "theta"}});
    matplotlibcpp::plot(distances_plot, itheta_plot, {{"label", "itheta"}});
    matplotlibcpp::plot(distances_plot, dtheta_plot, {{"label", "dtheta"}});
    matplotlibcpp::plot(distances_plot, idtheta_plot, {{"label", "idtheta"}});
    matplotlibcpp::legend();

    matplotlibcpp::show();
  }
#endif
}

INSTANTIATE_TEST_CASE_P(
    DistanceSplineTest, ParameterizedDistanceSplineTest,
    ::testing::Values(
        ::std::vector<Spline>(
            {Spline(Spline4To6((::Eigen::Matrix<double, 2, 4>() << 0.0, 0.5,
                                0.5, 1.0, 0.0, 0.0, 1.0, 1.0)
                                   .finished()))}),
        ::std::vector<Spline>(
            {Spline(Spline4To6((::Eigen::Matrix<double, 2, 4>() << 0.0, 0.5,
                                0.5, 1.0, 0.0, 0.0, 1.0, 1.0)
                                   .finished())),
             Spline(Spline4To6((::Eigen::Matrix<double, 2, 4>() << 1.0, 1.5,
                                1.5, 2.0, 1.0, 1.0, 0.0, 0.0)
                                   .finished()))})));

}  // namespace testing
}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
