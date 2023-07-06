#include "frc971/control_loops/drivetrain/spline.h"

#include <vector>

#include "gflags/gflags.h"
#include "gtest/gtest.h"

#include "frc971/analysis/in_process_plotter.h"

DEFINE_bool(plot, false, "If true, plot");

namespace frc971 {
namespace control_loops {
namespace drivetrain {
namespace testing {

std::string TestName() {
  const ::testing::TestInfo *info =
      ::testing::UnitTest::GetInstance()->current_test_info();
  return std::string(info->test_case_name()) + "." + std::string(info->name());
}

// Test fixture with a spline from 0, 0 to 1, 1
class SplineTest : public ::testing::Test {
 public:
  static void SetUpTestSuite() {
    if (FLAGS_plot) {
      plotter_ = std::make_unique<analysis::Plotter>();
    }
  }

  static void TearDownTestSuite() {
    if (FLAGS_plot) {
      plotter_->Spin();
    }
  }

 protected:
  SplineTest()
      : control_points_((::Eigen::Matrix<double, 2, 4>() << 0.0, 0.5, 0.5, 1.0,
                         0.0, 0.0, 1.0, 1.0)
                            .finished()),
        spline4_(control_points_),
        spline6_(Spline4To6(control_points_)) {
    if (FLAGS_plot) {
      CHECK(plotter_);
      plotter_->Title(TestName());
    }
  }
  ~SplineTest() {}

  void TearDown() override {
    if (FLAGS_plot) {
      plotter_->Publish();
    }
  }

  static std::unique_ptr<analysis::Plotter> plotter_;

  ::Eigen::Matrix<double, 2, 4> control_points_;
  NSpline<4> spline4_;
  NSpline<6> spline6_;
};

std::unique_ptr<analysis::Plotter> SplineTest::plotter_;

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
  ::Eigen::Matrix<double, 2, 1> point = spline6_.Point(0.0);
  ::Eigen::Matrix<double, 2, 1> dpoint = spline6_.DPoint(0.0);
  ::Eigen::Matrix<double, 2, 1> ddpoint = spline6_.DDPoint(0.0);

  const double dalpha = 1.0 / static_cast<double>(num_points - 1);
  for (int i = 0; i < num_points; ++i) {
    const double alpha =
        1.0 * static_cast<double>(i) / static_cast<double>(num_points - 1);
    const ::Eigen::Matrix<double, 2, 1> expected_point = spline6_.Point(alpha);
    const ::Eigen::Matrix<double, 2, 1> expected_dpoint =
        spline6_.DPoint(alpha);
    const ::Eigen::Matrix<double, 2, 1> expected_ddpoint =
        spline6_.DDPoint(alpha);

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
    EXPECT_LT((dpoint - expected_dpoint).norm(), 1e-2)
        << ": At alpha " << alpha;
    EXPECT_LT((ddpoint - expected_ddpoint).norm(), 1e-2)
        << ": At alpha " << alpha;

    // We need to record the starting state without integrating.
    if (i == 0) {
      continue;
    }

    point += dpoint * dalpha;
    dpoint += ddpoint * dalpha;
    ddpoint += spline6_.DDDPoint(alpha) * dalpha;
  }

  // Conditionally plot the functions and their integrals to aid debugging.
  if (FLAGS_plot) {
    plotter_->AddFigure("Spline Attributes Over Alpha");
    plotter_->AddLine(alphas_plot, x_plot, "X");
    plotter_->AddLine(alphas_plot, ix_plot, "Integrated X");
    plotter_->AddLine(alphas_plot, y_plot, "Y");
    plotter_->AddLine(alphas_plot, iy_plot, "Integrated Y");
    plotter_->AddLine(alphas_plot, dx_plot, "dX");
    plotter_->AddLine(alphas_plot, idx_plot, "Integrated dX");
    plotter_->AddLine(alphas_plot, dy_plot, "dY");
    plotter_->AddLine(alphas_plot, idy_plot, "Integrated dY");
    plotter_->XLabel("Spline Alpha");
    plotter_->YLabel("X/Y (m), dX, dY (m / alpha)");
    plotter_->Publish();

    plotter_->AddFigure("X/Y Plot of Spline Path");
    plotter_->AddLine(x_plot, y_plot, "spline");
    plotter_->XLabel("X (m)");
    plotter_->YLabel("Y (m)");
    plotter_->Publish();
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
  double theta = spline6_.Theta(0.0);
  double dtheta = spline6_.DTheta(0.0);

  const double dalpha = 1.0 / static_cast<double>(num_points - 1);
  for (int i = 0; i < num_points; ++i) {
    const double alpha =
        1.0 * static_cast<double>(i) / static_cast<double>(num_points - 1);
    const double expected_theta = spline6_.Theta(alpha);
    const double expected_dtheta = spline6_.DTheta(alpha);

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
    dtheta += spline6_.DDTheta(alpha) * dalpha;
  }

  // Conditionally plot the functions and their integrals to aid debugging.
  if (FLAGS_plot) {
    plotter_->AddFigure("Heading Plot");
    plotter_->AddLine(alphas_plot, theta_plot, "theta");
    plotter_->AddLine(alphas_plot, itheta_plot, "Integrated theta");
    plotter_->AddLine(alphas_plot, dtheta_plot, "dtheta");
    plotter_->AddLine(alphas_plot, idtheta_plot, "Integrated dtheta");
    plotter_->XLabel("Alpha");
    plotter_->YLabel("Theta (rad), Dtheta (rad / alpha)");
  }
}

// Tests that a 4 point spline has the same points as a 6 point spline built
// with Spline4To6.
TEST_F(SplineTest, FourToSixSpline) {
  const int num_points = 10000;

  ::std::vector<double> alphas_plot;
  ::std::vector<double> x_plot;
  ::std::vector<double> y_plot;

  const double dalpha = 1.0 / static_cast<double>(num_points - 1);
  for (int i = 0; i < num_points; ++i) {
    const double alpha = dalpha * static_cast<double>(i);

    const ::Eigen::Matrix<double, 2, 1> expected_point = spline4_.Point(alpha);
    const ::Eigen::Matrix<double, 2, 1> expected_dpoint =
        spline4_.DPoint(alpha);
    const ::Eigen::Matrix<double, 2, 1> expected_ddpoint =
        spline4_.DDPoint(alpha);
    const ::Eigen::Matrix<double, 2, 1> expected_dddpoint =
        spline4_.DDDPoint(alpha);

    const ::Eigen::Matrix<double, 2, 1> point = spline6_.Point(alpha);
    const ::Eigen::Matrix<double, 2, 1> dpoint = spline6_.DPoint(alpha);
    const ::Eigen::Matrix<double, 2, 1> ddpoint = spline6_.DDPoint(alpha);
    const ::Eigen::Matrix<double, 2, 1> dddpoint = spline6_.DDDPoint(alpha);

    alphas_plot.push_back(alpha);
    x_plot.push_back(point(0));
    y_plot.push_back(point(1));

    EXPECT_LT((point - expected_point).norm(), 1e-9) << ": At alpha " << alpha;
    EXPECT_LT((dpoint - expected_dpoint).norm(), 1e-9)
        << ": At alpha " << alpha;
    EXPECT_LT((ddpoint - expected_ddpoint).norm(), 1e-9)
        << ": At alpha " << alpha;
    EXPECT_LT((dddpoint - expected_dddpoint).norm(), 1e-9)
        << ": At alpha " << alpha;
  }

  // Conditionally plot the functions and their integrals to aid debugging.
  if (FLAGS_plot) {
    plotter_->AddFigure("Spline X/Y");
    plotter_->AddLine(alphas_plot, x_plot, "X");
    plotter_->AddLine(alphas_plot, y_plot, "Y");
    plotter_->XLabel("Alpha");
    plotter_->YLabel("X/Y (m)");

    ::std::vector<double> control4x;
    ::std::vector<double> control4y;
    ::std::vector<double> control6x;
    ::std::vector<double> control6y;
    for (int i = 0; i < 4; ++i) {
      control4x.push_back(spline4_.control_points()(0, i));
      control4y.push_back(spline4_.control_points()(1, i));
    }
    for (int i = 0; i < 6; ++i) {
      control6x.push_back(spline6_.control_points()(0, i));
      control6y.push_back(spline6_.control_points()(1, i));
    }

    plotter_->AddFigure("Spline Path Control Points Comparison");
    plotter_->AddLine(x_plot, y_plot, "Spline");
    plotter_->AddLine(control4x, control4y, "4-Spline Control Points");
    plotter_->AddLine(control6x, control6y, "6-Spline Control Points");
    plotter_->XLabel("X (m)");
    plotter_->YLabel("Y (m)");
  }
}

}  // namespace testing
}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
