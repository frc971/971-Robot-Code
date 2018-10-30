#include <chrono>
#include <cmath>
#include <thread>

#include "gflags/gflags.h"
#include "third_party/matplotlib-cpp/matplotlibcpp.h"
#include "y2018/control_loops/python/arm_bounds.h"

DEFINE_double(boundary_scalar, 20000.0, "quadratic slope");
DEFINE_double(boundary_rate, 1000.0, "linear slope");
DEFINE_double(bounds_offset, 0.02, "Offset the quadratic boundary in by this");

using ::y2018::control_loops::Point;

int main(int argc, char **argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, false);

  ::y2018::control_loops::BoundsCheck arm_space =
      ::y2018::control_loops::MakeClippedArmSpace();

  matplotlibcpp::figure();
  ::std::vector<double> bounds_x;
  ::std::vector<double> bounds_y;
  for (const Point p : arm_space.points()) {
    bounds_x.push_back(p.x());
    bounds_y.push_back(p.y());
  }
  matplotlibcpp::plot(bounds_x, bounds_y, {{"label", "actual region"}});
  matplotlibcpp::legend();

  ::std::vector<::std::vector<double>> cost_x;
  ::std::vector<::std::vector<double>> cost_y;
  ::std::vector<::std::vector<double>> cost_z;
  ::std::vector<::std::vector<double>> cost_z2;
  ::std::vector<::std::vector<double>> cost_z3;

  for (double x_coordinate = -0.5; x_coordinate < 1.2; x_coordinate += 0.05) {
    ::std::vector<double> cost_x_row;
    ::std::vector<double> cost_y_row;
    ::std::vector<double> cost_z_row;
    ::std::vector<double> cost_z2_row;
    ::std::vector<double> cost_z3_row;

    for (double y_coordinate = -1.0; y_coordinate < 5.0; y_coordinate += 0.05) {
      cost_x_row.push_back(x_coordinate);
      cost_y_row.push_back(y_coordinate);

      ::Eigen::Matrix<double, 2, 1> norm;
      double min_distance =
          arm_space.min_distance(Point(x_coordinate, y_coordinate), &norm);
      cost_z_row.push_back(::std::min(
          500.0 * ::std::max(min_distance + FLAGS_bounds_offset, 0.0), 40.0));

      cost_z2_row.push_back(::std::min(
          FLAGS_boundary_scalar *
                  ::std::max(0.0, min_distance + FLAGS_bounds_offset) *
                  ::std::max(0.0, min_distance + FLAGS_bounds_offset) +
              FLAGS_boundary_rate *
                  ::std::max(0.0, min_distance + FLAGS_bounds_offset),
          200.0));

      cost_z3_row.push_back(min_distance);
    }
    cost_x.push_back(cost_x_row);
    cost_y.push_back(cost_y_row);
    cost_z.push_back(cost_z_row);
    cost_z2.push_back(cost_z2_row);
    cost_z3.push_back(cost_z3_row);
  }

  matplotlibcpp::plot_surface(cost_x, cost_y, cost_z);
  matplotlibcpp::plot_surface(cost_x, cost_y, cost_z2);
  matplotlibcpp::plot_surface(cost_x, cost_y, cost_z3);
  matplotlibcpp::show();
}
