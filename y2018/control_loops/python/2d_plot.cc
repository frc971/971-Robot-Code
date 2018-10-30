#include <chrono>
#include <cmath>
#include <thread>

#include "gflags/gflags.h"
#include "third_party/matplotlib-cpp/matplotlibcpp.h"

DEFINE_double(yrange, 1.0, "+- y max");

double fx(double x, double yrange) {
  return 2.0 * ((1.0 / (1.0 + ::std::exp(-x * 2.0 / yrange)) - 0.5)) *
         yrange;
}

int main(int argc, char **argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, false);

  matplotlibcpp::figure();
  ::std::vector<double> x;
  ::std::vector<double> y;
  ::std::vector<double> slope_y;

  for (double i = -5.0; i < 5.0; i += 0.01) {
    x.push_back(i);
    y.push_back(fx(i, FLAGS_yrange));
    slope_y.push_back(
        (fx(i + 0.0001, FLAGS_yrange) - fx(i - 0.0001, FLAGS_yrange)) /
        (2.0 * 0.0001));
  }

  matplotlibcpp::plot(x, y, {{"label", "saturated x"}});
  matplotlibcpp::plot(x, slope_y, {{"label", "slope"}});
  matplotlibcpp::legend();
  matplotlibcpp::show();

}
