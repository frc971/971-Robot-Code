#include "aos/init.h"
#include "frc971/analysis/in_process_plotter.h"

// To run this example, do:
// bazel run -c opt //frc971/analysis:in_process_plotter_demo
// And then open localhost:8080, select "C++ Plotter" from the drop-down, and
// then select "TITLE!" or "Trig Functions" from the second drop-down to see
// each plot.
int main(int argc, char *argv[]) {
  aos::InitGoogle(&argc, &argv);
  frc971::analysis::Plotter plotter;
  plotter.Title("TITLE!");
  plotter.AddFigure("Fig Foo");
  plotter.ShareXAxis(true);
  plotter.AddLine({1, 2, 3, 4, 5}, {1, 2, 3, 4, 5}, "y = x");
  plotter.AddLine({5, 4, 3, 2, 1}, {1, 2, 3, 4, 5}, "y = -x");
  plotter.YLabel("Y Axis");
  plotter.AddFigure("Fig Bar");
  plotter.ShareXAxis(true);
  plotter.AddLine({1, 2, 3}, {3, 4, 5}, "y = x + 2");
  plotter.XLabel("X Axis (Linked to both above plots)");
  plotter.Publish();

  plotter.Title("Trig Functions");

  plotter.AddFigure("Sin & Cos");
  std::vector<double> x;
  std::vector<double> sinx;
  std::vector<double> cosx;
  constexpr int kNumPoints = 100000;
  for (int ii = 0; ii < kNumPoints; ++ii) {
    x.push_back(ii * 2 * M_PI / kNumPoints);
    sinx.push_back(std::sin(x.back()));
    cosx.push_back(std::cos(x.back()));
  }
  plotter.AddLine(x, sinx, "sin(x)");
  plotter.AddLine(x, cosx, "cos(x)");
  plotter.Publish();

  plotter.Spin();
}
