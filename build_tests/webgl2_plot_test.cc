#include <emscripten/emscripten.h>
#include <emscripten/html5.h>

#include <iostream>

#include "frc971/analysis/plotting/webgl2_plotter.h"
#include "frc971/analysis/plotting/webgl2_animator.h"

float rand1() {
  return static_cast<float>(rand()) / RAND_MAX;
}

int main() {
  // Note that the animation_state must last until Redraw stops being called,
  // which we cannot provide any bound on. As such, we don't currently destroy
  // the memory until the webpage is closed.
  frc971::plotting::Animator *animation_state =
      new frc971::plotting::Animator("#canvas");
  // Generate a bunch of lines with random y-values and evenly spaced x-values,
  // such that each line takes up a set amount of space in the y-space. If
  // that's unclear, then try running this and seeing what it looks like.
  constexpr size_t kNLines = 30;
  for (int jj = 0; jj < kNLines; ++jj) {
    frc971::plotting::Line *line = animation_state->plotter()->AddLine();
    // Randomly generate a color to use; each of r/g/b are between 0 and 1.
    line->SetColor({.r = rand1(), .g = rand1(), .b = rand1()});
    std::vector<Eigen::Vector2d> points;
    constexpr size_t kNPoints = 100000;
    for (int ii = 0; ii < kNPoints; ++ii) {
      const float x = static_cast<float>(ii) / kNPoints;
      points.emplace_back(x, std::sin(x + jj));
    }
    line->SetPoints(points);
  }
}
