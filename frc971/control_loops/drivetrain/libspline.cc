#include <string.h>

#include "Eigen/Dense"

#include "frc971/control_loops/drivetrain/spline.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

extern "C" {
  NSpline<6>* newSpline(double x[6], double y[6]) {
    return new NSpline<6>((::Eigen::Matrix<double, 2, 6>() << x[0], x[1], x[2],
                           x[3], x[4], x[5], y[0], y[1], y[2], y[3], y[4],
                           y[5]).finished());
  }

  void deleteSpline(NSpline<6>* spline) {
    delete spline;
  }

  void Point(NSpline<6>* spline, double alpha, double* res) {
    double* val = spline->Point(alpha).data();
    res[0] = val[0];
    res[1] = val[1];
  }

  void DPoint(NSpline<6>* spline, double alpha, double* res) {
    double* val = spline->DPoint(alpha).data();
    res[0] = val[0];
    res[1] = val[1];
  }

  void DDPoint(NSpline<6>* spline, double alpha, double* res) {
    double* val = spline->DDPoint(alpha).data();
    res[0] = val[0];
    res[1] = val[1];
  }

  void DDDPoint(NSpline<6>* spline, double alpha, double* res) {
    double* val = spline->DDDPoint(alpha).data();
    res[0] = val[0];
    res[1] = val[1];
  }

  double Theta(NSpline<6>* spline, double alpha) {
    return spline->Theta(alpha);
  }

  double DTheta(NSpline<6>* spline, double alpha) {
    return spline->DTheta(alpha);
  }

  double DDTheta(NSpline<6>* spline, double alpha) {
    return spline->DDTheta(alpha);
  }

  void control_points(NSpline<6>* spline, double* x, double* y) {
    auto points = spline->control_points();
    // Deal with incorrectly strided matrix.
    for (int i = 0; i < 6; ++i) {
      x[i] = points(0, i);
      y[i] = points(1, i);
    }
  }
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
