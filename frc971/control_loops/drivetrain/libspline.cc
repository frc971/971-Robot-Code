#include <vector>

#include "Eigen/Dense"

#include "frc971/control_loops/drivetrain/distance_spline.h"
#include "frc971/control_loops/drivetrain/spline.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

extern "C" {
  NSpline<6>* NewSpline(double x[6], double y[6]) {
    return new NSpline<6>((::Eigen::Matrix<double, 2, 6>() << x[0], x[1], x[2],
                           x[3], x[4], x[5], y[0], y[1], y[2], y[3], y[4],
                           y[5]).finished());
  }

  void deleteSpline(NSpline<6>* spline) {
    delete spline;
  }

  void SplinePoint(NSpline<6>* spline, double alpha, double* res) {
    double* val = spline->Point(alpha).data();
    res[0] = val[0];
    res[1] = val[1];
  }

  void SplineDPoint(NSpline<6>* spline, double alpha, double* res) {
    double* val = spline->DPoint(alpha).data();
    res[0] = val[0];
    res[1] = val[1];
  }

  void SplineDDPoint(NSpline<6>* spline, double alpha, double* res) {
    double* val = spline->DDPoint(alpha).data();
    res[0] = val[0];
    res[1] = val[1];
  }

  void SplineDDDPoint(NSpline<6>* spline, double alpha, double* res) {
    double* val = spline->DDDPoint(alpha).data();
    res[0] = val[0];
    res[1] = val[1];
  }

  double SplineTheta(NSpline<6>* spline, double alpha) {
    return spline->Theta(alpha);
  }

  double SplineDTheta(NSpline<6>* spline, double alpha) {
    return spline->DTheta(alpha);
  }

  double SplineDDTheta(NSpline<6>* spline, double alpha) {
    return spline->DDTheta(alpha);
  }

  void SplineControlPoints(NSpline<6>* spline, double* x, double* y) {
    auto points = spline->control_points();
    // Deal with incorrectly strided matrix.
    for (int i = 0; i < 6; ++i) {
      x[i] = points(0, i);
      y[i] = points(1, i);
    }
  }

  DistanceSpline* NewDistanceSpline(Spline** splines, int count) {
    ::std::vector<Spline> splines_;
    for (int i = 0; i < count; ++i) {
      splines_.push_back(*splines[i]);
    }
    return new DistanceSpline(::std::vector<Spline>(splines_));
  }

  void deleteDistanceSpline(DistanceSpline* spline) {
    delete spline;
  }

  void DistanceSplineXY(DistanceSpline *spline, double distance, double *res) {
    double *val = spline->XY(distance).data();
    res[0] = val[0];
    res[1] = val[1];
  }

  void DistanceSplineDXY(DistanceSpline *spline, double distance, double *res) {
    double *val = spline->DXY(distance).data();
    res[0] = val[0];
    res[1] = val[1];
  }

  void DistanceSplineDDXY(DistanceSpline *spline, double distance,
                          double *res) {
    double *val = spline->DDXY(distance).data();
    res[0] = val[0];
    res[1] = val[1];
  }

  double DistanceSplineTheta(DistanceSpline *spline, double distance) {
    return spline->Theta(distance);
  }

  double DistanceSplineDTheta(DistanceSpline *spline, double distance) {
    return spline->DTheta(distance);
  }

  double DistanceSplineDThetaDt(DistanceSpline *spline, double distance,
                                double velocity) {
    return spline->DThetaDt(distance, velocity);
  }

  double DistanceSplineDDTheta(DistanceSpline *spline, double distance) {
    return spline->DDTheta(distance);
  }

  double DistanceSplineLength(DistanceSpline *spline) {
    return spline->length();
  }
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
