#include <vector>
#include <string>

#include "Eigen/Dense"

#include "aos/logging/implementations.h"
#include "aos/network/team_number.h"
#include "frc971/control_loops/drivetrain/distance_spline.h"
#include "frc971/control_loops/drivetrain/spline.h"
#include "frc971/control_loops/drivetrain/trajectory.h"
#include "y2019/control_loops/drivetrain/drivetrain_base.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

extern "C" {
  // Based on spline.h
  NSpline<6> *NewSpline(double x[6], double y[6]) {
    return new NSpline<6>((::Eigen::Matrix<double, 2, 6>() << x[0], x[1], x[2],
                           x[3], x[4], x[5], y[0], y[1], y[2], y[3], y[4],
                           y[5]).finished());
  }

  void deleteSpline(NSpline<6> *spline) { delete spline; }

  void SplinePoint(NSpline<6> *spline, double alpha, double *res) {
    const Eigen::Vector2d xy = spline->Point(alpha);
    res[0] = xy.x();
    res[1] = xy.y();
  }

  void SplineDPoint(NSpline<6> *spline, double alpha, double *res) {
    const Eigen::Vector2d dxy = spline->DPoint(alpha);
    res[0] = dxy.x();
    res[1] = dxy.y();
  }

  void SplineDDPoint(NSpline<6> *spline, double alpha, double *res) {
    const Eigen::Vector2d ddxy = spline->DDPoint(alpha);
    res[0] = ddxy.x();
    res[1] = ddxy.y();
  }

  void SplineDDDPoint(NSpline<6> *spline, double alpha, double *res) {
    const Eigen::Vector2d dddxy = spline->DDDPoint(alpha);
    res[0] = dddxy.x();
    res[1] = dddxy.y();
  }

  double SplineTheta(NSpline<6> *spline, double alpha) {
    return spline->Theta(alpha);
  }

  double SplineDTheta(NSpline<6> *spline, double alpha) {
    return spline->DTheta(alpha);
  }

  double SplineDDTheta(NSpline<6> *spline, double alpha) {
    return spline->DDTheta(alpha);
  }

  void SplineControlPoints(NSpline<6> *spline, double *x, double *y) {
    auto points = spline->control_points();
    // Deal with incorrectly strided matrix.
    for (int i = 0; i < 6; ++i) {
      x[i] = points(0, i);
      y[i] = points(1, i);
    }
  }

  // Based on distance_spline.h
  DistanceSpline *NewDistanceSpline(Spline **splines, int count) {
    ::std::vector<Spline> splines_;
    for (int i = 0; i < count; ++i) {
      splines_.push_back(*splines[i]);
    }
    return new DistanceSpline(::std::vector<Spline>(splines_));
  }

  void deleteDistanceSpline(DistanceSpline *spline) { delete spline; }

  void DistanceSplineXY(DistanceSpline *spline, double distance, double *res) {
    const Eigen::Vector2d xy = spline->XY(distance);
    res[0] = xy.x();
    res[1] = xy.y();
  }

  void DistanceSplineDXY(DistanceSpline *spline, double distance, double *res) {
    const Eigen::Vector2d dxy = spline->DXY(distance);
    res[0] = dxy.x();
    res[1] = dxy.y();
  }

  void DistanceSplineDDXY(DistanceSpline *spline, double distance,
                          double *res) {
    const Eigen::Vector2d ddxy = spline->DDXY(distance);
    res[0] = ddxy.x();
    res[1] = ddxy.y();
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

  // Based on trajectory.h
  Trajectory *NewTrajectory(DistanceSpline *spline, double vmax,
                            int num_distance) {
    return new Trajectory(
        spline, ::y2019::control_loops::drivetrain::GetDrivetrainConfig(), vmax,
        num_distance);
  }

  void deleteTrajectory(Trajectory *t) { delete t; }

  void TrajectorySetLongitudinalAcceleration(Trajectory *t, double accel) {
    t->set_longitudinal_acceleration(accel);
  }

  void TrajectorySetLateralAcceleration(Trajectory *t, double accel) {
    t->set_lateral_acceleration(accel);
  }

  void TrajectorySetVoltageLimit(Trajectory *t, double limit) {
    t->set_voltage_limit(limit);
  }

  void TrajectoryLimitVelocity(Trajectory *t, double start, double end,
                               double max) {
    t->LimitVelocity(start, end, max);
  }

  void TrajectoryPlan(Trajectory *t) { t->Plan(); }

  void TrajectoryVoltage(Trajectory *t, double distance, double* res) {
    const Eigen::Vector2d ff_voltage = t->FFVoltage(distance);
    res[0] = ff_voltage.x();
    res[1] = ff_voltage.y();
  }

  double TrajectoryLength(Trajectory *t) { return t->length(); }

  int TrajectoryGetPathLength(Trajectory *t) { return t->plan().size(); }

  // This assumes that res is created in python to be getPathLength() long.
  // Likely to SEGFAULT otherwise.
  void Distances(Trajectory *t, double *res) {
    const ::std::vector<double> &distances = t->Distances();
    ::std::memcpy(res, distances.data(), sizeof(double) * distances.size());
  }

  double TrajectoryDistance(Trajectory *t, int index) {
    return t->Distance(index);
  }

  // This assumes that res is created in python to be getPathLength() long.
  // Likely to SEGFAULT otherwise.
  void TrajectoryGetPlan(Trajectory *t, double *res) {
    const ::std::vector<double> &velocities = t->plan();
    ::std::memcpy(res, velocities.data(), sizeof(double) * velocities.size());
  }

  // Time in in nanoseconds.
  ::std::vector<::Eigen::Matrix<double, 3, 1>> *TrajectoryGetPlanXVAPtr(
      Trajectory *t, int dt) {
    return new ::std::vector<::Eigen::Matrix<double, 3, 1>>(
        t->PlanXVA(::std::chrono::nanoseconds(dt)));
  }

  void TrajectoryDeleteVector(
      ::std::vector<::Eigen::Matrix<double, 3, 1>> *vec) {
    delete vec;
  }

  int TrajectoryGetVectorLength(
      ::std::vector<::Eigen::Matrix<double, 3, 1>> *vec) {
    return vec->size();
  }

  void TrajectoryGetPlanXVA(::std::vector<::Eigen::Matrix<double, 3, 1>> *vec,
                            double *X, double *V, double *A) {
    for (size_t i = 0; i < vec->size(); ++i) {
      X[i] = (*vec)[i][0];
      V[i] = (*vec)[i][1];
      A[i] = (*vec)[i][2];
    }
  }

  // Util
  void SetUpLogging() {
    ::aos::logging::Init();
    ::aos::network::OverrideTeamNumber(971);
  }
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
