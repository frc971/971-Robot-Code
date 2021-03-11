#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_DISTANCE_SPLINE_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_DISTANCE_SPLINE_H_

#include <vector>

#include "Eigen/Dense"
#include "frc971/control_loops/drivetrain/spline.h"
#include "frc971/control_loops/drivetrain/trajectory_generated.h"
#include "frc971/control_loops/fixed_quadrature.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

std::vector<Spline> FlatbufferToSplines(const MultiSpline *fb);

// Class to hold a spline as a function of distance.
class DistanceSpline {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DistanceSpline(const Spline &spline, int num_alpha = 0);
  DistanceSpline(::std::vector<Spline> &&splines, int num_alpha = 0);
  DistanceSpline(const MultiSpline *fb, int num_alpha = 0);
  // Copies the distances for the spline directly out of the provided buffer,
  // rather than constructing the distances from the original splines.
  DistanceSpline(const fb::DistanceSpline &fb);

  flatbuffers::Offset<fb::DistanceSpline> Serialize(
      flatbuffers::FlatBufferBuilder *fbb,
      flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Constraint>>>
          constraints) const;

  // Returns a point on the spline as a function of distance.
  ::Eigen::Matrix<double, 2, 1> XY(double distance) const {
    const AlphaAndIndex a = DistanceToAlpha(distance);
    return splines_[a.index].Point(a.alpha);
  }

  // Returns the velocity as a function of distance.
  ::Eigen::Matrix<double, 2, 1> DXY(double distance) const {
    const AlphaAndIndex a = DistanceToAlpha(distance);
    return splines_[a.index].DPoint(a.alpha).normalized();
  }

  // Returns the acceleration as a function of distance.
  ::Eigen::Matrix<double, 2, 1> DDXY(double distance) const;

  // Returns the heading as a function of distance.
  double Theta(double distance) const {
    const AlphaAndIndex a = DistanceToAlpha(distance);
    return splines_[a.index].Theta(a.alpha);
  }

  // Returns the angular velocity as a function of distance.
  // This is also equivalent to the curvature at the given point.
  double DTheta(double distance) const {
    // TODO(austin): We are re-computing DPoint here!
    const AlphaAndIndex a = DistanceToAlpha(distance);
    const Spline &spline = splines_[a.index];
    return spline.DTheta(a.alpha) / spline.DPoint(a.alpha).norm();
  }

  // Returns the derivative of heading with respect to time at a given
  // distance along the spline, if we are travelling at the provided velocity.
  double DThetaDt(double distance, double velocity) const {
    return DTheta(distance) * velocity;
  }

  // Returns the angular acceleration as a function of distance.
  double DDTheta(double distance) const;

  // Returns the length of the path in meters.
  double length() const { return distances_.back(); }

  const std::vector<float> &distances() const { return distances_; }
  const std::vector<Spline> &splines() const { return splines_; }

 private:
  struct AlphaAndIndex {
    size_t index;
    double alpha;
  };

  // Computes alpha for a distance
  AlphaAndIndex DistanceToAlpha(double distance) const;

  ::std::vector<float> BuildDistances(size_t num_alpha);

  // The spline we are converting to a distance.
  const ::std::vector<Spline> splines_;
  // An interpolation table of distances evenly distributed in alpha.
  const ::std::vector<float> distances_;
};

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_DISTANCE_SPLINE_H_
