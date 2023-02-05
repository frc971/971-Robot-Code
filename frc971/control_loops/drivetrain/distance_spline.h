#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_DISTANCE_SPLINE_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_DISTANCE_SPLINE_H_

#include <vector>

#include "Eigen/Dense"
#include "absl/types/span.h"
#include "aos/containers/sized_array.h"
#include "frc971/control_loops/drivetrain/spline.h"
#include "frc971/control_loops/drivetrain/trajectory_generated.h"
#include "frc971/control_loops/fixed_quadrature.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

std::vector<Spline> FlatbufferToSplines(const MultiSpline *fb);

// Class to hold a spline as a function of distance.
class DistanceSplineBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  virtual ~DistanceSplineBase() {}

  flatbuffers::Offset<fb::DistanceSpline> Serialize(
      flatbuffers::FlatBufferBuilder *fbb,
      flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Constraint>>>
          constraints) const;

  // Returns a point on the spline as a function of distance.
  ::Eigen::Matrix<double, 2, 1> XY(double distance) const {
    const AlphaAndIndex a = DistanceToAlpha(distance);
    return splines()[a.index].Point(a.alpha);
  }

  // Returns the velocity as a function of distance.
  ::Eigen::Matrix<double, 2, 1> DXY(double distance) const {
    const AlphaAndIndex a = DistanceToAlpha(distance);
    return splines()[a.index].DPoint(a.alpha).normalized();
  }

  // Returns the acceleration as a function of distance.
  ::Eigen::Matrix<double, 2, 1> DDXY(double distance) const;

  // Returns the heading as a function of distance.
  double Theta(double distance) const {
    const AlphaAndIndex a = DistanceToAlpha(distance);
    return splines()[a.index].Theta(a.alpha);
  }

  // Returns the angular velocity as a function of distance.
  // This is also equivalent to the curvature at the given point.
  double DTheta(double distance) const {
    // TODO(austin): We are re-computing DPoint here!
    const AlphaAndIndex a = DistanceToAlpha(distance);
    const Spline &spline = splines()[a.index];
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
  double length() const { return distances().back(); }

  virtual const absl::Span<const float> distances() const = 0;
  virtual const absl::Span<const Spline> splines() const = 0;

 private:
  struct AlphaAndIndex {
    size_t index;
    double alpha;
  };

  // Computes alpha for a distance
  AlphaAndIndex DistanceToAlpha(double distance) const;
};

// Class which computes a DistanceSpline and stores it.
class DistanceSpline final : public DistanceSplineBase {
 public:
  DistanceSpline(const Spline &spline, int num_alpha = 0);
  DistanceSpline(::std::vector<Spline> &&splines, int num_alpha = 0);
  DistanceSpline(const MultiSpline *fb, int num_alpha = 0);

  const absl::Span<const float> distances() const override {
    return distances_;
  }
  const absl::Span<const Spline> splines() const override { return splines_; }

 private:
  ::std::vector<float> BuildDistances(size_t num_alpha);

  // The spline we are converting to a distance.
  const ::std::vector<Spline> splines_;
  // An interpolation table of distances evenly distributed in alpha.
  const ::std::vector<float> distances_;
};

// Class exposing a finished DistanceSpline flatbuffer as a DistanceSpline.
//
// The lifetime of the provided fb::DistanceSpline needs to out-live this class.
class FinishedDistanceSpline final : public DistanceSplineBase {
 public:
  static constexpr size_t kMaxSplines = 6;
  FinishedDistanceSpline(const fb::DistanceSpline &fb);

  const absl::Span<const float> distances() const override {
    return distances_;
  }
  const absl::Span<const Spline> splines() const override { return splines_; }

 private:
  ::std::vector<float> BuildDistances(size_t num_alpha);

  // The spline we are converting to a distance.
  aos::SizedArray<Spline, kMaxSplines> splines_;
  // An interpolation table of distances evenly distributed in alpha.
  absl::Span<const float> distances_;
};

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_DISTANCE_SPLINE_H_
