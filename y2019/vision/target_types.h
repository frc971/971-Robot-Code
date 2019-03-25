#ifndef _Y2019_VISION_TARGET_TYPES_H_
#define _Y2019_VISION_TARGET_TYPES_H_

#include "aos/vision/math/segment.h"
#include "aos/vision/math/vector.h"
#include "y2019/vision/constants.h"

namespace y2019 {
namespace vision {

// This polynomial exists in transpose space.
struct TargetComponent {
  aos::vision::Vector<2> GetByIndex(size_t i) const {
    switch (i) {
      case 0:
        return top;
      case 1:
        return inside;
      case 2:
        return bottom;
      case 3:
        return outside;
      default:
        return aos::vision::Vector<2>();
    }
  }
  bool is_right;
  // The point which is the upper outside point on this side of the target pair.
  ::aos::vision::Vector<2> top;
  // The point which is the upper inside point on this side of the target pair.
  ::aos::vision::Vector<2> inside;
  // The point which is the outer bottom point on this side of the target pair.
  ::aos::vision::Vector<2> outside;
  // The point which is the inner bottom point on this side of the target pair.
  ::aos::vision::Vector<2> bottom;

  aos::vision::Segment<2> major_axis;

  // The point with is the "lowest" along the outer edge.  This point is useful
  // for making sure clipped targets are "big enough" to cover all the pixels.
  ::Eigen::Vector2f bottom_point;
};

// Convert back to screen space for final result.
struct Target {
  TargetComponent left;
  TargetComponent right;

  // Returns a target.  The resulting target is in meters with 0, 0 centered
  // between the upper inner corners of the two pieces of tape, y being up and x
  // being to the right.
  static Target MakeTemplate();
  // Get the points in some order (will match against the template).
  std::array<aos::vision::Vector<2>, 8> ToPointList() const;
};

struct ExtrinsicParams {
  static constexpr size_t kNumParams = 4;

  // Height of the target
  double y = 18.0 * 0.0254;
  // Distance to the target
  double z = 23.0 * 0.0254;
  // Skew of the target relative to the line-of-sight from the camera to the
  // target.
  double r1 = 1.0 / 180 * M_PI;
  // Heading from the camera to the target, relative to the center of the view
  // from the camera.
  double r2 = -1.0 / 180 * M_PI;

  void set(double *data) {
    data[0] = y;
    data[1] = z;
    data[2] = r1;
    data[3] = r2;
  }
  static ExtrinsicParams get(const double *data) {
    ExtrinsicParams out;
    out.y = data[0];
    out.z = data[1];
    out.r1 = data[2];
    out.r2 = data[3];
    return out;
  }
};

// Projects a point from idealized template space to camera space.
aos::vision::Vector<2> Project(aos::vision::Vector<2> pt,
                               const IntrinsicParams &intrinsics,
                               const ExtrinsicParams &extrinsics);

Target Project(const Target &target, const IntrinsicParams &intrinsics,
               const ExtrinsicParams &extrinsics);

// An intermediate for of the results.
// These are actual awnsers, but in a form from the solver that is not ready for
// the wire.
struct IntermediateResult {
  ExtrinsicParams extrinsics;

  // Error from solver calulations.
  double solver_error;

  // extrinsics and error from a more relaxed problem.
  ExtrinsicParams backup_extrinsics;

  double backup_solver_error;

  bool good_corners;
};

// Final foramtting ready for output on the wire.
struct TargetResult {
  // Distance to the target in meters. Specifically, the distance from the
  // center of the camera's image plane to the center of the target.
  float distance;
  // Height of the target in meters. Specifically, the distance from the floor
  // to the center of the target.
  float height;

  // Heading of the center of the target in radians. Zero is straight out
  // perpendicular to the camera's image plane. Images to the left (looking at a
  // camera image) are at a positive angle.
  float heading;

  // The angle between the target and the camera's image plane. This is
  // projected so both are assumed to be perpendicular to the floor. Parallel
  // targets have a skew of zero. Targets rotated such that their left edge
  // (looking at a camera image) is closer are at a positive angle.
  float skew;
};

}  // namespace vision
}  // namespace y2019

#endif
