#ifndef _Y2019_VISION_TARGET_TYPES_H_
#define _Y2019_VISION_TARGET_TYPES_H_

#include "Eigen/Dense"

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

  double width() const { return left.inside.DistanceTo(right.inside); }

  // Returns a target.  The resulting target is in meters with 0, 0 centered
  // between the upper inner corners of the two pieces of tape, y being up and x
  // being to the right.
  static Target MakeTemplate();
  // Get the points in some order (will match against the template).
  std::array<aos::vision::Vector<2>, 8> ToPointList() const;
};

template <typename Scalar>
struct TemplatedExtrinsicParams {
  static constexpr size_t kNumParams = 4;

  // Height of the target
  Scalar y = Scalar(18.0 * 0.0254);
  // Distance to the target
  Scalar z = Scalar(23.0 * 0.0254);
  // Skew of the target relative to the line-of-sight from the camera to the
  // target.
  Scalar r1 = Scalar(1.0 / 180 * M_PI);
  // Heading from the camera to the target, relative to the center of the view
  // from the camera.
  Scalar r2 = Scalar(-1.0 / 180 * M_PI);

  void set(Scalar *data) {
    data[0] = y;
    data[1] = z;
    data[2] = r1;
    data[3] = r2;
  }
  static TemplatedExtrinsicParams get(const Scalar *data) {
    TemplatedExtrinsicParams out;
    out.y = data[0];
    out.z = data[1];
    out.r1 = data[2];
    out.r2 = data[3];
    return out;
  }
};

using ExtrinsicParams = TemplatedExtrinsicParams<double>;

// Projects a point from idealized template space to camera space.
template <typename Extrinsics>
aos::vision::Vector<2> Project(aos::vision::Vector<2> pt,
                                  const IntrinsicParams &intrinsics,
                                  const Extrinsics &extrinsics);

template <typename T, typename Extrinsics>
::Eigen::Matrix<T, 2, 1> Project(::Eigen::Matrix<T, 2, 1> pt,
                                  const IntrinsicParams &intrinsics,
                                  const Extrinsics &extrinsics);

Target Project(const Target &target, const IntrinsicParams &intrinsics,
               const ExtrinsicParams &extrinsics);

// An intermediate for of the results.
// These are actual awnsers, but in a form from the solver that is not ready for
// the wire.
struct IntermediateResult {
  ExtrinsicParams extrinsics;

  // Width of the target in pixels. Distance from inner most points.
  double target_width;

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

template<typename T>
::Eigen::Matrix<T, 2, 1> ToEigenMatrix(aos::vision::Vector<2> pt) {
  return (::Eigen::Matrix<T, 2, 1>() << T(pt.x()), T(pt.y())).finished();
}

template <typename Extrinsics>
aos::vision::Vector<2> Project(aos::vision::Vector<2> pt,
                               const IntrinsicParams &intrinsics,
                               const Extrinsics &extrinsics) {
  const ::Eigen::Matrix<double, 2, 1> eigen_pt = ToEigenMatrix<double>(pt);
  const ::Eigen::Matrix<double, 2, 1> res =
      Project(eigen_pt, intrinsics, extrinsics);
  return aos::vision::Vector<2>(res(0, 0), res(1, 0));
}

template <typename T, typename Extrinsics>
::Eigen::Matrix<T, 2, 1> Project(::Eigen::Matrix<T, 2, 1> pt,
                              const IntrinsicParams &intrinsics,
                              const Extrinsics &extrinsics) {
  const T y = extrinsics.y;    // height
  const T z = extrinsics.z;    // distance
  const T r1 = extrinsics.r1;  // skew
  const T r2 = extrinsics.r2;  // heading
  const double rup = intrinsics.mount_angle;
  const double rbarrel = intrinsics.barrel_mount;
  const double fl = intrinsics.focal_length;

  // Start by translating point in target-space to be at correct height.
  ::Eigen::Matrix<T, 3, 1> pts{pt(0, 0), pt(1, 0) + y, T(0.0)};

  {
    // Rotate to compensate for skew angle, to get into a frame still at the
    // same (x, y) position as the target but rotated to be facing straight
    // towards the camera.
    const T theta = r1;
    const T s = sin(theta);
    const T c = cos(theta);
    pts = (::Eigen::Matrix<T, 3, 3>() << c, T(0), -s, T(0), T(1), T(0), s, T(0),
           c).finished() *
          pts;
  }

  // Translate the coordinate frame to have (x, y) centered at the camera, but
  // still oriented to be facing along the line from the camera to the target.
  pts(2) += z;

  {
    // Rotate out the heading so that the frame is oriented to line up with the
    // camera's viewpoint in the yaw-axis.
    const T theta = r2;
    const T s = sin(theta);
    const T c = cos(theta);
    pts = (::Eigen::Matrix<T, 3, 3>() << c, T(0), -s, T(0), T(1), T(0), s, T(0),
           c).finished() *
          pts;
  }

  // TODO: Apply 15 degree downward rotation.
  {
    // Compensate for rotation in the pitch of the camera up/down to get into
    // the coordinate frame lined up with the plane of the camera sensor.
    const double theta = rup;
    const T s = T(sin(theta));
    const T c = T(cos(theta));

    pts = (::Eigen::Matrix<T, 3, 3>() << T(1), T(0), T(0), T(0), c, -s, T(0), s,
           c).finished() *
          pts;
  }

  // Compensate for rotation of the barrel of the camera, i.e. about the axis
  // that points straight out from the camera lense, using an AngleAxis instead
  // of manually constructing the rotation matrices because once you get into
  // this frame you no longer need to be masochistic.
  // TODO: Maybe barrel should be extrinsics to allow rocking?
  // Also, in this case, barrel should go above the rotation above?
  pts = ::Eigen::AngleAxis<T>(T(rbarrel),
                              ::Eigen::Matrix<T, 3, 1>(T(0), T(0), T(1))) *
        pts;

  // TODO: Final image projection.
  const ::Eigen::Matrix<T, 3, 1> res = pts;

  // Finally, scale to account for focal length and translate to get into
  // pixel-space.
  const T scale = fl / res.z();
  return ::Eigen::Matrix<T, 2, 1>(res.x() * scale + 320.0,
                                   240.0 - res.y() * scale);
}

}  // namespace vision
}  // namespace y2019

#endif
