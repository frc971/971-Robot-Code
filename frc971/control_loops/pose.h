#ifndef FRC971_CONTROL_LOOPS_POSE_H_
#define FRC971_CONTROL_LOOPS_POSE_H_

#include <vector>

#include "Eigen/Dense"
#include "aos/util/math.h"

namespace frc971 {
namespace control_loops {

// Provides a representation of a transformation on the field.
// Currently, this is heavily geared towards things that occur in a 2-D plane.
// The Z-axis is rarely used (but still relevant; e.g., in 2019 some of the
// targets are at a different height).
// For rotations, we currently just represent the yaw axis (the rotation about
// the Z-axis).
// As a convention, we use right-handed coordinate systems; the positive Z
// axis will go up on the field, the positive X axis shall be "forwards" for
// some relevant meaning of forwards, and the origin shall be chosen as
// appropriate.
// For 2019, at least, the global origin will be on the ground at the center
// of the driver's station wall of your current alliance and the positive X-axis
// will point straight into the field from the driver's station.
// In future years this may need to change if the field's symmetry changes and
// we can't interchangeably handle which side of the field we are on.
// This means that if we had a Pose for the center of mass of the robot with a
// position of (10, -5, 0) and a yaw of pi / 2, that suggests the robot is
// facing straight to the left from the driver's perspective and is placed 10m
// from the driver's station wall and 5m to the right of the center of the wall.
//
// Furthermore, Poses can be chained such that a Pose can be placed relative to
// another Pose; the other Pose can dynamically update, thus allowing us to,
// e.g., provide a Pose for a camera that is relative to the Pose of the robot.
// Poses can also be in the global frame with no parent Pose.
template <typename Scalar = double>
class TypedPose {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  // The type that contains the translational (x, y, z) component of the Pose.
  typedef Eigen::Matrix<Scalar, 3, 1> Pos;

  // Provide a default constructor that crease a pose at the origin.
  TypedPose() : TypedPose({0.0, 0.0, 0.0}, 0.0) {}

  // Construct a Pose in the absolute frame with a particular position and yaw.
  TypedPose(const Pos &abs_pos, Scalar theta) : pos_(abs_pos), theta_(theta) {}
  // Construct a Pose relative to another Pose (base).
  // If you provide a base of nullptr, then this will
  // construct a Pose in the global frame.
  // Note that the lifetime of base should be greater than the lifetime of
  // the object being constructed.
  TypedPose(const TypedPose<Scalar> *base, const Pos &rel_pos, Scalar rel_theta)
      : base_(base), pos_(rel_pos), theta_(rel_theta) {}

  // Calculate the current global position of this Pose.
  Pos abs_pos() const {
    if (base_ == nullptr) {
      return pos_;
    }
    Pos base_pos = base_->abs_pos();
    Scalar base_theta = base_->abs_theta();
    return base_pos + YawRotation(base_theta) * pos_;
  }
  // Calculate the absolute yaw of this Pose. Since we only have a single
  // rotational axis, we can just sum the angle with that of the base Pose.
  Scalar abs_theta() const {
    if (base_ == nullptr) {
      return theta_;
    }
    return aos::math::NormalizeAngle(theta_ + base_->abs_theta());
  }
  // Provide access to the position and yaw relative to the base Pose.
  Pos rel_pos() const { return pos_; }
  Scalar rel_theta() const { return theta_; }
  const TypedPose<Scalar> *base() const { return base_; }

  Pos *mutable_pos() { return &pos_; }
  void set_theta(Scalar theta) { theta_ = theta; }
  // Swap out the base Pose, keeping the current relative position/angle.
  void set_base(const TypedPose<Scalar> *new_base) { base_ = new_base; }

  // For 2-D calculation, provide the heading, which is distinct from the
  // yaw/theta value. heading is the heading relative to the base Pose if you
  // were to draw a line from the base to this Pose. i.e., if heading() is zero
  // then you are directly in front of the base Pose.
  Scalar heading() const { return ::std::atan2(pos_.y(), pos_.x()); }
  // The 2-D distance from the base Pose to this Pose.
  Scalar xy_norm() const { return pos_.template topRows<2>().norm(); }
  // Return the absolute xy position.
  Eigen::Matrix<Scalar, 2, 1> abs_xy() const {
    return abs_pos().template topRows<2>();
  }

  // Provide a copy of this that is set to have the same
  // current absolute Pose as this, but have a different base.
  // This can be used, e.g., to compute a Pose for a vision target that is
  // relative to the camera instead of relative to the field. You can then
  // access the rel_* variables to get what the position of the target is
  // relative to the robot/camera.
  // If new_base == nullptr, provides a Pose referenced to the global frame.
  // Note that the lifetime of new_base should be greater than the lifetime of
  // the returned object (unless new_base == nullptr).
  TypedPose Rebase(const TypedPose<Scalar> *new_base) const;

 private:
  // A rotation-matrix like representation of the rotation for a given angle.
  inline static Eigen::AngleAxis<Scalar> YawRotation(double theta) {
    return Eigen::AngleAxis<Scalar>(theta, Pos::UnitZ());
  }

  // A pointer to the base Pose. If uninitialized, then this Pose is in the
  // global frame.
  const TypedPose<Scalar> *base_ = nullptr;
  // Position and yaw relative to base_.
  Pos pos_;
  Scalar theta_;
}; // class TypedPose

typedef TypedPose<double> Pose;

template <typename Scalar>
TypedPose<Scalar> TypedPose<Scalar>::Rebase(
    const TypedPose<Scalar> *new_base) const {
  if (new_base == nullptr) {
    return TypedPose<Scalar>(nullptr, abs_pos(), abs_theta());
  }
  // Calculate the absolute position/yaws of this and of the new_base, and then
  // calculate where we are relative to new_base, essentially reversing the
  // calculation in abs_*.
  Pos base_pos = new_base->abs_pos();
  Scalar base_theta = new_base->abs_theta();
  Pos self_pos = abs_pos();
  Scalar self_theta = abs_theta();
  Scalar diff_theta = ::aos::math::DiffAngle(self_theta, base_theta);
  Pos diff_pos = YawRotation(-base_theta) * (self_pos - base_pos);
  return TypedPose<Scalar>(new_base, diff_pos, diff_theta);
}

// Represents a 2D line segment constructed from a pair of Poses.
// The line segment goes between the two Poses, but for calculating
// intersections we use the 2D projection of the Poses onto the global X-Y
// plane.
template <typename Scalar = double>
class TypedLineSegment {
 public:
  TypedLineSegment() {}
  TypedLineSegment(const TypedPose<Scalar> &pose1,
                   const TypedPose<Scalar> &pose2)
      : pose1_(pose1), pose2_(pose2) {}
  // Detects if two line segments intersect.
  // When at least one end of one line segment is collinear with the other,
  // the line segments are treated as not intersecting.
  bool Intersects(const TypedLineSegment<Scalar> &other) const {
    // Source for algorithm:
    // https://bryceboe.com/2006/10/23/line-segment-intersection-algorithm/
    // Method:
    // We will consider the four triangles that can be made out of any 3 points
    // from the pair of line segments.
    // Basically, if you consider one line segment the base of the triangle,
    // then the two points of the other line segment should be on opposite
    // sides of the first line segment (we use the PointsAreCCW function for
    // this). This must hold when splitting off of both line segments.
    Eigen::Matrix<Scalar, 2, 1> p1 = pose1_.abs_xy();
    Eigen::Matrix<Scalar, 2, 1> p2 = pose2_.abs_xy();
    Eigen::Matrix<Scalar, 2, 1> q1 = other.pose1_.abs_xy();
    Eigen::Matrix<Scalar, 2, 1> q2 = other.pose2_.abs_xy();
    return (::aos::math::PointsAreCCW<Scalar>(p1, q1, q2) !=
            ::aos::math::PointsAreCCW<Scalar>(p2, q1, q2)) &&
           (::aos::math::PointsAreCCW<Scalar>(p1, p2, q1) !=
            ::aos::math::PointsAreCCW<Scalar>(p1, p2, q2));
  }

  TypedPose<Scalar> pose1() const { return pose1_; }
  TypedPose<Scalar> pose2() const { return pose2_; }
  TypedPose<Scalar> *mutable_pose1() { return &pose1_; }
  TypedPose<Scalar> *mutable_pose2() { return &pose2_; }

  ::std::vector<TypedPose<Scalar>> PlotPoints() const {
    return {pose1_, pose2_};
  }
 private:
  TypedPose<Scalar> pose1_;
  TypedPose<Scalar> pose2_;
};  // class TypedLineSegment

typedef TypedLineSegment<double> LineSegment;

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_POSE_H_
