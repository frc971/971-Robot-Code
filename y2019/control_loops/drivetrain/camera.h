#ifndef Y2019_CONTROL_LOOPS_DRIVETRAIN_CAMERA_H_
#define Y2019_CONTROL_LOOPS_DRIVETRAIN_CAMERA_H_

#include <vector>

#include "aos/containers/sized_array.h"
#include "frc971/control_loops/pose.h"

namespace y2019 {
namespace control_loops {

// Represents a target on the field. Currently just consists of a pose and a
// indicator for whether it is occluded (occlusion is only used by the simulator
// for testing).
// Orientation convention:
// -The absolute position of the pose is the center of the vision target on the
//  field.
// -The yaw of the pose shall be such that the positive X-axis in the Target's
//  frame wil be pointed straight through the target--i.e., if you are looking
//  at the target head-on, then you will be facing in the same direction as the
//  positive X-axis.
// E.g., if the Target has a global position of (1, 1, 0) and yaw of pi / 2,
// then it is at position (1, 1, 0) on the field and is oriented so that if
// someone were to stand at (1, 0, 0) and turn themselves to have a yaw of
// pi / 2, they would see the target 1 meter straight ahead of them.
//
// Generally, the position of a target should not change dynamically; if we do
// start having targets that move, we may want to start optimizing certain
// things (e.g., caching the position of the Target--currently, if the Pose of a
// target is in an absolute frame, then calling abs_pos will be inexpensive; if
// that changes, then we start having to recalculate transformations on every
// frame).
template <typename Scalar = double>
class TypedTarget {
 public:
  typedef ::frc971::control_loops::TypedPose<Scalar> Pose;
  // The nature of the target as a goal--to mark what modes it is a valid
  // potential goal pose and to mark targets on the opposite side of the field
  // as not being viable targets.
  enum class GoalType {
    // None marks targets that are on the opposite side of the field and not
    // viable goal poses.
    kNone,
    // Spots where we can touch hatch panels.
    kHatches,
    // Spots where we can mess with balls.
    kBalls,
    // Spots for both (cargo ship, human loading).
    kBoth,
  };
  // Which target this is within a given field quadrant:
  enum class TargetType {
    kHPSlot,
    kFaceCargoBay,
    kNearSideCargoBay,
    kMidSideCargoBay,
    kFarSideCargoBay,
    kNearRocket,
    kFarRocket,
    kRocketPortal,
  };
  TypedTarget(const Pose &pose, double radius = 0,
              TargetType target_type = TargetType::kHPSlot,
              GoalType goal_type = GoalType::kBoth)
      : pose_(pose),
        radius_(radius),
        target_type_(target_type),
        goal_type_(goal_type) {}
  TypedTarget() {}
  Pose pose() const { return pose_; }
  Pose *mutable_pose() { return &pose_; }

  bool occluded() const { return occluded_; }
  void set_occluded(bool occluded) { occluded_ = occluded; }
  double radius() const { return radius_; }
  GoalType goal_type() const { return goal_type_; }
  void set_goal_type(GoalType goal_type) { goal_type_ = goal_type; }
  TargetType target_type() const { return target_type_; }
  void set_target_type(TargetType target_type) { target_type_ = target_type; }

  // Get a list of points for plotting. These points should be plotted on
  // an x/y plane in the global frame with lines connecting the points.
  // Essentially, this provides a Polygon that is a reasonable representation
  // of a Target.
  // This should not be called from real-time code, as it will probably
  // dynamically allocate memory.
  ::std::vector<Pose> PlotPoints() const {
    // For the actual representation, we will use a triangle such that the
    // base of the triangle corresponds to the surface the target is on.
    // The third point is shown behind the target, so that the user can
    // visually identify which side of the target is the front.
    Pose base1(&pose_, {0, 0.125, 0}, 0);
    Pose base2(&pose_, {0, -0.125, 0}, 0);
    Pose behind(&pose_, {0.05, 0, 0}, 0);
    // Include behind at the start and end to indicate that we want to draw
    // a closed polygon.
    return {behind, base1, base2, behind};
  }

 private:
  Pose pose_;
  bool occluded_ = false;
  // The effective radius of the target--for placing discs, this should be the
  // radius of the disc; for fetching discs and working with balls this should
  // be near zero.
  // TODO(james): We may actually want a non-zero (possibly negative?) number
  // here for balls.
  double radius_ = 0.0;
  TargetType target_type_ = TargetType::kHPSlot;
  GoalType goal_type_ = GoalType::kBoth;
};  // class TypedTarget

typedef TypedTarget<double> Target;

// Represents a camera that can see targets and provide information about their
// relative positions.
// Note on coordinate systems:
// -The camera's Pose shall be such that the camera frame's positive X-axis is
//  pointed straight out of the lens (as always, positive Z will be up; we
//  assume that all cameras mounted level, or compensated for such that this
//  code won't care).
// -The origin of the camera shall be "at" the camera. For this code, I don't
//  think we care too much about the details of the camera model, so we can just
//  assume that it is an idealized pinhole camera with the pinhole being the
//  location of the camera.
//
// Template parameters:
// -num_targets: The number of targets on the field, should be the same for
//   all the actual cameras on the robot (although it may change in tests).
// -Scalar: The floating point type to use (double vs. float).
// -num_obstacles: The number of obstacles on the field to account for; like
//   the number of targets, it should be consistent across actual cameras,
//   although for simulation we may add extra obstacles for testing.
template <int num_targets, int num_obstacles, typename Scalar = double>
class TypedCamera {
 public:
  typedef ::frc971::control_loops::TypedPose<Scalar> Pose;
  typedef ::frc971::control_loops::TypedLineSegment<Scalar> LineSegment;

  // TargetView contains the information associated with a sensor reading
  // from the camera--the readings themselves and noise values, *from the
  // Camera's persective* for each reading.
  // Note that the noise terms are just accounting for the inaccuracy you
  // expect to get due to visual noise, pixel-level resolution, etc. These
  // do not account for the fact that, e.g., there is noise in the Pose of the
  // robot which can translate into noise in the target reading.
  // The noise terms are standard deviations, and so have units identical
  // to that of the actual readings.
  struct TargetView {
    struct Reading {
      // The heading as reported from the camera; zero = straight ahead,
      // positive = target in the left half of the image.
      Scalar heading;   // radians
      // The distance from the camera to the target.
      Scalar distance;  // meters
      // Height of the target from the camera.
      Scalar height;    // meters
      // The angle of the target relative to line between the camera and
      // the center of the target.
      Scalar skew;      // radians
    };
    Reading reading;
    Reading noise;

    // The target that this view corresponds to.
    const TypedTarget<Scalar> *target = nullptr;
    // The Pose the camera was at when viewing the target:
    Pose camera_pose;
  };

  // Important parameters for dealing with camera noise calculations.
  // Ultimately, this should end up coming from the constants file.
  struct NoiseParameters {
    // The maximum distance from which we can see a target head-on (when the
    // target is not head-on, we adjust for that).
    Scalar max_viewable_distance;  // meters

    // All noises are standard deviations of the noise, assuming an ~normal
    // distribution.

    // Noise in the heading measurement, which should be constant regardless of
    // other factors.
    Scalar heading_noise;  // radians
    // Noise in the distance measurement when the target is 1m away and head-on
    // to us. This is adjusted by assuming the noise is proportional to the
    // apparent width of the target (because the target also has height, this
    // may not be strictly correct).
    // TODO(james): Is this a good model? It should be reasonable, but there
    // may be more complexity somewhere.
    Scalar nominal_distance_noise;  // meters
    // The noise in the skew measurement when the target is 1m away and head-on
    // to us. Calculated in the same manner with the same caveats as the
    // distance noise.
    Scalar nominal_skew_noise;  // radians
    // Noise in the height measurement, same rules as for skew and distance.
    // TODO(james): Figure out how much noise we will actually get in the
    // height, since there will be extremely low resolution on it.
    Scalar nominal_height_noise;  // meters
  };

  // Provide a default constructor to make life easier.
  TypedCamera() {}

  // Creates a camera:
  // pose: The Pose of the camera, relative to the robot at least transitively.
  // fov: The field-of-view of the camera, in radians. Note that this is the
  //   *total* field-of-view in the horizontal plane (left-right), so the angle
  //   from the left edge of the image to the right edge.
  // targets: The list of targets on the field that could be seen by the camera.
  // obstacles: The list of known obstacles on the field.
  TypedCamera(const Pose &pose, Scalar fov,
              const NoiseParameters &noise_parameters,
              const ::std::array<TypedTarget<Scalar>, num_targets> &targets,
              const ::std::array<LineSegment, num_obstacles> &obstacles)
      : pose_(pose),
        fov_(fov),
        noise_parameters_(noise_parameters),
        targets_(targets),
        obstacles_(obstacles) {}

  // Returns a list of TargetViews for all the currently visible targets.
  // These will contain ground-truth TargetViews, so the readings will be
  // perfect; a pseudo-random number generator should be used to add noise
  // separately for simulation.
  ::aos::SizedArray<TargetView, num_targets> target_views() const {
    ::aos::SizedArray<TargetView, num_targets> views;
    Pose camera_abs_pose = pose_.Rebase(nullptr);
    // Because there are num_targets in targets_ and because AddTargetIfVisible
    // adds at most 1 view to views, we should never exceed the size of
    // SizedArray.
    for (const auto &target : targets_) {
      AddTargetIfVisible(target, camera_abs_pose, &views);
    }
    return views;
  }

  // Returns a list of list of points for plotting. Each list of points should
  // be plotted as a line; currently, each list is just a pair drawing a line
  // from the camera aperture to the target location.
  // This should not be called from real-time code, as it will probably
  // dynamically allocate memory.
  ::std::vector<::std::vector<Pose>> PlotPoints() const {
    ::std::vector<::std::vector<Pose>> list_of_lists;
    for (const auto &view : target_views()) {
      list_of_lists.push_back({pose_, view.target->pose().Rebase(&pose_)});
    }
    return list_of_lists;
  }

  const Pose &pose() const { return pose_; }
  Scalar fov() const { return fov_; }

  // Estimates the noise values of a target based on the raw readings.
  // Also estimates whether we would expect the target to be visible, and
  // populates is_visible if is_visible is not nullptr.
  void PopulateNoise(TargetView *view, bool *is_visible = nullptr) const {
    // Calculate the width of the target as it appears in the image.
    // This number is unitless and if greater than 1, implies that the target is
    // visible to the camera and if less than 1 implies it is too small to be
    // registered on the camera.
    const Scalar cosskew = ::std::cos(view->reading.skew);
    Scalar apparent_width = ::std::max<Scalar>(
        0.0, cosskew * noise_parameters_.max_viewable_distance /
                 view->reading.distance);
    // If we got wildly invalid distance or skew measurements, then set a very
    // small apparent width.
    if (view->reading.distance < 0 || cosskew < 0) {
      apparent_width = 0.01;
    }
    // As both a sanity check and for the sake of numerical stability, manually
    // set apparent_width to something "very small" if it is near zero.
    if (apparent_width < 0.01) {
      apparent_width = 0.01;
    }

    if (is_visible != nullptr) {
      *is_visible = apparent_width >= 1.0;
    }

    view->noise.heading = noise_parameters_.heading_noise;

    const Scalar normalized_width =
        apparent_width / noise_parameters_.max_viewable_distance;
    view->noise.distance =
        noise_parameters_.nominal_distance_noise / normalized_width;
    view->noise.skew =
        noise_parameters_.nominal_skew_noise / normalized_width;
    view->noise.height =
        noise_parameters_.nominal_height_noise / normalized_width;
  }

 private:

  // If the specified target is visible from the current camera Pose, adds it to
  // the views array.
  void AddTargetIfVisible(
      const TypedTarget<Scalar> &target, const Pose &camera_abs_pose,
      ::aos::SizedArray<TargetView, num_targets> *views) const;

  // The Pose of this camera.
  Pose pose_;

  // Field of view of the camera, in radians.
  Scalar fov_;

  // Various constants to calclate sensor noise; see definition of
  // NoiseParameters for more detail.
  NoiseParameters noise_parameters_;

  // A list of all the targets on the field.
  // TODO(james): Is it worth creating some sort of cache for the targets and
  // obstacles? e.g., passing around pointer to the targets/obstacles.
  ::std::array<TypedTarget<Scalar>, num_targets> targets_;
  // Known obstacles on the field which can interfere with our view of the
  // targets. An "obstacle" is a line segment which we cannot see through, as
  // such a logical obstacle (e.g., the cargo ship) may consist of many
  // obstacles in this list to account for all of its sides.
  ::std::array<LineSegment, num_obstacles> obstacles_;
}; // class TypedCamera

template <int num_targets, int num_obstacles, typename Scalar>
void TypedCamera<num_targets, num_obstacles, Scalar>::AddTargetIfVisible(
    const TypedTarget<Scalar> &target, const Pose &camera_abs_pose,
    ::aos::SizedArray<TargetView, num_targets> *views) const {
  if (target.occluded()) {
    return;
  }

  // Precompute the current absolute pose of the camera, because we will reuse
  // it a bunch.
  const Pose relative_pose = target.pose().Rebase(&camera_abs_pose);
  const Scalar heading = relative_pose.heading();
  const Scalar distance = relative_pose.xy_norm();
  const Scalar skew =
      ::aos::math::NormalizeAngle(relative_pose.rel_theta() - heading);

  // Check if the camera is in the angular FOV.
  if (::std::abs(heading) > fov_ / 2.0) {
    return;
  }

  TargetView view;
  view.reading.heading = heading;
  view.reading.distance = distance;
  view.reading.skew = skew;
  view.reading.height = relative_pose.rel_pos().z();
  view.target = &target;
  view.camera_pose = camera_abs_pose;

  bool is_visible = false;

  PopulateNoise(&view, &is_visible);

  if (!is_visible) {
    return;
  }

  // Final visibility check is for whether there are any obstacles blocking or
  // line of sight.
  for (const auto &obstacle : obstacles_) {
    if (obstacle.Intersects({camera_abs_pose, target.pose()})) {
      return;
    }
  }

  // At this point, we've passed all the checks to ensure that the target is
  // visible and we can add it to the list of targets.
  views->push_back(view);
}

}  // namespace control_loops
}  // namespace y2019

#endif  // Y2019_CONTROL_LOOPS_DRIVETRAIN_CAMERA_H_
