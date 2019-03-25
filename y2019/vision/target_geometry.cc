#include "y2019/vision/target_finder.h"

#include "ceres/ceres.h"

#include <math.h>

#include "aos/util/math.h"

using ceres::NumericDiffCostFunction;
using ceres::CENTRAL;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

namespace y2019 {
namespace vision {

static constexpr double kInchesToMeters = 0.0254;

using namespace aos::vision;
using aos::vision::Vector;

Target Target::MakeTemplate() {
  Target out;
  // This is how off-vertical the tape is.
  const double theta = 14.5 * M_PI / 180.0;

  const double tape_offset = 4 * kInchesToMeters;
  const double tape_width = 2 * kInchesToMeters;
  const double tape_length = 5.5 * kInchesToMeters;

  const double s = sin(theta);
  const double c = cos(theta);
  out.right.top = Vector<2>(tape_offset, 0.0);
  out.right.inside = Vector<2>(tape_offset + tape_width * c, tape_width * s);
  out.right.bottom = Vector<2>(tape_offset + tape_width * c + tape_length * s,
                               tape_width * s - tape_length * c);
  out.right.outside =
      Vector<2>(tape_offset + tape_length * s, -tape_length * c);

  out.right.is_right = true;
  out.left.top = Vector<2>(-out.right.top.x(), out.right.top.y());
  out.left.inside = Vector<2>(-out.right.inside.x(), out.right.inside.y());
  out.left.bottom = Vector<2>(-out.right.bottom.x(), out.right.bottom.y());
  out.left.outside = Vector<2>(-out.right.outside.x(), out.right.outside.y());
  return out;
}

std::array<Vector<2>, 8> Target::ToPointList() const {
  // Note, the even points are fit with the line solver in the 4 point solution
  // while the odds are fit with the point matcher.
  return std::array<Vector<2>, 8>{{right.top, right.outside, right.inside,
                                   right.bottom, left.top, left.outside,
                                   left.inside, left.bottom}};
}

Vector<2> Project(Vector<2> pt, const IntrinsicParams &intrinsics,
                  const ExtrinsicParams &extrinsics) {
  const double y = extrinsics.y;    // height
  const double z = extrinsics.z;    // distance
  const double r1 = extrinsics.r1;  // skew
  const double r2 = extrinsics.r2;  // heading
  const double rup = intrinsics.mount_angle;
  const double rbarrel = intrinsics.barrel_mount;
  const double fl = intrinsics.focal_length;

  // Start by translating point in target-space to be at correct height.
  ::Eigen::Matrix<double, 1, 3> pts{pt.x(), pt.y() + y, 0.0};

  {
    // Rotate to compensate for skew angle, to get into a frame still at the
    // same (x, y) position as the target but rotated to be facing straight
    // towards the camera.
    const double theta = r1;
    const double s = sin(theta);
    const double c = cos(theta);
    pts = (::Eigen::Matrix<double, 3, 3>() << c, 0, -s, 0, 1, 0, s, 0,
           c).finished() *
          pts.transpose();
  }

  // Translate the coordinate frame to have (x, y) centered at the camera, but
  // still oriented to be facing along the line from the camera to the target.
  pts(2) += z;

  {
    // Rotate out the heading so that the frame is oriented to line up with the
    // camera's viewpoint in the yaw-axis.
    const double theta = r2;
    const double s = sin(theta);
    const double c = cos(theta);
    pts = (::Eigen::Matrix<double, 3, 3>() << c, 0, -s, 0, 1, 0, s, 0,
           c).finished() *
          pts.transpose();
  }

  // TODO: Apply 15 degree downward rotation.
  {
    // Compensate for rotation in the pitch of the camera up/down to get into
    // the coordinate frame lined up with the plane of the camera sensor.
    const double theta = rup;
    const double s = sin(theta);
    const double c = cos(theta);

    pts = (::Eigen::Matrix<double, 3, 3>() << 1, 0, 0, 0, c, -s, 0, s,
           c).finished() *
          pts.transpose();
  }

  // Compensate for rotation of the barrel of the camera, i.e. about the axis
  // that points straight out from the camera lense, using an AngleAxis instead
  // of manually constructing the rotation matrices because once you get into
  // this frame you no longer need to be masochistic.
  // TODO: Maybe barrel should be extrinsics to allow rocking?
  // Also, in this case, barrel should go above the rotation above?
  pts = ::Eigen::AngleAxis<double>(rbarrel, ::Eigen::Vector3d(0.0, 0.0, 1.0)) *
        pts.transpose();

  // TODO: Final image projection.
  const ::Eigen::Matrix<double, 1, 3> res = pts;

  // Finally, scale to account for focal length and translate to get into
  // pixel-space.
  const float scale = fl / res.z();
  return Vector<2>(res.x() * scale + 320.0, 240.0 - res.y() * scale);
}

Target Project(const Target &target, const IntrinsicParams &intrinsics,
               const ExtrinsicParams &extrinsics) {
  Target new_targ;
  new_targ.right.is_right = true;
  new_targ.right.top = Project(target.right.top, intrinsics, extrinsics);
  new_targ.right.inside = Project(target.right.inside, intrinsics, extrinsics);
  new_targ.right.bottom = Project(target.right.bottom, intrinsics, extrinsics);
  new_targ.right.outside =
      Project(target.right.outside, intrinsics, extrinsics);

  new_targ.left.top = Project(target.left.top, intrinsics, extrinsics);
  new_targ.left.inside = Project(target.left.inside, intrinsics, extrinsics);
  new_targ.left.bottom = Project(target.left.bottom, intrinsics, extrinsics);
  new_targ.left.outside = Project(target.left.outside, intrinsics, extrinsics);

  return new_targ;
}

// Used at runtime on a single image given camera parameters.
struct PointCostFunctor {
  PointCostFunctor(Vector<2> result, Vector<2> template_pt,
                     IntrinsicParams intrinsics)
      : result(result), template_pt(template_pt), intrinsics(intrinsics) {}

  bool operator()(const double *const x, double *residual) const {
    auto extrinsics = ExtrinsicParams::get(x);
    auto pt = result - Project(template_pt, intrinsics, extrinsics);
    residual[0] = pt.x();
    residual[1] = pt.y();
    return true;
  }

  Vector<2> result;
  Vector<2> template_pt;
  IntrinsicParams intrinsics;
};

// Find the distance from a lower target point to the 'vertical' line it should
// be on.
struct LineCostFunctor {
  LineCostFunctor(Vector<2> result, Segment<2> template_seg,
                  IntrinsicParams intrinsics)
      : result(result), template_seg(template_seg), intrinsics(intrinsics) {}

  bool operator()(const double *const x, double *residual) const {
    const auto extrinsics = ExtrinsicParams::get(x);
    const Vector<2> p1 = Project(template_seg.A(), intrinsics, extrinsics);
    const Vector<2> p2 = Project(template_seg.B(), intrinsics, extrinsics);
    // distance from line (P1, P2) to point result
    double dx = p2.x() - p1.x();
    double dy = p2.y() - p1.y();
    double denom = p2.DistanceTo(p1);
    residual[0] = ::std::abs(dy * result.x() - dx * result.y() +
                             p2.x() * p1.y() - p2.y() * p1.x()) /
                  denom;
    return true;
  }

  Vector<2> result;
  Segment<2> template_seg;
  IntrinsicParams intrinsics;
};

// Find the distance that the bottom point is outside the target and penalize
// that linearly.
class BottomPointCostFunctor {
 public:
  BottomPointCostFunctor(::Eigen::Vector2f bottom_point,
                         Segment<2> template_seg, IntrinsicParams intrinsics)
      : bottom_point_(bottom_point.x(), bottom_point.y()),
        template_seg_(template_seg),
        intrinsics_(intrinsics) {}

  bool operator()(const double *const x, double *residual) const {
    const ExtrinsicParams extrinsics = ExtrinsicParams::get(x);
    const Vector<2> p1 = Project(template_seg_.A(), intrinsics_, extrinsics);
    const Vector<2> p2 = Project(template_seg_.B(), intrinsics_, extrinsics);

    // Construct a vector pointed perpendicular to the line.  This vector is
    // pointed down out the bottom of the target.
    ::Eigen::Vector2d down_axis(-(p1.y() - p2.y()), p1.x() - p2.x());
    down_axis.normalize();

    // Positive means out.
    const double component =
        down_axis.transpose() * (bottom_point_ - p1.GetData().transpose());

    if (component > 0) {
      residual[0] = component * 1.0;
    } else {
      residual[0] = 0.0;
    }
    return true;
  }

 private:
  ::Eigen::Vector2d bottom_point_;
  Segment<2> template_seg_;

  IntrinsicParams intrinsics_;
};

IntermediateResult TargetFinder::ProcessTargetToResult(const Target &target,
                                                       bool verbose) {
  // Memory for the ceres solver.
  double params_8point[ExtrinsicParams::kNumParams];
  default_extrinsics_.set(&params_8point[0]);
  double params_4point[ExtrinsicParams::kNumParams];
  default_extrinsics_.set(&params_4point[0]);

  Problem::Options problem_options;
  problem_options.context = ceres_context_.get();
  Problem problem_8point(problem_options);
  Problem problem_4point(problem_options);

  ::std::array<aos::vision::Vector<2>, 8> target_value = target.ToPointList();
  ::std::array<aos::vision::Vector<2>, 8> template_value =
      target_template_.ToPointList();

  for (size_t i = 0; i < 8; ++i) {
    aos::vision::Vector<2> a = template_value[i];
    aos::vision::Vector<2> b = target_value[i];

    if (i % 2 == 1) {
      aos::vision::Vector<2> a2 = template_value[i-1];
      aos::vision::Segment<2> line = Segment<2>(a, a2);

      problem_4point.AddResidualBlock(
          new NumericDiffCostFunction<LineCostFunctor, CENTRAL, 1, 4>(
              new LineCostFunctor(b, line, intrinsics_)),
          NULL, &params_4point[0]);
    } else {
      problem_4point.AddResidualBlock(
          new NumericDiffCostFunction<PointCostFunctor, CENTRAL, 2, 4>(
              new PointCostFunctor(b, a, intrinsics_)),
          NULL, &params_4point[0]);
    }

    problem_8point.AddResidualBlock(
        new NumericDiffCostFunction<PointCostFunctor, CENTRAL, 2, 4>(
            new PointCostFunctor(b, a, intrinsics_)),
        NULL, &params_8point[0]);
  }

  Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  Solver::Summary summary_8point;
  Solve(options, &problem_8point, &summary_8point);


  // So, let's sneak up on it.  Start by warm-starting it with where we got on the 8 point solution.
  ExtrinsicParams::get(&params_8point[0]).set(&params_4point[0]);
  // Then solve without the bottom constraint.
  Solver::Summary summary_4point1;
  Solve(options, &problem_4point, &summary_4point1);

  // Now, add a large cost for the bottom point being below the bottom line.
  problem_4point.AddResidualBlock(
      new NumericDiffCostFunction<BottomPointCostFunctor, CENTRAL, 1, 4>(
          new BottomPointCostFunctor(target.left.bottom_point,
                                     Segment<2>(target_template_.left.outside,
                                                target_template_.left.bottom),
                                     intrinsics_)),
      NULL, &params_4point[0]);
  // Make sure to point the segment the other way so when we do a -pi/2 rotation
  // on the line, it points down in target space.
  problem_4point.AddResidualBlock(
      new NumericDiffCostFunction<BottomPointCostFunctor, CENTRAL, 1, 4>(
          new BottomPointCostFunctor(target.right.bottom_point,
                                     Segment<2>(target_template_.right.bottom,
                                                target_template_.right.outside),
                                     intrinsics_)),
      NULL, &params_4point[0]);

  // And then re-solve.
  Solver::Summary summary_4point2;
  Solve(options, &problem_4point, &summary_4point2);

  IntermediateResult IR;
  IR.extrinsics = ExtrinsicParams::get(&params_8point[0]);
  IR.solver_error = summary_8point.final_cost;
  IR.backup_extrinsics = ExtrinsicParams::get(&params_4point[0]);
  IR.backup_solver_error = summary_4point2.final_cost;

  // Normalize all angles to (-M_PI, M_PI]
  IR.extrinsics.r1 = ::aos::math::NormalizeAngle(IR.extrinsics.r1);
  IR.extrinsics.r2 = ::aos::math::NormalizeAngle(IR.extrinsics.r2);
  IR.backup_extrinsics.r1 =
      ::aos::math::NormalizeAngle(IR.backup_extrinsics.r1);
  IR.backup_extrinsics.r2 =
      ::aos::math::NormalizeAngle(IR.backup_extrinsics.r2);

  // Ok, let's look at how perpendicular the corners are.
  // Vector from the outside to inside along the top on the left.
  const ::Eigen::Vector2d top_left_vector =
      (target.left.top.GetData() - target.left.inside.GetData())
          .transpose()
          .normalized();
  // Vector up the outside of the left target.
  const ::Eigen::Vector2d outer_left_vector =
      (target.left.top.GetData() - target.left.outside.GetData())
          .transpose()
          .normalized();
  // Vector up the inside of the left target.
  const ::Eigen::Vector2d inner_left_vector =
      (target.left.inside.GetData() - target.left.bottom.GetData())
          .transpose()
          .normalized();

  // Vector from the outside to inside along the top on the right.
  const ::Eigen::Vector2d top_right_vector =
      (target.right.top.GetData() - target.right.inside.GetData())
          .transpose()
          .normalized();
  // Vector up the outside of the right target.
  const ::Eigen::Vector2d outer_right_vector =
      (target.right.top.GetData() - target.right.outside.GetData())
          .transpose()
          .normalized();
  // Vector up the inside of the right target.
  const ::Eigen::Vector2d inner_right_vector =
      (target.right.inside.GetData() - target.right.bottom.GetData())
          .transpose()
          .normalized();

  // Now dot the vectors and use that to compute angles.
  // Left side, outside corner.
  const double left_outer_corner_dot =
      (outer_left_vector.transpose() * top_left_vector)(0);
  // Left side, inside corner.
  const double left_inner_corner_dot =
      (inner_left_vector.transpose() * top_left_vector)(0);
  // Right side, outside corner.
  const double right_outer_corner_dot =
      (outer_right_vector.transpose() * top_right_vector)(0);
  // Right side, inside corner.
  const double right_inner_corner_dot =
      (inner_right_vector.transpose() * top_right_vector)(0);

  constexpr double kCornerThreshold = 0.35;
  if (::std::abs(left_outer_corner_dot) < kCornerThreshold &&
      ::std::abs(left_inner_corner_dot) < kCornerThreshold &&
      ::std::abs(right_outer_corner_dot) < kCornerThreshold &&
      ::std::abs(right_inner_corner_dot) < kCornerThreshold) {
    IR.good_corners = true;
  } else {
    IR.good_corners = false;
  }

  if (verbose) {
    std::cout << "rup = " << intrinsics_.mount_angle * 180 / M_PI << ";\n";
    std::cout << "fl = " << intrinsics_.focal_length << ";\n";
    std::cout << "8 points:\n";
    std::cout << summary_8point.BriefReport() << "\n";
    std::cout << "error = " << summary_8point.final_cost << ";\n";
    std::cout << "y = " << IR.extrinsics.y / kInchesToMeters << ";\n";
    std::cout << "z = " << IR.extrinsics.z / kInchesToMeters << ";\n";
    std::cout << "r1 = " << IR.extrinsics.r1 * 180 / M_PI << ";\n";
    std::cout << "r2 = " << IR.extrinsics.r2 * 180 / M_PI << ";\n";
    std::cout << "4 points:\n";
    std::cout << summary_4point1.BriefReport() << "\n";
    std::cout << "error = " << summary_4point1.final_cost << ";\n\n";
    std::cout << "4 points:\n";
    std::cout << summary_4point2.BriefReport() << "\n";
    std::cout << "error = " << summary_4point2.final_cost << ";\n\n";
    std::cout << "y = " << IR.backup_extrinsics.y / kInchesToMeters << ";\n";
    std::cout << "z = " << IR.backup_extrinsics.z / kInchesToMeters << ";\n";
    std::cout << "r1 = " << IR.backup_extrinsics.r1 * 180 / M_PI << ";\n";
    std::cout << "r2 = " << IR.backup_extrinsics.r2 * 180 / M_PI << ";\n";


    printf("left upper outer corner angle: %f, top (%f, %f), outer (%f, %f)\n",
           (outer_left_vector.transpose() * top_left_vector)(0),
           top_left_vector(0, 0), top_left_vector(1, 0),
           outer_left_vector(0, 0), outer_left_vector(1, 0));
    printf("left upper inner corner angle: %f\n",
           (inner_left_vector.transpose() * top_left_vector)(0));

    printf("right upper outer corner angle: %f, top (%f, %f), outer (%f, %f)\n",
           (outer_right_vector.transpose() * top_right_vector)(0),
           top_right_vector(0, 0), top_right_vector(1, 0),
           outer_right_vector(0, 0), outer_right_vector(1, 0));
    printf("right upper inner corner angle: %f\n",
           (inner_right_vector.transpose() * top_right_vector)(0));
  }
  return IR;
}

}  // namespace vision
}  // namespace y2019
