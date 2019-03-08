#include "y2019/vision/target_finder.h"

#include "ceres/ceres.h"

#include <math.h>

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
  return std::array<Vector<2>, 8>{{right.top, right.inside, right.bottom,
                                   right.outside, left.top, left.inside,
                                   left.bottom, left.outside}};
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
  auto project = [&](Vector<2> pt) {
    return Project(pt, intrinsics, extrinsics);
  };
  Target new_targ;
  new_targ.right.is_right = true;
  new_targ.right.top = project(target.right.top);
  new_targ.right.inside = project(target.right.inside);
  new_targ.right.bottom = project(target.right.bottom);
  new_targ.right.outside = project(target.right.outside);

  new_targ.left.top = project(target.left.top);
  new_targ.left.inside = project(target.left.inside);
  new_targ.left.bottom = project(target.left.bottom);
  new_targ.left.outside = project(target.left.outside);

  return new_targ;
}

// Used at runtime on a single image given camera parameters.
struct RuntimeCostFunctor {
  RuntimeCostFunctor(Vector<2> result, Vector<2> template_pt,
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

IntermediateResult TargetFinder::ProcessTargetToResult(const Target &target,
                                                       bool verbose) {
  // Memory for the ceres solver.
  double params[ExtrinsicParams::kNumParams];
  default_extrinsics_.set(&params[0]);

  Problem problem;

  ::std::array<aos::vision::Vector<2>, 8> target_value = target.ToPointList();
  ::std::array<aos::vision::Vector<2>, 8> template_value =
      target_template_.ToPointList();

  for (size_t i = 0; i < 8; ++i) {
    aos::vision::Vector<2> a = template_value[i];
    aos::vision::Vector<2> b = target_value[i];

    problem.AddResidualBlock(
        new NumericDiffCostFunction<RuntimeCostFunctor, CENTRAL, 2, 4>(
            new RuntimeCostFunctor(b, a, intrinsics_)),
        NULL, &params[0]);
  }

  Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  Solver::Summary summary;
  Solve(options, &problem, &summary);

  IntermediateResult IR;
  IR.extrinsics = ExtrinsicParams::get(&params[0]);
  IR.solver_error = summary.final_cost;

  if (verbose) {
    std::cout << summary.BriefReport() << "\n";
    std::cout << "y = " << IR.extrinsics.y / kInchesToMeters << ";\n";
    std::cout << "z = " << IR.extrinsics.z / kInchesToMeters << ";\n";
    std::cout << "r1 = " << IR.extrinsics.r1 * 180 / M_PI << ";\n";
    std::cout << "r2 = " << IR.extrinsics.r2 * 180 / M_PI << ";\n";
    std::cout << "rup = " << intrinsics_.mount_angle * 180 / M_PI << ";\n";
    std::cout << "fl = " << intrinsics_.focal_length << ";\n";
    std::cout << "error = " << summary.final_cost << ";\n";
  }
  return IR;
}

}  // namespace vision
}  // namespace y2019
