#include <fstream>

#include "aos/logging/implementations.h"
#include "aos/logging/logging.h"
#include "aos/vision/blob/codec.h"
#include "aos/vision/blob/find_blob.h"
#include "aos/vision/events/socket_types.h"
#include "aos/vision/events/udp.h"
#include "aos/vision/image/image_dataset.h"
#include "aos/vision/image/image_stream.h"
#include "aos/vision/image/reader.h"
#include "y2019/vision/target_finder.h"

// CERES Clashes with logging symbols...
#include "ceres/ceres.h"

DEFINE_int32(camera_id, -1, "The camera ID to calibrate");
DEFINE_string(prefix, "", "The image filename prefix");

DEFINE_string(constants, "y2019/vision/constants.cc",
              "Path to the constants file to update");

DEFINE_double(beginning_tape_measure_reading, 11,
             "The tape measure measurement (in inches) of the first image.");
DEFINE_int32(image_count, 75, "The number of images to capture");
DEFINE_double(
    tape_start_x, -12.5,
    "The starting location of the tape measure in x relative to the CG in inches.");
DEFINE_double(
    tape_start_y, -0.5,
    "The starting location of the tape measure in y relative to the CG in inches.");

DEFINE_double(
    tape_direction_x, -1.0,
    "The x component of \"1\" inch along the tape measure in meters.");
DEFINE_double(
    tape_direction_y, 0.0,
    "The y component of \"1\" inch along the tape measure in meters.");

using ::aos::events::DataSocket;
using ::aos::events::RXUdpSocket;
using ::aos::events::TCPServer;
using ::aos::vision::DataRef;
using ::aos::vision::Int32Codec;
using ::aos::monotonic_clock;
using aos::vision::Segment;

using ceres::NumericDiffCostFunction;
using ceres::CENTRAL;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

namespace y2019 {
namespace vision {
namespace {

constexpr double kInchesToMeters = 0.0254;

}  // namespace

::std::array<double, 3> GetError(const DatasetInfo &info,
                                 const double *const extrinsics,
                                 const double *const geometry, int i) {
  const ExtrinsicParams extrinsic_params = ExtrinsicParams::get(extrinsics);
  const CameraGeometry geo = CameraGeometry::get(geometry);

  const double s = sin(geo.heading + extrinsic_params.r2);
  const double c = cos(geo.heading + extrinsic_params.r2);

  // Take the tape measure starting point (this will be at the perimeter of the
  // robot), add the offset to the first sample, and then add the per sample
  // offset.
  const double dx = (info.to_tape_measure_start[0] +
                     (info.beginning_tape_measure_reading + i) *
                         info.tape_measure_direction[0]) -
                    (geo.location[0] + c * extrinsic_params.z);
  const double dy = (info.to_tape_measure_start[1] +
                     (info.beginning_tape_measure_reading + i) *
                         info.tape_measure_direction[1]) -
                    (geo.location[1] + s * extrinsic_params.z);

  constexpr double kCalibrationTargetHeight = 28.5 * kInchesToMeters;
  const double dz =
      kCalibrationTargetHeight - (geo.location[2] + extrinsic_params.y);
  return {{dx, dy, dz}};
}

void main(int argc, char **argv) {
  using namespace y2019::vision;
  ::gflags::ParseCommandLineFlags(&argc, &argv, false);

  DatasetInfo info = {
      FLAGS_camera_id,
      {{FLAGS_tape_start_x * kInchesToMeters,
        FLAGS_tape_start_y * kInchesToMeters}},
      {{FLAGS_tape_direction_x * kInchesToMeters,
        FLAGS_tape_direction_y * kInchesToMeters}},
      FLAGS_beginning_tape_measure_reading,
      FLAGS_prefix.c_str(),
      FLAGS_image_count,
  };

  ::aos::logging::Init();
  ::aos::logging::AddImplementation(
      new ::aos::logging::StreamLogImplementation(stderr));

  TargetFinder target_finder;

  ceres::Problem problem;

  struct Sample {
    ::std::unique_ptr<double[]> extrinsics;
    int i;
  };
  ::std::vector<Sample> all_extrinsics;
  double intrinsics[IntrinsicParams::kNumParams];
  IntrinsicParams().set(&intrinsics[0]);

  double geometry[CameraGeometry::kNumParams];
  {
    // Assume the camera is near the center of the robot, and start by pointing
    // it from the center of the robot to the location of the first target.
    CameraGeometry camera_geometry;
    const double x =
        info.to_tape_measure_start[0] +
        info.beginning_tape_measure_reading * info.tape_measure_direction[0];
    const double y =
        info.to_tape_measure_start[1] +
        info.beginning_tape_measure_reading * info.tape_measure_direction[1];
    camera_geometry.heading = ::std::atan2(y, x);
    printf("Inital heading is %f\n", camera_geometry.heading);
    camera_geometry.set(&geometry[0]);
  }

  Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  Solver::Summary summary;

  ::std::cout << summary.BriefReport() << "\n";
  IntrinsicParams intrinsics_ = IntrinsicParams::get(&intrinsics[0]);
  ::std::cout << "rup = " << intrinsics_.mount_angle * 180 / M_PI << ";\n";
  ::std::cout << "fl = " << intrinsics_.focal_length << ";\n";
  ::std::cout << "error = " << summary.final_cost << ";\n";

  for (int i = 0; i < info.num_images; ++i) {
    ::aos::vision::DatasetFrame frame =
        aos::vision::LoadFile(FLAGS_prefix + std::to_string(i) + ".yuyv");

    const ::aos::vision::ImageFormat fmt{640, 480};
    ::aos::vision::BlobList imgs =
        ::aos::vision::FindBlobs(aos::vision::SlowYuyvYThreshold(
            fmt, frame.data.data(), TargetFinder::GetThresholdValue()));
    target_finder.PreFilter(&imgs);

    constexpr bool verbose = false;
    ::std::vector<Polygon> raw_polys;
    for (const RangeImage &blob : imgs) {
      // Convert blobs to contours in the corrected space.
      ContourNode *contour = target_finder.GetContour(blob);
      ::std::vector<::Eigen::Vector2f> unwarped_contour =
          target_finder.UnWarpContour(contour);
      const Polygon polygon =
          target_finder.FindPolygon(::std::move(unwarped_contour), verbose);
      if (!polygon.segments.empty()) {
        raw_polys.push_back(polygon);
      }
    }

    // Calculate each component side of a possible target.
    const ::std::vector<TargetComponent> target_component_list =
        target_finder.FillTargetComponentList(raw_polys, verbose);

    // Put the compenents together into targets.
    const ::std::vector<Target> target_list =
        target_finder.FindTargetsFromComponents(target_component_list, verbose);

    // Now, iterate over the targets (in pixel space).  Generate a residual
    // block for each valid target which compares the actual pixel locations
    // with the expected pixel locations given the intrinsics and extrinsics.
    for (const Target &target : target_list) {
      const ::std::array<::aos::vision::Vector<2>, 8> target_value =
          target.ToPointList();
      const ::std::array<::aos::vision::Vector<2>, 8> template_value =
          target_finder.GetTemplateTarget().ToPointList();

      all_extrinsics.push_back(
          {::std::unique_ptr<double[]>(new double[ExtrinsicParams::kNumParams]),
           i});
      double *extrinsics = all_extrinsics.back().extrinsics.get();
      ExtrinsicParams().set(extrinsics);

      for (size_t j = 0; j < 8; ++j) {
        // Template target in target space as documented by GetTemplateTarget()
        const ::aos::vision::Vector<2> template_point = template_value[j];
        // Target in pixel space.
        const ::aos::vision::Vector<2> target_point = target_value[j];

        // Now build up the residual block.
        auto ftor = [template_point, target_point](
            const double *const intrinsics, const double *const extrinsics,
            double *residual) {
          const IntrinsicParams intrinsic_params =
              IntrinsicParams::get(intrinsics);
          const ExtrinsicParams extrinsic_params =
              ExtrinsicParams::get(extrinsics);
          // Project the target location into pixel coordinates.
          const ::aos::vision::Vector<2> projected_point =
              Project(template_point, intrinsic_params, extrinsic_params);
          const ::aos::vision::Vector<2> residual_point =
              target_point - projected_point;
          residual[0] = residual_point.x();
          residual[1] = residual_point.y();
          return true;
        };
        problem.AddResidualBlock(
            new NumericDiffCostFunction<decltype(ftor), CENTRAL, 2,
                                        IntrinsicParams::kNumParams,
                                        ExtrinsicParams::kNumParams>(
                new decltype(ftor)(::std::move(ftor))),
            NULL, &intrinsics[0], extrinsics);
      }

      // Now, compare the estimated location of the target that we just solved
      // for with the extrinsic params above with the known location of the
      // target.
      auto ftor = [&info, i](const double *const extrinsics,
                             const double *const geometry, double *residual) {
        const ::std::array<double, 3> err =
            GetError(info, extrinsics, geometry, i);
        // We care a lot more about geometry than pixels.  Especially since
        // pixels are small and meters are big, so there's a small penalty to
        // being off by meters.
        residual[0] = err[0] * 1259.0;
        residual[1] = err[1] * 1259.0;
        residual[2] = err[2] * 1259.0;
        return true;
      };

      problem.AddResidualBlock(
          new NumericDiffCostFunction<decltype(ftor), CENTRAL, 3,
                                      ExtrinsicParams::kNumParams,
                                      CameraGeometry::kNumParams>(
              new decltype(ftor)(::std::move(ftor))),
          NULL, extrinsics, &geometry[0]);
    }
  }
  // TODO: Debug solver convergence?
  Solve(options, &problem, &summary);
  Solve(options, &problem, &summary);
  Solve(options, &problem, &summary);

  {
    ::std::cout << summary.BriefReport() << ::std::endl;
    IntrinsicParams intrinsics_ = IntrinsicParams::get(&intrinsics[0]);
    CameraGeometry geometry_ = CameraGeometry::get(&geometry[0]);
    ::std::cout << "rup = " << intrinsics_.mount_angle * 180 / M_PI << ";"
                << ::std::endl;
    ::std::cout << "fl = " << intrinsics_.focal_length << ";" << ::std::endl;
    ::std::cout << "error = " << summary.final_cost << ";" << ::std::endl;

    ::std::cout << "camera_angle = " << geometry_.heading * 180 / M_PI
                << ::std::endl;
    ::std::cout << "camera_x = " << geometry_.location[0] << ::std::endl;
    ::std::cout << "camera_y = " << geometry_.location[1] << ::std::endl;
    ::std::cout << "camera_z = " << geometry_.location[2] << ::std::endl;
    ::std::cout << "camera_barrel = " << intrinsics_.barrel_mount * 180.0 / M_PI
                << ::std::endl;

    for (const Sample &sample : all_extrinsics) {
      const int i = sample.i;
      double *data = sample.extrinsics.get();

      const ExtrinsicParams extrinsic_params = ExtrinsicParams::get(data);

      const ::std::array<double, 3> error =
          GetError(info, data, &geometry[0], i);

      ::std::cout << i << ", ";
      ::std::cout << extrinsic_params.z << "m, ";
      ::std::cout << extrinsic_params.y << "m, ";
      ::std::cout << extrinsic_params.r1 * 180 / M_PI << ", ";
      ::std::cout << extrinsic_params.r2 * 180 / M_PI << ", ";
      // TODO: Methodology problem: This should really have a separate solve for
      // extrinsics...
      ::std::cout << error[0] << "m, ";
      ::std::cout << error[1] << "m, ";
      ::std::cout << error[2] << "m" << ::std::endl;
    }
  }

  CameraCalibration results;
  results.dataset = info;
  results.intrinsics = IntrinsicParams::get(&intrinsics[0]);
  results.geometry = CameraGeometry::get(&geometry[0]);
  DumpCameraConstants(FLAGS_constants.c_str(), info.camera_id, results);
}

}  // namespace y2019
}  // namespace vision

int main(int argc, char **argv) { y2019::vision::main(argc, argv); }
