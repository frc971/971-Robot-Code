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

#undef CHECK_NOTNULL
#undef CHECK_OP
#undef PCHECK
// CERES Clashes with logging symbols...
#include "ceres/ceres.h"

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

constexpr double kInchesToMeters = 0.0254;

template <size_t k, size_t n, size_t n2, typename T>
T *MakeFunctor(T &&t) {
  return new T(std::move(t));
}

std::array<double, 3> GetError(const double *const extrinsics,
                               const double *const geometry, int i) {
  auto ex = ExtrinsicParams::get(extrinsics);

  double s = sin(geometry[2] + ex.r2);
  double c = cos(geometry[2] + ex.r2);

  // TODO: Generalize this from being just for a single calibration.
  double dx = 12.5 + 26.0 + i - (geometry[0] + c * ex.z) / kInchesToMeters;
  double dy = 12.0 - (geometry[1] + s * ex.z) / kInchesToMeters;
  double dz = 28.5 - (geometry[3] + ex.y) / kInchesToMeters;
  return {{dx, dy, dz}};
}

void main(int argc, char **argv) {
  (void)argc;
  (void)argv;
  using namespace y2019::vision;
  // gflags::ParseCommandLineFlags(&argc, &argv, false);
  ::aos::logging::Init();
  ::aos::logging::AddImplementation(
      new ::aos::logging::StreamLogImplementation(stderr));

  TargetFinder finder_;

  ceres::Problem problem;

  struct Sample {
    std::unique_ptr<double[]> extrinsics;
    int i;
  };
  std::vector<Sample> all_extrinsics;
  double intrinsics[IntrinsicParams::kNumParams];
  IntrinsicParams().set(&intrinsics[0]);

  // To know the meaning, see the printout below...
  constexpr size_t GeometrykNumParams = 4;
  double geometry[GeometrykNumParams] = {0, 0, 0, 0};

  Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  Solver::Summary summary;

  std::cout << summary.BriefReport() << "\n";
  auto intrinsics_ = IntrinsicParams::get(&intrinsics[0]);
  std::cout << "rup = " << intrinsics_.mount_angle * 180 / M_PI << ";\n";
  std::cout << "fl = " << intrinsics_.focal_length << ";\n";
  std::cout << "error = " << summary.final_cost << ";\n";

  int nimgs = 56;
  for (int i = 0; i < nimgs; ++i) {
    auto frame = aos::vision::LoadFile(
        "/home/parker/data/frc/2019_calibration/cam4_0/debug_viewer_jpeg_" +
        std::to_string(i) + ".yuyv");

    aos::vision::ImageFormat fmt{640, 480};
    aos::vision::BlobList imgs = aos::vision::FindBlobs(
        aos::vision::DoThresholdYUYV(fmt, frame.data.data(), 120));
    finder_.PreFilter(&imgs);

    bool verbose = false;
    std::vector<std::vector<Segment<2>>> raw_polys;
    for (const RangeImage &blob : imgs) {
      std::vector<Segment<2>> polygon = finder_.FillPolygon(blob, verbose);
      if (polygon.empty()) {
      } else {
        raw_polys.push_back(polygon);
      }
    }

    // Calculate each component side of a possible target.
    std::vector<TargetComponent> target_component_list =
        finder_.FillTargetComponentList(raw_polys);

    // Put the compenents together into targets.
    std::vector<Target> target_list =
        finder_.FindTargetsFromComponents(target_component_list, verbose);

    // Use the solver to generate an intermediate version of our results.
    std::vector<IntermediateResult> results;
    for (const Target &target : target_list) {
      auto target_value = target.toPointList();
      auto template_value = finder_.GetTemplateTarget().toPointList();

      auto *extrinsics = new double[ExtrinsicParams::kNumParams];
      ExtrinsicParams().set(extrinsics);
      all_extrinsics.push_back({std::unique_ptr<double[]>(extrinsics), i});

      for (size_t j = 0; j < 8; ++j) {
        auto temp = template_value[j];
        auto targ = target_value[j];

        auto ftor = [temp, targ, i](const double *const intrinsics,
                                    const double *const extrinsics,
                                    double *residual) {
          auto in = IntrinsicParams::get(intrinsics);
          auto ex = ExtrinsicParams::get(extrinsics);
          auto pt = targ - Project(temp, in, ex);
          residual[0] = pt.x();
          residual[1] = pt.y();
          return true;
        };
        problem.AddResidualBlock(
            new NumericDiffCostFunction<decltype(ftor), CENTRAL, 2,
                                        IntrinsicParams::kNumParams,
                                        ExtrinsicParams::kNumParams>(
                new decltype(ftor)(std::move(ftor))),
            NULL, &intrinsics[0], extrinsics);
      }

      auto ftor = [i](const double *const extrinsics,
                      const double *const geometry, double *residual) {
        auto err = GetError(extrinsics, geometry, i);
        residual[0] = 32.0 * err[0];
        residual[1] = 32.0 * err[1];
        residual[2] = 32.0 * err[2];
        return true;
      };

      problem.AddResidualBlock(
          new NumericDiffCostFunction<decltype(ftor), CENTRAL, 3,
                                      ExtrinsicParams::kNumParams,
                                      GeometrykNumParams>(
              new decltype(ftor)(std::move(ftor))),
          NULL, extrinsics, &geometry[0]);
    }
  }
  // TODO: Debug solver convergence?
  Solve(options, &problem, &summary);
  Solve(options, &problem, &summary);
  Solve(options, &problem, &summary);

  {
    std::cout << summary.BriefReport() << "\n";
    auto intrinsics_ = IntrinsicParams::get(&intrinsics[0]);
    std::cout << "rup = " << intrinsics_.mount_angle * 180 / M_PI << ";\n";
    std::cout << "fl = " << intrinsics_.focal_length << ";\n";
    std::cout << "error = " << summary.final_cost << ";\n";

    std::cout << "camera_height = " << geometry[3] / kInchesToMeters << "\n";
    std::cout << "camera_angle = " << geometry[2] * 180 / M_PI << "\n";
    std::cout << "camera_x = " << geometry[0] / kInchesToMeters << "\n";
    std::cout << "camera_y = " << geometry[1] / kInchesToMeters << "\n";
    std::cout << "camera_barrel = " << intrinsics_.barrel_mount * 180.0 / M_PI
              << "\n";

    for (auto &sample : all_extrinsics) {
      int i = sample.i;
      double *data = sample.extrinsics.get();

      auto extn = ExtrinsicParams::get(data);

      auto err = GetError(data, &geometry[0], i);

      std::cout << i << ", ";
      std::cout << extn.z / kInchesToMeters << ", ";
      std::cout << extn.y / kInchesToMeters << ", ";
      std::cout << extn.r1 * 180 / M_PI << ", ";
      std::cout << extn.r2 * 180 / M_PI << ", ";
      // TODO: Methodology problem: This should really have a separate solve for
      // extrinsics...
      std::cout << err[0] << ", ";
      std::cout << err[1] << ", ";
      std::cout << err[2] << "\n";
    }
  }
}

}  // namespace y2019
}  // namespace vision

int main(int argc, char **argv) { y2019::vision::main(argc, argv); }
