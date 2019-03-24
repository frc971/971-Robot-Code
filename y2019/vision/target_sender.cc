#include <fstream>

#include "aos/logging/implementations.h"
#include "aos/logging/logging.h"
#include "aos/vision/blob/codec.h"
#include "aos/vision/blob/find_blob.h"
#include "aos/vision/events/socket_types.h"
#include "aos/vision/events/udp.h"
#include "y2019/jevois/camera/image_stream.h"
#include "y2019/jevois/camera/reader.h"

#include "y2019/jevois/serial.h"
#include "y2019/jevois/structures.h"
#include "y2019/jevois/uart.h"
#include "y2019/vision/target_finder.h"

using ::aos::events::DataSocket;
using ::aos::events::RXUdpSocket;
using ::aos::events::TCPServer;
using ::aos::vision::DataRef;
using ::aos::vision::Int32Codec;
using ::aos::monotonic_clock;
using ::y2019::jevois::open_via_terminos;
using aos::vision::Segment;

class CameraStream : public ::y2019::camera::ImageStreamEvent {
 public:
  CameraStream(::aos::vision::CameraParams params, const ::std::string &fname)
      : ImageStreamEvent(fname, params) {}

  void ProcessImage(DataRef data, monotonic_clock::time_point monotonic_now) {
    LOG(INFO, "got frame: %d\n", (int)data.size());

    if (on_frame_) on_frame_(data, monotonic_now);
  }

  void set_on_frame(const std::function<
                    void(DataRef, monotonic_clock::time_point)> &on_frame) {
    on_frame_ = on_frame;
  }

 private:
  std::function<void(DataRef, monotonic_clock::time_point)> on_frame_;
};

int open_terminos(const char *tty_name) { return open_via_terminos(tty_name); }

std::string GetFileContents(const std::string &filename) {
  std::ifstream in(filename, std::ios::in | std::ios::binary);
  if (in) {
    std::string contents;
    in.seekg(0, std::ios::end);
    contents.resize(in.tellg());
    in.seekg(0, std::ios::beg);
    in.read(&contents[0], contents.size());
    in.close();
    return (contents);
  }
  fprintf(stderr, "Could not read file: %s\n", filename.c_str());
  exit(-1);
}

using aos::vision::ImageRange;
using aos::vision::RangeImage;
using aos::vision::ImageFormat;

int main(int argc, char **argv) {
  (void)argc;
  (void)argv;
  using namespace y2019::vision;
  using frc971::jevois::CameraCommand;
  // gflags::ParseCommandLineFlags(&argc, &argv, false);
  ::aos::logging::Init();
  ::aos::logging::AddImplementation(
      new ::aos::logging::StreamLogImplementation(stderr));

  int itsDev = open_terminos("/dev/ttyS0");
  frc971::jevois::CobsPacketizer<frc971::jevois::uart_to_camera_size()> cobs;
  // Uncomment these to printf directly to stdout to get debug info...
  // dup2(itsDev, 1);
  // dup2(itsDev, 2);

  TargetFinder finder_;

  aos::vision::CameraParams params0;
  params0.set_exposure(50);
  params0.set_brightness(40);
  params0.set_width(640);
  params0.set_fps(15);
  params0.set_height(480);

  ::std::unique_ptr<CameraStream> camera0(
      new CameraStream(params0, "/dev/video0"));
  camera0->set_on_frame([&](DataRef data,
                            monotonic_clock::time_point monotonic_now) {
    aos::vision::ImageFormat fmt{640, 480};
    aos::vision::BlobList imgs = aos::vision::FindBlobs(
        aos::vision::FastYuyvYThreshold(fmt, data.data(), 120));
    finder_.PreFilter(&imgs);
    LOG(INFO, "Blobs: (%zu).\n", imgs.size());

    constexpr bool verbose = false;
    ::std::vector<Polygon> raw_polys;
    for (const RangeImage &blob : imgs) {
      // Convert blobs to contours in the corrected space.
      ContourNode *contour = finder_.GetContour(blob);
      ::std::vector<::Eigen::Vector2f> unwarped_contour =
          finder_.UnWarpContour(contour);
      const Polygon polygon =
          finder_.FindPolygon(::std::move(unwarped_contour), verbose);
      if (!polygon.segments.empty()) {
        raw_polys.push_back(polygon);
      }
    }
    LOG(INFO, "Polygons: (%zu).\n", raw_polys.size());

    // Calculate each component side of a possible target.
    ::std::vector<TargetComponent> target_component_list =
        finder_.FillTargetComponentList(raw_polys, verbose);
    LOG(INFO, "Components: (%zu).\n", target_component_list.size());

    // Put the compenents together into targets.
    ::std::vector<Target> target_list =
        finder_.FindTargetsFromComponents(target_component_list, verbose);
    LOG(INFO, "Potential Target: (%zu).\n", target_list.size());

    // Use the solver to generate an intermediate version of our results.
    ::std::vector<IntermediateResult> results;
    for (const Target &target : target_list) {
      results.emplace_back(finder_.ProcessTargetToResult(target, verbose));
    }
    LOG(INFO, "Raw Results: (%zu).\n", results.size());

    results = finder_.FilterResults(results, 30, verbose);
    LOG(INFO, "Results: (%zu).\n", results.size());

    // TODO: Select top 3 (randomly?)

    frc971::jevois::CameraFrame frame{};

    for (size_t i = 0; i < results.size() && i < frame.targets.max_size();
         ++i) {
      const auto &result = results[i].extrinsics;
      frame.targets.push_back(frc971::jevois::Target{
          static_cast<float>(result.z), static_cast<float>(result.y),
          static_cast<float>(result.r2), static_cast<float>(result.r1)});
    }

    frame.age = std::chrono::duration_cast<frc971::jevois::camera_duration>(
        aos::monotonic_clock::now() - monotonic_now);

    // If we succeed in writing our delimiter, then write out the rest of
    // the frame. If not, no point in continuing.
    if (write(itsDev, "\0", 1) == 1) {
      const auto serialized_frame = frc971::jevois::UartPackToTeensy(frame);
      // We don't really care if this succeeds or not. If it fails for some
      // reason, we'll just try again with the next frame, and the other end
      // will find the new packet just fine.
      ssize_t n =
          write(itsDev, serialized_frame.data(), serialized_frame.size());

      if (n != (ssize_t)serialized_frame.size()) {
        LOG(INFO, "Some problem happened");
      }
    }
  });

  aos::events::EpollLoop loop;

  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    camera0->ReadEvent();

    {
      constexpr size_t kBufferSize = frc971::jevois::uart_to_teensy_size();
      char data[kBufferSize];
      ssize_t n = read(itsDev, &data[0], kBufferSize);
      if (n >= 1) {
        cobs.ParseData(gsl::span<const char>(&data[0], n));
        auto packet = cobs.received_packet();
        if (!packet.empty()) {
          auto calibration_question =
              frc971::jevois::UartUnpackToCamera(packet);
          if (calibration_question) {
            const auto &calibration = *calibration_question;
            IntrinsicParams *intrinsics = finder_.mutable_intrinsics();
            intrinsics->mount_angle = calibration.calibration(0, 0);
            intrinsics->focal_length = calibration.calibration(0, 1);
            intrinsics->barrel_mount = calibration.calibration(0, 2);

            switch (calibration.camera_command) {
              case CameraCommand::kNormal:
              case CameraCommand::kAs:
                break;
              case CameraCommand::kUsb:
                return 0;
              case CameraCommand::kCameraPassthrough:
                return system("touch /tmp/do_not_export_sd_card");
            }
          } else {
            printf("bad frame\n");
          }
          cobs.clear_received_packet();
        }
      }
    }
  }

  // TODO: Fix event loop on jevois:
  // loop.Add(camera0.get());
  // loop.Run();
}
