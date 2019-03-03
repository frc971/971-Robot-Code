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

#define MASH(v0, v1, v2, v3, v4)                                  \
  ((uint8_t(v0) << 4) | (uint8_t(v1) << 3) | (uint8_t(v2) << 2) | \
   (uint8_t(v3) << 1) | (uint8_t(v4)))

// YUYV image types:
inline RangeImage DoThresholdYUYV(ImageFormat fmt, const char *data,
                                  uint8_t value) {
  std::vector<std::vector<ImageRange>> ranges;
  ranges.reserve(fmt.h);
  for (int y = 0; y < fmt.h; ++y) {
    const char *row = fmt.w * y * 2 + data;
    bool p_score = false;
    int pstart = -1;
    std::vector<ImageRange> rngs;
    for (int x = 0; x < fmt.w / 4; ++x) {
      uint8_t v[8];
      memcpy(&v[0], row + x * 4 * 2, 8);
      uint8_t pattern =
          MASH(p_score, v[0] > value, v[2] > value, v[4] > value, v[6] > value);
      switch (pattern) {
        /*
# Ruby code to generate the below code:
32.times do |v|
        puts "case MASH(#{[v[4], v[3], v[2], v[1], v[0]].join(", ")}):"
        p_score = v[4]
        pstart = "pstart"
        4.times do |i|
                if v[3 - i] != p_score
                        if (p_score == 1)
                                puts "  rngs.emplace_back(ImageRange(#{pstart},
x * 4 + #{i}));"
                        else
                                pstart = "x * 4 + #{i}"
                        end
                        p_score = v[3 - i]
                end
        end
        if (pstart != "pstart")
                puts "  pstart = #{pstart};"
        end
        if (p_score != v[4])
                puts "  p_score = #{["false", "true"][v[0]]};"
        end
        puts "  break;"
end
*/
        case MASH(0, 0, 0, 0, 0):
          break;
        case MASH(0, 0, 0, 0, 1):
          pstart = x * 4 + 3;
          p_score = true;
          break;
        case MASH(0, 0, 0, 1, 0):
          rngs.emplace_back(ImageRange(x * 4 + 2, x * 4 + 3));
          pstart = x * 4 + 2;
          break;
        case MASH(0, 0, 0, 1, 1):
          pstart = x * 4 + 2;
          p_score = true;
          break;
        case MASH(0, 0, 1, 0, 0):
          rngs.emplace_back(ImageRange(x * 4 + 1, x * 4 + 2));
          pstart = x * 4 + 1;
          break;
        case MASH(0, 0, 1, 0, 1):
          rngs.emplace_back(ImageRange(x * 4 + 1, x * 4 + 2));
          pstart = x * 4 + 3;
          p_score = true;
          break;
        case MASH(0, 0, 1, 1, 0):
          rngs.emplace_back(ImageRange(x * 4 + 1, x * 4 + 3));
          pstart = x * 4 + 1;
          break;
        case MASH(0, 0, 1, 1, 1):
          pstart = x * 4 + 1;
          p_score = true;
          break;
        case MASH(0, 1, 0, 0, 0):
          rngs.emplace_back(ImageRange(x * 4 + 0, x * 4 + 1));
          pstart = x * 4 + 0;
          break;
        case MASH(0, 1, 0, 0, 1):
          rngs.emplace_back(ImageRange(x * 4 + 0, x * 4 + 1));
          pstart = x * 4 + 3;
          p_score = true;
          break;
        case MASH(0, 1, 0, 1, 0):
          rngs.emplace_back(ImageRange(x * 4 + 0, x * 4 + 1));
          rngs.emplace_back(ImageRange(x * 4 + 2, x * 4 + 3));
          pstart = x * 4 + 2;
          break;
        case MASH(0, 1, 0, 1, 1):
          rngs.emplace_back(ImageRange(x * 4 + 0, x * 4 + 1));
          pstart = x * 4 + 2;
          p_score = true;
          break;
        case MASH(0, 1, 1, 0, 0):
          rngs.emplace_back(ImageRange(x * 4 + 0, x * 4 + 2));
          pstart = x * 4 + 0;
          break;
        case MASH(0, 1, 1, 0, 1):
          rngs.emplace_back(ImageRange(x * 4 + 0, x * 4 + 2));
          pstart = x * 4 + 3;
          p_score = true;
          break;
        case MASH(0, 1, 1, 1, 0):
          rngs.emplace_back(ImageRange(x * 4 + 0, x * 4 + 3));
          pstart = x * 4 + 0;
          break;
        case MASH(0, 1, 1, 1, 1):
          pstart = x * 4 + 0;
          p_score = true;
          break;
        case MASH(1, 0, 0, 0, 0):
          rngs.emplace_back(ImageRange(pstart, x * 4 + 0));
          p_score = false;
          break;
        case MASH(1, 0, 0, 0, 1):
          rngs.emplace_back(ImageRange(pstart, x * 4 + 0));
          pstart = x * 4 + 3;
          break;
        case MASH(1, 0, 0, 1, 0):
          rngs.emplace_back(ImageRange(pstart, x * 4 + 0));
          rngs.emplace_back(ImageRange(x * 4 + 2, x * 4 + 3));
          pstart = x * 4 + 2;
          p_score = false;
          break;
        case MASH(1, 0, 0, 1, 1):
          rngs.emplace_back(ImageRange(pstart, x * 4 + 0));
          pstart = x * 4 + 2;
          break;
        case MASH(1, 0, 1, 0, 0):
          rngs.emplace_back(ImageRange(pstart, x * 4 + 0));
          rngs.emplace_back(ImageRange(x * 4 + 1, x * 4 + 2));
          pstart = x * 4 + 1;
          p_score = false;
          break;
        case MASH(1, 0, 1, 0, 1):
          rngs.emplace_back(ImageRange(pstart, x * 4 + 0));
          rngs.emplace_back(ImageRange(x * 4 + 1, x * 4 + 2));
          pstart = x * 4 + 3;
          break;
        case MASH(1, 0, 1, 1, 0):
          rngs.emplace_back(ImageRange(pstart, x * 4 + 0));
          rngs.emplace_back(ImageRange(x * 4 + 1, x * 4 + 3));
          pstart = x * 4 + 1;
          p_score = false;
          break;
        case MASH(1, 0, 1, 1, 1):
          rngs.emplace_back(ImageRange(pstart, x * 4 + 0));
          pstart = x * 4 + 1;
          break;
        case MASH(1, 1, 0, 0, 0):
          rngs.emplace_back(ImageRange(pstart, x * 4 + 1));
          p_score = false;
          break;
        case MASH(1, 1, 0, 0, 1):
          rngs.emplace_back(ImageRange(pstart, x * 4 + 1));
          pstart = x * 4 + 3;
          break;
        case MASH(1, 1, 0, 1, 0):
          rngs.emplace_back(ImageRange(pstart, x * 4 + 1));
          rngs.emplace_back(ImageRange(x * 4 + 2, x * 4 + 3));
          pstart = x * 4 + 2;
          p_score = false;
          break;
        case MASH(1, 1, 0, 1, 1):
          rngs.emplace_back(ImageRange(pstart, x * 4 + 1));
          pstart = x * 4 + 2;
          break;
        case MASH(1, 1, 1, 0, 0):
          rngs.emplace_back(ImageRange(pstart, x * 4 + 2));
          p_score = false;
          break;
        case MASH(1, 1, 1, 0, 1):
          rngs.emplace_back(ImageRange(pstart, x * 4 + 2));
          pstart = x * 4 + 3;
          break;
        case MASH(1, 1, 1, 1, 0):
          rngs.emplace_back(ImageRange(pstart, x * 4 + 3));
          p_score = false;
          break;
        case MASH(1, 1, 1, 1, 1):
          break;
      }

      for (int i = 0; i < 4; ++i) {
        if ((v[i * 2] > value) != p_score) {
          if (p_score) {
            rngs.emplace_back(ImageRange(pstart, x * 4 + i));
          } else {
            pstart = x * 4 + i;
          }
          p_score = !p_score;
        }
      }
    }
    if (p_score) {
      rngs.emplace_back(ImageRange(pstart, fmt.w));
    }
    ranges.push_back(rngs);
  }
  return RangeImage(0, std::move(ranges));
}

#undef MASH

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

  // Check that our current results match possible solutions.
  aos::vision::CameraParams params0;
  params0.set_exposure(400);
  params0.set_brightness(40);
  params0.set_width(640);
  // params0.set_fps(10);
  params0.set_height(480);

  ::std::unique_ptr<CameraStream> camera0(
      new CameraStream(params0, "/dev/video0"));
  camera0->set_on_frame([&](DataRef data,
                            monotonic_clock::time_point monotonic_now) {
    aos::vision::ImageFormat fmt{640, 480};
    aos::vision::BlobList imgs =
        aos::vision::FindBlobs(::DoThresholdYUYV(fmt, data.data(), 120));
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
      results.emplace_back(finder_.ProcessTargetToResult(target, verbose));
    }

    results = finder_.FilterResults(results);

    // TODO: Select top 3 (randomly?)

    frc971::jevois::Frame frame{};

    for (size_t i = 0; i < results.size() && i < frame.targets.max_size();
         ++i) {
      const auto &result = results[i].extrinsics;
      frame.targets.push_back(frc971::jevois::Target{
          static_cast<float>(result.z), static_cast<float>(result.y),
          static_cast<float>(result.r2), static_cast<float>(result.r1)});
    }

    frame.age = std::chrono::duration_cast<frc971::jevois::camera_duration>(
        aos::monotonic_clock::now() - monotonic_now);

    // If we succeed in writing our delimiter, then write out the rest of the
    // frame. If not, no point in continuing.
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
