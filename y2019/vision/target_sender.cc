#include <fstream>

#include "aos/logging/implementations.h"
#include "aos/logging/logging.h"
#include "aos/vision/blob/codec.h"
#include "aos/vision/blob/find_blob.h"
#include "aos/vision/events/socket_types.h"
#include "aos/vision/events/udp.h"
#include "aos/vision/image/image_stream.h"
#include "aos/vision/image/reader.h"

#include "y2019/jevois/serial.h"
#include "y2019/vision/target_finder.h"

using ::aos::events::DataSocket;
using ::aos::events::RXUdpSocket;
using ::aos::events::TCPServer;
using ::aos::vision::DataRef;
using ::aos::vision::Int32Codec;
using ::aos::monotonic_clock;
using ::y2019::jevois::open_via_terminos;
using aos::vision::Segment;

class CameraStream : public ::aos::vision::ImageStreamEvent {
 public:
  CameraStream(::aos::vision::CameraParams params, const ::std::string &fname)
      : ImageStreamEvent(fname, params) {}

  void ProcessImage(DataRef data, monotonic_clock::time_point monotonic_now) {
    LOG(INFO, "got frame: %d\n", (int)data.size());

    static unsigned i = 0;

    /*
          std::ofstream ofs(std::string("/jevois/data/debug_viewer_jpeg_") +
                                std::to_string(i) + ".yuyv",
                            std::ofstream::out);
          ofs << data;
          ofs.close();
          */
    if (on_frame) on_frame(data, monotonic_now);
    ++i;

    if (i == 200) exit(-1);
  }

  std::function<void(DataRef, monotonic_clock::time_point)> on_frame;
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

int main(int argc, char **argv) {
  (void)argc;
  (void)argv;
  using namespace y2019::vision;
  // gflags::ParseCommandLineFlags(&argc, &argv, false);
  ::aos::logging::Init();
  ::aos::logging::AddImplementation(
      new ::aos::logging::StreamLogImplementation(stderr));

  int itsDev = open_terminos("/dev/ttyS0");
  dup2(itsDev, 1);
  dup2(itsDev, 2);

  TargetFinder finder_;

  // Check that our current results match possible solutions.
  aos::vision::CameraParams params0;
  params0.set_exposure(0);
  params0.set_brightness(40);
  params0.set_width(640);
  // params0.set_fps(10);
  params0.set_height(480);

  ::std::unique_ptr<CameraStream> camera0(
      new CameraStream(params0, "/dev/video0"));
  camera0->on_frame = [&](DataRef data,
                          monotonic_clock::time_point /*monotonic_now*/) {
    aos::vision::ImageFormat fmt{640, 480};
    aos::vision::BlobList imgs = aos::vision::FindBlobs(
        aos::vision::DoThresholdYUYV(fmt, data.data(), 120));
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
  };

  aos::events::EpollLoop loop;

  for (int i = 0; i < 100; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    camera0->ReadEvent();
  }

  // TODO: Fix event loop on jevois:
  // loop.Add(camera0.get());
  // loop.Run();
}
