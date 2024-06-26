#include <unistd.h>

#include <cstdlib>

#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"

#include "aos/events/logging/log_reader.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/init.h"
#include "aos/scoped/scoped_fd.h"
#include "frc971/vision/vision_generated.h"

ABSL_FLAG(std::string, channel, "/camera", "Channel name for the image.");
ABSL_FLAG(int32_t, width, 1280, "Width of the image");
ABSL_FLAG(int32_t, height, 720, "Height of the image");
ABSL_FLAG(std::string, ffmpeg_binary, "external/ffmpeg/ffmpeg",
          "The path to the ffmpeg binary");
ABSL_FLAG(std::string, output_path, "video_ripper_output.mp4",
          "The path to output the mp4 video");
ABSL_FLAG(bool, flip, true, "If true, rotate the video 180 deg.");

// Replays a log and dumps the contents of /camera frc971.vision.CameraImage
// directly to stdout.
int main(int argc, char *argv[]) {
  aos::InitGoogle(&argc, &argv);
  const std::vector<aos::logger::LogFile> logfiles =
      aos::logger::SortParts(aos::logger::FindLogs(argc, argv));
  CHECK(!logfiles.empty());

  // Start ffmpeg
  std::stringstream command;
  command << absl::GetFlag(FLAGS_ffmpeg_binary);
  command << " -framerate 30 -f rawvideo -pix_fmt yuyv422";
  command << " -s " << absl::GetFlag(FLAGS_width) << "x"
          << absl::GetFlag(FLAGS_height);
  command << " -i pipe:";
  command << " -c:v libx264 -f mp4";
  if (absl::GetFlag(FLAGS_flip)) {
    command << " -vf rotate=PI";
  }
  command << " \"" << absl::GetFlag(FLAGS_output_path) << "\"";

  FILE *stream = popen(command.str().c_str(), "w");

  const std::string replay_node = logfiles.at(0).logger_node;
  LOG(INFO) << "Replaying as \"" << replay_node << "\"";

  aos::logger::LogReader reader(logfiles, nullptr);
  aos::SimulatedEventLoopFactory factory(reader.configuration());
  reader.RegisterWithoutStarting(&factory);

  const aos::Node *node =
      (replay_node.empty() ||
       !aos::configuration::MultiNode(reader.configuration()))
          ? nullptr
          : aos::configuration::GetNode(reader.configuration(), replay_node);

  std::unique_ptr<aos::EventLoop> event_loop;
  factory.GetNodeEventLoopFactory(node)->OnStartup([&stream, &event_loop,
                                                    &reader, node]() {
    event_loop =
        reader.event_loop_factory()->MakeEventLoop("video_ripper", node);
    event_loop->MakeWatcher(
        absl::GetFlag(FLAGS_channel),
        [&stream](const frc971::vision::CameraImage &image) {
          CHECK_EQ(absl::GetFlag(FLAGS_width), image.cols())
              << "Image width needs to match the images in the logfile";
          CHECK_EQ(absl::GetFlag(FLAGS_height), image.rows())
              << "Image width needs to match the images in the logfile";

          const size_t bytes_written =
              write(fileno(stream), image.data()->data(), image.data()->size());
          PCHECK(bytes_written == image.data()->size());
        });
  });

  reader.event_loop_factory()->Run();
  reader.Deregister();

  pclose(stream);
}
