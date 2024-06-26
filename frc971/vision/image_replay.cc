#include "absl/flags/flag.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "aos/events/logging/log_reader.h"
#include "aos/events/logging/log_writer.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/logging/log_message_generated.h"
#include "frc971/vision/vision_generated.h"

ABSL_FLAG(std::string, node, "orin1", "The node to view the log from");
ABSL_FLAG(std::string, channel, "/camera0", "The channel to view the log from");

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  // open logfiles
  aos::logger::LogReader reader(
      aos::logger::SortParts(aos::logger::FindLogs(argc, argv)));

  aos::SimulatedEventLoopFactory factory(reader.configuration());
  reader.Register(&factory);

  aos::NodeEventLoopFactory *node =
      factory.GetNodeEventLoopFactory(absl::GetFlag(FLAGS_node));

  std::unique_ptr<aos::EventLoop> image_loop = node->MakeEventLoop("image");
  image_loop->MakeWatcher(
      "/" + absl::GetFlag(FLAGS_node) + "/" + absl::GetFlag(FLAGS_channel),
      [](const frc971::vision::CameraImage &msg) {
        cv::Mat color_image(cv::Size(msg.cols(), msg.rows()), CV_8UC2,
                            (void *)msg.data()->data());

        cv::Mat bgr(color_image.size(), CV_8UC3);
        cv::cvtColor(color_image, bgr, cv::COLOR_YUV2BGR_YUYV);

        cv::imshow("Replay", bgr);
        cv::waitKey(1);
      });

  factory.Run();

  reader.Deregister();

  return 0;
}
