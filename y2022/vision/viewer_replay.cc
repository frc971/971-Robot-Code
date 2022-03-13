#include "aos/events/logging/log_reader.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/init.h"
#include "frc971/vision/vision_generated.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "y2022/vision/blob_detector.h"

DEFINE_string(node, "pi1", "Node name to replay.");
DEFINE_string(image_save_prefix, "/tmp/img",
              "Prefix to use for saving images from the logfile.");
DEFINE_bool(display, false, "If true, display the images with a timeout.");
DEFINE_bool(detected_only, false,
            "If true, only write images which had blobs (unfiltered) detected");
DEFINE_bool(filtered_only, false,
            "If true, only write images which had blobs (filtered) detected");

namespace y2022 {
namespace vision {
namespace {

void ViewerMain(int argc, char *argv[]) {
  std::vector<std::string> unsorted_logfiles =
      aos::logger::FindLogs(argc, argv);

  // Open logfiles
  aos::logger::LogReader reader(aos::logger::SortParts(unsorted_logfiles));
  reader.Register();
  const aos::Node *node = nullptr;
  if (aos::configuration::MultiNode(reader.configuration())) {
    node = aos::configuration::GetNode(reader.configuration(), FLAGS_node);
  }
  std::unique_ptr<aos::EventLoop> event_loop =
      reader.event_loop_factory()->MakeEventLoop("player", node);

  int image_count = 0;
  event_loop->MakeWatcher(
      "/camera/decimated",
      [&image_count](const frc971::vision::CameraImage &image) {
        // Create color image:
        cv::Mat image_color_mat(cv::Size(image.cols(), image.rows()), CV_8UC2,
                                (void *)image.data()->data());
        cv::Mat image_mat(cv::Size(image.cols(), image.rows()), CV_8UC3);
        cv::cvtColor(image_color_mat, image_mat, cv::COLOR_YUV2BGR_YUYV);

        bool use_image = true;
        if (FLAGS_detected_only || FLAGS_filtered_only) {
          BlobDetector::BlobResult blob_result;
          BlobDetector::ExtractBlobs(image_mat, &blob_result);

          use_image =
              ((FLAGS_filtered_only ? blob_result.filtered_blobs.size()
                                    : blob_result.unfiltered_blobs.size()) > 0);
        }
        if (use_image) {
          if (!FLAGS_image_save_prefix.empty()) {
            cv::imwrite(FLAGS_image_save_prefix +
                            std::to_string(image_count++) + ".png",
                        image_mat);
          }
          if (FLAGS_display) {
            cv::imshow("Display", image_mat);
            cv::waitKey(FLAGS_detected_only || FLAGS_filtered_only ? 10 : 1);
          }
        }
      });

  reader.event_loop_factory()->Run();
}

}  // namespace
}  // namespace vision
}  // namespace y2022

// Quick and lightweight viewer for image logs
int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  y2022::vision::ViewerMain(argc, argv);
}
