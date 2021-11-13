#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "y2020/vision/camera_reader.h"

// config used to allow running camera_reader independently.  E.g.,
// bazel run //y2020/vision:camera_reader -- --config y2020/config.json
//   --override_hostname pi-7971-1  --ignore_timestamps true
DEFINE_string(config, "config.json", "Path to the config file to use.");
namespace frc971 {
namespace vision {
namespace {

void CameraReaderMain() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  const aos::FlatbufferSpan<sift::TrainingData> training_data(
      SiftTrainingData());
  CHECK(training_data.Verify());

  const auto index_params = cv::makePtr<cv::flann::IndexParams>();
  index_params->setAlgorithm(cvflann::FLANN_INDEX_KDTREE);
  index_params->setInt("trees", 5);
  const auto search_params =
      cv::makePtr<cv::flann::SearchParams>(/* checks */ 50);
  cv::FlannBasedMatcher matcher(index_params, search_params);

  aos::ShmEventLoop event_loop(&config.message());

  // First, log the data for future reference.
  {
    aos::Sender<sift::TrainingData> training_data_sender =
        event_loop.MakeSender<sift::TrainingData>("/camera");
    CHECK_EQ(training_data_sender.Send(training_data),
             aos::RawSender::Error::kOk);
  }

  V4L2Reader v4l2_reader(&event_loop, "/dev/video0");
  CameraReader camera_reader(&event_loop, &training_data.message(),
                             &v4l2_reader, index_params, search_params);

  event_loop.Run();
}

}  // namespace
}  // namespace vision
}  // namespace frc971

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  frc971::vision::CameraReaderMain();
}
