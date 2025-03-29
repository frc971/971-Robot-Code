#ifndef Y2025_FRAME_LOGGER_H_
#define Y2025_FRAME_LOGGER_H_

#include <string>

#include <opencv2/opencv.hpp>

#include "aos/events/event_loop.h"
#include "aos/util/filesystem_generated.h"
#include "frc971/vision/vision_generated.h"

namespace y2025::vision {

using namespace frc971::vision;

class FrameLogger {
 public:
  FrameLogger(aos::EventLoop *event_loop);

  void LogImage(const std::string channel_name, const CameraImage *image);

 private:
  aos::Fetcher<frc971::vision::CameraImage> camera_zero_fetcher_;
  aos::Fetcher<frc971::vision::CameraImage> camera_one_fetcher_;
  std::unique_ptr<aos::PhasedLoopHandler> phased_loop_handle_;
  aos::Fetcher<aos::util::FilesystemStatus> filesystem_status_;
  std::string last_logged_ = "";
  int index_;
};

}  // namespace y2025::vision

#endif
