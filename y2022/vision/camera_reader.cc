#include "y2022/vision/camera_reader.h"

#include <math.h>

#include <opencv2/imgproc.hpp>

#include "aos/events/event_loop.h"
#include "aos/flatbuffer_merge.h"
#include "aos/network/team_number.h"
#include "frc971/vision/v4l2_reader.h"
#include "frc971/vision/vision_generated.h"
#include "y2020/vision/sift/sift_generated.h"
#include "y2020/vision/sift/sift_training_generated.h"
#include "y2020/vision/tools/python_code/sift_training_data.h"

namespace y2022 {
namespace vision {

using namespace frc971::vision;

const sift::CameraCalibration *CameraReader::FindCameraCalibration() const {
  const std::string_view node_name = event_loop_->node()->name()->string_view();
  const int team_number = aos::network::GetTeamNumber();
  for (const sift::CameraCalibration *candidate :
       *training_data_->camera_calibrations()) {
    if (candidate->node_name()->string_view() != node_name) {
      continue;
    }
    if (candidate->team_number() != team_number) {
      continue;
    }
    return candidate;
  }
  LOG(FATAL) << ": Failed to find camera calibration for " << node_name
             << " on " << team_number;
}

void CameraReader::ProcessImage(const CameraImage &image) {
  // Remember, we're getting YUYV images, so we start by converting to RGB

  // TOOD: Need to code this up for blob detection
  cv::Mat image_mat(image.rows(), image.cols(), CV_8U);
  CHECK(image_mat.isContinuous());
  const int number_pixels = image.rows() * image.cols();
  for (int i = 0; i < number_pixels; ++i) {
    reinterpret_cast<uint8_t *>(image_mat.data)[i] =
        image.data()->data()[i * 2];
  }

  // Now, send our two messages-- one large, with details for remote
  // debugging(features), and one smaller
  // TODO: Send blobdetection and pose results
}

void CameraReader::ReadImage() {
  if (!reader_->ReadLatestImage()) {
    read_image_timer_->Setup(event_loop_->monotonic_now() +
                             std::chrono::milliseconds(10));
    return;
  }

  ProcessImage(reader_->LatestImage());

  reader_->SendLatestImage();
  read_image_timer_->Setup(event_loop_->monotonic_now());
}

}  // namespace vision
}  // namespace y2022
