#ifndef Y2022_VISION_CAMERA_READER_H_
#define Y2022_VISION_CAMERA_READER_H_

#include <math.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include "aos/events/event_loop.h"
#include "aos/flatbuffer_merge.h"
#include "aos/network/team_number.h"
#include "frc971/vision/v4l2_reader.h"
#include "frc971/vision/vision_generated.h"
#include "y2020/vision/sift/sift_generated.h"
#include "y2020/vision/sift/sift_training_generated.h"
#include "y2020/vision/tools/python_code/sift_training_data.h"
#include "y2022/vision/target_estimate_generated.h"

namespace y2022 {
namespace vision {

using namespace frc971::vision;

// TODO<Jim/Milind>: Need to add in senders to send out the blob data/stats

class CameraReader {
 public:
  CameraReader(aos::EventLoop *event_loop,
               const sift::TrainingData *training_data, V4L2Reader *reader)
      : event_loop_(event_loop),
        training_data_(training_data),
        camera_calibration_(FindCameraCalibration()),
        reader_(reader),
        image_sender_(event_loop->MakeSender<CameraImage>("/camera")),
        target_estimate_sender_(
            event_loop->MakeSender<TargetEstimate>("/camera")),
        read_image_timer_(event_loop->AddTimer([this]() { ReadImage(); })) {
    event_loop->OnRun(
        [this]() { read_image_timer_->Setup(event_loop_->monotonic_now()); });
  }

 private:
  const sift::CameraCalibration *FindCameraCalibration() const;

  // Processes an image (including sending the results).
  void ProcessImage(const CameraImage &image);

  // Reads an image, and then performs all of our processing on it.
  void ReadImage();

  cv::Mat CameraIntrinsics() const {
    const cv::Mat result(3, 3, CV_32F,
                         const_cast<void *>(static_cast<const void *>(
                             camera_calibration_->intrinsics()->data())));
    CHECK_EQ(result.total(), camera_calibration_->intrinsics()->size());
    return result;
  }

  cv::Mat CameraExtrinsics() const {
    const cv::Mat result(
        4, 4, CV_32F,
        const_cast<void *>(static_cast<const void *>(
            camera_calibration_->fixed_extrinsics()->data()->data())));
    CHECK_EQ(result.total(),
             camera_calibration_->fixed_extrinsics()->data()->size());
    return result;
  }

  cv::Mat CameraDistCoeffs() const {
    const cv::Mat result(5, 1, CV_32F,
                         const_cast<void *>(static_cast<const void *>(
                             camera_calibration_->dist_coeffs()->data())));
    CHECK_EQ(result.total(), camera_calibration_->dist_coeffs()->size());
    return result;
  }

  aos::EventLoop *const event_loop_;
  const sift::TrainingData *const training_data_;
  const sift::CameraCalibration *const camera_calibration_;
  V4L2Reader *const reader_;
  aos::Sender<CameraImage> image_sender_;
  aos::Sender<TargetEstimate> target_estimate_sender_;

  // We schedule this immediately to read an image. Having it on a timer
  // means other things can run on the event loop in between.
  aos::TimerHandler *const read_image_timer_;
};

}  // namespace vision
}  // namespace y2022
#endif  // Y2022_VISION_CAMERA_READER_H_
