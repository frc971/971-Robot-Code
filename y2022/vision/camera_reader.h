#ifndef Y2022_VISION_CAMERA_READER_H_
#define Y2022_VISION_CAMERA_READER_H_

#include <math.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "aos/events/shm_event_loop.h"
#include "aos/flatbuffer_merge.h"
#include "aos/network/team_number.h"
#include "frc971/vision/v4l2_reader.h"
#include "frc971/vision/vision_generated.h"
#include "y2022/localizer/localizer_output_generated.h"
#include "y2022/vision/calibration_data.h"
#include "y2022/vision/calibration_generated.h"
#include "y2022/vision/gpio.h"
#include "y2022/vision/target_estimate_generated.h"
#include "y2022/vision/target_estimator.h"

namespace y2022 {
namespace vision {

using namespace frc971::vision;
using frc971::controls::LedOutput;

// TODO<jim>: Probably need to break out LED control to separate process
class CameraReader {
 public:
  CameraReader(aos::ShmEventLoop *event_loop,
               const calibration::CalibrationData *calibration_data,
               V4L2Reader *reader)
      : event_loop_(event_loop),
        calibration_data_(calibration_data),
        camera_calibration_(FindCameraCalibration()),
        reader_(reader),
        image_sender_(event_loop->MakeSender<CameraImage>("/camera")),
        target_estimator_(CameraIntrinsics(), CameraExtrinsics()),
        target_estimate_sender_(
            event_loop->MakeSender<TargetEstimate>("/camera")),
        localizer_output_fetcher_(
            event_loop->MakeFetcher<frc971::controls::LocalizerOutput>(
                "/localizer")),
        read_image_timer_(event_loop->AddTimer([this]() { ReadImage(); })),
        gpio_imu_pin_(GPIOControl(GPIO_PIN_SCLK_IMU, kGPIOIn)),
        gpio_pwm_control_(GPIOPWMControl(GPIO_PIN_SCK_PWM, duty_cycle_)),
        gpio_disable_control_(
            GPIOControl(GPIO_PIN_MOSI_DISABLE, kGPIOOut, kGPIOLow)) {
    event_loop->OnRun(
        [this]() { read_image_timer_->Setup(event_loop_->monotonic_now()); });
  }

  void SetDutyCycle(double duty_cycle) {
    duty_cycle_ = duty_cycle;
    gpio_pwm_control_.setPWMDutyCycle(duty_cycle_);
  }

  double GetDutyCycle() { return duty_cycle_; }

 private:
  const calibration::CameraCalibration *FindCameraCalibration() const;

  // Processes an image (including sending the results).
  void ProcessImage(cv::Mat image_mat_distorted,
                    int64_t image_monotonic_timestamp_ns);

  // Reads an image, and then performs all of our processing on it.
  void ReadImage();

  cv::Mat CameraIntrinsics() const {
    cv::Mat result(3, 3, CV_32F,
                   const_cast<void *>(static_cast<const void *>(
                       camera_calibration_->intrinsics()->data())));
    result.convertTo(result, CV_64F);
    CHECK_EQ(result.total(), camera_calibration_->intrinsics()->size());
    return result;
  }

  cv::Mat CameraExtrinsics() const {
    // TODO(james): What's the principled way to handle non-z-axis turrets?
    const frc971::vision::calibration::TransformationMatrix *transform =
        camera_calibration_->has_turret_extrinsics()
            ? camera_calibration_->turret_extrinsics()
            : camera_calibration_->fixed_extrinsics();

    cv::Mat result(4, 4, CV_32F,
                   const_cast<void *>(
                       static_cast<const void *>(transform->data()->data())));
    result.convertTo(result, CV_64F);
    CHECK_EQ(result.total(), transform->data()->size());
    return result;
  }

  cv::Mat CameraDistCoeffs() const {
    const cv::Mat result(5, 1, CV_32F,
                         const_cast<void *>(static_cast<const void *>(
                             camera_calibration_->dist_coeffs()->data())));
    result.convertTo(result, CV_64F);
    CHECK_EQ(result.total(), camera_calibration_->dist_coeffs()->size());
    return result;
  }

  aos::ShmEventLoop *const event_loop_;
  const calibration::CalibrationData *const calibration_data_;
  const calibration::CameraCalibration *const camera_calibration_;
  V4L2Reader *const reader_;
  aos::Sender<CameraImage> image_sender_;
  TargetEstimator target_estimator_;
  aos::Sender<TargetEstimate> target_estimate_sender_;

  LedOutput prev_led_output_ = LedOutput::ON;
  aos::Fetcher<frc971::controls::LocalizerOutput> localizer_output_fetcher_;

  // We schedule this immediately to read an image. Having it on a timer
  // means other things can run on the event loop in between.
  aos::TimerHandler *const read_image_timer_;

  double duty_cycle_ = 0.0;
  GPIOControl gpio_imu_pin_;
  GPIOPWMControl gpio_pwm_control_;
  GPIOControl gpio_disable_control_;
};

}  // namespace vision
}  // namespace y2022
#endif  // Y2022_VISION_CAMERA_READER_H_
