#ifndef Y2022_VISION_BALL_COLOR_H_
#define Y2022_VISION_BALL_COLOR_H_

#include <opencv2/imgproc.hpp>

#include "aos/events/shm_event_loop.h"
#include "frc971/input/joystick_state_generated.h"
#include "frc971/vision/vision_generated.h"
#include "y2022/constants.h"
#include "y2022/vision/ball_color_generated.h"

namespace y2022 {
namespace vision {

using namespace frc971::vision;

// Takes in camera images and detects what color the loaded ball is
// Does not detect if there is a ball, and will output bad measurements in
// the case that that there is not a ball.
class BallColorDetector {
 public:
  // The size image that the reference rectangles were measure with
  // These constants will be scaled if the image sent is not the same size
  static const cv::Size kMeasurementsImageSize() { return {640, 480}; }

  // Constants used to filter out pixels that don't have good color information
  static constexpr double kMinSaturation = 128;
  static constexpr double kMinValue = 25;
  static constexpr double kMaxValue = 230;

  static constexpr double kMaxHueDistance = 10;

  BallColorDetector(aos::EventLoop *event_loop,
                    std::shared_ptr<const constants::Values> values);

  void ProcessImage(const CameraImage &camera_image);

  // We look at three parts of the image: two reference locations where there
  // will be red and blue markers that should match the ball, and then the
  // location in the catapult where we expect to see the ball. We then compute
  // the average hue of each patch but discard pixels that we deem not colorful
  // enough. Then we decide whether the ball color looks close enough to either
  // of the reference colors. If no good color is detected, outputs kInvalid.
  aos::Alliance DetectColor(cv::Mat image);

  cv::Rect reference_red() const { return reference_red_; }
  cv::Rect reference_blue() const { return reference_blue_; }
  cv::Rect ball_location() const { return ball_location_; }

  static cv::Mat SubImage(cv::Mat image, cv::Rect location);

  static cv::Rect RescaleRect(cv::Mat image, cv::Rect location,
                              cv::Size original_size);
  static double mean_hue(cv::Mat hsv_image);

 private:
  aos::Sender<BallColor> ball_color_sender_;

  std::shared_ptr<const constants::Values> values_;

  const cv::Rect reference_red_;
  const cv::Rect reference_blue_;
  const cv::Rect ball_location_;
};
}  // namespace vision
}  // namespace y2022
#endif
