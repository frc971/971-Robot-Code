#ifndef Y2022_VISION_BALL_COLOR_H_
#define Y2022_VISION_BALL_COLOR_H_

#include <opencv2/imgproc.hpp>

#include "aos/events/shm_event_loop.h"
#include "frc971/input/joystick_state_generated.h"
#include "frc971/vision/vision_generated.h"
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
  static const cv::Size kMeasurementsImageSize() { return {640, 480}; };
  static const cv::Rect kReferenceRed() { return {440, 150, 50, 130}; };
  static const cv::Rect kReferenceBlue() { return {440, 350, 30, 100}; };
  static const cv::Rect kBallLocation() { return {100, 400, 140, 50}; };

  // Constants used to filter out pixels that don't have good color information
  static constexpr double kMinSaturation = 128;
  static constexpr double kMinValue = 25;
  static constexpr double kMaxValue = 230;

  static constexpr double kMaxHueDistance = 10;

  BallColorDetector(aos::EventLoop *event_loop);

  void ProcessImage(const CameraImage &camera_image);

  // We look at three parts of the image: two reference locations where there
  // will be red and blue markers that should match the ball, and then the
  // location in the catapult where we expect to see the ball. We then compute
  // the average hue of each patch but discard pixels that we deem not colorful
  // enough. Then we decide whether the ball color looks close enough to either
  // of the reference colors. If no good color is detected, outputs kInvalid.
  static aos::Alliance DetectColor(cv::Mat image);

  static cv::Mat SubImage(cv::Mat image, cv::Rect location);

  static cv::Rect RescaleRect(cv::Mat image, cv::Rect location,
                              cv::Size original_size);
  static double mean_hue(cv::Mat hsv_image);

 private:
  aos::Sender<BallColor> ball_color_sender_;
};
}  // namespace vision
}  // namespace y2022
#endif
