#include "y2022/vision/ball_color.h"

#include <chrono>
#include <cmath>
#include <thread>

#include "glog/logging.h"
#include "opencv2/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>

#include "aos/events/event_loop.h"
#include "aos/events/shm_event_loop.h"
#include "frc971/input/joystick_state_generated.h"
#include "frc971/vision/vision_generated.h"

namespace y2022 {
namespace vision {

namespace {
cv::Rect ArrayToRect(const std::array<int, 4> &values) {
  return cv::Rect{values[0], values[1], values[2], values[3]};
}
}  // namespace

BallColorDetector::BallColorDetector(
    aos::EventLoop *event_loop, std::shared_ptr<const constants::Values> values)
    : ball_color_sender_(event_loop->MakeSender<BallColor>("/superstructure")),
      values_(values),
      reference_red_(ArrayToRect(values_->ball_color.reference_red)),
      reference_blue_(ArrayToRect(values_->ball_color.reference_blue)),
      ball_location_(ArrayToRect(values_->ball_color.ball_location)) {
  event_loop->MakeWatcher("/camera", [this](const CameraImage &camera_image) {
    this->ProcessImage(camera_image);
  });
}

void BallColorDetector::ProcessImage(const CameraImage &image) {
  cv::Mat image_color_mat(cv::Size(image.cols(), image.rows()), CV_8UC2,
                          (void *)image.data()->data());
  cv::Mat image_mat(cv::Size(image.cols(), image.rows()), CV_8UC3);
  cv::cvtColor(image_color_mat, image_mat, cv::COLOR_YUV2BGR_YUYV);

  aos::Alliance detected_color = DetectColor(image_mat);

  auto builder = ball_color_sender_.MakeBuilder();
  auto ball_color_builder = builder.MakeBuilder<BallColor>();
  ball_color_builder.add_ball_color(detected_color);
  builder.CheckOk(builder.Send(ball_color_builder.Finish()));
}

aos::Alliance BallColorDetector::DetectColor(cv::Mat image) {
  cv::Mat hsv(cv::Size(image.cols, image.rows), CV_8UC3);

  cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

  // Look at 3 chunks of the image
  cv::Mat reference_red_mat = BallColorDetector::SubImage(hsv, reference_red_);

  cv::Mat reference_blue_mat =
      BallColorDetector::SubImage(hsv, reference_blue_);
  cv::Mat ball_location_mat = BallColorDetector::SubImage(hsv, ball_location_);

  // OpenCV HSV hues go from [0 to 179]
  // Average the average color of each patch in both directions
  // Rejecting pixels that have too low saturation or to bright or dark value
  // And dealing with the wrapping of the red hues by shifting the wrap to be
  // around 90 instead of 180. 90 is a color we don't care about.
  double red = BallColorDetector::mean_hue(reference_red_mat);
  double blue = BallColorDetector::mean_hue(reference_blue_mat);
  double ball = BallColorDetector::mean_hue(ball_location_mat);

  // Just look at the hue values for distance
  const double distance_to_blue = std::abs(ball - blue);
  const double distance_to_red = std::abs(ball - red);

  VLOG(1) << "\n"
          << "Red: " << red << " deg\n"
          << "Blue: " << blue << " deg\n"
          << "Ball: " << ball << " deg\n"
          << "distance to blue: " << distance_to_blue << " "
          << "distance_to_red: " << distance_to_red;

  // Is the ball location close enough to being the same hue as the blue
  // reference or the red reference?

  if (distance_to_blue < distance_to_red &&
      distance_to_blue < kMaxHueDistance) {
    return aos::Alliance::kBlue;
  } else if (distance_to_red < distance_to_blue &&
             distance_to_red < kMaxHueDistance) {
    return aos::Alliance::kRed;
  }

  return aos::Alliance::kInvalid;
}

cv::Mat BallColorDetector::SubImage(cv::Mat image, cv::Rect location) {
  cv::Rect new_location = BallColorDetector::RescaleRect(
      image, location, BallColorDetector::kMeasurementsImageSize());
  return image(new_location);
}

// Handle varying size images by scaling our constants rectangles
cv::Rect BallColorDetector::RescaleRect(cv::Mat image, cv::Rect location,
                                        cv::Size original_size) {
  const double x_scale = static_cast<double>(image.cols) / original_size.width;
  const double y_scale = static_cast<double>(image.rows) / original_size.height;

  cv::Rect new_location(location.x * x_scale, location.y * y_scale,
                        location.width * x_scale, location.height * y_scale);

  return new_location;
}

double BallColorDetector::mean_hue(cv::Mat hsv_image) {
  double num_pixels_selected = 0;
  double sum = 0;

  for (int i = 0; i < hsv_image.rows; ++i) {
    for (int j = 0; j < hsv_image.cols; ++j) {
      const cv::Vec3b &color = hsv_image.at<cv::Vec3b>(i, j);
      double value = static_cast<double>(color(2));
      double saturation = static_cast<double>(color(1));

      if (value < kMinValue || value > kMaxValue ||
          saturation < kMinSaturation) {
        continue;
      }

      // unwrap hue so that break is around 90 instead of 180
      // ex. a hue of 180 goes to 0, a hue of 120 goes to -60
      // but there's still a break around 90 where it will be either +- 90
      // depending on which side it's on
      double hue = static_cast<double>(color(0));
      if (hue > 90) {
        hue = hue - 180;
      }

      num_pixels_selected++;
      sum += hue;
    }
  }

  return sum / num_pixels_selected;
}

}  // namespace vision
}  // namespace y2022
