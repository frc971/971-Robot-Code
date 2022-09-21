#include "y2022/vision/ball_color.h"

#include <cmath>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "aos/events/simulated_event_loop.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/testing/test_logging.h"
#include "glog/logging.h"
#include "gtest/gtest.h"
#include "y2022/constants.h"

DEFINE_string(output_folder, "",
              "If set, logs all channels to the provided logfile.");

namespace y2022::vision::testing {

class BallColorTest : public ::testing::Test {
 public:
  BallColorTest()
      : config_(aos::configuration::ReadConfig("y2022/aos_config.json")),
        event_loop_factory_(&config_.message()),
        logger_pi_(aos::configuration::GetNode(
            event_loop_factory_.configuration(), "logger")),
        roborio_(aos::configuration::GetNode(
            event_loop_factory_.configuration(), "roborio")),
        camera_event_loop_(
            event_loop_factory_.MakeEventLoop("Camera", logger_pi_)),
        color_detector_event_loop_(event_loop_factory_.MakeEventLoop(
            "Ball color detector", logger_pi_)),
        superstructure_event_loop_(
            event_loop_factory_.MakeEventLoop("Superstructure", roborio_)),
        ball_color_fetcher_(superstructure_event_loop_->MakeFetcher<BallColor>(
            "/superstructure")),
        image_sender_(camera_event_loop_->MakeSender<CameraImage>("/camera")),
        detector_(color_detector_event_loop_.get(),
                  std::make_shared<const constants::Values>(
                      constants::MakeValues(constants::kCompTeamNumber))) {}

  // copied from camera_reader.cc
  void SendImage(cv::Mat bgr_image) {
    cv::Mat image_color_mat;
    cv::cvtColor(bgr_image, image_color_mat, cv::COLOR_BGR2YUV);

    // Convert YUV (3 channels) to YUYV (stacked format)
    std::vector<uint8_t> yuyv;
    for (int i = 0; i < image_color_mat.rows; i++) {
      for (int j = 0; j < image_color_mat.cols; j++) {
        // Always push a Y value
        yuyv.emplace_back(image_color_mat.at<cv::Vec3b>(i, j)[0]);
        if ((j % 2) == 0) {
          // If column # is even, push a U value.
          yuyv.emplace_back(image_color_mat.at<cv::Vec3b>(i, j)[1]);
        } else {
          // If column # is odd, push a V value.
          yuyv.emplace_back(image_color_mat.at<cv::Vec3b>(i, j)[2]);
        }
      }
    }

    CHECK_EQ(static_cast<int>(yuyv.size()),
             image_color_mat.rows * image_color_mat.cols * 2);

    auto builder = image_sender_.MakeBuilder();
    auto image_offset = builder.fbb()->CreateVector(yuyv);
    auto image_builder = builder.MakeBuilder<CameraImage>();

    int64_t timestamp = aos::monotonic_clock::now().time_since_epoch().count();

    image_builder.add_rows(image_color_mat.rows);
    image_builder.add_cols(image_color_mat.cols);
    image_builder.add_data(image_offset);
    image_builder.add_monotonic_timestamp_ns(timestamp);

    builder.CheckOk(builder.Send(image_builder.Finish()));
  }

  aos::FlatbufferDetachedBuffer<aos::Configuration> config_;
  aos::SimulatedEventLoopFactory event_loop_factory_;
  const aos::Node *const logger_pi_;
  const aos::Node *const roborio_;
  ::std::unique_ptr<::aos::EventLoop> camera_event_loop_;
  ::std::unique_ptr<::aos::EventLoop> color_detector_event_loop_;
  ::std::unique_ptr<::aos::EventLoop> superstructure_event_loop_;
  aos::Fetcher<BallColor> ball_color_fetcher_;
  aos::Sender<CameraImage> image_sender_;

  BallColorDetector detector_;
};

TEST_F(BallColorTest, DetectColorFromTestImage) {
  cv::Mat bgr_image =
      cv::imread("y2022/vision/test_ball_color_image.jpg", cv::IMREAD_COLOR);

  ASSERT_TRUE(bgr_image.data != nullptr);

  aos::Alliance detected_color = detector_.DetectColor(bgr_image);

  EXPECT_EQ(detected_color, aos::Alliance::kRed);
}

TEST_F(BallColorTest, DetectColorFromTestImageInEventLoop) {
  cv::Mat bgr_image =
      cv::imread("y2022/vision/test_ball_color_image.jpg", cv::IMREAD_COLOR);
  ASSERT_TRUE(bgr_image.data != nullptr);

  camera_event_loop_->OnRun([this, bgr_image]() { SendImage(bgr_image); });

  event_loop_factory_.RunFor(std::chrono::milliseconds(5));

  ASSERT_TRUE(ball_color_fetcher_.Fetch());

  EXPECT_TRUE(ball_color_fetcher_->has_ball_color());
  EXPECT_EQ(ball_color_fetcher_->ball_color(), aos::Alliance::kRed);
}

TEST_F(BallColorTest, TestRescaling) {
  cv::Mat mat(cv::Size(320, 240), CV_8UC3);
  cv::Rect new_rect = BallColorDetector::RescaleRect(
      mat, cv::Rect(30, 30, 30, 30), cv::Size(1920, 1080));

  EXPECT_EQ(new_rect, cv::Rect(5, 6, 5, 6));
}

TEST_F(BallColorTest, TestAreas) {
  cv::Mat bgr_image =
      cv::imread("y2022/vision/test_ball_color_image.jpg", cv::IMREAD_COLOR);
  ASSERT_TRUE(bgr_image.data != nullptr);

  cv::Rect reference_red = BallColorDetector::RescaleRect(
      bgr_image, detector_.reference_red(),
      BallColorDetector::kMeasurementsImageSize());
  cv::Rect reference_blue = BallColorDetector::RescaleRect(
      bgr_image, detector_.reference_blue(),
      BallColorDetector::kMeasurementsImageSize());
  cv::Rect ball_location = BallColorDetector::RescaleRect(
      bgr_image, detector_.ball_location(),
      BallColorDetector::kMeasurementsImageSize());

  cv::rectangle(bgr_image, reference_red, cv::Scalar(0, 0, 255));
  cv::rectangle(bgr_image, reference_blue, cv::Scalar(255, 0, 0));
  cv::rectangle(bgr_image, ball_location, cv::Scalar(0, 255, 0));

  cv::imwrite("/tmp/rectangles.jpg", bgr_image);
}

}  // namespace y2022::vision::testing
