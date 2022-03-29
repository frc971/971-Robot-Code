#include "y2022/vision/camera_reader.h"

#include <chrono>
#include <cmath>
#include <thread>

#include "aos/events/event_loop.h"
#include "aos/events/shm_event_loop.h"
#include "aos/flatbuffer_merge.h"
#include "aos/network/team_number.h"
#include "frc971/vision/v4l2_reader.h"
#include "frc971/vision/vision_generated.h"
#include "opencv2/imgproc.hpp"
#include "y2022/vision/blob_detector.h"
#include "y2022/vision/calibration_generated.h"

DEFINE_string(image_png, "", "A set of PNG images");

namespace y2022 {
namespace vision {

using namespace frc971::vision;

const calibration::CameraCalibration *CameraReader::FindCameraCalibration()
    const {
  const std::string_view node_name = event_loop_->node()->name()->string_view();
  const int team_number = aos::network::GetTeamNumber();
  for (const calibration::CameraCalibration *candidate :
       *calibration_data_->camera_calibrations()) {
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

namespace {
flatbuffers::Offset<flatbuffers::Vector<const Point *>> CvPointsToFbs(
    const std::vector<cv::Point> &points,
    aos::Sender<TargetEstimate>::Builder *builder) {
  std::vector<Point> points_fbs;
  for (auto p : points) {
    points_fbs.push_back(Point{p.x, p.y});
  }
  return builder->fbb()->CreateVectorOfStructs(points_fbs);
}

constexpr size_t kMaxBlobsForDebug = 100;

// Converts a vector of cv::Point to PointT for the flatbuffer
flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Blob>>>
CvBlobsToFbs(const std::vector<std::vector<cv::Point>> &blobs,
             aos::Sender<TargetEstimate>::Builder *builder) {
  std::vector<flatbuffers::Offset<Blob>> blobs_fbs;
  for (auto &blob : blobs) {
    const auto points_offset = CvPointsToFbs(blob, builder);
    auto blob_builder = builder->MakeBuilder<Blob>();
    blob_builder.add_points(points_offset);
    blobs_fbs.emplace_back(blob_builder.Finish());
    if (blobs_fbs.size() == kMaxBlobsForDebug) {
      break;
    }
  }
  return builder->fbb()->CreateVector(blobs_fbs.data(), blobs_fbs.size());
}

flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<BlobStatsFbs>>>
BlobStatsToFbs(const std::vector<BlobDetector::BlobStats> blob_stats,
               aos::Sender<TargetEstimate>::Builder *builder) {
  std::vector<flatbuffers::Offset<BlobStatsFbs>> stats_fbs;
  for (auto &stats : blob_stats) {
    // Make BlobStatsFbs builder then fill each field using the BlobStats
    // struct, then you finish it and add it to stats_fbs.
    auto stats_builder = builder->MakeBuilder<BlobStatsFbs>();
    Point centroid_fbs = Point{stats.centroid.x, stats.centroid.y};
    stats_builder.add_centroid(&centroid_fbs);
    stats_builder.add_aspect_ratio(stats.aspect_ratio);
    stats_builder.add_area(stats.area);
    stats_builder.add_num_points(stats.num_points);

    auto current_stats = stats_builder.Finish();
    stats_fbs.emplace_back(current_stats);
  }
  return builder->fbb()->CreateVector(stats_fbs.data(), stats_fbs.size());
}
}  // namespace

void CameraReader::ProcessImage(cv::Mat image_mat_distorted,
                                int64_t image_monotonic_timestamp_ns) {
  cv::Mat image_mat;
  cv::undistort(image_mat_distorted, image_mat, CameraIntrinsics(),
                CameraDistCoeffs());

  BlobDetector::BlobResult blob_result;
  BlobDetector::ExtractBlobs(image_mat, &blob_result);
  auto builder = target_estimate_sender_.MakeBuilder();
  flatbuffers::Offset<BlobResultFbs> blob_result_offset;
  {
    const auto filtered_blobs_offset =
        CvBlobsToFbs(blob_result.filtered_blobs, &builder);
    const auto unfiltered_blobs_offset =
        CvBlobsToFbs(blob_result.unfiltered_blobs, &builder);
    const auto blob_stats_offset =
        BlobStatsToFbs(blob_result.blob_stats, &builder);
    const auto filtered_stats_offset =
        BlobStatsToFbs(blob_result.filtered_stats, &builder);
    const Point centroid_fbs =
        Point{blob_result.centroid.x, blob_result.centroid.y};

    auto blob_result_builder = builder.MakeBuilder<BlobResultFbs>();
    blob_result_builder.add_filtered_blobs(filtered_blobs_offset);
    blob_result_builder.add_unfiltered_blobs(unfiltered_blobs_offset);
    blob_result_builder.add_blob_stats(blob_stats_offset);
    blob_result_builder.add_filtered_stats(filtered_stats_offset);
    blob_result_builder.add_centroid(&centroid_fbs);
    blob_result_offset = blob_result_builder.Finish();
  }

  target_estimator_.Solve(blob_result.filtered_stats, std::nullopt);

  const auto camera_calibration_offset =
      aos::RecursiveCopyFlatBuffer(camera_calibration_, builder.fbb());

  const auto rotation =
      Rotation{target_estimator_.roll(), target_estimator_.pitch(),
               target_estimator_.yaw()};

  auto target_estimate_builder = builder.MakeBuilder<TargetEstimate>();

  target_estimate_builder.add_distance(target_estimator_.distance());
  target_estimate_builder.add_angle_to_target(
      target_estimator_.angle_to_target());
  target_estimate_builder.add_angle_to_camera(
      target_estimator_.angle_to_camera());
  target_estimate_builder.add_rotation_camera_hub(&rotation);
  target_estimate_builder.add_confidence(target_estimator_.confidence());
  target_estimate_builder.add_blob_result(blob_result_offset);
  target_estimate_builder.add_camera_calibration(camera_calibration_offset);
  target_estimate_builder.add_image_monotonic_timestamp_ns(
      image_monotonic_timestamp_ns);
  builder.CheckOk(builder.Send(target_estimate_builder.Finish()));
}

void CameraReader::ReadImage() {
  // Path is for reading from the Disk.
  if (FLAGS_image_png != "") {
    std::vector<cv::String> file_list;
    cv::glob(FLAGS_image_png + "/*.png", file_list, false);
    for (auto file : file_list) {
      // Sleep for 0.05 seconds in order to not reach the max number of messages
      // that can be sent in a second.
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      LOG(INFO) << "Reading file " << file;
      cv::Mat bgr_image = cv::imread(file.c_str());
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

      int64_t timestamp =
          aos::monotonic_clock::now().time_since_epoch().count();

      image_builder.add_rows(image_color_mat.rows);
      image_builder.add_cols(image_color_mat.cols);
      image_builder.add_data(image_offset);
      image_builder.add_monotonic_timestamp_ns(timestamp);

      ProcessImage(bgr_image, timestamp);
      builder.CheckOk(builder.Send(image_builder.Finish()));
    }
    event_loop_->Exit();
    return;
  }
  // If we are not reading from the disk, we read the live camera stream.
  if (!reader_->ReadLatestImage()) {
    read_image_timer_->Setup(event_loop_->monotonic_now() +
                             std::chrono::milliseconds(10));
    return;
  }

  const CameraImage &image = reader_->LatestImage();

  cv::Mat image_color_mat(cv::Size(image.cols(), image.rows()), CV_8UC2,
                          (void *)image.data()->data());
  cv::Mat image_mat(cv::Size(image.cols(), image.rows()), CV_8UC3);
  cv::cvtColor(image_color_mat, image_mat, cv::COLOR_YUV2BGR_YUYV);

  ProcessImage(image_mat, image.monotonic_timestamp_ns());

  reader_->SendLatestImage();
  read_image_timer_->Setup(event_loop_->monotonic_now());

  // Disable the LEDs based on localizer output
  if (localizer_output_fetcher_.Fetch()) {
    const auto node_name = event_loop_->node()->name()->string_view();
    const size_t pi_number =
        std::atol(node_name.substr(node_name.size() - 1).data()) - 1;

    CHECK(localizer_output_fetcher_->has_led_outputs() &&
          localizer_output_fetcher_->led_outputs()->size() > pi_number);

    const LedOutput led_output =
        localizer_output_fetcher_->led_outputs()->Get(pi_number);

    if (led_output != prev_led_output_) {
      gpio_disable_control_.GPIOWrite(led_output == LedOutput::OFF ? kGPIOHigh
                                                                   : kGPIOLow);

      prev_led_output_ = led_output;
    }
  }
}

}  // namespace vision
}  // namespace y2022
