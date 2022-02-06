#include "y2022/vision/camera_reader.h"

#include <cmath>
#include <chrono>
#include <thread>

#include <opencv2/imgproc.hpp>

#include "aos/events/event_loop.h"
#include "aos/events/shm_event_loop.h"
#include "aos/flatbuffer_merge.h"
#include "aos/network/team_number.h"
#include "frc971/vision/v4l2_reader.h"
#include "frc971/vision/vision_generated.h"
#include "y2022/vision/blob_detector.h"
#include "y2022/vision/calibration_generated.h"
#include "y2022/vision/target_estimator.h"

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
// Converts a vector of cv::Point to PointT for the flatbuffer
flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Blob>>>
CvBlobsToFbs(const std::vector<std::vector<cv::Point>> &blobs,
             aos::Sender<TargetEstimate>::Builder &builder) {
  std::vector<flatbuffers::Offset<Blob>> blobs_fbs;
  for (auto &blob : blobs) {
    std::vector<Point> points_fbs;
    for (auto p : blob) {
      points_fbs.push_back(Point{p.x, p.y});
    }
    auto points_offset = builder.fbb()->CreateVectorOfStructs(points_fbs);
    auto blob_builder = builder.MakeBuilder<Blob>();
    blob_builder.add_points(points_offset);
    blobs_fbs.emplace_back(blob_builder.Finish());
  }
  return builder.fbb()->CreateVector(blobs_fbs.data(), blobs_fbs.size());
}

flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<BlobStatsFbs>>>
BlobStatsToFbs(const std::vector<BlobDetector::BlobStats> blob_stats,
               aos::Sender<TargetEstimate>::Builder &builder) {
  std::vector<flatbuffers::Offset<BlobStatsFbs>> stats_fbs;
  for (auto &stats : blob_stats) {
    // Make BlobStatsFbs builder then fill each field using the BlobStats
    // struct, then you finish it and add it to stats_fbs.
    auto stats_builder = builder.MakeBuilder<BlobStatsFbs>();
    Point centroid_fbs = Point{stats.centroid.x, stats.centroid.y};
    stats_builder.add_centroid(&centroid_fbs);
    stats_builder.add_aspect_ratio(stats.aspect_ratio);
    stats_builder.add_area(stats.area);
    stats_builder.add_num_points(stats.num_points);

    auto current_stats = stats_builder.Finish();
    stats_fbs.emplace_back(current_stats);
  }
  return builder.fbb()->CreateVector(stats_fbs.data(), stats_fbs.size());
}
}  // namespace

void CameraReader::ProcessImage(cv::Mat image_mat) {
  // Remember, we're getting YUYV images, so we start by converting to RGB

  std::vector<std::vector<cv::Point>> filtered_blobs, unfiltered_blobs;
  std::vector<BlobDetector::BlobStats> blob_stats;
  cv::Mat binarized_image =
      cv::Mat::zeros(cv::Size(image_mat.cols, image_mat.rows), CV_8UC1);
  cv::Point centroid;
  BlobDetector::ExtractBlobs(image_mat, binarized_image, filtered_blobs,
                             unfiltered_blobs, blob_stats, centroid);
  auto builder = target_estimate_sender_.MakeBuilder();
  flatbuffers::Offset<BlobResult> blob_result_offset;
  {
    const auto filtered_blobs_offset = CvBlobsToFbs(filtered_blobs, builder);
    const auto unfiltered_blobs_offset =
        CvBlobsToFbs(unfiltered_blobs, builder);
    const auto blob_stats_offset = BlobStatsToFbs(blob_stats, builder);
    const Point centroid_fbs = Point{centroid.x, centroid.y};

    auto blob_result_builder = builder.MakeBuilder<BlobResult>();
    blob_result_builder.add_filtered_blobs(filtered_blobs_offset);
    blob_result_builder.add_unfiltered_blobs(unfiltered_blobs_offset);
    blob_result_builder.add_blob_stats(blob_stats_offset);
    blob_result_builder.add_centroid(&centroid_fbs);
    blob_result_offset = blob_result_builder.Finish();
  }

  auto target_estimate_builder = builder.MakeBuilder<TargetEstimate>();
  TargetEstimator::EstimateTargetLocation(centroid, CameraIntrinsics(),
                                          CameraExtrinsics(),
                                          &target_estimate_builder);
  target_estimate_builder.add_blob_result(blob_result_offset);

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
      cv::Mat rgb_image = cv::imread(file.c_str());
      ProcessImage(rgb_image);
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
  cv::Mat image_mat(image.rows(), image.cols(), CV_8U);
  CHECK(image_mat.isContinuous());

  const int number_pixels = image.rows() * image.cols();
  for (int i = 0; i < number_pixels; ++i) {
    reinterpret_cast<uint8_t *>(image_mat.data)[i] =
        image.data()->data()[i * 2];
  }

  ProcessImage(image_mat);

  reader_->SendLatestImage();
  read_image_timer_->Setup(event_loop_->monotonic_now());
}

}  // namespace vision
}  // namespace y2022
