#include <map>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <random>

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/time/time.h"
#include "frc971/vision/vision_generated.h"
#include "y2022/vision/blob_detector.h"
#include "y2022/vision/calibration_data.h"
#include "y2022/vision/target_estimate_generated.h"
#include "y2022/vision/target_estimator.h"

DEFINE_string(capture, "",
              "If set, capture a single image and save it to this filename.");
DEFINE_string(channel, "/camera", "Channel name for the image.");
DEFINE_string(config, "aos_config.json", "Path to the config file to use.");
DEFINE_string(png_dir, "", "Path to a set of images to display.");
DEFINE_string(calibration_node, "",
              "If reading locally, use the calibration for this node");
DEFINE_int32(
    calibration_team_number, 971,
    "If reading locally, use the calibration for a node with this team number");
DEFINE_uint64(skip, 0,
              "Number of images to skip if doing local reading (png_dir set).");
DEFINE_bool(show_features, true, "Show the blobs.");
DEFINE_bool(display_estimation, false,
            "If true, display the target estimation graphically");

namespace y2022 {
namespace vision {
namespace {

using namespace frc971::vision;

std::map<int64_t, BlobDetector::BlobResult> target_est_map;
aos::Fetcher<frc971::vision::CameraImage> image_fetcher;
aos::Fetcher<y2022::vision::TargetEstimate> target_estimate_fetcher;

std::vector<cv::Point> FbsToCvPoints(
    const flatbuffers::Vector<const Point *> &points_fbs) {
  std::vector<cv::Point> points;
  for (const Point *point : points_fbs) {
    points.emplace_back(point->x(), point->y());
  }
  return points;
}

std::vector<std::vector<cv::Point>> FbsToCvBlobs(
    const flatbuffers::Vector<flatbuffers::Offset<Blob>> *blobs_fbs) {
  if (blobs_fbs == nullptr) {
    return {};
  }
  std::vector<std::vector<cv::Point>> blobs;
  for (const auto blob : *blobs_fbs) {
    blobs.emplace_back(FbsToCvPoints(*blob->points()));
  }
  return blobs;
}

std::vector<BlobDetector::BlobStats> FbsToBlobStats(
    const flatbuffers::Vector<flatbuffers::Offset<BlobStatsFbs>>
        &blob_stats_fbs) {
  std::vector<BlobDetector::BlobStats> blob_stats;
  for (const auto stats_fbs : blob_stats_fbs) {
    cv::Point centroid{stats_fbs->centroid()->x(), stats_fbs->centroid()->y()};
    cv::Size size{stats_fbs->size()->width(), stats_fbs->size()->height()};
    blob_stats.emplace_back(BlobDetector::BlobStats{
        centroid, size, stats_fbs->aspect_ratio(), stats_fbs->area(),
        static_cast<size_t>(stats_fbs->num_points())});
  }
  return blob_stats;
}

bool DisplayLoop() {
  int64_t target_timestamp = 0;
  if (target_estimate_fetcher.Fetch()) {
    const TargetEstimate *target_est = target_estimate_fetcher.get();
    CHECK(target_est != nullptr)
        << "Got null when trying to fetch target estimate";

    target_timestamp = target_est->image_monotonic_timestamp_ns();
    if (target_est->blob_result()->filtered_blobs()->size() > 0) {
      VLOG(2) << "Got blobs for timestamp " << target_est << "\n";
    }
    // Store the TargetEstimate data so we can match timestamp with image
    target_est_map[target_timestamp] = BlobDetector::BlobResult{
        cv::Mat(),
        FbsToCvBlobs(target_est->blob_result()->filtered_blobs()),
        FbsToCvBlobs(target_est->blob_result()->unfiltered_blobs()),
        FbsToBlobStats(*target_est->blob_result()->blob_stats()),
        FbsToBlobStats(*target_est->blob_result()->filtered_stats()),
        cv::Point{target_est->blob_result()->centroid()->x(),
                  target_est->blob_result()->centroid()->y()}};
    // Only keep last 10 matches
    while (target_est_map.size() > 10u) {
      target_est_map.erase(target_est_map.begin());
    }
  }
  int64_t image_timestamp = 0;
  if (!image_fetcher.Fetch()) {
    VLOG(2) << "Couldn't fetch image";
    return true;
  }
  const CameraImage *image = image_fetcher.get();
  CHECK(image != nullptr) << "Couldn't read image";
  image_timestamp = image->monotonic_timestamp_ns();
  VLOG(2) << "Got image at timestamp: " << image_timestamp;

  // Create color image:
  cv::Mat image_color_mat(cv::Size(image->cols(), image->rows()), CV_8UC2,
                          (void *)image->data()->data());
  cv::Mat bgr_image(cv::Size(image->cols(), image->rows()), CV_8UC3);
  cv::cvtColor(image_color_mat, bgr_image, cv::COLOR_YUV2BGR_YUYV);

  if (!FLAGS_capture.empty()) {
    cv::imwrite(FLAGS_capture, bgr_image);
    return false;
  }

  auto target_est_it = target_est_map.find(image_timestamp);
  if (target_est_it != target_est_map.end()) {
    LOG(INFO) << image->monotonic_timestamp_ns() << ": # unfiltered blobs: "
              << target_est_it->second.unfiltered_blobs.size()
              << "; # filtered blobs: "
              << target_est_it->second.filtered_blobs.size();

    cv::Mat ret_image =
        cv::Mat::zeros(cv::Size(image->cols(), image->rows()), CV_8UC3);
    BlobDetector::DrawBlobs(target_est_it->second, ret_image);
    cv::imshow("blobs", ret_image);
  }

  cv::imshow("image", bgr_image);

  int keystroke = cv::waitKey(1);
  if ((keystroke & 0xFF) == static_cast<int>('c')) {
    // Convert again, to get clean image
    cv::cvtColor(image_color_mat, bgr_image, cv::COLOR_YUV2BGR_YUYV);
    std::stringstream name;
    name << "capture-" << aos::realtime_clock::now() << ".png";
    cv::imwrite(name.str(), bgr_image);
    LOG(INFO) << "Saved image file: " << name.str();
  } else if ((keystroke & 0xFF) == static_cast<int>('q')) {
    return false;
  }
  return true;
}

void ViewerMain() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  aos::ShmEventLoop event_loop(&config.message());

  image_fetcher =
      event_loop.MakeFetcher<frc971::vision::CameraImage>(FLAGS_channel);

  target_estimate_fetcher =
      event_loop.MakeFetcher<y2022::vision::TargetEstimate>(FLAGS_channel);

  // Run the display loop
  event_loop.AddPhasedLoop(
      [&event_loop](int) {
        if (!DisplayLoop()) {
          LOG(INFO) << "Calling event_loop Exit";
          event_loop.Exit();
        };
      },
      ::std::chrono::milliseconds(100));

  event_loop.Run();
}

// TODO(milind): delete this when viewer can accumulate local images and results
void ViewerLocal() {
  std::vector<cv::String> file_list;
  cv::glob(FLAGS_png_dir + "/*.png", file_list, false);

  const aos::FlatbufferSpan<calibration::CalibrationData> calibration_data(
      CalibrationData());

  const calibration::CameraCalibration *calibration = nullptr;
  for (const calibration::CameraCalibration *candidate :
       *calibration_data.message().camera_calibrations()) {
    if ((candidate->node_name()->string_view() == FLAGS_calibration_node) &&
        (candidate->team_number() == FLAGS_calibration_team_number)) {
      calibration = candidate;
      break;
    }
  }

  CHECK(calibration) << "No calibration data found for node \""
                     << FLAGS_calibration_node << "\" with team number "
                     << FLAGS_calibration_team_number;

  auto intrinsics_float = cv::Mat(3, 3, CV_32F,
                                  const_cast<void *>(static_cast<const void *>(
                                      calibration->intrinsics()->data())));
  cv::Mat intrinsics;
  intrinsics_float.convertTo(intrinsics, CV_64F);

  const auto extrinsics_float =
      cv::Mat(4, 4, CV_32F,
              const_cast<void *>(static_cast<const void *>(
                  calibration->fixed_extrinsics()->data()->data())));
  cv::Mat extrinsics;
  extrinsics_float.convertTo(extrinsics, CV_64F);

  TargetEstimator estimator(intrinsics, extrinsics);

  for (auto it = file_list.begin() + FLAGS_skip; it < file_list.end(); it++) {
    LOG(INFO) << "Reading file " << *it;
    cv::Mat image_mat = cv::imread(it->c_str());
    BlobDetector::BlobResult blob_result;
    blob_result.binarized_image =
        cv::Mat::zeros(cv::Size(image_mat.cols, image_mat.rows), CV_8UC1);
    BlobDetector::ExtractBlobs(image_mat, &blob_result);

    cv::Mat ret_image =
        cv::Mat::zeros(cv::Size(image_mat.cols, image_mat.rows), CV_8UC3);
    BlobDetector::DrawBlobs(blob_result, ret_image);

    LOG(INFO) << ": # blobs: " << blob_result.filtered_blobs.size()
              << " (# removed: "
              << blob_result.unfiltered_blobs.size() -
                     blob_result.filtered_blobs.size()
              << ")";

    if (blob_result.filtered_blobs.size() > 0) {
      estimator.Solve(blob_result.filtered_stats,
                      FLAGS_display_estimation ? std::make_optional(ret_image)
                                               : std::nullopt);
      estimator.DrawEstimate(ret_image);
    }

    cv::imshow("image", image_mat);
    cv::imshow("mask", blob_result.binarized_image);
    cv::imshow("blobs", ret_image);

    int keystroke = cv::waitKey(0);
    if ((keystroke & 0xFF) == static_cast<int>('q')) {
      return;
    }
  }
}
}  // namespace
}  // namespace vision
}  // namespace y2022

// Quick and lightweight viewer for images
int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  if (FLAGS_png_dir != "")
    y2022::vision::ViewerLocal();
  else
    y2022::vision::ViewerMain();
}
