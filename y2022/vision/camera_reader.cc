#include "y2022/vision/camera_reader.h"

#include <math.h>

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

void CameraReader::ProcessImage(const cv::Mat &image_mat) {
  // Remember, we're getting YUYV images, so we start by converting to RGB

  // TOOD: Need to code this up for blob detection

  std::vector<std::vector<cv::Point>> filtered_blobs, unfiltered_blobs;
  std::vector<BlobDetector::BlobStats> blob_stats;
  cv::Mat binarized_image =
      cv::Mat::zeros(cv::Size(image_mat.cols, image_mat.rows), CV_8UC1);
  cv::Mat ret_image =
      cv::Mat::zeros(cv::Size(image_mat.cols, image_mat.rows), CV_8UC3);
  cv::Point centroid;
  BlobDetector::ExtractBlobs(image_mat, binarized_image, ret_image,
                             filtered_blobs, unfiltered_blobs, blob_stats,
                             centroid);
  TargetEstimateT target = TargetEstimator::EstimateTargetLocation(
      centroid, CameraIntrinsics(), CameraExtrinsics());

  auto builder = target_estimate_sender_.MakeBuilder();
  builder.CheckOk(builder.Send(TargetEstimate::Pack(*builder.fbb(), &target)));
}

void CameraReader::ReadImage() {
  // Path is for reading from the Disk.
  if (FLAGS_image_png != "") {
    std::vector<cv::String> file_list;
    cv::glob(FLAGS_image_png + "/*.png", file_list, false);

    for (auto file : file_list) {
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
