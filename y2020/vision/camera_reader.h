#ifndef Y2020_VISION_CAMERA_READER_H_
#define Y2020_VISION_CAMERA_READER_H_

#include <math.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include "aos/events/event_loop.h"
#include "aos/flatbuffer_merge.h"
#include "aos/network/team_number.h"
#include "frc971/vision/v4l2_reader.h"
#include "frc971/vision/vision_generated.h"
#include "y2020/vision/sift/sift971.h"
#include "y2020/vision/sift/sift_generated.h"
#include "y2020/vision/sift/sift_training_generated.h"
#include "y2020/vision/tools/python_code/sift_training_data.h"

namespace frc971 {
namespace vision {

class CameraReader {
 public:
  CameraReader(aos::EventLoop *event_loop,
               const sift::TrainingData *training_data, V4L2Reader *reader,
               const cv::Ptr<cv::flann::IndexParams> &index_params,
               const cv::Ptr<cv::flann::SearchParams> &search_params)
      : event_loop_(event_loop),
        training_data_(training_data),
        camera_calibration_(FindCameraCalibration()),
        reader_(reader),
        image_sender_(event_loop->MakeSender<CameraImage>("/camera")),
        result_sender_(
            event_loop->MakeSender<sift::ImageMatchResult>("/camera")),
        detailed_result_sender_(
            event_loop->MakeSender<sift::ImageMatchResult>("/camera/detailed")),
        read_image_timer_(event_loop->AddTimer([this]() { ReadImage(); })) {
    for (int ii = 0; ii < number_training_images(); ++ii) {
      matchers_.push_back(cv::FlannBasedMatcher(index_params, search_params));
      prev_camera_field_R_vec_list_.push_back(cv::Mat::zeros(3, 1, CV_32F));
      prev_camera_field_T_list_.push_back(cv::Mat::zeros(3, 1, CV_32F));
    }
    CopyTrainingFeatures();

    for (auto &matcher : matchers_) {
      matcher.train();
    }

    event_loop->OnRun(
        [this]() { read_image_timer_->Setup(event_loop_->monotonic_now()); });
  }

 private:
  const sift::CameraCalibration *FindCameraCalibration() const;

  // Copies the information from training_data_ into matcher_.
  void CopyTrainingFeatures();
  // Processes an image (including sending the results).
  void ProcessImage(const CameraImage &image);
  // Reads an image, and then performs all of our processing on it.
  void ReadImage();

  flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<sift::ImageMatch>>>
  PackImageMatches(flatbuffers::FlatBufferBuilder *fbb,
                   const std::vector<std::vector<cv::DMatch>> &matches);
  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<sift::Feature>>>
  PackFeatures(flatbuffers::FlatBufferBuilder *fbb,
               const std::vector<cv::KeyPoint> &keypoints,
               const cv::Mat &descriptors);

  void SendImageMatchResult(const CameraImage &image,
                            const std::vector<cv::KeyPoint> &keypoints,
                            const cv::Mat &descriptors,
                            const std::vector<std::vector<cv::DMatch>> &matches,
                            const std::vector<cv::Mat> &camera_target_list,
                            const std::vector<cv::Mat> &field_camera_list,
                            const std::vector<cv::Point2f> &target_point_vector,
                            const std::vector<float> &target_radius_vector,
                            const std::vector<int> &training_image_indices,
                            const std::vector<int> &homography_feature_counts,
                            aos::Sender<sift::ImageMatchResult> *result_sender,
                            bool send_details);

  // Returns the 2D (image) location for the specified training feature.
  cv::Point2f Training2dPoint(int training_image_index,
                              int feature_index) const {
    const float x = training_data_->images()
                        ->Get(training_image_index)
                        ->features()
                        ->Get(feature_index)
                        ->x();
    const float y = training_data_->images()
                        ->Get(training_image_index)
                        ->features()
                        ->Get(feature_index)
                        ->y();
    return cv::Point2f(x, y);
  }

  // Returns the 3D location for the specified training feature.
  cv::Point3f Training3dPoint(int training_image_index,
                              int feature_index) const {
    const sift::KeypointFieldLocation *const location =
        training_data_->images()
            ->Get(training_image_index)
            ->features()
            ->Get(feature_index)
            ->field_location();
    return cv::Point3f(location->x(), location->y(), location->z());
  }

  const sift::TransformationMatrix *FieldToTarget(int training_image_index) {
    return training_data_->images()
        ->Get(training_image_index)
        ->field_to_target();
  }

  void TargetLocation(int training_image_index, cv::Point2f &target_location,
                      float &target_radius) {
    target_location.x =
        training_data_->images()->Get(training_image_index)->target_point_x();
    target_location.y =
        training_data_->images()->Get(training_image_index)->target_point_y();
    target_radius = training_data_->images()
                        ->Get(training_image_index)
                        ->target_point_radius();
  }

  int number_training_images() const {
    return training_data_->images()->size();
  }

  cv::Mat CameraIntrinsics() const {
    const cv::Mat result(3, 3, CV_32F,
                         const_cast<void *>(static_cast<const void *>(
                             camera_calibration_->intrinsics()->data())));
    CHECK_EQ(result.total(), camera_calibration_->intrinsics()->size());
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
  std::vector<cv::FlannBasedMatcher> matchers_;
  aos::Sender<CameraImage> image_sender_;
  aos::Sender<sift::ImageMatchResult> result_sender_;
  aos::SendFailureCounter result_failure_counter_;
  aos::Sender<sift::ImageMatchResult> detailed_result_sender_;
  // We schedule this immediately to read an image. Having it on a timer means
  // other things can run on the event loop in between.
  aos::TimerHandler *const read_image_timer_;

  // Storage for when we want to use the previous estimates of pose
  std::vector<cv::Mat> prev_camera_field_R_vec_list_;
  std::vector<cv::Mat> prev_camera_field_T_list_;

  const std::unique_ptr<frc971::vision::SIFT971_Impl> sift_{
      new frc971::vision::SIFT971_Impl()};
};

}  // namespace vision
}  // namespace frc971
#endif  // Y2020_VISION_CAMERA_READER_H_
