#include <unistd.h>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include "glog/logging.h"
#include "gtest/gtest.h"

#if 1
#include "y2020/vision/tools/python_code/sift_training_data_test.h"
#else
// Using this include (and changing BUILD file) allows to test on real data
#include "y2020/vision/tools/python_code/sift_training_data.h"
#endif

#include "frc971/vision/vision_generated.h"
#include "y2020/vision/sift/sift_generated.h"
#include "y2020/vision/sift/sift_training_generated.h"

namespace frc971 {
namespace vision {
namespace {

class Feature {
 public:
  std::vector<float> descriptor_;
  /*	float x_;
  float y_;
  float size_;
  float angle_;
  float response_;
  int octave_;
  cv::Point3f keypoint_field_location_;*/
};

class CameraCalibration {
 public:
  cv::Mat intrinsics_;
  cv::Mat fixed_extrinsics_;
};

class TrainingImage {
 public:
  cv::Mat features_;
  float target_point_x_;
  float target_point_y_;
  cv::Mat field_to_target_;
};

class TrainingData {
 public:
  std::vector<TrainingImage> images_;
  CameraCalibration camera_calibration_;
};

class CameraParamTest {
 public:
  CameraParamTest() {
    const auto training_data_bfbs = SiftTrainingData();
    sift_training_data_ =
        flatbuffers::GetRoot<sift::TrainingData>(training_data_bfbs.data());
    {
      flatbuffers::Verifier verifier(
          reinterpret_cast<const uint8_t *>(training_data_bfbs.data()),
          training_data_bfbs.size());

      CHECK(sift_training_data_->Verify(verifier));
    }

    CopyTrainingFeatures();
    sift_camera_calibration_ = CameraParamTest::FindCameraCalibration();
    camera_intrinsic_matrix_ = CameraIntrinsicMatrix();
    camera_dist_coeffs_ = CameraDistCoeffs();
    camera_extrinsics_ = CameraExtrinsics();
  }

  // Copies the information from sift_training_data_ into matcher_.
  void CopyTrainingFeatures();

  const sift::CameraCalibration *FindCameraCalibration() const;

  // Returns the 2D image location for the specified training feature.
  cv::Point2f Training2dPoint(int training_image_index, int feature_index);
  // Returns the 3D location for the specified training feature.
  cv::Point3f Training3dPoint(int training_image_index, int feature_index);

  const sift::TransformationMatrix *FieldToTarget(int training_image_index) {
    return sift_training_data_->images()
        ->Get(training_image_index)
        ->field_to_target();
  }

  cv::Mat CameraIntrinsicMatrix() const {
    const cv::Mat result(3, 3, CV_32F,
                         const_cast<void *>(static_cast<const void *>(
                             sift_camera_calibration_->intrinsics()->data())));
    CHECK_EQ(result.total(), sift_camera_calibration_->intrinsics()->size());
    return result;
  }

  cv::Mat CameraDistCoeffs() const {
    const cv::Mat result(5, 1, CV_32F,
                         const_cast<void *>(static_cast<const void *>(
                             sift_camera_calibration_->dist_coeffs()->data())));
    CHECK_EQ(result.total(), sift_camera_calibration_->dist_coeffs()->size());
    return result;
  }

  cv::Mat CameraExtrinsics() const {
    const cv::Mat result(
        4, 4, CV_32F,
        const_cast<void *>(static_cast<const void *>(
            sift_camera_calibration_->fixed_extrinsics()->data()->data())));
    return result;
  }

  int number_training_images() const {
    return sift_training_data_->images()->size();
  }

  const std::string node_name_ = "pi-3";
  const int team_number_ = 971;

  // We'll just extract the one that matches our current module
  const sift::CameraCalibration *sift_camera_calibration_;
  cv::Mat camera_intrinsic_matrix_;
  cv::Mat camera_dist_coeffs_;
  cv::Mat camera_extrinsics_;

  TrainingData training_data_;
  const sift::TrainingData *sift_training_data_;
  std::vector<sift::TransformationMatrix *> field_to_targets_;
  std::vector<cv::Point2f> point_list_2d_;
  std::vector<cv::Point3f> point_list_3d_;
};

// Copy in the keypoint descriptors, the 2d (image) location,
// and the 3d (field) location
void CameraParamTest::CopyTrainingFeatures() {
  int train_image_index = 0;
  for (const sift::TrainingImage *training_image :
       *sift_training_data_->images()) {
    TrainingImage tmp_training_image_data;
    cv::Mat features(training_image->features()->size(), 128, CV_32F);
    for (size_t i = 0; i < training_image->features()->size(); ++i) {
      const sift::Feature *feature_table = training_image->features()->Get(i);
      const flatbuffers::Vector<uint8_t> *const descriptor =
          feature_table->descriptor();
      CHECK_EQ(descriptor->size(), 128u) << ": Unsupported feature size";
      const auto in_mat = cv::Mat(
          1, descriptor->size(), CV_8U,
          const_cast<void *>(static_cast<const void *>(descriptor->data())));
      const auto out_mat = features(cv::Range(i, i + 1), cv::Range(0, 128));
      in_mat.convertTo(out_mat, CV_32F);

      cv::Point2f point_2d = Training2dPoint(train_image_index, i);
      point_list_2d_.push_back(point_2d);
      cv::Point3f point_3d = Training3dPoint(train_image_index, i);
      point_list_3d_.push_back(point_3d);
    }
    const sift::TransformationMatrix *field_to_target_ =
        FieldToTarget(train_image_index);
    const cv::Mat field_to_target_mat(
        4, 4, CV_32F,
        const_cast<void *>(
            static_cast<const void *>(field_to_target_->data()->data())));
    tmp_training_image_data.features_ = features;
    tmp_training_image_data.field_to_target_ = field_to_target_mat;
    tmp_training_image_data.target_point_x_ =
        sift_training_data_->images()->Get(train_image_index)->target_point_x();
    tmp_training_image_data.target_point_y_ =
        sift_training_data_->images()->Get(train_image_index)->target_point_y();

    training_data_.images_.push_back(tmp_training_image_data);
    train_image_index++;
  }
}

const sift::CameraCalibration *CameraParamTest::FindCameraCalibration() const {
  for (const sift::CameraCalibration *candidate :
       *sift_training_data_->camera_calibrations()) {
    if (candidate->node_name()->string_view() != node_name_) {
      continue;
    }
    if (candidate->team_number() != team_number_) {
      continue;
    }
    return candidate;
  }
  LOG(INFO) << ": Failed to find camera calibration for " << node_name_
            << " on " << team_number_;
  return NULL;
}

// Returns the 2D image location for the specified training feature.
cv::Point2f CameraParamTest::Training2dPoint(int training_image_index,
                                             int feature_index) {
  const float x = sift_training_data_->images()
                      ->Get(training_image_index)
                      ->features()
                      ->Get(feature_index)
                      ->x();
  const float y = sift_training_data_->images()
                      ->Get(training_image_index)
                      ->features()
                      ->Get(feature_index)
                      ->y();
  return cv::Point2f(x, y);
}

// Returns the 3D location for the specified training feature.
cv::Point3f CameraParamTest::Training3dPoint(int training_image_index,
                                             int feature_index) {
  const sift::KeypointFieldLocation *const location =
      sift_training_data_->images()
          ->Get(training_image_index)
          ->features()
          ->Get(feature_index)
          ->field_location();
  return cv::Point3f(location->x(), location->y(), location->z());
}

TEST(CameraParamTest, TargetDataTest) {
  CameraParamTest camera_params;

  ASSERT_EQ(camera_params.number_training_images(), 1);
  ASSERT_EQ(camera_params.point_list_2d_.size(), 676);
  ASSERT_EQ(camera_params.point_list_2d_[0].x, 0);
  ASSERT_EQ(camera_params.point_list_2d_[0].y, 1);
  ASSERT_EQ(camera_params.point_list_3d_.size(), 676);
  ASSERT_EQ(camera_params.point_list_3d_[0].x, 0);
  ASSERT_EQ(camera_params.point_list_3d_[1].y, 1);
  ASSERT_EQ(camera_params.point_list_3d_[2].z, 2);

  float ftt_mat[16] = {1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 2, 0, 0, 0, 1};
  cv::Mat field_to_targets_0 = cv::Mat(4, 4, CV_32F, ftt_mat);
  cv::Mat ftt_diff =
      (camera_params.training_data_.images_[0].field_to_target_ !=
       field_to_targets_0);
  bool ftt_eq = (cv::countNonZero(ftt_diff) == 0);
  ASSERT_TRUE(ftt_eq)
      << "Mismatch on field_to_target: "
      << camera_params.training_data_.images_[0].field_to_target_ << "\nvs.\n"
      << field_to_targets_0;

  ASSERT_EQ(camera_params.training_data_.images_[0].target_point_x_, 10.);
  ASSERT_EQ(camera_params.training_data_.images_[0].target_point_y_, 20.);

  float intrinsic_mat[9] = {810, 0, 320, 0, 810, 240, 0, 0, 1};
  cv::Mat intrinsic = cv::Mat(3, 3, CV_32F, intrinsic_mat);
  cv::Mat intrinsic_diff =
      (intrinsic != camera_params.camera_intrinsic_matrix_);
  bool intrinsic_eq = (cv::countNonZero(intrinsic_diff) == 0);
  ASSERT_TRUE(intrinsic_eq)
      << "Mismatch on camera intrinsic matrix: " << intrinsic << "\nvs.\n"
      << camera_params.camera_intrinsic_matrix_;

  float dist_coeffs_mat[5] = {0., 0., 0., 0., 0.};
  cv::Mat dist_coeffs = cv::Mat(5, 1, CV_32F, dist_coeffs_mat);
  cv::Mat dist_coeffs_diff = (dist_coeffs != camera_params.camera_dist_coeffs_);
  bool dist_coeffs_eq = (cv::countNonZero(dist_coeffs_diff) == 0);
  ASSERT_TRUE(dist_coeffs_eq)
      << "Mismatch on camera distortion coefficients: " << dist_coeffs
      << "\nvs.\n"
      << camera_params.camera_dist_coeffs_;

  float i_f = 3.0;
  float extrinsic_mat[16] = {0, 0,  1, i_f, -1, 0, 0, i_f,
                             0, -1, 0, i_f, 0,  0, 0, 1};
  cv::Mat extrinsic = cv::Mat(4, 4, CV_32F, extrinsic_mat);
  cv::Mat extrinsic_diff = (extrinsic != camera_params.camera_extrinsics_);
  bool extrinsic_eq = (cv::countNonZero(extrinsic_diff) == 0);
  ASSERT_TRUE(extrinsic_eq)
      << "Mismatch of extrinsic: " << extrinsic << "\nvs.\n"
      << camera_params.camera_extrinsics_;
}

}  // namespace
}  // namespace vision
}  // namespace frc971
