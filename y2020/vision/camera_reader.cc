#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include "aos/events/shm_event_loop.h"
#include "aos/flatbuffer_merge.h"
#include "aos/init.h"
#include "aos/network/team_number.h"

#include "y2020/vision/sift/sift971.h"
#include "y2020/vision/sift/sift_generated.h"
#include "y2020/vision/sift/sift_training_generated.h"
#include "y2020/vision/tools/python_code/sift_training_data.h"
#include "y2020/vision/v4l2_reader.h"
#include "y2020/vision/vision_generated.h"

namespace frc971 {
namespace vision {
namespace {

class CameraReader {
 public:
  CameraReader(aos::EventLoop *event_loop,
               const sift::TrainingData *training_data, V4L2Reader *reader,
               cv::FlannBasedMatcher *matcher)
      : event_loop_(event_loop),
        training_data_(training_data),
        camera_calibration_(FindCameraCalibration()),
        reader_(reader),
        matcher_(matcher),
        image_sender_(event_loop->MakeSender<CameraImage>("/camera")),
        result_sender_(
            event_loop->MakeSender<sift::ImageMatchResult>("/camera")),
        detailed_result_sender_(
            event_loop->MakeSender<sift::ImageMatchResult>("/camera/detailed")),
        read_image_timer_(event_loop->AddTimer([this]() {
          ReadImage();
          read_image_timer_->Setup(event_loop_->monotonic_now());
        })) {
    CopyTrainingFeatures();
    // Technically we don't need to do this, but doing it now avoids the first
    // match attempt being slow.
    matcher_->train();

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
                            aos::Sender<sift::ImageMatchResult> *result_sender,
                            bool send_details);

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
  cv::FlannBasedMatcher *const matcher_;
  aos::Sender<CameraImage> image_sender_;
  aos::Sender<sift::ImageMatchResult> result_sender_;
  aos::Sender<sift::ImageMatchResult> detailed_result_sender_;
  // We schedule this immediately to read an image. Having it on a timer means
  // other things can run on the event loop in between.
  aos::TimerHandler *const read_image_timer_;

  const std::unique_ptr<frc971::vision::SIFT971_Impl> sift_{
      new frc971::vision::SIFT971_Impl()};
};

const sift::CameraCalibration *CameraReader::FindCameraCalibration() const {
  const std::string_view node_name = event_loop_->node()->name()->string_view();
  const int team_number = aos::network::GetTeamNumber();
  for (const sift::CameraCalibration *candidate :
       *training_data_->camera_calibrations()) {
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

void CameraReader::CopyTrainingFeatures() {
  for (const sift::TrainingImage *training_image : *training_data_->images()) {
    cv::Mat features(training_image->features()->size(), 128, CV_32F);
    for (size_t i = 0; i < training_image->features()->size(); ++i) {
      const sift::Feature *feature_table = training_image->features()->Get(i);

      // We don't need this information right now, but make sure it's here to
      // avoid crashes that only occur when specific features are matched.
      CHECK(feature_table->has_field_location());

      const flatbuffers::Vector<float> *const descriptor =
          feature_table->descriptor();
      CHECK_EQ(descriptor->size(), 128u) << ": Unsupported feature size";
      cv::Mat(1, descriptor->size(), CV_32F,
              const_cast<void *>(static_cast<const void *>(descriptor->data())))
          .copyTo(features(cv::Range(i, i + 1), cv::Range(0, 128)));
    }
    matcher_->add(features);
  }
}

void CameraReader::SendImageMatchResult(
    const CameraImage &image, const std::vector<cv::KeyPoint> &keypoints,
    const cv::Mat &descriptors,
    const std::vector<std::vector<cv::DMatch>> &matches,
    const std::vector<cv::Mat> &camera_target_list,
    aos::Sender<sift::ImageMatchResult> *result_sender, bool send_details) {
  auto builder = result_sender->MakeBuilder();
  const auto camera_calibration_offset =
      aos::CopyFlatBuffer(camera_calibration_, builder.fbb());

  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<sift::Feature>>>
      features_offset;
  flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<sift::ImageMatch>>>
      image_matches_offset;
  if (send_details) {
    features_offset = PackFeatures(builder.fbb(), keypoints, descriptors);
    image_matches_offset = PackImageMatches(builder.fbb(), matches);
  }

  std::vector<flatbuffers::Offset<sift::CameraPose>> camera_poses;

  for (size_t i = 0; i < camera_target_list.size(); ++i) {
    cv::Mat camera_target = camera_target_list[i];
    CHECK(camera_target.isContinuous());
    const auto data_offset = builder.fbb()->CreateVector<float>(
        reinterpret_cast<float *>(camera_target.data), camera_target.total());
    const flatbuffers::Offset<sift::TransformationMatrix> transform_offset =
        sift::CreateTransformationMatrix(*builder.fbb(), data_offset);
    const flatbuffers::Offset<sift::TransformationMatrix>
        field_to_target_offset =
            aos::CopyFlatBuffer(FieldToTarget(i), builder.fbb());

    sift::CameraPose::Builder pose_builder(*builder.fbb());
    pose_builder.add_camera_to_target(transform_offset);
    pose_builder.add_field_to_target(field_to_target_offset);
    camera_poses.emplace_back(pose_builder.Finish());
  }
  const auto camera_poses_offset = builder.fbb()->CreateVector(camera_poses);

  sift::ImageMatchResult::Builder result_builder(*builder.fbb());
  result_builder.add_camera_poses(camera_poses_offset);
  if (send_details) {
    result_builder.add_image_matches(image_matches_offset);
    result_builder.add_features(features_offset);
  }
  result_builder.add_image_monotonic_timestamp_ns(
      image.monotonic_timestamp_ns());
  result_builder.add_camera_calibration(camera_calibration_offset);

  // TODO<Jim>: Need to add target point computed from matches and
  // mapped by homography
  builder.Send(result_builder.Finish());
}

void CameraReader::ProcessImage(const CameraImage &image) {
  // First, we need to extract the brightness information. This can't really be
  // fused into the beginning of the SIFT algorithm because the algorithm needs
  // to look at the base image directly. It also only takes 2ms on our images.
  // This is converting from YUYV to a grayscale image.
  cv::Mat image_mat(image.rows(), image.cols(), CV_8U);
  CHECK(image_mat.isContinuous());
  const int number_pixels = image.rows() * image.cols();
  for (int i = 0; i < number_pixels; ++i) {
    reinterpret_cast<uint8_t *>(image_mat.data)[i] =
        image.data()->data()[i * 2];
  }

  // Next, grab the features from the image.
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  sift_->detectAndCompute(image_mat, cv::noArray(), keypoints, descriptors);

  // Then, match those features against our training data.
  std::vector<std::vector<cv::DMatch>> matches;
  matcher_->knnMatch(/* queryDescriptors */ descriptors, matches, /* k */ 2);

  struct PerImageMatches {
    std::vector<const std::vector<cv::DMatch> *> matches;
    std::vector<cv::Point3f> training_points_3d;
    std::vector<cv::Point2f> query_points;
  };
  std::vector<PerImageMatches> per_image_matches(number_training_images());

  // Pull out the good matches which we want for each image.
  // Discard the bad matches per Lowe's ratio test.
  // (Lowe originally proposed 0.7 ratio, but 0.75 was later proposed as a
  // better option.  We'll go with the more conservative (fewer, better matches)
  // for now).
  for (const std::vector<cv::DMatch> &match : matches) {
    CHECK_EQ(2u, match.size());
    CHECK_LE(match[0].distance, match[1].distance);
    CHECK_LT(match[0].imgIdx, number_training_images());
    CHECK_LT(match[1].imgIdx, number_training_images());
    CHECK_EQ(match[0].queryIdx, match[1].queryIdx);
    if (!(match[0].distance < 0.7 * match[1].distance)) {
      continue;
    }

    const int training_image = match[0].imgIdx;
    CHECK_LT(training_image, static_cast<int>(per_image_matches.size()));
    PerImageMatches *const per_image = &per_image_matches[training_image];
    per_image->matches.push_back(&match);
    per_image->training_points_3d.push_back(
        Training3dPoint(training_image, match[0].trainIdx));

    const cv::KeyPoint &keypoint = keypoints[match[0].queryIdx];
    per_image->query_points.push_back(keypoint.pt);
  }

  // The minimum number of matches in a training image for us to use it.
  static constexpr int kMinimumMatchCount = 10;
  std::vector<cv::Mat> camera_target_list;

  for (size_t i = 0; i < per_image_matches.size(); ++i) {
    const PerImageMatches &per_image = per_image_matches[i];
    if (per_image.matches.size() < kMinimumMatchCount) {
      continue;
    }

    cv::Mat R_camera_target_vec, R_camera_target, T_camera_target;
    // Compute the pose of the camera (global origin relative to camera)
    cv::solvePnPRansac(per_image.training_points_3d, per_image.query_points,
                       CameraIntrinsics(), CameraDistCoeffs(),
                       R_camera_target_vec, T_camera_target);
    T_camera_target = T_camera_target.t();
    // Convert Camera from angle-axis (3x1) to homogenous (3x3) representation
    cv::Rodrigues(R_camera_target_vec, R_camera_target);

    {
      CHECK_EQ(cv::Size(3, 3), R_camera_target.size());
      CHECK_EQ(cv::Size(3, 1), T_camera_target.size());
      cv::Mat camera_target = cv::Mat::zeros(4, 4, CV_32F);
      R_camera_target.copyTo(camera_target(cv::Range(0, 3), cv::Range(0, 3)));
      T_camera_target.copyTo(camera_target(cv::Range(3, 4), cv::Range(0, 3)));
      camera_target.at<float>(3, 3) = 1;
      CHECK(camera_target.isContinuous());
      camera_target_list.push_back(camera_target);
    }
  }
  // Now, send our two messages-- one large, with details for remote
  // debugging(features), and one smaller
  SendImageMatchResult(image, keypoints, descriptors, matches,
                       camera_target_list, &detailed_result_sender_, true);
  SendImageMatchResult(image, keypoints, descriptors, matches,
                       camera_target_list, &result_sender_, false);
}

void CameraReader::ReadImage() {
  if (!reader_->ReadLatestImage()) {
    LOG(INFO) << "No image, sleeping";
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    return;
  }

  ProcessImage(reader_->LatestImage());

  reader_->SendLatestImage();
}

flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<sift::ImageMatch>>>
CameraReader::PackImageMatches(
    flatbuffers::FlatBufferBuilder *fbb,
    const std::vector<std::vector<cv::DMatch>> &matches) {
  // First, we need to pull out all the matches for each image. Might as well
  // build up the Match tables at the same time.
  std::vector<std::vector<sift::Match>> per_image_matches(
      number_training_images());
  for (const std::vector<cv::DMatch> &image_matches : matches) {
    for (const cv::DMatch &image_match : image_matches) {
      CHECK_LT(image_match.imgIdx, number_training_images());
      per_image_matches[image_match.imgIdx].emplace_back();
      sift::Match *const match = &per_image_matches[image_match.imgIdx].back();
      match->mutate_query_feature(image_match.queryIdx);
      match->mutate_train_feature(image_match.trainIdx);
      match->mutate_distance(image_match.distance);
    }
  }

  // Then, we need to build up each ImageMatch table.
  std::vector<flatbuffers::Offset<sift::ImageMatch>> image_match_tables;
  for (size_t i = 0; i < per_image_matches.size(); ++i) {
    const std::vector<sift::Match> &this_image_matches = per_image_matches[i];
    if (this_image_matches.empty()) {
      continue;
    }
    const auto vector_offset = fbb->CreateVectorOfStructs(this_image_matches);
    sift::ImageMatch::Builder image_builder(*fbb);
    image_builder.add_train_image(i);
    image_builder.add_matches(vector_offset);
    image_match_tables.emplace_back(image_builder.Finish());
  }

  return fbb->CreateVector(image_match_tables);
}

flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<sift::Feature>>>
CameraReader::PackFeatures(flatbuffers::FlatBufferBuilder *fbb,
                           const std::vector<cv::KeyPoint> &keypoints,
                           const cv::Mat &descriptors) {
  const int number_features = keypoints.size();
  CHECK_EQ(descriptors.rows, number_features);
  std::vector<flatbuffers::Offset<sift::Feature>> features_vector(
      number_features);
  for (int i = 0; i < number_features; ++i) {
    const auto submat = descriptors(cv::Range(i, i + 1), cv::Range(0, 128));
    CHECK(submat.isContinuous());
    const auto descriptor_offset =
        fbb->CreateVector(reinterpret_cast<float *>(submat.data), 128);
    sift::Feature::Builder feature_builder(*fbb);
    feature_builder.add_descriptor(descriptor_offset);
    feature_builder.add_x(keypoints[i].pt.x);
    feature_builder.add_y(keypoints[i].pt.y);
    feature_builder.add_size(keypoints[i].size);
    feature_builder.add_angle(keypoints[i].angle);
    feature_builder.add_response(keypoints[i].response);
    feature_builder.add_octave(keypoints[i].octave);
    CHECK_EQ(-1, keypoints[i].class_id)
        << ": Not sure what to do with a class id";
    features_vector[i] = feature_builder.Finish();
  }
  return fbb->CreateVector(features_vector);
}

void CameraReaderMain() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("config.json");

  const auto training_data_bfbs = SiftTrainingData();
  const sift::TrainingData *const training_data =
      flatbuffers::GetRoot<sift::TrainingData>(training_data_bfbs.data());
  {
    flatbuffers::Verifier verifier(
        reinterpret_cast<const uint8_t *>(training_data_bfbs.data()),
        training_data_bfbs.size());
    CHECK(training_data->Verify(verifier));
  }

  const auto index_params = cv::makePtr<cv::flann::IndexParams>();
  index_params->setAlgorithm(cvflann::FLANN_INDEX_KDTREE);
  index_params->setInt("trees", 5);
  const auto search_params =
      cv::makePtr<cv::flann::SearchParams>(/* checks */ 50);
  cv::FlannBasedMatcher matcher(index_params, search_params);

  aos::ShmEventLoop event_loop(&config.message());

  // First, log the data for future reference.
  {
    aos::Sender<sift::TrainingData> training_data_sender =
        event_loop.MakeSender<sift::TrainingData>("/camera");
    training_data_sender.Send(
        aos::FlatbufferString<sift::TrainingData>(training_data_bfbs));
  }

  V4L2Reader v4l2_reader(&event_loop, "/dev/video0");
  CameraReader camera_reader(&event_loop, training_data, &v4l2_reader,
                             &matcher);

  event_loop.Run();
}

}  // namespace
}  // namespace vision
}  // namespace frc971

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  frc971::vision::CameraReaderMain();
}
