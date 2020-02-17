#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"

#include "y2020/vision/sift/demo_sift.h"
#include "y2020/vision/sift/sift971.h"
#include "y2020/vision/sift/sift_generated.h"
#include "y2020/vision/sift/sift_training_generated.h"
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
        reader_(reader),
        matcher_(matcher),
        image_sender_(event_loop->MakeSender<CameraImage>("/camera")),
        result_sender_(
            event_loop->MakeSender<sift::ImageMatchResult>("/camera")),
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

  // Returns the 3D location for the specified training feature.
  cv::Point3f Training3dPoint(int training_image_index, int feature_index) {
    const sift::KeypointFieldLocation *const location =
        training_data_->images()
            ->Get(training_image_index)
            ->features()
            ->Get(feature_index)
            ->field_location();
    return cv::Point3f(location->x(), location->y(), location->z());
  }

  int number_training_images() const {
    return training_data_->images()->size();
  }

  aos::EventLoop *const event_loop_;
  const sift::TrainingData *const training_data_;
  V4L2Reader *const reader_;
  cv::FlannBasedMatcher *const matcher_;
  aos::Sender<CameraImage> image_sender_;
  aos::Sender<sift::ImageMatchResult> result_sender_;
  // We schedule this immediately to read an image. Having it on a timer means
  // other things can run on the event loop in between.
  aos::TimerHandler *const read_image_timer_;

  const std::unique_ptr<frc971::vision::SIFT971_Impl> sift_{
      new frc971::vision::SIFT971_Impl()};
};

void CameraReader::CopyTrainingFeatures() {
  for (const sift::TrainingImage *training_image : *training_data_->images()) {
    cv::Mat features(training_image->features()->size(), 128, CV_32F);
    for (size_t i = 0; i <  training_image->features()->size(); ++i) {
      const sift::Feature *feature_table = training_image->features()->Get(i);
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

void CameraReader::ProcessImage(const CameraImage &image) {
  // First, we need to extract the brightness information. This can't really be
  // fused into the beginning of the SIFT algorithm because the algorithm needs
  // to look at the base image directly. It also only takes 2ms on our images.
  // This is converting from YUYV to a grayscale image.
  cv::Mat image_mat(
      image.rows(), image.cols(), CV_8U);
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

  // Now, pack the results up and send them out.
  auto builder = result_sender_.MakeBuilder();

  const auto image_matches_offset = PackImageMatches(builder.fbb(), matches);
  // TODO(Brian): PackCameraPoses (and put it in the result)
  const auto features_offset =
      PackFeatures(builder.fbb(), keypoints, descriptors);

  sift::ImageMatchResult::Builder result_builder(*builder.fbb());
  result_builder.add_image_matches(image_matches_offset);
  result_builder.add_features(features_offset);
  result_builder.add_image_monotonic_timestamp_ns(
      image.monotonic_timestamp_ns());
  builder.Send(result_builder.Finish());
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

  const auto training_data_bfbs = DemoSiftData();
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
  V4L2Reader v4l2_reader(&event_loop, "/dev/video0");
  CameraReader camera_reader(&event_loop, training_data, &v4l2_reader, &matcher);

  event_loop.Run();
}

}  // namespace
}  // namespace vision
}  // namespace frc971


int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  frc971::vision::CameraReaderMain();
}
