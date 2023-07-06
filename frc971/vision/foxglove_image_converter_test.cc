#include "gtest/gtest.h"

#include "aos/events/simulated_event_loop.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/testing/path.h"
#include "aos/testing/tmpdir.h"
#include "frc971/vision/foxglove_image_converter_lib.h"

DECLARE_int32(jpeg_quality);

namespace frc971::vision {
std::ostream &operator<<(std::ostream &os, ImageCompression compression) {
  os << ExtensionForCompression(compression);
  return os;
}
namespace testing {
class ImageConverterTest : public ::testing::TestWithParam<ImageCompression> {
 protected:
  ImageConverterTest()
      : config_(aos::configuration::ReadConfig(
            aos::testing::ArtifactPath("frc971/vision/converter_config.json"))),
        factory_(&config_.message()),
        camera_image_(
            aos::FileToFlatbuffer<CameraImage>(aos::testing::ArtifactPath(
                "external/april_tag_test_image/test.bfbs"))),
        node_(aos::configuration::GetNode(&config_.message(), "test")),
        test_event_loop_(factory_.MakeEventLoop("test", node_)),
        image_sender_(test_event_loop_->MakeSender<CameraImage>("/camera")),
        converter_event_loop_(factory_.MakeEventLoop("converter", node_)),
        converter_(converter_event_loop_.get(), "/camera", "/visualize",
                   GetParam()),
        output_path_(absl::StrCat(aos::testing::TestTmpDir(), "/test.",
                                  ExtensionForCompression(GetParam()))) {
    // Because our test image for comparison was generated with a JPEG quality
    // of 95, we need to use that for the test to work. This also protects the
    // tests against future changes to the default JPEG quality.
    FLAGS_jpeg_quality = 95;
    test_event_loop_->OnRun(
        [this]() { image_sender_.CheckOk(image_sender_.Send(camera_image_)); });
    test_event_loop_->MakeWatcher(
        "/visualize", [this](const foxglove::CompressedImage &image) {
          ASSERT_TRUE(image.has_data());
          std::string expected_contents =
              aos::util::ReadFileToStringOrDie(aos::testing::ArtifactPath(
                  absl::StrCat("external/april_tag_test_image/expected.",
                               ExtensionForCompression(GetParam()))));
          std::string_view data(
              reinterpret_cast<const char *>(image.data()->data()),
              image.data()->size());
          EXPECT_EQ(expected_contents, data);
          aos::util::WriteStringToFileOrDie(output_path_, data);
          factory_.Exit();
        });
  }

  aos::FlatbufferDetachedBuffer<aos::Configuration> config_;
  aos::SimulatedEventLoopFactory factory_;
  aos::FlatbufferVector<CameraImage> camera_image_;
  const aos::Node *const node_;
  std::unique_ptr<aos::EventLoop> test_event_loop_;
  aos::Sender<CameraImage> image_sender_;
  std::unique_ptr<aos::EventLoop> converter_event_loop_;
  FoxgloveImageConverter converter_;
  std::string output_path_;
};

TEST_P(ImageConverterTest, ImageToFoxglove) { factory_.Run(); }

INSTANTIATE_TEST_SUITE_P(CompressionOptions, ImageConverterTest,
                         ::testing::Values(ImageCompression::kJpeg,
                                           ImageCompression::kPng));

}  // namespace testing
}  // namespace frc971::vision
