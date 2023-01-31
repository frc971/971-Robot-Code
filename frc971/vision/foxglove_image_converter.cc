#include "frc971/vision/foxglove_image_converter.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace frc971::vision {
namespace {
std::string_view ExtensionForCompression(ImageCompression compression) {
  switch (compression) {
    case ImageCompression::kJpeg:
      return "jpeg";
    case ImageCompression::kPng:
      return "png";
  }
}
}  // namespace
flatbuffers::Offset<foxglove::CompressedImage> CompressImage(
    const CameraImage *raw_image, flatbuffers::FlatBufferBuilder *fbb,
    ImageCompression compression) {
  std::string_view format = ExtensionForCompression(compression);
  // imencode doesn't let us pass in anything other than an std::vector, and
  // performance isn't yet a big enough issue to try to avoid the copy.
  std::vector<uint8_t> buffer;
  CHECK(raw_image->has_data());
  cv::Mat image_color_mat(cv::Size(raw_image->cols(), raw_image->rows()),
                          CV_8UC2, (void *)raw_image->data()->data());
  cv::Mat bgr_image(cv::Size(raw_image->cols(), raw_image->rows()), CV_8UC3);
  cv::cvtColor(image_color_mat, bgr_image, cv::COLOR_YUV2BGR_YUYV);
  CHECK(cv::imencode(absl::StrCat(".", format), bgr_image, buffer));
  const flatbuffers::Offset<flatbuffers::Vector<uint8_t>> data_offset =
      fbb->CreateVector(buffer);
  const struct timespec timestamp_t =
      aos::time::to_timespec(aos::monotonic_clock::time_point(
          std::chrono::nanoseconds(raw_image->monotonic_timestamp_ns())));
  const foxglove::Time time{static_cast<uint32_t>(timestamp_t.tv_sec),
                            static_cast<uint32_t>(timestamp_t.tv_nsec)};
  const flatbuffers::Offset<flatbuffers::String> format_offset =
      fbb->CreateString(format);
  foxglove::CompressedImage::Builder builder(*fbb);
  builder.add_timestamp(&time);
  builder.add_data(data_offset);
  builder.add_format(format_offset);
  return builder.Finish();
}

FoxgloveImageConverter::FoxgloveImageConverter(aos::EventLoop *event_loop,
                                               std::string_view input_channel,
                                               std::string_view output_channel,
                                               ImageCompression compression)
    : event_loop_(event_loop),
      sender_(
          event_loop_->MakeSender<foxglove::CompressedImage>(output_channel)) {
  event_loop_->MakeWatcher(input_channel, [this, compression](
                                              const CameraImage &image) {
    auto builder = sender_.MakeBuilder();
    builder.CheckOk(
        builder.Send(CompressImage(&image, builder.fbb(), compression)));
  });
}
}  // namespace frc971::vision
