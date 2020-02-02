#ifndef Y2020_VISION_V4L2_READER_H_
#define Y2020_VISION_V4L2_READER_H_

#include <array>
#include <string>

#include "absl/types/span.h"
#include "glog/logging.h"

#include "aos/events/event_loop.h"
#include "aos/scoped/scoped_fd.h"
#include "y2020/vision/vision_generated.h"

namespace frc971 {
namespace vision {

// Reads images from a V4L2 capture device (aka camera).
class V4L2Reader {
 public:
  // device_name is the name of the device file (like "/dev/video0").
  V4L2Reader(aos::EventLoop *event_loop, const std::string &device_name);

  V4L2Reader(const V4L2Reader &) = delete;
  V4L2Reader &operator=(const V4L2Reader &) = delete;

  // Reads the latest image.
  //
  // Returns an empty span if no image was available since this object was
  // created. The data referenced in the return value is valid until this method
  // is called again.
  absl::Span<const char> ReadLatestImage();

  // Sends the latest image.
  //
  // ReadLatestImage() must have returned a non-empty span the last time it was
  // called. After calling this, the data which was returned from
  // ReadLatestImage() will no longer be valid.
  void SendLatestImage();

 private:
  static constexpr int kNumberBuffers = 16;

  struct Buffer {
    void InitializeMessage(size_t max_image_size) {
      builder = aos::Sender<CameraImage>::Builder();
      builder = sender.MakeBuilder();
      // The kernel has an undocumented requirement that the buffer is aligned
      // to 64 bytes. If you give it a nonaligned pointer, it will return EINVAL
      // and only print something in dmesg with the relevant dynamic debug
      // prints turned on.
      builder.fbb()->StartIndeterminateVector(max_image_size, 1, 64,
                                              &data_pointer);
      CHECK_EQ(reinterpret_cast<uintptr_t>(data_pointer) % 64, 0u)
          << ": Flatbuffers failed to align things as requested";
    }

    void Send(int rows, int cols, size_t image_size) {
      const auto data_offset =
          builder.fbb()->EndIndeterminateVector(image_size, 1);
      auto image_builder = builder.MakeBuilder<CameraImage>();
      image_builder.add_data(data_offset);
      image_builder.add_rows(rows);
      image_builder.add_cols(cols);
      builder.Send(image_builder.Finish());
      data_pointer = nullptr;
    }

    absl::Span<const char> DataSpan(size_t image_size) {
      return absl::Span<const char>(reinterpret_cast<char *>(data_pointer),
                                    image_size);
    }

    aos::Sender<CameraImage> sender;
    aos::Sender<CameraImage>::Builder builder;

    uint8_t *data_pointer = nullptr;
  };

  // TODO(Brian): This concept won't exist once we start using variable-size
  // H.264 frames.
  size_t ImageSize() const { return rows_ * cols_ * 2 /* bytes per pixel */; }

  // Attempts to dequeue a buffer (nonblocking). Returns the index of the new
  // buffer, or -1 if there wasn't a frame to dequeue.
  int DequeueBuffer();

  void EnqueueBuffer(int buffer);

  int Ioctl(unsigned long number, void *arg);

  // The mmaped V4L2 buffers.
  std::array<Buffer, kNumberBuffers> buffers_;

  // If this is non-negative, it's the buffer number we're currently holding
  // onto.
  int saved_buffer_ = -1;

  const int rows_ = 480;
  const int cols_ = 640;

  aos::ScopedFD fd_;
};

}  // namespace vision
}  // namespace frc971

#endif  // Y2020_VISION_V4L2_READER_H_
