#ifndef FRC971_VISION_V4L2_READER_H_
#define FRC971_VISION_V4L2_READER_H_

#include <array>
#include <string>

#include "absl/types/span.h"
#include "glog/logging.h"

#include "aos/events/event_loop.h"
#include "aos/scoped/scoped_fd.h"
#include "frc971/vision/vision_generated.h"

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
  // Returns false if no image was available since the last image was read.
  // Call LatestImage() to get a reference to the data, which will be valid
  // until this method is called again.
  bool ReadLatestImage();

  // Sends the latest image.
  //
  // ReadLatestImage() must have returned a non-empty span the last time it was
  // called. After calling this, the data which was returned from
  // ReadLatestImage() will no longer be valid.
  void SendLatestImage();

  const CameraImage &LatestImage() {
    Buffer *const buffer = &buffers_[saved_buffer_.index];
    return *flatbuffers::GetTemporaryPointer(*buffer->builder.fbb(),
                                             buffer->message_offset);
  }

  // Sets the exposure duration of the camera. duration is the number of 100
  // microsecond units.
  void SetExposure(size_t duration);

  // Switches from manual to auto exposure.
  void UseAutoExposure();

 private:
  static constexpr int kNumberBuffers = 16;

  struct Buffer {
    void InitializeMessage(size_t max_image_size);

    void PrepareMessage(int rows, int cols, size_t image_size,
                        aos::monotonic_clock::time_point monotonic_eof);

    void Send() {
      (void)builder.Send(message_offset);
      message_offset = flatbuffers::Offset<CameraImage>();
    }

    absl::Span<const char> DataSpan(size_t image_size) {
      return absl::Span<const char>(
          reinterpret_cast<char *>(CHECK_NOTNULL(data_pointer)), image_size);
    }

    aos::Sender<CameraImage> sender;
    aos::Sender<CameraImage>::Builder builder;
    flatbuffers::Offset<CameraImage> message_offset;

    uint8_t *data_pointer = nullptr;
  };

  struct BufferInfo {
    int index = -1;
    aos::monotonic_clock::time_point monotonic_eof =
        aos::monotonic_clock::min_time;

    explicit operator bool() const { return index != -1; }

    void Clear() {
      index = -1;
      monotonic_eof = aos::monotonic_clock::min_time;
    }
  };

  // TODO(Brian): This concept won't exist once we start using variable-size
  // H.264 frames.
  size_t ImageSize() const { return rows_ * cols_ * 2 /* bytes per pixel */; }

  // Attempts to dequeue a buffer (nonblocking). Returns the index of the new
  // buffer, or BufferInfo() if there wasn't a frame to dequeue.
  BufferInfo DequeueBuffer();

  void EnqueueBuffer(int buffer);

  int Ioctl(unsigned long number, void *arg);

  void StreamOff();

  // The mmaped V4L2 buffers.
  std::array<Buffer, kNumberBuffers> buffers_;

  // If this is non-negative, it's the buffer number we're currently holding
  // onto.
  BufferInfo saved_buffer_;

  const int rows_ = 480;
  const int cols_ = 640;

  aos::ScopedFD fd_;
};

}  // namespace vision
}  // namespace frc971

#endif  // FRC971_VISION_V4L2_READER_H_
