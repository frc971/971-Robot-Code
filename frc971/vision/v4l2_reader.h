#ifndef FRC971_VISION_V4L2_READER_H_
#define FRC971_VISION_V4L2_READER_H_

#include <array>
#include <string>

#include "absl/types/span.h"
#include "glog/logging.h"

#include "aos/containers/ring_buffer.h"
#include "aos/events/epoll.h"
#include "aos/events/event_loop.h"
#include "aos/ftrace.h"
#include "aos/realtime.h"
#include "aos/scoped/scoped_fd.h"
#include "aos/util/threaded_consumer.h"
#include "frc971/vision/vision_generated.h"

namespace frc971 {
namespace vision {

// Reads images from a V4L2 capture device (aka camera).
class V4L2ReaderBase {
 public:
  // device_name is the name of the device file (like "/dev/video0").
  // image_channel is the channel to send images on
  V4L2ReaderBase(aos::EventLoop *event_loop, std::string_view device_name,
                 std::string_view image_channel);

  V4L2ReaderBase(const V4L2ReaderBase &) = delete;
  V4L2ReaderBase &operator=(const V4L2ReaderBase &) = delete;

  // Reads the latest image.
  //
  // Returns false if no image was available since the last image was read.
  // Call LatestImage() to get a reference to the data, which will be valid
  // until this method is called again.
  bool ReadLatestImage();

  void MaybeEnqueue();

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
  virtual void SetExposure(size_t duration);

  // Switches from manual to auto exposure.
  void UseAutoExposure();

 protected:
  void StreamOff();
  void StreamOn();

  // Enqueues a buffer for v4l2 to stream into (expensive).
  void EnqueueBuffer(int buffer_index);

  // Initializations that need to happen in the main thread.
  //
  // Implementations of MarkBufferToBeEnqueued should call this before calling
  // EnqueueBuffer.
  void ReinitializeBuffer(int buffer_index) {
    CHECK_GE(buffer_index, 0);
    CHECK_LT(buffer_index, static_cast<int>(buffers_.size()));
    buffers_[buffer_index].InitializeMessage(ImageSize());
  }

  // Submits a buffer to be enqueued later in a lower priority thread.
  // Legacy V4L2Reader still does this in the main thread.
  virtual void MarkBufferToBeEnqueued(int buffer_index);

  int Ioctl(unsigned long number, void *arg);

  bool multiplanar() const { return multiplanar_; }

  // TODO(Brian): This concept won't exist once we start using variable-size
  // H.264 frames.
  size_t ImageSize() const { return ImageSize(rows_, cols_); }
  static size_t ImageSize(int rows, int cols) {
    return rows * cols * 2 /* bytes per pixel */;
  }

  const aos::ScopedFD &fd() { return fd_; };

  static constexpr int kNumberBuffers = 4;

 private:
  struct Buffer {
    void InitializeMessage(size_t max_image_size);

    void PrepareMessage(int rows, int cols, size_t image_size,
                        aos::monotonic_clock::time_point monotonic_eof);

    void Send() {
      builder.CheckOk(builder.Send(message_offset));
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

    std::string_view image_channel_;
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

  // Attempts to dequeue a buffer (nonblocking). Returns the index of the new
  // buffer, or BufferInfo() if there wasn't a frame to dequeue.
  BufferInfo DequeueBuffer();

  // The mmaped V4L2 buffers.
  std::array<Buffer, kNumberBuffers> buffers_;

  // If this is non-negative, it's the buffer number we're currently holding
  // onto.
  BufferInfo saved_buffer_;

  bool multiplanar_ = false;

  int rows_ = 0;
  int cols_ = 0;

  aos::ScopedFD fd_;

  aos::EventLoop *event_loop_;
  aos::Ftrace ftrace_;

  std::string_view image_channel_;
};

// Generic V4L2 reader for pi's and older.
class V4L2Reader : public V4L2ReaderBase {
 public:
  V4L2Reader(aos::EventLoop *event_loop, std::string_view device_name,
             std::string_view image_channel = "/camera");
};

// Rockpi specific v4l2 reader.  This assumes that the media device has been
// properly configured before this class is constructed.
class RockchipV4L2Reader : public V4L2ReaderBase {
 public:
  RockchipV4L2Reader(aos::EventLoop *event_loop, aos::internal::EPoll *epoll,
                     std::string_view device_name,
                     std::string_view image_sensor_subdev,
                     std::string_view image_channel = "/camera");

  virtual ~RockchipV4L2Reader();

  void SetExposure(size_t duration) override;

  void SetGain(size_t gain);
  void SetGainExt(size_t gain);

  void SetVerticalBlanking(size_t vblank);

 private:
  void OnImageReady();

  void MarkBufferToBeEnqueued(int buffer) override;

  int ImageSensorIoctl(unsigned long number, void *arg);

  aos::internal::EPoll *epoll_;

  aos::ScopedFD image_sensor_fd_;

  static constexpr int kEnqueueFifoPriority = 1;

  aos::util::ThreadedConsumer<int, kNumberBuffers> buffer_requeuer_;
};

}  // namespace vision
}  // namespace frc971

#endif  // FRC971_VISION_V4L2_READER_H_
