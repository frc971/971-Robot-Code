#include "y2020/vision/v4l2_reader.h"

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>

DEFINE_bool(ignore_timestamps, false,
            "Don't require timestamps on images.  Used to allow webcams");

namespace frc971 {
namespace vision {

V4L2Reader::V4L2Reader(aos::EventLoop *event_loop,
                       const std::string &device_name)
    : fd_(open(device_name.c_str(), O_RDWR | O_NONBLOCK)) {
  PCHECK(fd_.get() != -1);

  // First, clean up after anybody else who left the device streaming.
  StreamOff();

  {
    struct v4l2_format format;
    memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = cols_;
    format.fmt.pix.height = rows_;
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    // This means we want to capture from a progressive (non-interlaced) source.
    format.fmt.pix.field = V4L2_FIELD_NONE;
    PCHECK(Ioctl(VIDIOC_S_FMT, &format) == 0);
    CHECK_EQ(static_cast<int>(format.fmt.pix.width), cols_);
    CHECK_EQ(static_cast<int>(format.fmt.pix.height), rows_);
    CHECK_EQ(static_cast<int>(format.fmt.pix.bytesperline),
             cols_ * 2 /* bytes per pixel */);
    CHECK_EQ(format.fmt.pix.sizeimage, ImageSize());
  }

  {
    struct v4l2_requestbuffers request;
    memset(&request, 0, sizeof(request));
    request.count = buffers_.size();
    request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    request.memory = V4L2_MEMORY_USERPTR;
    PCHECK(Ioctl(VIDIOC_REQBUFS, &request) == 0);
    CHECK_EQ(request.count, buffers_.size())
        << ": Kernel refused to give us the number of buffers we asked for";
  }

  for (size_t i = 0; i < buffers_.size(); ++i) {
    buffers_[i].sender = event_loop->MakeSender<CameraImage>("/camera");
    EnqueueBuffer(i);
  }

  {
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    PCHECK(Ioctl(VIDIOC_STREAMON, &type) == 0);
  }
}

bool V4L2Reader::ReadLatestImage() {
  // First, enqueue any old buffer we already have. This is the one which may
  // have been sent.
  if (saved_buffer_) {
    EnqueueBuffer(saved_buffer_.index);
    saved_buffer_.Clear();
  }
  while (true) {
    const BufferInfo previous_buffer = saved_buffer_;
    saved_buffer_ = DequeueBuffer();
    if (saved_buffer_) {
      // We got a new buffer. Return the previous one (if relevant) and keep
      // going.
      if (previous_buffer) {
        EnqueueBuffer(previous_buffer.index);
      }
      continue;
    }
    if (!previous_buffer) {
      // There were no images to read. Return an indication of that.
      return false;
    }
    // We didn't get a new one, but we already got one in a previous
    // iteration, which means we found an image so return it.
    saved_buffer_ = previous_buffer;
    buffers_[saved_buffer_.index].PrepareMessage(rows_, cols_, ImageSize(),
                                                 saved_buffer_.monotonic_eof);
    return true;
  }
}

void V4L2Reader::SendLatestImage() { buffers_[saved_buffer_.index].Send(); }

void V4L2Reader::SetExposure(size_t duration) {
  v4l2_control manual_control;
  manual_control.id = V4L2_CID_EXPOSURE_AUTO;
  manual_control.value = V4L2_EXPOSURE_MANUAL;
  PCHECK(Ioctl(VIDIOC_S_CTRL, &manual_control) == 0);

  v4l2_control exposure_control;
  exposure_control.id = V4L2_CID_EXPOSURE_ABSOLUTE;
  exposure_control.value = static_cast<int>(duration);  // 100 micro s units
  PCHECK(Ioctl(VIDIOC_S_CTRL, &exposure_control) == 0);
}

void V4L2Reader::UseAutoExposure() {
  v4l2_control control;
  control.id = V4L2_CID_EXPOSURE_AUTO;
  control.value = V4L2_EXPOSURE_AUTO;
  PCHECK(Ioctl(VIDIOC_S_CTRL, &control) == 0);
}

void V4L2Reader::Buffer::InitializeMessage(size_t max_image_size) {
  message_offset = flatbuffers::Offset<CameraImage>();
  builder = aos::Sender<CameraImage>::Builder();
  builder = sender.MakeBuilder();
  // The kernel has an undocumented requirement that the buffer is aligned
  // to 64 bytes. If you give it a nonaligned pointer, it will return EINVAL
  // and only print something in dmesg with the relevant dynamic debug
  // prints turned on.
  builder.fbb()->StartIndeterminateVector(max_image_size, 1, 64, &data_pointer);
  CHECK_EQ(reinterpret_cast<uintptr_t>(data_pointer) % 64, 0u)
      << ": Flatbuffers failed to align things as requested";
}

void V4L2Reader::Buffer::PrepareMessage(
    int rows, int cols, size_t image_size,
    aos::monotonic_clock::time_point monotonic_eof) {
  CHECK(data_pointer != nullptr);
  data_pointer = nullptr;

  const auto data_offset = builder.fbb()->EndIndeterminateVector(image_size, 1);
  auto image_builder = builder.MakeBuilder<CameraImage>();
  image_builder.add_data(data_offset);
  image_builder.add_rows(rows);
  image_builder.add_cols(cols);
  image_builder.add_monotonic_timestamp_ns(
      std::chrono::nanoseconds(monotonic_eof.time_since_epoch()).count());
  message_offset = image_builder.Finish();
}

int V4L2Reader::Ioctl(unsigned long number, void *arg) {
  return ioctl(fd_.get(), number, arg);
}

V4L2Reader::BufferInfo V4L2Reader::DequeueBuffer() {
  struct v4l2_buffer buffer;
  memset(&buffer, 0, sizeof(buffer));
  buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buffer.memory = V4L2_MEMORY_USERPTR;
  const int result = Ioctl(VIDIOC_DQBUF, &buffer);
  if (result == -1 && errno == EAGAIN) {
    return BufferInfo();
  }
  PCHECK(result == 0) << ": VIDIOC_DQBUF failed";
  CHECK_LT(buffer.index, buffers_.size());
  CHECK_EQ(reinterpret_cast<uintptr_t>(buffers_[buffer.index].data_pointer),
           buffer.m.userptr);
  CHECK_EQ(ImageSize(), buffer.length);
  CHECK(buffer.flags & V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC);
  if (!FLAGS_ignore_timestamps) {
    // Require that we have good timestamp on images
    CHECK_EQ(buffer.flags & V4L2_BUF_FLAG_TSTAMP_SRC_MASK,
             static_cast<uint32_t>(V4L2_BUF_FLAG_TSTAMP_SRC_EOF));
  }
  return {static_cast<int>(buffer.index),
          aos::time::from_timeval(buffer.timestamp)};
}

void V4L2Reader::EnqueueBuffer(int buffer_number) {
  CHECK_GE(buffer_number, 0);
  CHECK_LT(buffer_number, static_cast<int>(buffers_.size()));
  buffers_[buffer_number].InitializeMessage(ImageSize());
  struct v4l2_buffer buffer;
  memset(&buffer, 0, sizeof(buffer));
  buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buffer.memory = V4L2_MEMORY_USERPTR;
  buffer.index = buffer_number;
  buffer.m.userptr =
      reinterpret_cast<uintptr_t>(buffers_[buffer_number].data_pointer);
  buffer.length = ImageSize();
  PCHECK(Ioctl(VIDIOC_QBUF, &buffer) == 0);
}

void V4L2Reader::StreamOff() {
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  const int result = Ioctl(VIDIOC_STREAMOFF, &type);
  if (result == 0) {
    return;
  }
  // Some devices (like Alex's webcam) return this if streaming isn't currently
  // on, unlike what the documentations says should happen.
  if (errno == EBUSY) {
    return;
  }
  PLOG(FATAL) << "VIDIOC_STREAMOFF failed";
}

}  // namespace vision
}  // namespace frc971
