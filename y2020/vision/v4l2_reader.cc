#include "y2020/vision/v4l2_reader.h"

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>

namespace frc971 {
namespace vision {

V4L2Reader::V4L2Reader(aos::EventLoop *event_loop,
                       const std::string &device_name)
    : fd_(open(device_name.c_str(), O_RDWR | O_NONBLOCK)) {
  PCHECK(fd_.get() != -1);

  // First, clean up after anybody else who left the device streaming.
  {
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    PCHECK(Ioctl(VIDIOC_STREAMOFF, &type) == 0);
  }

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

absl::Span<const char> V4L2Reader::ReadLatestImage() {
  // First, enqueue any old buffer we already have. This is the one which may
  // have been sent.
  if (saved_buffer_ != -1) {
    EnqueueBuffer(saved_buffer_);
    saved_buffer_ = -1;
  }
  while (true) {
    const int previous_buffer = saved_buffer_;
    saved_buffer_ = DequeueBuffer();
    if (saved_buffer_ != -1) {
      // We got a new buffer. Return the previous one (if relevant) and keep
      // going.
      if (previous_buffer != -1) {
        EnqueueBuffer(previous_buffer);
      }
      continue;
    }
    if (previous_buffer == -1) {
      // There were no images to read. Return an indication of that.
      return absl::Span<const char>();
    }
    // We didn't get a new one, but we already got one in a previous
    // iteration, which means we found an image so return it.
    saved_buffer_ = previous_buffer;
    return buffers_[saved_buffer_].DataSpan(ImageSize());
  }
}

void V4L2Reader::SendLatestImage() {
  buffers_[saved_buffer_].Send(rows_, cols_, ImageSize());
}

int V4L2Reader::Ioctl(unsigned long number, void *arg) {
  return ioctl(fd_.get(), number, arg);
}

int V4L2Reader::DequeueBuffer() {
  struct v4l2_buffer buffer;
  memset(&buffer, 0, sizeof(buffer));
  buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buffer.memory = V4L2_MEMORY_USERPTR;
  const int result = Ioctl(VIDIOC_DQBUF, &buffer);
  if (result == -1 && errno == EAGAIN) {
    return -1;
  }
  PCHECK(result == 0) << ": VIDIOC_DQBUF failed";
  CHECK_LT(buffer.index, buffers_.size());
  LOG(INFO) << "dequeued " << buffer.index;
  CHECK_EQ(reinterpret_cast<uintptr_t>(buffers_[buffer.index].data_pointer),
           buffer.m.userptr);
  CHECK_EQ(ImageSize(), buffer.length);
  return buffer.index;
}

void V4L2Reader::EnqueueBuffer(int buffer_number) {
  LOG(INFO) << "enqueueing " << buffer_number;
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

}  // namespace vision
}  // namespace frc971
