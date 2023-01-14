#include "frc971/vision/v4l2_reader.h"

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>

DEFINE_bool(ignore_timestamps, false,
            "Don't require timestamps on images.  Used to allow webcams");

namespace frc971 {
namespace vision {

V4L2ReaderBase::V4L2ReaderBase(aos::EventLoop *event_loop,
                               const std::string &device_name)
    : fd_(open(device_name.c_str(), O_RDWR | O_NONBLOCK)),
      event_loop_(event_loop) {
  PCHECK(fd_.get() != -1) << " Failed to open device " << device_name;

  // Figure out if we are multi-planar or not.
  {
    struct v4l2_capability capability;
    memset(&capability, 0, sizeof(capability));
    PCHECK(Ioctl(VIDIOC_QUERYCAP, &capability) == 0);

    LOG(INFO) << "Opening " << device_name;
    LOG(INFO) << "  driver " << capability.driver;
    LOG(INFO) << "  card " << capability.card;
    LOG(INFO) << "  bus_info " << capability.bus_info;
    if (capability.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE) {
      LOG(INFO) << "  Multi-planar";
      multiplanar_ = true;
    }
  }

  // First, clean up after anybody else who left the device streaming.
  StreamOff();
}

void V4L2ReaderBase::StreamOn() {
  {
    struct v4l2_requestbuffers request;
    memset(&request, 0, sizeof(request));
    request.count = buffers_.size();
    request.type = multiplanar() ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
                                 : V4L2_BUF_TYPE_VIDEO_CAPTURE;
    request.memory = V4L2_MEMORY_USERPTR;
    PCHECK(Ioctl(VIDIOC_REQBUFS, &request) == 0);
    CHECK_EQ(request.count, buffers_.size())
        << ": Kernel refused to give us the number of buffers we asked for";
  }

  {
    struct v4l2_format format;
    memset(&format, 0, sizeof(format));
    format.type = multiplanar() ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
                                : V4L2_BUF_TYPE_VIDEO_CAPTURE;
    PCHECK(Ioctl(VIDIOC_G_FMT, &format) == 0);

    if (multiplanar()) {
      cols_ = format.fmt.pix_mp.width;
      rows_ = format.fmt.pix_mp.height;
      LOG(INFO) << "Format is " << cols_ << ", " << rows_;
      CHECK_EQ(format.fmt.pix_mp.pixelformat, V4L2_PIX_FMT_YUYV)
          << ": Invalid pixel format";

      CHECK_EQ(format.fmt.pix_mp.num_planes, 1u);

      CHECK_EQ(static_cast<int>(format.fmt.pix_mp.plane_fmt[0].bytesperline),
               cols_ * 2 /* bytes per pixel */);
      CHECK_EQ(format.fmt.pix_mp.plane_fmt[0].sizeimage, ImageSize());
    } else {
      cols_ = format.fmt.pix.width;
      rows_ = format.fmt.pix.height;
      LOG(INFO) << "Format is " << cols_ << ", " << rows_;
      CHECK_EQ(format.fmt.pix.pixelformat, V4L2_PIX_FMT_YUYV)
          << ": Invalid pixel format";

      CHECK_EQ(static_cast<int>(format.fmt.pix.bytesperline),
               cols_ * 2 /* bytes per pixel */);
      CHECK_EQ(format.fmt.pix.sizeimage, ImageSize());
    }
  }

  for (size_t i = 0; i < buffers_.size(); ++i) {
    buffers_[i].sender = event_loop_->MakeSender<CameraImage>("/camera");
    EnqueueBuffer(i);
  }
  int type = multiplanar() ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
                           : V4L2_BUF_TYPE_VIDEO_CAPTURE;
  PCHECK(Ioctl(VIDIOC_STREAMON, &type) == 0);
}

void V4L2ReaderBase::MaybeEnqueue() {
  // First, enqueue any old buffer we already have. This is the one which
  // may have been sent.
  if (saved_buffer_) {
    EnqueueBuffer(saved_buffer_.index);
    saved_buffer_.Clear();
  }
  ftrace_.FormatMessage("Enqueued previous buffer %d", saved_buffer_.index);
}

bool V4L2ReaderBase::ReadLatestImage() {
  MaybeEnqueue();

  while (true) {
    const BufferInfo previous_buffer = saved_buffer_;
    saved_buffer_ = DequeueBuffer();
    ftrace_.FormatMessage("Dequeued %d", saved_buffer_.index);
    if (saved_buffer_) {
      // We got a new buffer. Return the previous one (if relevant) and keep
      // going.
      if (previous_buffer) {
        ftrace_.FormatMessage("Previous %d", previous_buffer.index);
        EnqueueBuffer(previous_buffer.index);
      }
      continue;
    }
    if (!previous_buffer) {
      // There were no images to read. Return an indication of that.
      ftrace_.FormatMessage("No images to read");
      return false;
    }
    // We didn't get a new one, but we already got one in a previous
    // iteration, which means we found an image so return it.
    ftrace_.FormatMessage("Got saved buffer %d", saved_buffer_.index);
    saved_buffer_ = previous_buffer;
    buffers_[saved_buffer_.index].PrepareMessage(rows_, cols_, ImageSize(),
                                                 saved_buffer_.monotonic_eof);
    return true;
  }
}

void V4L2ReaderBase::SendLatestImage() { buffers_[saved_buffer_.index].Send(); }

void V4L2ReaderBase::SetExposure(size_t duration) {
  v4l2_control manual_control;
  manual_control.id = V4L2_CID_EXPOSURE_AUTO;
  manual_control.value = V4L2_EXPOSURE_MANUAL;
  PCHECK(Ioctl(VIDIOC_S_CTRL, &manual_control) == 0);

  v4l2_control exposure_control;
  exposure_control.id = V4L2_CID_EXPOSURE_ABSOLUTE;
  exposure_control.value = static_cast<int>(duration);  // 100 micro s units
  PCHECK(Ioctl(VIDIOC_S_CTRL, &exposure_control) == 0);
}

void V4L2ReaderBase::UseAutoExposure() {
  v4l2_control control;
  control.id = V4L2_CID_EXPOSURE_AUTO;
  control.value = V4L2_EXPOSURE_AUTO;
  PCHECK(Ioctl(VIDIOC_S_CTRL, &control) == 0);
}

void V4L2ReaderBase::Buffer::InitializeMessage(size_t max_image_size) {
  message_offset = flatbuffers::Offset<CameraImage>();
  builder = aos::Sender<CameraImage>::Builder();
  builder = sender.MakeBuilder();
  // The kernel has an undocumented requirement that the buffer is aligned
  // to 128 bytes. If you give it a nonaligned pointer, it will return EINVAL
  // and only print something in dmesg with the relevant dynamic debug
  // prints turned on.
  builder.fbb()->StartIndeterminateVector(max_image_size, 1, 128,
                                          &data_pointer);
  CHECK_EQ(reinterpret_cast<uintptr_t>(data_pointer) % 128, 0u)
      << ": Flatbuffers failed to align things as requested";
}

void V4L2ReaderBase::Buffer::PrepareMessage(
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

int V4L2ReaderBase::Ioctl(unsigned long number, void *arg) {
  return ioctl(fd_.get(), number, arg);
}

V4L2ReaderBase::BufferInfo V4L2ReaderBase::DequeueBuffer() {
  struct v4l2_buffer buffer;
  memset(&buffer, 0, sizeof(buffer));
  buffer.memory = V4L2_MEMORY_USERPTR;
  if (multiplanar()) {
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    struct v4l2_plane planes[1];
    std::memset(planes, 0, sizeof(planes));
    buffer.m.planes = planes;
    buffer.length = 1;
    const int result = Ioctl(VIDIOC_DQBUF, &buffer);
    if (result == -1 && errno == EAGAIN) {
      return BufferInfo();
    }
    PCHECK(result == 0) << ": VIDIOC_DQBUF failed";
    CHECK_LT(buffer.index, buffers_.size());

    CHECK_EQ(reinterpret_cast<uintptr_t>(buffers_[buffer.index].data_pointer),
             planes[0].m.userptr);

    CHECK_EQ(ImageSize(), planes[0].length);
  } else {
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    const int result = Ioctl(VIDIOC_DQBUF, &buffer);
    if (result == -1 && errno == EAGAIN) {
      return BufferInfo();
    }
    PCHECK(result == 0) << ": VIDIOC_DQBUF failed";
    CHECK_LT(buffer.index, buffers_.size());
    CHECK_EQ(reinterpret_cast<uintptr_t>(buffers_[buffer.index].data_pointer),
             buffer.m.userptr);
    CHECK_EQ(ImageSize(), buffer.length);
  }
  CHECK(buffer.flags & V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC);
  if (!FLAGS_ignore_timestamps) {
    // Require that we have good timestamp on images
    CHECK_EQ(buffer.flags & V4L2_BUF_FLAG_TSTAMP_SRC_MASK,
             static_cast<uint32_t>(V4L2_BUF_FLAG_TSTAMP_SRC_EOF));
  }
  return {static_cast<int>(buffer.index),
          aos::time::from_timeval(buffer.timestamp)};
}

void V4L2ReaderBase::EnqueueBuffer(int buffer_number) {
  // TODO(austin): Detect multiplanar and do this all automatically.

  CHECK_GE(buffer_number, 0);
  CHECK_LT(buffer_number, static_cast<int>(buffers_.size()));
  buffers_[buffer_number].InitializeMessage(ImageSize());
  struct v4l2_buffer buffer;
  struct v4l2_plane planes[1];
  memset(&buffer, 0, sizeof(buffer));
  memset(&planes, 0, sizeof(planes));
  buffer.memory = V4L2_MEMORY_USERPTR;
  buffer.index = buffer_number;
  if (multiplanar()) {
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    buffer.m.planes = planes;
    buffer.length = 1;
    planes[0].m.userptr =
        reinterpret_cast<uintptr_t>(buffers_[buffer_number].data_pointer);
    planes[0].length = ImageSize();
    planes[0].bytesused = planes[0].length;
  } else {
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.m.userptr =
        reinterpret_cast<uintptr_t>(buffers_[buffer_number].data_pointer);
    buffer.length = ImageSize();
  }

  PCHECK(Ioctl(VIDIOC_QBUF, &buffer) == 0);
}

void V4L2ReaderBase::StreamOff() {
  int type = multiplanar() ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
                           : V4L2_BUF_TYPE_VIDEO_CAPTURE;
  const int result = Ioctl(VIDIOC_STREAMOFF, &type);
  if (result == 0) {
    return;
  }
  // Some devices (like Alex's webcam) return this if streaming isn't
  // currently on, unlike what the documentations says should happen.
  if (errno == EBUSY) {
    return;
  }
  PLOG(FATAL) << "VIDIOC_STREAMOFF failed";
}

V4L2Reader::V4L2Reader(aos::EventLoop *event_loop,
                       const std::string &device_name)
    : V4L2ReaderBase(event_loop, device_name) {
  // Don't know why this magic call to SetExposure is required (before the
  // camera settings are configured) to make things work on boot of the pi, but
  // it seems to be-- without it, the image exposure is wrong (too dark). Note--
  // any valid value seems to work-- just choosing 1 for now

  SetExposure(1);

  struct v4l2_format format;
  memset(&format, 0, sizeof(format));
  format.type = multiplanar() ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
                              : V4L2_BUF_TYPE_VIDEO_CAPTURE;
  format.fmt.pix.width = cols_;
  format.fmt.pix.height = rows_;
  format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
  // This means we want to capture from a progressive (non-interlaced)
  // source.
  format.fmt.pix.field = V4L2_FIELD_NONE;
  PCHECK(Ioctl(VIDIOC_S_FMT, &format) == 0);
  CHECK_EQ(static_cast<int>(format.fmt.pix.width), cols_);
  CHECK_EQ(static_cast<int>(format.fmt.pix.height), rows_);
  CHECK_EQ(static_cast<int>(format.fmt.pix.bytesperline),
           cols_ * 2 /* bytes per pixel */);
  CHECK_EQ(format.fmt.pix.sizeimage, ImageSize());

  StreamOn();
}

RockchipV4L2Reader::RockchipV4L2Reader(aos::EventLoop *event_loop,
                                       aos::internal::EPoll *epoll,
                                       const std::string &device_name,
                                       const std::string &image_sensor_subdev)
    : V4L2ReaderBase(event_loop, device_name),
      epoll_(epoll),
      image_sensor_fd_(open(image_sensor_subdev.c_str(), O_RDWR | O_NONBLOCK)) {
  PCHECK(image_sensor_fd_.get() != -1)
      << " Failed to open device " << device_name;

  StreamOn();
  epoll_->OnReadable(fd().get(), [this]() { OnImageReady(); });
}

void RockchipV4L2Reader::OnImageReady() {
  if (!ReadLatestImage()) {
    return;
  }

  SendLatestImage();
}

int RockchipV4L2Reader::ImageSensorIoctl(unsigned long number, void *arg) {
  return ioctl(image_sensor_fd_.get(), number, arg);
}

void RockchipV4L2Reader::SetExposure(size_t duration) {
  v4l2_control exposure_control;
  exposure_control.id = V4L2_CID_EXPOSURE;
  exposure_control.value = static_cast<int>(duration);
  PCHECK(ImageSensorIoctl(VIDIOC_S_CTRL, &exposure_control) == 0);
}

void RockchipV4L2Reader::SetGain(size_t gain) {
  struct v4l2_ext_controls controls;
  memset(&controls, 0, sizeof(controls));
  struct v4l2_ext_control control[1];
  memset(&control, 0, sizeof(control));

  controls.ctrl_class = V4L2_CTRL_CLASS_IMAGE_SOURCE;
  controls.count = 1;
  controls.controls = control;
  control[0].id = V4L2_CID_ANALOGUE_GAIN;
  control[0].value = gain;

  PCHECK(ImageSensorIoctl(VIDIOC_S_EXT_CTRLS, &controls) == 0);
}

}  // namespace vision
}  // namespace frc971
