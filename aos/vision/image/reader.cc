#include "aos/vision/image/reader.h"

#include <errno.h>
#include <fcntl.h>
#include <malloc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include "aos/common/logging/logging.h"
#include "aos/common/time.h"

#define CLEAR(x) memset(&(x), 0, sizeof(x))

namespace camera {

struct Reader::Buffer {
  void *start;
  size_t length;  // for munmap
};

aos::vision::CameraParams MakeCameraParams(int32_t width, int32_t height,
                                           int32_t exposure, int32_t brightness,
                                           int32_t gain, int32_t fps) {
  aos::vision::CameraParams cam;
  cam.set_width(width);
  cam.set_height(height);
  cam.set_exposure(exposure);
  cam.set_brightness(brightness);
  cam.set_gain(gain);
  cam.set_fps(fps);
  return cam;
}

Reader::Reader(const std::string &dev_name, ProcessCb process,
               aos::vision::CameraParams params)
    : dev_name_(dev_name), process_(std::move(process)), params_(params) {
  struct stat st;
  if (stat(dev_name.c_str(), &st) == -1) {
    PLOG(FATAL, "Cannot identify '%s'", dev_name.c_str());
  }
  if (!S_ISCHR(st.st_mode)) {
    PLOG(FATAL, "%s is no device\n", dev_name.c_str());
  }

  fd_ = open(dev_name.c_str(), O_RDWR /* required */ | O_NONBLOCK, 0);
  if (fd_ == -1) {
    PLOG(FATAL, "Cannot open '%s'", dev_name.c_str());
  }

  Init();

  InitMMap();
  LOG(INFO, "Bat Vision Successfully Initialized.\n");
}

void Reader::QueueBuffer(v4l2_buffer *buf) {
  if (xioctl(fd_, VIDIOC_QBUF, buf) == -1) {
    PLOG(WARNING,
         "ioctl VIDIOC_QBUF(%d, %p)."
         " losing buf #%" PRIu32 "\n",
         fd_, &buf, buf->index);
  } else {
    //    LOG(DEBUG, "put buf #%" PRIu32 " into driver's queue\n", buf->index);
    ++queued_;
  }
}

void Reader::HandleFrame() {
  v4l2_buffer buf;
  CLEAR(buf);
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;

  if (xioctl(fd_, VIDIOC_DQBUF, &buf) == -1) {
    if (errno != EAGAIN) {
      PLOG(ERROR, "ioctl VIDIOC_DQBUF(%d, %p)", fd_, &buf);
    }
    return;
  }
  --queued_;

  if (tick_id_ % 10 == 0) {
    if (!SetCameraControl(V4L2_CID_EXPOSURE_AUTO, "V4L2_CID_EXPOSURE_AUTO",
                          V4L2_EXPOSURE_MANUAL)) {
      LOG(FATAL, "Failed to set exposure\n");
    }

    if (!SetCameraControl(V4L2_CID_EXPOSURE_ABSOLUTE,
                          "V4L2_CID_EXPOSURE_ABSOLUTE", params_.exposure())) {
      LOG(FATAL, "Failed to set exposure\n");
    }
  }
  ++tick_id_;
  // Get a timestamp now as proxy for when the image was taken
  // TODO(ben): the image should come with a timestamp, parker
  // will know how to get it.
  auto time = aos::monotonic_clock::now();

  process_(aos::vision::DataRef(
               reinterpret_cast<const char *>(buffers_[buf.index].start),
               buf.bytesused),
           time);

  QueueBuffer(&buf);
}

void Reader::MMapBuffers() {
  buffers_ = new Buffer[kNumBuffers];
  v4l2_buffer buf;
  for (unsigned int n = 0; n < kNumBuffers; ++n) {
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = n;
    if (xioctl(fd_, VIDIOC_QUERYBUF, &buf) == -1) {
      PLOG(FATAL, "ioctl VIDIOC_QUERYBUF(%d, %p)", fd_, &buf);
    }
    buffers_[n].length = buf.length;
    buffers_[n].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                             MAP_SHARED, fd_, buf.m.offset);
    if (buffers_[n].start == MAP_FAILED) {
      PLOG(FATAL,
           "mmap(NULL, %zd, PROT_READ | PROT_WRITE, MAP_SHARED, %d, %jd)",
           (size_t)buf.length, fd_, static_cast<intmax_t>(buf.m.offset));
    }
  }
}

void Reader::InitMMap() {
  v4l2_requestbuffers req;
  CLEAR(req);
  req.count = kNumBuffers;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (xioctl(fd_, VIDIOC_REQBUFS, &req) == -1) {
    if (EINVAL == errno) {
      LOG(FATAL, "%s does not support memory mapping\n", dev_name_.c_str());
    } else {
      PLOG(FATAL, "ioctl VIDIOC_REQBUFS(%d, %p)\n", fd_, &req);
    }
  }
  queued_ = kNumBuffers;
  if (req.count != kNumBuffers) {
    LOG(FATAL, "Insufficient buffer memory on %s\n", dev_name_.c_str());
  }
}

// Sets one of the camera's user-control values.
// Prints the old and new values.
// Just prints a message if the camera doesn't support this control or value.
bool Reader::SetCameraControl(uint32_t id, const char *name, int value) {
  struct v4l2_control getArg = {id, 0U};
  int r = xioctl(fd_, VIDIOC_G_CTRL, &getArg);
  if (r == 0) {
    if (getArg.value == value) {
      LOG(DEBUG, "Camera control %s was already %d\n", name, getArg.value);
      return true;
    }
  } else if (errno == EINVAL) {
    LOG(DEBUG, "Camera control %s is invalid\n", name);
    errno = 0;
    return false;
  }

  struct v4l2_control setArg = {id, value};
  r = xioctl(fd_, VIDIOC_S_CTRL, &setArg);
  if (r == 0) {
    LOG(DEBUG, "Set camera control %s from %d to %d\n", name, getArg.value,
        value);
    return true;
  }

  LOG(DEBUG, "Couldn't set camera control %s to %d", name, value);
  errno = 0;
  return false;
}

void Reader::Init() {
  v4l2_capability cap;
  if (xioctl(fd_, VIDIOC_QUERYCAP, &cap) == -1) {
    if (EINVAL == errno) {
      LOG(FATAL, "%s is no V4L2 device\n", dev_name_.c_str());
    } else {
      PLOG(FATAL, "ioctl VIDIOC_QUERYCAP(%d, %p)", fd_, &cap);
    }
  }
  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    LOG(FATAL, "%s is no video capture device\n", dev_name_.c_str());
  }
  if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
    LOG(FATAL, "%s does not support streaming i/o\n", dev_name_.c_str());
  }

  /* Select video input, video standard and tune here. */

  v4l2_cropcap cropcap;
  CLEAR(cropcap);
  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd_, VIDIOC_CROPCAP, &cropcap) == 0) {
    v4l2_crop crop;
    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    crop.c = cropcap.defrect; /* reset to default */

    if (xioctl(fd_, VIDIOC_S_CROP, &crop) == -1) {
      switch (errno) {
        case EINVAL:
          /* Cropping not supported. */
          break;
        default:
          /* Errors ignored. */
          PLOG(WARNING, "xioctl VIDIOC_S_CROP");
          break;
      }
    }
  } else {
    PLOG(WARNING, "xioctl VIDIOC_CROPCAP");
  }

  v4l2_format fmt;
  CLEAR(fmt);
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = params_.width();
  fmt.fmt.pix.height = params_.height();
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
  fmt.fmt.pix.field = V4L2_FIELD_ANY;
  // fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
  // fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
  if (xioctl(fd_, VIDIOC_S_FMT, &fmt) == -1) {
    LOG(FATAL, "ioctl VIDIC_S_FMT(%d, %p) failed with %d: %s\n", fd_, &fmt,
        errno, strerror(errno));
  }
  /* Note VIDIOC_S_FMT may change width and height. */

  /* Buggy driver paranoia. */
  unsigned int min = fmt.fmt.pix.width * 2;
  if (fmt.fmt.pix.bytesperline < min) fmt.fmt.pix.bytesperline = min;
  min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
  if (fmt.fmt.pix.sizeimage < min) fmt.fmt.pix.sizeimage = min;

  if (!SetCameraControl(V4L2_CID_EXPOSURE_AUTO, "V4L2_CID_EXPOSURE_AUTO",
                        V4L2_EXPOSURE_MANUAL)) {
    LOG(FATAL, "Failed to set exposure\n");
  }

  if (!SetCameraControl(V4L2_CID_EXPOSURE_ABSOLUTE,
                        "V4L2_CID_EXPOSURE_ABSOLUTE", params_.exposure())) {
    LOG(FATAL, "Failed to set exposure\n");
  }

  if (!SetCameraControl(V4L2_CID_BRIGHTNESS, "V4L2_CID_BRIGHTNESS",
                        params_.brightness())) {
    LOG(FATAL, "Failed to set up camera\n");
  }

  if (!SetCameraControl(V4L2_CID_GAIN, "V4L2_CID_GAIN", params_.gain())) {
    LOG(FATAL, "Failed to set up camera\n");
  }

  // set framerate
  struct v4l2_streamparm *setfps;
  setfps = (struct v4l2_streamparm *)calloc(1, sizeof(struct v4l2_streamparm));
  memset(setfps, 0, sizeof(struct v4l2_streamparm));
  setfps->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  setfps->parm.capture.timeperframe.numerator = 1;
  setfps->parm.capture.timeperframe.denominator = params_.fps();
  if (xioctl(fd_, VIDIOC_S_PARM, setfps) == -1) {
    PLOG(FATAL, "ioctl VIDIOC_S_PARM(%d, %p)\n", fd_, setfps);
  }
  LOG(INFO, "framerate ended up at %d/%d\n",
      setfps->parm.capture.timeperframe.numerator,
      setfps->parm.capture.timeperframe.denominator);
}

aos::vision::ImageFormat Reader::get_format() {
  struct v4l2_format fmt;
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd_, VIDIOC_G_FMT, &fmt) == -1) {
    PLOG(FATAL, "ioctl VIDIC_G_FMT(%d, %p)\n", fd_, &fmt);
  }

  return aos::vision::ImageFormat{(int)fmt.fmt.pix.width,
                                  (int)fmt.fmt.pix.height};
}

void Reader::Start() {
  LOG(DEBUG, "queueing buffers for the first time\n");
  v4l2_buffer buf;
  for (unsigned int i = 0; i < kNumBuffers; ++i) {
    CLEAR(buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    QueueBuffer(&buf);
  }
  LOG(DEBUG, "done with first queue\n");

  v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd_, VIDIOC_STREAMON, &type) == -1) {
    PLOG(FATAL, "ioctl VIDIOC_STREAMON(%d, %p)\n", fd_, &type);
  }
}

}  // namespace camera
