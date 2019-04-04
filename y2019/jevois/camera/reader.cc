#include "y2019/jevois/camera/reader.h"

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

#include "aos/time/time.h"
#include "glog/logging.h"

#define CLEAR(x) memset(&(x), 0, sizeof(x))

namespace y2019 {
namespace camera {

using ::camera::xioctl;

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
    PLOG(FATAL) << "Cannot identify '" << dev_name << "'";
  }
  if (!S_ISCHR(st.st_mode)) {
    PLOG(FATAL) << dev_name << " is no device";
  }

  fd_ = open(dev_name.c_str(), O_RDWR /* required */ | O_NONBLOCK, 0);
  if (fd_ == -1) {
    PLOG(FATAL) << "Cannot open '" << dev_name << "'";
  }

  Init();

  InitMMap();
  LOG(INFO) << "Bat Vision Successfully Initialized.";
}

void Reader::QueueBuffer(v4l2_buffer *buf) {
  if (xioctl(fd_, VIDIOC_QBUF, buf) == -1) {
    PLOG(WARNING) << "ioctl VIDIOC_QBUF(" << fd_ << ", " << &buf
                  << "). losing buf #" << buf->index;
  } else {
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
      PLOG(ERROR) << "ioctl VIDIOC_DQBUF(" << fd_ << ", " << &buf << ")";
    }
    return;
  }
  --queued_;

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
      PLOG(FATAL) << "ioctl VIDIOC_QUERYBUF(" << fd_ << ", " << &buf << ")";
    }
    buffers_[n].length = buf.length;
    buffers_[n].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                             MAP_SHARED, fd_, buf.m.offset);
    if (buffers_[n].start == MAP_FAILED) {
      PLOG(FATAL) << "mmap(NULL, " << buf.length
                  << ", PROT_READ | PROT_WRITE, MAP_SHARED, " << fd_ << ", "
                  << buf.m.offset << ")";
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
      LOG(FATAL) << dev_name_ << " does not support memory mapping\n";
    } else {
      LOG(FATAL) << "ioctl VIDIOC_REQBUFS(" << fd_ << ", " << &req << ")";
    }
  }
  queued_ = kNumBuffers;
  if (req.count != kNumBuffers) {
    LOG(FATAL) << "Insufficient buffer memory on " << dev_name_;
  }
}

// Sets one of the camera's user-control values.
// Prints the old and new values.
// Just prints a message if the camera doesn't support this control or value.
bool Reader::SetCameraControl(uint32_t id, const char *name, int value) {
  struct v4l2_control getArg = {id, 0U};
  int r;
  // Used to be: r = xioctl(fd_, VIDIOC_S_CTRL, &getArg);
  // Jevois wants this incorrect number below:.
  r = xioctl(fd_, 0xc00c561b, &getArg);
  if (r == 0) {
    if (getArg.value == value) {
      VLOG(1) << "Camera control " << name << " was already " << getArg.value;
      return true;
    }
  } else if (errno == EINVAL) {
    VLOG(1) << "Camera control " << name << " is invalid";
    errno = 0;
    return false;
  }

  struct v4l2_control setArg = {id, value};
  // Should be: r = xioctl(fd_, VIDIOC_S_CTRL, &setArg);
  // Jevois wants this incorrect number below:.
  r = xioctl(fd_, 0xc00c561c, &setArg);
  if (r == 0) {
    VLOG(1) << "Set camera control " << name << " from " << getArg.value
            << " to " << value;
    return true;
  }

  VLOG(1) << "Couldn't set camera control " << name << " to " << value;
  errno = 0;
  return false;
}

void Reader::Init() {
  v4l2_capability cap;
  if (xioctl(fd_, VIDIOC_QUERYCAP, &cap) == -1) {
    if (EINVAL == errno) {
      LOG(FATAL) << dev_name_ << " is no V4L2 device";
    } else {
      PLOG(FATAL) << "ioctl VIDIOC_QUERYCAP(" << fd_ << ", " << &cap << ")";
    }
  }
  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    LOG(FATAL) << dev_name_ << " is no video capture device";
  }
  if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
    LOG(FATAL) << dev_name_ << " does not support streaming i/o";
  }

  int camidx = -1;
  struct v4l2_input inp = {};
  while (true) {
    if (xioctl(fd_, VIDIOC_ENUMINPUT, &inp) == -1) {
      break;
    }
    if (inp.type == V4L2_INPUT_TYPE_CAMERA) {
      if (camidx == -1) camidx = inp.index;
      printf("Input %d [ %s ] is a camera sensor\n", inp.index, inp.name);
    } else
      printf("Input %d [ %s ] is not a camera sensor\n", inp.index, inp.name);
    ++inp.index;
  }

  if (xioctl(fd_, VIDIOC_S_INPUT, &camidx) == -1) {
    PLOG(FATAL) << "ioctl VIDIOC_S_INPUT(" << fd_ << ") failed";
  }
  printf("camera idx: %d\n", camidx);

  /* Select video input, video standard and tune here. */

  v4l2_format fmt;
  CLEAR(fmt);

  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd_, VIDIOC_G_FMT, &fmt) == -1) {
    PLOG(FATAL) << "ioctl VIDIC_G_FMT(" << fd_ << ", " << &fmt << ") failed";
  }

  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = params_.width();
  fmt.fmt.pix.height = params_.height();
  printf("setting format: %d, %d\n", params_.width(), params_.height());
  fmt.fmt.pix.field = V4L2_FIELD_NONE;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
  if (xioctl(fd_, VIDIOC_S_FMT, &fmt) == -1) {
    PLOG(FATAL) << "ioctl VIDIC_S_FMT(" << fd_ << ", " << &fmt << ") failed";
  }
  /* Note VIDIOC_S_FMT may change width and height. */

  /* Buggy driver paranoia. */
  unsigned int min = fmt.fmt.pix.width * 2;
  if (fmt.fmt.pix.bytesperline < min) fmt.fmt.pix.bytesperline = min;
  min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
  if (fmt.fmt.pix.sizeimage < min) fmt.fmt.pix.sizeimage = min;

  // set framerate
  struct v4l2_streamparm *setfps;
  setfps = (struct v4l2_streamparm *)calloc(1, sizeof(struct v4l2_streamparm));
  memset(setfps, 0, sizeof(struct v4l2_streamparm));
  setfps->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  setfps->parm.capture.timeperframe.numerator = 1;
  setfps->parm.capture.timeperframe.denominator = params_.fps();
  if (xioctl(fd_, VIDIOC_S_PARM, setfps) == -1) {
    PLOG(FATAL) << "ioctl VIDIOC_S_PARM(" << fd_ << ", " << setfps << ")";
  }
  LOG(INFO) << "framerate ended up at "
            << setfps->parm.capture.timeperframe.numerator << "/"
            << setfps->parm.capture.timeperframe.denominator;

  for (int j = 0; j < 2; ++j) {
    if (!SetCameraControl(V4L2_CID_EXPOSURE_AUTO, "V4L2_CID_EXPOSURE_AUTO",
                          V4L2_EXPOSURE_MANUAL)) {
      LOG(FATAL) << "Failed to set exposure";
    }

    if (!SetCameraControl(V4L2_CID_EXPOSURE_ABSOLUTE,
                          "V4L2_CID_EXPOSURE_ABSOLUTE", params_.exposure())) {
      LOG(FATAL) << "Failed to set exposure";
    }
    sleep(1);
  }
}

aos::vision::ImageFormat Reader::get_format() {
  struct v4l2_format fmt;
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd_, VIDIOC_G_FMT, &fmt) == -1) {
    PLOG(FATAL) << "ioctl VIDIC_G_FMT(" << fd_ << ", " << &fmt << ")";
  }

  return aos::vision::ImageFormat{(int)fmt.fmt.pix.width,
                                  (int)fmt.fmt.pix.height};
}

void Reader::Start() {
  VLOG(1) << "queueing buffers for the first time";
  v4l2_buffer buf;
  for (unsigned int i = 0; i < kNumBuffers; ++i) {
    CLEAR(buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    QueueBuffer(&buf);
  }
  VLOG(1) << "done with first queue";

  v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd_, VIDIOC_STREAMON, &type) == -1) {
    PLOG(FATAL) << "ioctl VIDIOC_STREAMON(" << fd_ << ", " << &type << ")";
  }
}

}  // namespace camera
}  // namespace y2019
