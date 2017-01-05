#ifndef AOS_VISION_IMAGE_READER_H_
#define AOS_VISION_IMAGE_READER_H_
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

#include <inttypes.h>
#include <functional>
#include <string>

#include "aos/common/time.h"
#include "aos/vision/image/V4L2.h"
#include "aos/vision/image/image_types.h"

namespace camera {

struct CameraParams {
  int32_t width;
  int32_t height;
  int32_t exposure;
  int32_t brightness;
  int32_t gain;
  int32_t fps;
};

class Reader {
 public:
  using ProcessCb = std::function<void(
      aos::vision::DataRef data, aos::monotonic_clock::time_point timestamp)>;
  Reader(const std::string &dev_name, ProcessCb process, CameraParams params);

  aos::vision::ImageFormat get_format();

  void HandleFrame();
  void StartAsync() {
    Start();
    MMapBuffers();
  }
  int fd() { return fd_; }

 private:
  void QueueBuffer(v4l2_buffer *buf);
  void InitMMap();
  bool SetCameraControl(uint32_t id, const char *name, int value);
  void Init();
  void Start();
  void MMapBuffers();
  // File descriptor of the camera
  int fd_;
  // Name of the camera device.
  std::string dev_name_;

  ProcessCb process_;

  // The number of buffers currently queued in v4l2.
  uint32_t queued_;
  struct Buffer;
  // TODO(parker): This should be a smart pointer, but it cannot
  // because the buffers are not ummapped.
  Buffer *buffers_;

  static const unsigned int kNumBuffers = 10;

  // set only at initialize
  CameraParams params_;
};

}  // namespace camera

#endif  // AOS_VISION_IMAGE_READER_H_
