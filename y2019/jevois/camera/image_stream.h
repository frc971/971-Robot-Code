#ifndef Y2019_JEVOIS_CAMERA_IMAGE_STREAM_H_
#define Y2019_JEVOIS_CAMERA_IMAGE_STREAM_H_

#include "aos/vision/events/epoll_events.h"
#include "aos/vision/image/camera_params.pb.h"
#include "y2019/jevois/camera/reader.h"

#include <memory>

namespace y2019 {
namespace camera {

// Converts a camera reader into a virtual base class that calls ProcessImage
// on each new image.
class ImageStreamEvent : public ::aos::events::EpollEvent {
 public:
  static std::unique_ptr<Reader> GetCamera(const std::string &fname,
                                           ImageStreamEvent *obj,
                                           aos::vision::CameraParams params) {
    using namespace std::placeholders;
    std::unique_ptr<Reader> camread(new Reader(
        fname, std::bind(&ImageStreamEvent::ProcessHelper, obj, _1, _2),
        params));
    camread->StartAsync();
    return camread;
  }

  explicit ImageStreamEvent(std::unique_ptr<Reader> reader)
      : ::aos::events::EpollEvent(reader->fd()), reader_(std::move(reader)) {}

  explicit ImageStreamEvent(const std::string &fname,
                            aos::vision::CameraParams params)
      : ImageStreamEvent(GetCamera(fname, this, params)) {}

  void ProcessHelper(aos::vision::DataRef data,
                     aos::monotonic_clock::time_point timestamp);
  virtual void ProcessImage(aos::vision::DataRef data,
                            aos::monotonic_clock::time_point timestamp) = 0;

  void ReadEvent() override { reader_->HandleFrame(); }

  bool SetExposure(int abs_exp) {
    return reader_->SetCameraControl(V4L2_CID_EXPOSURE_ABSOLUTE,
                                     "V4L2_CID_EXPOSURE_ABSOLUTE", abs_exp);
  }

 private:
  std::unique_ptr<Reader> reader_;
};

}  // namespace camera
}  // namespace y2019

#endif  // Y2019_JEVOIS_CAMERA_IMAGE_STREAM_H_
