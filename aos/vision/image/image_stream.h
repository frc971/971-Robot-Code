#ifndef _AOS_VISION_IMAGE_IMAGE_STREAM_H_
#define _AOS_VISION_IMAGE_IMAGE_STREAM_H_

#include "aos/vision/events/epoll_events.h"
#include "aos/vision/image/reader.h"

#include <memory>

namespace aos {
namespace vision {

class ImageStreamEvent : public ::aos::events::EpollEvent {
 public:
  static std::unique_ptr<::camera::Reader> GetCamera(
      const std::string &fname, ImageStreamEvent *obj,
      camera::CameraParams params) {
    using namespace std::placeholders;
    std::unique_ptr<::camera::Reader> camread(new ::camera::Reader(
        fname,
        std::bind(&ImageStreamEvent::ProcessHelper, obj, _1, _2), params);
    camread->StartAsync();
    return camread;
  }

  explicit ImageStreamEvent(std::unique_ptr<::camera::Reader> reader)
      : ::aos::events::EpollEvent(reader->fd()), reader_(reader) {}

  explicit ImageStreamEvent(const std::string &fname,
                            camera::CameraParams params)
      : ImageStreamEvent(GetCamera(fname, this, params)) {}

  void ProcessHelper(DataRef data, uint64_t timestamp) {
    if (data.size() < 300) {
      LOG(INFO, "got bad img: %d of size(%lu)\n", (int)timestamp, data.size());
      return;
    }
    ProcessImage(data, timestamp);
  }
  virtual void ProcessImage(DataRef data, uint64_t timestamp) = 0;

  void ReadEvent(Context /*ctx*/) override { reader_->HandleFrame(); }

 private:
  std::unique_ptr<::camera::Reader> reader_;
};

}  // namespace vision
}  // namespace aos

#endif  // _AOS_VISION_DEBUG_IMAGE_STREAM_H_
