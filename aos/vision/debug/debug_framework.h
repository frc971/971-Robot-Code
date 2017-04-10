#ifndef _AOS_VISION_DEBUG_DEBUG_FRAMEWORK_H_
#define _AOS_VISION_DEBUG_DEBUG_FRAMEWORK_H_

#include "aos/common/util/global_factory.h"
#include "aos/vision/blob/range_image.h"
#include "aos/vision/events/epoll_events.h"
#include "aos/vision/image/camera_params.pb.h"
#include "aos/vision/image/image_types.h"

namespace aos {
namespace vision {

class BlobStreamViewer;

// Implement per-filter to draw debug viewer information from the filter to
// the debug BlobStreamViewer.
class FilterHarness {
 public:
  virtual ~FilterHarness() {}

  // Apply the filter-specific thresholding logic.
  // Blob sources may not have this called at all.
  virtual RangeImage Threshold(ImagePtr image) = 0;

  // Each filter can only be used by one debug viewer. This will
  // get called before calling any other methods.
  virtual void InstallViewer(BlobStreamViewer * /*viewer*/) {}

  // One frame worth of blobs. Returns if the frame is "interesting".
  virtual bool HandleBlobs(BlobList imgs, ImageFormat fmt) = 0;

  // One frame worth of blobs. Returns if the frame is "interesting".
  // Fast version that does no drawing.
  virtual bool JustCheckForTarget(BlobList imgs, ImageFormat fmt) {
    return HandleBlobs(std::move(imgs), fmt);
  }

  // Register key press handler.
  virtual std::function<void(uint32_t)> RegisterKeyPress() {
    return std::function<void(uint32_t)>();
  }
};

// For ImageSource implementations only. Allows registering key press events
// and installing new blob lists and jpegs.
class DebugFrameworkInterface {
 public:
  virtual ~DebugFrameworkInterface() {}

  void InstallKeyPress(std::function<void(uint32_t)> key_press_event) {
    key_press_events_.emplace_back(std::move(key_press_event));
  }

  // The return value bool here for all of these is
  // if the frame is "interesting" ie has a target.
  virtual bool NewJpeg(DataRef data) = 0;

  virtual bool NewBlobList(BlobList blob_list, ImageFormat fmt) = 0;

  virtual bool JustCheckForTarget(BlobList imgs, ImageFormat fmt) = 0;

  // Expose a EpollLoop to allow waiting for events.
  virtual aos::events::EpollLoop *Loop() = 0;

  virtual const CameraParams &camera_params() = 0;

  virtual BlobStreamViewer *viewer() = 0;

 protected:
  const std::vector<std::function<void(uint32_t)>> &key_press_events() {
    return key_press_events_;
  }

 private:
  std::vector<std::function<void(uint32_t)>> key_press_events_;
};

// Implemented by each source type. Will stream frames to
// DebugFrameworkInterface.
class ImageSource {
 public:
  virtual ~ImageSource() {}

  // Printed when you call: debug_viewer help.
  virtual const char *GetHelpMessage() { return "    No help string :(\n"; }

  // Start streaming frames to DebugFrameworkInterface.
  virtual void Init(const std::string &args,
                    DebugFrameworkInterface *interface) = 0;
};

// Factory for ImageSource.
SETUP_FACTORY(ImageSource);

#define REGISTER_IMAGE_SOURCE(key, SubClass) \
  REGISTER_SUBCLASS_BY_KEY(key, ::aos::vision::ImageSource, SubClass)

// Runs loop and never returns.
// Feeds into a generic filter.
void DebugFrameworkMain(int argc, char **argv, FilterHarness *filter,
                        CameraParams camera_params);

}  // namespace vision
}  // namespace aos

#endif  // _AOS_VISION_DEBUG_DEBUG_FRAMEWORK_H_
